#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

// --- CẤU HÌNH THÔNG SỐ HỆ THỐNG ---
#define ENCODER_PULSES_PER_REV  1980.0f  // Tổng số xung/vòng trục chính (Đã nhân 4 ở chế độ X4)
#define MAX_SPEED_LIMIT         60.0f   // Ngưỡng tốc độ cảnh báo nguy hiểm (RPM)
#define SPEED_STEP              10.0f    // Mức tăng giảm tốc độ mỗi lần bấm nút
#define LCD_I2C_ADDR            0x4C     // Địa chỉ I2C: 0x27 << 1 (Đổi thành 0x7E nếu dùng chip 0x3F)

// Các bit điều khiển của module PCF8574 nối với LCD
#define LCD_BIT_RS              (1 << 0)
#define LCD_BIT_RW              (1 << 1)
#define LCD_BIT_EN              (1 << 2)
#define LCD_BIT_BACKLIGHT       (1 << 3)

// --- BIẾN TOÀN CỤC ---
volatile float target_speed = 0.0f; // Setpoint chung cho cả 2 bánh (RPM)
volatile float speed_m1 = 0.0f;
volatile float speed_m2 = 0.0f;

// Hệ số bộ điều khiển PID (Có thể tinh chỉnh lại dựa trên tải trọng thực tế của xe)
float Kp = 2.38f, Ki = 0.95f, Kd = 0.1f;
float err_m1 = 0, err_m2 = 0;
float prev_err_m1 = 0, prev_err_m2 = 0;
float integral_m1 = 0, integral_m2 = 0;
float derivative_m1 = 0, derivative_m2 = 0;

// --- 1. KHỞI TẠO CÁC CHÂN GPIO (NÚT BẤM, LED, CÒI, CHÂN HƯỚNG MOTOR) ---
void GPIO_Init(void) {
    // Cấp xung Clock cho Port A, B, C
    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);

    // Chân PA0, PA1: Cấu hình Input (00) cho Nút bấm
    GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    // Bật Điện trở kéo lên Pull-up (01) cho chân PA0, PA1
    GPIOA->PUPDR &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOA->PUPDR |=  ((1 << (0 * 2)) | (1 << (1 * 2)));

    // SỬA ĐỔI: Cấu hình chân PA5 làm Output (01) cho LED cảnh báo (thay vì PC13)
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |=  (1 << (5 * 2));

    // Cấu hình các chân điều khiển trên PORT B làm Output (01):
    // PB0 (Buzzer), PB1 (IN2), PB2 (IN4), PB4 (IN1), PB5 (IN3)
    GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->MODER |=  ((1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2)));
}

// --- 2. KHỞI TẠO NGOẠI VI I2C1 ĐỂ ĐIỀU KHIỂN MÀN HÌNH (PB8, PB9) ---
void I2C1_Init(void) {
    RCC->AHB1ENR |= (1 << 1);  // Bật Clock GPIOB
    RCC->APB1ENR |= (1 << 21); // Bật Clock bộ I2C1

    // Chuyển chân PB8, PB9 sang Alternate Function Mode (10)
    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |=  ((2 << (8 * 2)) | (2 << (9 * 2)));

    // Gán chức năng AF4 (I2C1) cho chân PB8, PB9
    GPIOB->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOB->AFR[1] |=  ((4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4)));

    // Ép chân thành cấu hình Output Open-Drain (Bắt buộc đối với đường truyền Bus I2C)
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);

    // Thiết lập vận tốc Standard Mode (100 kHz) tại tần số APB1 = 16 MHz
    I2C1->CR2 &= ~(0x3F);
    I2C1->CR2 |= 16;           // Chu kỳ xung nhịp ngoại vi là 16 MHz
    I2C1->CCR &= ~(1 << 15);   // Chọn chế độ Standard
    I2C1->CCR |= 80;           // CCR = 16,000,000 / (2 * 100,000) = 80
    I2C1->TRISE = 17;          // Thơi gian sườn lên cực đại = (1000ns / 62.5ns) + 1 = 17

    I2C1->CR1 |= (1 << 0);     // Kích hoạt bộ I2C1 (PE = 1)
}

void I2C1_WriteByte(uint8_t dev_addr, uint8_t data) {
    I2C1->CR1 |= (1 << 8);                     // Tạo tín hiệu START
    while (!(I2C1->SR1 & (1 << 0)));           // Chờ cờ SB dựng (Start thành công)

    I2C1->DR = dev_addr;                       // Gửi địa chỉ Slave + bit Write
    while (!(I2C1->SR1 & (1 << 1)));           // Chờ cờ ADDR dựng (Gửi địa chỉ xong)
    (void)I2C1->SR2;                           // Đọc thanh ghi SR2 để xóa cờ ADDR

    while (!(I2C1->SR1 & (1 << 7)));           // Chờ thanh ghi dữ liệu trống (TXE)
    I2C1->DR = data;                           // Nạp dữ liệu thô vào thanh ghi dịch
    while (!(I2C1->SR1 & (1 << 2)));           // Chờ dữ liệu truyền đi hoàn tất (BTF)

    I2C1->CR1 |= (1 << 9);                     // Tạo tín hiệu STOP giải phóng Bus
}

// --- 3. CÁC HÀM ĐIỀU KHIỂN GIAO TIẾP LCD I2C ---
void LCD_I2C_SendCommand(uint8_t cmd) {
    uint8_t data_u = (cmd & 0xF0) | LCD_BIT_BACKLIGHT;
    uint8_t data_l = ((cmd << 4) & 0xF0) | LCD_BIT_BACKLIGHT;

    // Gửi nửa Byte cao (Nháy chân EN để chốt dữ liệu)
    I2C1_WriteByte(LCD_I2C_ADDR, data_u | LCD_BIT_EN);  HAL_Delay(1);
    I2C1_WriteByte(LCD_I2C_ADDR, data_u);               HAL_Delay(1);
    // Gửi nửa Byte thấp
    I2C1_WriteByte(LCD_I2C_ADDR, data_l | LCD_BIT_EN);  HAL_Delay(1);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l);               HAL_Delay(1);
}

void LCD_I2C_SendData(uint8_t data) {
    uint8_t data_u = (data & 0xF0) | LCD_BIT_RS | LCD_BIT_BACKLIGHT;
    uint8_t data_l = ((data << 4) & 0xF0) | LCD_BIT_RS | LCD_BIT_BACKLIGHT;

    I2C1_WriteByte(LCD_I2C_ADDR, data_u | LCD_BIT_EN);  HAL_Delay(1);
    I2C1_WriteByte(LCD_I2C_ADDR, data_u);               HAL_Delay(1);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l | LCD_BIT_EN);  HAL_Delay(1);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l);               HAL_Delay(1);
}

void LCD_I2C_Init(void) {
    HAL_Delay(50);
    LCD_I2C_SendCommand(0x33); // Định dạng hàng lệnh chuẩn
    LCD_I2C_SendCommand(0x32); // Ép màn hình chuyển hẳn sang Mode 4-bit
    LCD_I2C_SendCommand(0x28); // Thiết lập hiển thị 2 dòng, font ký tự 5x8
    LCD_I2C_SendCommand(0x0C); // Mở màn hình hiển thị, ẩn thanh con trỏ nhấp nháy
    LCD_I2C_SendCommand(0x06); // Tự động tăng vị trí con trỏ sang phải khi in ký tự
    LCD_I2C_SendCommand(0x01); // Xóa sạch bộ nhớ màn hình
    HAL_Delay(2);
}

void LCD_I2C_Print(char *str) {
    while (*str) LCD_I2C_SendData(*str++);
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_I2C_SendCommand(address);
}

// --- 4. KHỞI TẠO BỘ PHÁT XUNG PWM TẦN SỐ CAO 10KHZ (TIM1_CH1 -> PA8, TIM1_CH2 -> PA9) ---
void Motor_PWM_Init(void) {
    RCC->APB2ENR |= (1 << 0); // Kích hoạt xung cho Advanced Timer 1

    // Cấu hình chân PA8, PA9 sang chế độ Alternate Function (10)
    GPIOA->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOA->MODER |=  ((2 << (8 * 2)) | (2 << (9 * 2)));

    // Gán chức năng AF1 (TIM1) cho chân PA8 và PA9
    GPIOA->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOA->AFR[1] |=  ((1 << ((8 - 8) * 4)) | (1 << ((9 - 8) * 4)));

    // Thiết lập thông số cơ sở Thời gian cho TIM1 đạt tần số PWM lý tưởng = 10 kHz
    TIM1->PSC = 15;   // Tần số đếm đếm = 16MHz / (15 + 1) = 1 MHz
    TIM1->ARR = 99;   // Độ phân giải chu kỳ gồm 100 bước (Vận tốc đặt chạy từ 0 -> 99%)

    // Cấu hình thanh ghi điều khiển kênh PWM Mode 1 cho CH1 và CH2
    TIM1->CCMR1 &= ~((0x7 << 4) | (0x7 << 12));
    TIM1->CCMR1 |=  ((6 << 4) | (6 << 12));
    TIM1->CCMR1 |=  ((1 << 3) | (1 << 11)); // Bật Preload Enable tăng độ ổn định nạp dữ liệu xung

    TIM1->CCER |= (1 << 0) | (1 << 4);     // Bật rơ-le ngõ ra vật lý cho cả hai kênh
    TIM1->BDTR |= (1 << 15);                // BẮT BUỘC: Bật cờ MOE (Main Output Enable) đối với Timer nâng cao
    TIM1->CR1  |= (1 << 0);                 // Khởi chạy Timer 1
}

// Hàm xuất tín hiệu điều khiển hướng tách biệt 4 chân độc lập kèm phanh điện từ chủ động
void Set_Motor_Outputs(int pwm_m1, int pwm_m2) {
    // --- XỬ LÝ ĐỘNG CƠ 1 (BÁNH TRÁI) ---
    if (pwm_m1 > 0) {
        GPIOB->ODR |=  (1 << 4); // PB4 (IN1) = 1 (Quay thuận)
        GPIOB->ODR &= ~(1 << 1); // PB1 (IN2) = 0
        TIM1->CCR1 = pwm_m1;
    } else if (pwm_m1 < 0) {
        GPIOB->ODR &= ~(1 << 4); // PB4 (IN1) = 0 (Quay nghịch)
        GPIOB->ODR |=  (1 << 1); // PB1 (IN2) = 1
        TIM1->CCR1 = -pwm_m1;
    } else {
        // pwm == 0 -> Kích hoạt chế độ phanh ngắn mạch cuộn dây (IN1 = 1, IN2 = 1) giúp xe dừng lập tức
        GPIOB->ODR |= (1 << 4);
        GPIOB->ODR |= (1 << 1);
        TIM1->CCR1 = 0;
    }

    // --- XỬ LÝ ĐỘNG CƠ 2 (BÁNH PHẢI) ---
    if (pwm_m2 > 0) {
        GPIOB->ODR |=  (1 << 5); // PB5 (IN3) = 1 (Quay thuận)
        GPIOB->ODR &= ~(1 << 2); // PB2 (IN4) = 0
        TIM1->CCR2 = pwm_m2;
    } else if (pwm_m2 < 0) {
        GPIOB->ODR &= ~(1 << 5); // PB5 (IN3) = 0 (Quay nghịch) - ĐÃ SỬA LỖI CÚ PHÁP THỪA KÝ TỰ
        GPIOB->ODR |=  (1 << 2); // PB2 (IN4) = 1
        TIM1->CCR2 = -pwm_m2;
    } else {
        // pwm == 0 -> Kích hoạt chế độ phanh ngắn mạch cuộn dây (IN3 = 1, IN4 = 1)
        GPIOB->ODR |= (1 << 5);
        GPIOB->ODR |= (1 << 2);
        TIM1->CCR2 = 0;
    }
}

// --- 5. CẤU HÌNH ĐỌC ENCODER TỰ ĐỘNG CHẾ ĐỘ X4 (TIM3 VÀ TIM4) ---
void Encoder_Init(void) {
    RCC->APB1ENR |= (1 << 1) | (1 << 2); // Cấp xung nuôi bộ đếm Timer 3 và Timer 4

    // --- CẤU HÌNH CHO MOTOR 1 (TIM3: PA6, PA7) ---
    // Chuyển PA6, PA7 sang Alternate Function Mode (10)
    GPIOA->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->MODER |=  ((2 << (6 * 2)) | (2 << (7 * 2)));
    // Gán chức năng AF2 (TIM3)
    GPIOA->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOA->AFR[0] |=  ((2 << (6 * 4)) | (2 << (7 * 4)));
    // KHẮC PHỤC: Bật điện trở kéo lên Pull-up (Ghi giá trị 01 vào cặp bit)
    GPIOA->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->PUPDR |=  ((1 << (6 * 2)) | (1 << (7 * 2)));

    // --- CẤU HÌNH CHO MOTOR 2 (TIM4: PB6, PB7) ---
    // Chuyển PB6, PB7 sang Alternate Function Mode (10)
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |=  ((2 << (6 * 2)) | (2 << (7 * 2)));
    // Gán chức năng AF2 (TIM4)
    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |=  ((2 << (6 * 4)) | (2 << (7 * 4)));
    // KHẮC PHỤC: Bật điện trở kéo lên Pull-up (Ghi giá trị 01 vào cặp bit)
    GPIOB->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->PUPDR |=  ((1 << (6 * 2)) | (1 << (7 * 2)));

    // Ép hai bộ Timer hoạt động ở chế độ giải mã Encoder Mode 3 (X4)
    TIM3->SMCR |= 3; TIM4->SMCR |= 3;
    TIM3->CCMR1 |= (1 << 0) | (1 << 8); // Bộ lọc nhiễu đầu vào ngõ hiển thị
    TIM4->CCMR1 |= (1 << 0) | (1 << 8);

    TIM3->ARR = 0xFFFFFFFF; TIM3->CR1 |= (1 << 0);
    TIM4->ARR = 0xFFFFFFFF; TIM4->CR1 |= (1 << 0);
}

// --- 6. KHỞI TẠO TRUYỀN THÔNG SERIAL MONITOR (USART2 -> BAUD 115200) ---
void UART2_Init(void) {
    RCC->APB1ENR |= (1 << 17); // Xung USART2
    GPIOA->MODER  &= ~(3 << (2 * 2));
    GPIOA->MODER  |=  (2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |=  (7 << (2 * 4)); // AF7 tương ứng USART2_TX

    // Cài đặt Baudrate = 115200 với Clock hệ thống 16MHz
    USART2->BRR = (8 << 4) | 11;
    USART2->CR1 |= (1 << 3) | (1 << 13); // Bật bộ truyền (TE) và Bật toàn bộ UART (UE)
}

void UART2_SendString(char *str) {
    while (*str) {
        while (!(USART2->SR & (1 << 7))); // Chờ thanh ghi đệm trống (TXE)
        USART2->DR = *str++;
    }
}

// --- 7. BỘ TIMER NGẮT TRUNG TÂM CHU KỲ CHUẨN 50MS VÀ TÍNH TOÁN VÒNG PID SONG SONG (TIM5) ---
void Timer5_PID_Interrupt_Init(void) {
    RCC->APB1ENR |= (1 << 3); // Cấp xung nuôi Timer 5 (Bộ đếm 32-bit rộng)
    TIM5->PSC = 15999;        // Tần số tick = 16MHz / 16000 = 1000Hz (1ms/tick)
    TIM5->ARR = 49;           // Đặt giá trị chu kỳ chốt ngắt bằng 50 ticks = 50ms
    TIM5->DIER |= (1 << 0);   // Kích hoạt cờ ngắt khi cập nhật chu kỳ (UIE)

    NVIC_SetPriority(TIM5_IRQn, 0); // Đặt mức ưu tiên cao nhất cho tác vụ lõi điều khiển
    NVIC_EnableIRQ(TIM5_IRQn);      // Đăng ký ngắt vào trình quản lý vector NVIC
    TIM5->CR1 |= (1 << 0);          // Kích hoạt Timer 5
}

void TIM5_IRQHandler(void) {
    if (TIM5->SR & (1 << 0)) {
        // Đọc thanh ghi dưới dạng số nguyên 16-bit có dấu
        int16_t cnt_m1 = (int16_t)TIM3->CNT; TIM3->CNT = 0;
        int16_t cnt_m2 = (int16_t)TIM4->CNT; TIM4->CNT = 0;


        // Tính toán tốc độ thực tế
        speed_m1 = ((float)cnt_m1 / 1980.0f) * 1200.0f;
        speed_m2 = ((float)cnt_m2 / 1980.0f) * 1200.0f;

        // --- GIẢI THUẬT ĐIỀU KHIỂN PID MẠCH KÍN CHO ĐỘNG CƠ TRÁI (MOTOR 1) ---
        err_m1 = target_speed - speed_m1;
        integral_m1 += err_m1 * 0.05f;
        if (integral_m1 > 150.0f) integral_m1 = 150.0f; else if (integral_m1 < -150.0f) integral_m1 = -150.0f; // Chống bão hòa tích phân
        derivative_m1 = (err_m1 - prev_err_m1) / 0.05f;
        int output_m1 = (int)((Kp * err_m1) + (Ki * integral_m1) + (Kd * derivative_m1));
        prev_err_m1 = err_m1;

        // --- GIẢI THUẬT ĐIỀU KHIỂN PID MẠCH KÍN CHO ĐỘNG CƠ PHẢI (MOTOR 2) ---
        err_m2 = target_speed - speed_m2;
        integral_m2 += err_m2 * 0.05f;
        if (integral_m2 > 50.0f) integral_m2 = 50.0f; else if (integral_m2 < -50.0f) integral_m2 = -50.0f;
        derivative_m2 = (err_m2 - prev_err_m2) / 0.05f;
        int output_m2 = (int)((Kp * err_m2) + (Ki * integral_m2) + (Kd * derivative_m2));
        prev_err_m2 = err_m2;

        // GIỚI HẠN ĐẦU RA PID (SATURATION CLAMPING):
        // Vì TIM1->ARR đặt bằng 99, giá trị băm xung chỉ được phép nằm trong khoảng từ -99 đến 99.
        if (output_m1 > 99)  output_m1 = 99;
        if (output_m1 < -99) output_m1 = -99;
        if (output_m2 > 99)  output_m2 = 99;
        if (output_m2 < -99) output_m2 = -99;

        // Nếu tốc độ đặt bằng 0, cưỡng bức dừng hẳn và xóa tích phân chống trôi xe
        if (target_speed == 0.0f) {
            Set_Motor_Outputs(0, 0);
            integral_m1 = 0;
            integral_m2 = 0;
        } else {
            // Xuất giá trị đã được giới hạn an toàn ra động cơ
            Set_Motor_Outputs(output_m1, output_m2);
        }

        TIM5->SR &= ~(1 << 0); // Xóa cờ ngắt phần cứng sau khi hoàn tất tính toán
    }
}

// --- 8. QUÉT TRẠNG THÁI NÚT BẤM VÀ XỬ LÝ CHỐNG RUNG PHÍM ---
void Process_Buttons(void) {
    // Nhấn nút tăng tốc (PA0)
    if (!(GPIOA->IDR & (1 << 0))) {
        HAL_Delay(20); // Trễ 20ms chống nhiễu sườn điện áp vật lý
        if (!(GPIOA->IDR & (1 << 0))) {
            target_speed += SPEED_STEP;
            if (target_speed > 130.0f) target_speed = 130.0f; // Giới hạn trần tốc độ phần cứng GA25
            while (!(GPIOA->IDR & (1 << 0))); // Chờ nhả phím hoàn toàn
        }
    }
    // Nhấn nút giảm tốc (PA1)
    if (!(GPIOA->IDR & (1 << 1))) {
        HAL_Delay(20);
        if (!(GPIOA->IDR & (1 << 1))) {
            target_speed -= SPEED_STEP;
            if (target_speed < 0.0f) target_speed = 0.0f; // Chặn tốc độ không âm (Chỉ cho tiến thẳng)
            while (!(GPIOA->IDR & (1 << 1)));
        }
    }
}

// --- CHƯƠNG TRÌNH CHÍNH (MAIN LOOP) ---
int main(void) {
    HAL_Init(); // Khởi tạo thư viện HAL (Sử dụng hàm nền HAL_Delay())

    // Khởi tạo toàn bộ các module phần cứng
    GPIO_Init();
    I2C1_Init();
    LCD_I2C_Init();
    Motor_PWM_Init();
    Encoder_Init();
    UART2_Init();
    Timer5_PID_Interrupt_Init();

    char tx_buffer[128];
    char lcd_buffer[16];

    // Màn hình khởi động chào mừng
    LCD_I2C_SetCursor(0, 0);
    LCD_I2C_Print("AGV SYS ACTIVE");
    HAL_Delay(1000);
    LCD_I2C_SendCommand(0x01); // Xóa màn hình chuẩn bị vào vòng lặp

    while (1) {
        Process_Buttons(); // Liên tục quét nút bấm tăng/giảm Setpoint

        // --- HỆ THỐNG GIÁM SÁT AN TOÀN VÀ CẢNH BÁO QUÁ TỐC ĐỘ ---
        if (speed_m1 > MAX_SPEED_LIMIT || speed_m2 > MAX_SPEED_LIMIT) {
            // SỬA ĐỔI: Chuyển sang kích hoạt LED tại chân PA5 (Mạch Nucleo-F401RE có LED onboard nối PA5 dạng Active High)
            GPIOA->ODR |=  (1 << 5);  // Bật LED cảnh báo PA5 (Mức cao)
            GPIOB->ODR |=  (1 << 0);  // Bật còi hú liên tục báo động
        } else {
            GPIOA->ODR &= ~(1 << 5);  // Tắt LED PA5 (Mức thấp)
            GPIOB->ODR &= ~(1 << 0);  // Tắt còi
        }
        char tx_buffer_1[64];
        sprintf(tx_buffer_1, "%.1f %.1f %.1f\r\n", target_speed, speed_m1, speed_m2);

        // Gửi chuỗi này qua cổng UART (Tên hàm tùy thuộc vào code của bạn, ví dụ: UART2_SendString)
        UART2_SendString(tx_buffer_1);
        // --- ĐẨY DỮ LIỆU TELEMETRY LÊN MÁY TÍNH QUA SERIAL MONITOR ---
        sprintf(tx_buffer, "Setpoint: %.1f | Left_M1: %.1f | Right_M2: %.1f RPM\r\n",
                target_speed, speed_m1, speed_m2);
        UART2_SendString(tx_buffer);

        // --- CẬP NHẬT THÔNG TIN HIỂN THỊ TRÊN MÀN HÌNH LCD I2C ---
        LCD_I2C_SetCursor(0, 0);
        sprintf(lcd_buffer, "SP:%.1f RPM     ", target_speed);
        LCD_I2C_Print(lcd_buffer);

        LCD_I2C_SetCursor(1, 0);
        sprintf(lcd_buffer, "L:%.1f R:%.1f ", speed_m1, speed_m2);
        LCD_I2C_Print(lcd_buffer);

        HAL_Delay(100); // Chu kỳ làm mới màn hình hiển thị 10Hz tránh nghẽn Bus
    }
}
