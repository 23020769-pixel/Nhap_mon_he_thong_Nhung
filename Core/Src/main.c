#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

// --- CẤU HÌNH THÔNG SỐ HỆ THỐNG ---
#define ENCODER_PULSES_PER_REV  1980.0f  // Tổng số xung/vòng trục chính (Đã nhân 4 ở chế độ X4)
#define MAX_SPEED_LIMIT         60.0f    // Ngưỡng tốc độ cảnh báo nguy hiểm (RPM)
#define SPEED_STEP              10.0f    // Mức tăng giảm tốc độ mỗi lần bấm nút
#define LCD_I2C_ADDR            0x4C     // Địa chỉ I2C: 0x27 << 1 (Đổi thành 0x7E nếu dùng chip 0x3F)

// Các bit điều khiển của module PCF8574 nối với LCD
#define LCD_BIT_RS              (1 << 0)
#define LCD_BIT_RW              (1 << 1)
#define LCD_BIT_EN              (1 << 2)
#define LCD_BIT_BACKLIGHT       (1 << 3)
#define BUTTON_DEBOUNCE_TIME    20       // Thời gian chống nhiễu nút bấm (ms)

// --- BIẾN TOÀN CỤC ---
volatile float target_speed = 0.0f; // Setpoint chung cho cả 2 bánh (RPM)
volatile float speed_m1 = 0.0f;
volatile float speed_m2 = 0.0f;

// Hệ số bộ điều khiển PID
float Kp = 2.0f, Ki = 0.5f, Kd = 0.05f;
float err_m1 = 0, err_m2 = 0;
float prev_err_m1 = 0, prev_err_m2 = 0;
float integral_m1 = 0, integral_m2 = 0;
float derivative_m1 = 0, derivative_m2 = 0;

// Định nghĩa trạng thái nút bấm
typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_DEBOUNCE,
    BTN_STATE_WAIT_RELEASE
} ButtonState_t;

// ==============================================================================
// CÁC HÀM TIỆN ÍCH CƠ SỞ (PHẢI ĐẶT TRƯỚC ĐỂ CÁC HÀM DƯỚI GỌI KHÔNG BỊ LỖI)
// ==============================================================================

// Hàm delay micro-giây chính xác tương đối cho STM32F4
void delay_us(uint32_t us) {
    // SystemCoreClock mặc định là 16000000 (16MHz) nếu không cấu hình lại PLL
    uint32_t count = us * (SystemCoreClock / 4000000);
    while (count--) {
        __NOP(); // Lệnh rỗng tiêu tốn 1 chu kỳ máy
    }
}

// ==============================================================================
// KHỞI TẠO PHẦN CỨNG
// ==============================================================================

// --- 1. KHỞI TẠO CÁC CHÂN GPIO (NÚT BẤM, LED, CÒI, CHÂN HƯỚNG MOTOR) ---
void GPIO_Init(void) {
    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2); // Clock Port A, B, C

    // Nút bấm PA0, PA1 (Input, Pull-up)
    GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOA->PUPDR &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOA->PUPDR |=  ((1 << (0 * 2)) | (1 << (1 * 2)));

    // LED cảnh báo PA5 (Output)
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |=  (1 << (5 * 2));

    // Motor Direction & Buzzer PB0, PB1, PB2, PB4, PB5 (Output)
    GPIOB->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->MODER |=  ((1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (2 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2)));
}

// --- 2. KHỞI TẠO NGOẠI VI I2C1 (PB8, PB9) ---
void I2C1_Init(void) {
    RCC->AHB1ENR |= (1 << 1);
    RCC->APB1ENR |= (1 << 21);

    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |=  ((2 << (8 * 2)) | (2 << (9 * 2)));
    GPIOB->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOB->AFR[1] |=  ((4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4)));
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);

    I2C1->CR2 &= ~(0x3F);
    I2C1->CR2 |= 16;
    I2C1->CCR &= ~(1 << 15);
    I2C1->CCR |= 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= (1 << 0);
}

void I2C1_WriteByte(uint8_t dev_addr, uint8_t data) {
    I2C1->CR1 |= (1 << 8);
    while (!(I2C1->SR1 & (1 << 0)));
    I2C1->DR = dev_addr;
    while (!(I2C1->SR1 & (1 << 1)));
    (void)I2C1->SR2;
    while (!(I2C1->SR1 & (1 << 7)));
    I2C1->DR = data;
    while (!(I2C1->SR1 & (1 << 2)));
    I2C1->CR1 |= (1 << 9);
}

// --- 3. ĐIỀU KHIỂN GIAO TIẾP LCD I2C ---
void LCD_I2C_SendCommand(uint8_t cmd) {
    uint8_t data_u = (cmd & 0xF0) | LCD_BIT_BACKLIGHT;
    uint8_t data_l = ((cmd << 4) & 0xF0) | LCD_BIT_BACKLIGHT;

    I2C1_WriteByte(LCD_I2C_ADDR, data_u | LCD_BIT_EN);  delay_us(50);
    I2C1_WriteByte(LCD_I2C_ADDR, data_u);               delay_us(50);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l | LCD_BIT_EN);  delay_us(50);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l);               delay_us(50);
}

void LCD_I2C_SendData(uint8_t data) {
    uint8_t data_u = (data & 0xF0) | LCD_BIT_RS | LCD_BIT_BACKLIGHT;
    uint8_t data_l = ((data << 4) & 0xF0) | LCD_BIT_RS | LCD_BIT_BACKLIGHT;

    I2C1_WriteByte(LCD_I2C_ADDR, data_u | LCD_BIT_EN);  delay_us(50);
    I2C1_WriteByte(LCD_I2C_ADDR, data_u);               delay_us(50);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l | LCD_BIT_EN);  delay_us(50);
    I2C1_WriteByte(LCD_I2C_ADDR, data_l);               delay_us(50);
}

void LCD_I2C_Init(void) {
    delay_us(50000); // Thay thế HAL_Delay(50)
    LCD_I2C_SendCommand(0x33);
    LCD_I2C_SendCommand(0x32);
    LCD_I2C_SendCommand(0x28);
    LCD_I2C_SendCommand(0x0C);
    LCD_I2C_SendCommand(0x06);
    LCD_I2C_SendCommand(0x01);
    delay_us(2000);  // Thay thế HAL_Delay(2)
}

void LCD_I2C_Print(char *str) {
    while (*str) LCD_I2C_SendData(*str++);
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_I2C_SendCommand(address);
}

// --- 4. KHỞI TẠO BỘ PHÁT XUNG PWM TẦN SỐ CAO 10KHZ (TIM1) ---
void Motor_PWM_Init(void) {
    RCC->APB2ENR |= (1 << 0);
    GPIOA->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOA->MODER |=  ((2 << (8 * 2)) | (2 << (9 * 2)));
    GPIOA->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOA->AFR[1] |=  ((1 << ((8 - 8) * 4)) | (1 << ((9 - 8) * 4)));

    TIM1->PSC = 15;
    TIM1->ARR = 99;
    TIM1->CCMR1 &= ~((0x7 << 4) | (0x7 << 12));
    TIM1->CCMR1 |=  ((6 << 4) | (6 << 12));
    TIM1->CCMR1 |=  ((1 << 3) | (1 << 11));
    TIM1->CCER |= (1 << 0) | (1 << 4);
    TIM1->BDTR |= (1 << 15);
    TIM1->CR1  |= (1 << 0);
}

void Set_Motor_Outputs(int pwm_m1, int pwm_m2) {
    // --- MOTOR 1 ---
    if (pwm_m1 > 0) {
        // Chạy tiến
        GPIOB->ODR |=  (1 << 4);
        GPIOB->ODR &= ~(1 << 1);
        TIM1->CCR1 = pwm_m1;
    } else {
        // Phanh/Dừng (Không bao giờ chạy lùi)
        GPIOB->ODR |= (1 << 4);
        GPIOB->ODR |= (1 << 1);
        TIM1->CCR1 = 0;
    }

    // --- MOTOR 2 ---
    if (pwm_m2 > 0) {
        // Chạy tiến
        GPIOB->ODR |=  (1 << 5);
        GPIOB->ODR &= ~(1 << 2);
        TIM1->CCR2 = pwm_m2;
    } else {
        // Phanh/Dừng (Không bao giờ chạy lùi)
        GPIOB->ODR |= (1 << 5);
        GPIOB->ODR |= (1 << 2);
        TIM1->CCR2 = 0;
    }
}

// --- 5. CẤU HÌNH ĐỌC ENCODER TỰ ĐỘNG CHẾ ĐỘ X4 (TIM3 VÀ TIM4) ---
void Encoder_Init(void) {
    RCC->APB1ENR |= (1 << 1) | (1 << 2);

    // TIM3 (PA6, PA7)
    GPIOA->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->MODER |=  ((2 << (6 * 2)) | (2 << (7 * 2)));
    GPIOA->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOA->AFR[0] |=  ((2 << (6 * 4)) | (2 << (7 * 4)));
    GPIOA->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOA->PUPDR |=  ((1 << (6 * 2)) | (1 << (7 * 2)));

    // TIM4 (PB6, PB7)
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |=  ((2 << (6 * 2)) | (2 << (7 * 2)));
    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |=  ((2 << (6 * 4)) | (2 << (7 * 4)));
    GPIOB->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->PUPDR |=  ((1 << (6 * 2)) | (1 << (7 * 2)));

    TIM3->SMCR |= 3; TIM4->SMCR |= 3;
    TIM3->CCMR1 |= (1 << 0) | (1 << 8);
    TIM4->CCMR1 |= (1 << 0) | (1 << 8);
    TIM3->ARR = 0xFFFFFFFF; TIM3->CR1 |= (1 << 0);
    TIM4->ARR = 0xFFFFFFFF; TIM4->CR1 |= (1 << 0);
}

// --- 6. KHỞI TẠO TRUYỀN THÔNG SERIAL MONITOR (USART2) ---
void UART2_Init(void) {
    RCC->APB1ENR |= (1 << 17);
    GPIOA->MODER  &= ~(3 << (2 * 2));
    GPIOA->MODER  |=  (2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |=  (7 << (2 * 4));
    USART2->BRR = (8 << 4) | 11;
    USART2->CR1 |= (1 << 3) | (1 << 13);
}

void UART2_SendString(char *str) {
    while (*str) {
        while (!(USART2->SR & (1 << 7)));
        USART2->DR = *str++;
    }
}

// --- 7. BỘ TIMER NGẮT TRUNG TÂM CHU KỲ CHUẨN 50MS (TIM5) ---
void Timer5_PID_Interrupt_Init(void) {
    RCC->APB1ENR |= (1 << 3);
    TIM5->PSC = 15999;
    TIM5->ARR = 49;
    TIM5->DIER |= (1 << 0);
    NVIC_SetPriority(TIM5_IRQn, 0);
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 |= (1 << 0);
}

void TIM5_IRQHandler(void) {
    if (TIM5->SR & (1 << 0)) {
        int16_t cnt_m1 = (int16_t)TIM3->CNT; TIM3->CNT = 0;
        int16_t cnt_m2 = (int16_t)TIM4->CNT; TIM4->CNT = 0;

        speed_m1 = ((float)cnt_m1 / ENCODER_PULSES_PER_REV) * 1200.0f;
        speed_m2 = ((float)cnt_m2 / ENCODER_PULSES_PER_REV) * 1200.0f;

        // PID Motor 1
        err_m1 = target_speed - speed_m1;
        integral_m1 += err_m1 * 0.05f;
        if (integral_m1 > 150.0f) integral_m1 = 150.0f; else if (integral_m1 < -150.0f) integral_m1 = -150.0f;
        derivative_m1 = (err_m1 - prev_err_m1) / 0.05f;
        int output_m1 = (int)((Kp * err_m1) + (Ki * integral_m1) + (Kd * derivative_m1));
        prev_err_m1 = err_m1;

        // PID Motor 2
        err_m2 = target_speed - speed_m2;
        integral_m2 += err_m2 * 0.05f;
        if (integral_m2 > 50.0f) integral_m2 = 50.0f; else if (integral_m2 < -50.0f) integral_m2 = -50.0f;
        derivative_m2 = (err_m2 - prev_err_m2) / 0.05f;
        int output_m2 = (int)((Kp * err_m2) + (Ki * integral_m2) + (Kd * derivative_m2));
        prev_err_m2 = err_m2;

        if (output_m1 > 99) {
        	output_m1 = 99;
        } else if (output_m1 < 0) {
            output_m1 = 0;
        }

                // Giới hạn Motor 2: Max 99, Min 0 (Chỉ cho phép tiến)
        if (output_m2 > 99) {
            output_m2 = 99;
        } else if (output_m2 < 0) {
            output_m2 = 0;
        }

        if (target_speed == 0.0f) {
            Set_Motor_Outputs(0, 0);
            integral_m1 = 0; integral_m2 = 0;
        } else {
            Set_Motor_Outputs(output_m1, output_m2);
        }

        TIM5->SR &= ~(1 << 0);
    }
}

// --- 8. QUÉT TRẠNG THÁI NÚT BẤM NON-BLOCKING ---
void Process_Buttons_NonBlocking(void) {
    static ButtonState_t state_inc = BTN_STATE_IDLE;
    static ButtonState_t state_dec = BTN_STATE_IDLE;
    static uint32_t last_tick_inc = 0;
    static uint32_t last_tick_dec = 0;
    uint32_t current_tick = HAL_GetTick();

    // Nút tăng tốc (PA0)
    switch (state_inc) {
        case BTN_STATE_IDLE:
            if (!(GPIOA->IDR & (1 << 0))) {
                last_tick_inc = current_tick;
                state_inc = BTN_STATE_DEBOUNCE;
            }
            break;
        case BTN_STATE_DEBOUNCE:
            if (current_tick - last_tick_inc >= BUTTON_DEBOUNCE_TIME) {
                if (!(GPIOA->IDR & (1 << 0))) {
                    target_speed += SPEED_STEP;
                    if (target_speed > 130.0f) target_speed = 130.0f;
                    state_inc = BTN_STATE_WAIT_RELEASE;
                } else {
                    state_inc = BTN_STATE_IDLE;
                }
            }
            break;
        case BTN_STATE_WAIT_RELEASE:
            if (GPIOA->IDR & (1 << 0)) state_inc = BTN_STATE_IDLE;
            break;
    }

    // Nút giảm tốc (PA1)
    switch (state_dec) {
        case BTN_STATE_IDLE:
            if (!(GPIOA->IDR & (1 << 1))) {
                last_tick_dec = current_tick;
                state_dec = BTN_STATE_DEBOUNCE;
            }
            break;
        case BTN_STATE_DEBOUNCE:
            if (current_tick - last_tick_dec >= BUTTON_DEBOUNCE_TIME) {
                if (!(GPIOA->IDR & (1 << 1))) {
                    target_speed -= SPEED_STEP;
                    if (target_speed < 0.0f) target_speed = 0.0f;
                    state_dec = BTN_STATE_WAIT_RELEASE;
                } else {
                    state_dec = BTN_STATE_IDLE;
                }
            }
            break;
        case BTN_STATE_WAIT_RELEASE:
            if (GPIOA->IDR & (1 << 1)) state_dec = BTN_STATE_IDLE;
            break;
    }
}

// ==============================================================================
// CHƯƠNG TRÌNH CHÍNH
// ==============================================================================
int main(void) {
    HAL_Init(); // BẮT BUỘC GIỮ LẠI: Khởi tạo biến thời gian hệ thống cho HAL_GetTick()

    // Khởi tạo phần cứng
    GPIO_Init();
    I2C1_Init();
    LCD_I2C_Init();
    Motor_PWM_Init();
    Encoder_Init();
    UART2_Init();
    Timer5_PID_Interrupt_Init();

    char tx_buffer[128];
    char lcd_buffer[16];
    uint32_t last_display_time = 0;

    // --- MÀN HÌNH CHÀO MỪNG ---
    LCD_I2C_SetCursor(0, 0);
    LCD_I2C_Print("AGV SYS ACTIVE");

    // CHÚ Ý: Chỗ này ĐƯỢC PHÉP dùng HAL_Delay vì nó chỉ chạy 1 lần duy nhất lúc cấp nguồn,
    // chưa bước vào vòng lặp điều khiển thời gian thực.
    HAL_Delay(1000);
    LCD_I2C_SendCommand(0x01); // Xóa màn hình

    // --- VÒNG LẶP ĐIỀU KHIỂN THỜI GIAN THỰC (KHÔNG CÓ DELAY CHẾT) ---
    while (1) {
        // 1. Quét nút bấm liên tục (Không delay)
        Process_Buttons_NonBlocking();

        // 2. Cảnh báo quá tốc độ lập tức (Không delay)
        if (speed_m1 > MAX_SPEED_LIMIT || speed_m2 > MAX_SPEED_LIMIT) {
            GPIOA->ODR |=  (1 << 5);  // Bật LED
            GPIOB->ODR |=  (1 << 0);  // Bật Còi
        } else {
            GPIOA->ODR &= ~(1 << 5);  // Tắt LED
            GPIOB->ODR &= ~(1 << 0);  // Tắt Còi
        }

        // 3. Tác vụ in ấn LCD và gửi Serial (Chỉ chạy mỗi 100ms)
        if (HAL_GetTick() - last_display_time >= 100) {
            last_display_time = HAL_GetTick();

            // Gửi UART cho máy tính vẽ đồ thị
            char tx_buffer_1[64];
            sprintf(tx_buffer_1, "%.1f %.1f %.1f\r\n", target_speed, speed_m1, speed_m2);
            UART2_SendString(tx_buffer_1);

            sprintf(tx_buffer, "Setpoint: %.1f | Left_M1: %.1f | Right_M2: %.1f RPM\r\n",
                    target_speed, speed_m1, speed_m2);
            UART2_SendString(tx_buffer);

            // Cập nhật LCD
            LCD_I2C_SetCursor(0, 0);
            sprintf(lcd_buffer, "SP:%.1f RPM     ", target_speed);
            LCD_I2C_Print(lcd_buffer);

            LCD_I2C_SetCursor(1, 0);
            sprintf(lcd_buffer, "L:%.1f R:%.1f ", speed_m1, speed_m2);
            LCD_I2C_Print(lcd_buffer);
        }
    }
}
