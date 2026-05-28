# HỆ THỐNG ĐIỀU KHIỂN TỐC ĐỘ ĐỘNG CƠ DC SỬ DỤNG PID VÀ CẢNH BÁO QUÁ TỐC ĐỘ

## 👥 Thành viên thực hiện

* **Mạc Văn Tùng** - 23020769
* **Hà Mạnh Hưng** - 23021837

## 👨‍🏫 Giảng viên hướng dẫn

* TS. Nguyễn Kiêm Hùng
* KS. Phạm Quang Hùng

---

# 🛠 Phần cứng sử dụng

* STM32F401RE (Nucleo-F401RE)
* Động cơ DC GA25 Encoder
* Driver L298N / MD10C
* Pin Li-ion 3 cell
* LED / Buzzer cảnh báo

---

# 💻 Chức năng chính

* Điều khiển tốc độ động cơ DC bằng thuật toán PID
* Đọc tốc độ bằng Encoder
* Xuất PWM điều khiển động cơ
* Cảnh báo khi vượt quá tốc độ cho phép

---

# 📂 Cấu trúc dự án

```text
final_project/
├── Core/
├── Drivers/
├── final_project.ioc
└── README.md
```

---

# 🚀 Hướng dẫn chạy dự án

## 1. Clone project

```bash
git clone [<repository_link>](https://github.com/23020769-pixel/Nhap_mon_he_thong_Nhung.git)
```

---

## 2. Mở project bằng STM32CubeIDE

```text
File → Open Projects from File System...
```

Chọn thư mục:

```text
final_project/
```

---

## 3. Build project

Nhấn biểu tượng 🔨 Build.

Đảm bảo Console hiển thị:

```text
0 errors, 0 warnings
```

---

## 4. Nạp code xuống board

* Kết nối Nucleo-F401RE bằng cáp USB
* Nhấn `Run` hoặc `Debug`

---

## 5. Theo dõi dữ liệu Serial

Sử dụng:

* VOFA+
* hoặc Arduino Serial Plotter

Cấu hình:

```text
Baudrate: 115200
```

Quan sát:

* `target_speed`
* `speed_actual`

---
#🎥 Video demo hệ thống

Google Drive:

[https://drive.google.com/xxxxxxxx](https://drive.google.com/file/d/1tiGrz6wLs8Gjv_RqGqYkjEnPx9ricCJP/view?usp=sharing)
# 📌 Công nghệ sử dụng

* STM32 HAL
* PWM
* Encoder Mode
* PID Control
* UART Communication
