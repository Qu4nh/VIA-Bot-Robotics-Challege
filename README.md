# FIRMWARE ĐIỀU KHIỂN - ROBOTICS CHALLENGE (VRC)

Repo này chứa firmware lập trình sănx cho hai loại robot sử dụng Mạch VIA B - Bánh Mì (VIA 2025), đã từng thi đấu trong cuộc thi robotics challenge:
1. Robot 2 bánh (thư mục `2_wheels`)
2. Robot 3 bánh toàn hướng Omni (thư mục `3_wheels`)

## Cấu trúc dự án

```
This Repo/
  ├── 2_wheels/                # Robot 2 bánh
  │   ├── 2_wheels.ino        # File chính Arduino
  │   ├── config.h            # Cấu hình chân và thông số
  │   ├── PS2_controller.cpp  # Xử lý tay cầm PS2
  │   ├── PS2_controller.h    # Header xử lý tay cầm PS2
  │   ├── robot_mechanisms.cpp # Điều khiển cơ cấu robot
  │   └── robot_mechanisms.h  # Header điều khiển cơ cấu
  │
  └── 3_wheels/         # Robot 3 bánh toàn hướng
      ├── 3_wheels.ino        # File chính Arduino
      ├── config.h            # Cấu hình chân và thông số
      ├── PS2_controller.cpp  # Xử lý tay cầm PS2
      ├── PS2_controller.h    # Header xử lý tay cầm PS2
      ├── robot_mechanisms.cpp # Điều khiển cơ cấu robot
      └── robot_mechanisms.h  # Header điều khiển cơ cấu
```

## Yêu cầu phần cứng

### Thành phần chung
- Mạch VIA Bánh Mì (MakerViet)
- Tay cầm PS2 không dây và bộ thu
- Pin Li-Po 2S hoặc 3S (7.4V-11.1V)
- Động cơ DC 12V loại 555 hoặc 545 
- Servo MG996R

### Robot 2 bánh
- 2 động cơ DC với bánh xe
- Servo 180° và 360° cho các cơ cấu phụ trợ
- Servo cho chốt hãm
- Động cơ DC cho cơ cấu nâng hạ

### Robot 3 bánh toàn hướng
- 3 động cơ DC với bánh xe toàn hướng (omni wheel)
- Servo 180° và 360° cho cơ cấu thả bóng
- Động cơ DC cho cơ cấu nâng hạ

## Thư viện yêu cầu

- [Arduino-PS2X-ESP32-Makerbot](https://github.com/makerviet/Arduino-PS2X-ESP32-Makerbot) - Thư viện điều khiển tay cầm PS2
- [Adafruit_PWMServoDriver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) - Thư viện điều khiển PCA9685
- esp32 by Espressif Systems - Board manager
- Additional Board Manager URLs: ```https://dl.espressif.com/dl/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json```

## Tính năng chính

### Robot 2 bánh
- Điều khiển di chuyển với nhiều chế độ: thường, nhanh, chậm, xoay tại chỗ
- Cơ cấu nâng hạ với chốt hãm servo 180°
- Điều khiển servo 360° cho các cơ cấu phụ trợ
- Hệ thống phanh thông minh

### Robot 3 bánh toàn hướng
- Di chuyển toàn hướng 8 hướng với 3 bánh
- Chế độ di chuyển chậm cho điều khiển chính xác
- Cơ cấu nâng hạ sử dụng động cơ DC
- Hệ thống thả bóng sử dụng servo 180° hoặc 360°
- Hệ thống giảm tốc thích ứng

## Hướng dẫn sử dụng

### Cài đặt
1. Cài đặt Arduino IDE và thêm hỗ trợ cho VIA B
2. Cài đặt các thư viện yêu cầu
3. Kết nối VIA B với máy tính
4. Mở file `.ino` tương ứng với robot của bạn
5. Kiểm tra và điều chỉnh các thông số trong `config.h` nếu cần
6. Biên dịch và tải lên VIA B

## Cấu trúc code

Mỗi dự án robot được tổ chức thành các module:

1. **File chính (.ino)**
   - Khởi tạo hệ thống
   - Vòng lặp chính

2. **Config (config.h)**
   - Định nghĩa chân kết nối
   - Thông số điều khiển
   - Cấu hình servo và động cơ

3. **PS2 Controller (PS2_controller.h/.cpp)**
   - Khởi tạo tay cầm PS2
   - Xử lý đầu vào từ tay cầm
   - Chuyển đổi tín hiệu thành lệnh điều khiển

4. **Robot Mechanisms (robot_mechanisms.h/.cpp)**
   - Điều khiển động cơ di chuyển
   - Điều khiển servo và cơ cấu phụ trợ
   - Xử lý các trạng thái robot

## Tùy chỉnh

Bạn có thể tùy chỉnh các thông số trong file `config.h` để phù hợp với phần cứng của mình:

- Chân kết nối tay cầm PS2
- Kênh động cơ và servo trên PCA9685
- Thông số điều khiển (tốc độ, độ nhạy, v.v.)
- Góc servo và thông số xung PWM

## Phát triển bởi

Qu4nh

## Giấy phép

Dự án này được phân phối theo giấy phép MIT. 