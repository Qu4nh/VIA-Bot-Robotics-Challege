/*
* ROBOT 3 BÁNH TOÀN HƯỚNG - V1.4
* -------------------------------------
* Code điều khiển cho robot toàn hướng 3 bánh
* Lưu ý: Điều chỉnh cấu hình trong config.h trước khi sử dụng!
*/

//========== THƯ VIỆN ==========
// Thư viện hệ thống
#include <Arduino.h>
#include <Wire.h>

// Thư viện ngoại vi
#include <PS2X_lib.h>         
#include <Adafruit_PWMServoDriver.h>

// Header tùy chỉnh
#include "config.h"           
#include "PS2_controller.h"    
#include "robot_mechanisms.h"  

//========== HƯỚNG DẪN SỬ DỤNG ==========
void displayHelp() {
  Serial.println("\n===== HƯỚNG DẪN SỬ DỤNG =====");
  Serial.println("- Joystick trái: Di chuyển robot theo 8 hướng");
  Serial.println("- Joystick phải (trái/phải): Xoay robot tại chỗ");
  Serial.println("- Nút R1: Chế độ di chuyển chậm");
  Serial.println("- Nút O: Nâng lên");
  Serial.println("- Nút X: Hạ xuống");
  Serial.println("- PAD_UP/PAD_DOWN: Điều khiển servo thả bóng");
  Serial.println("* LƯU Ý: Robot dùng hệ thống giảm tốc thích ứng");
  Serial.println("===============================");
}

//========== KHỞI TẠO ==========
void setup() {
  Serial.begin(115200);
  Serial.println("\n===== ROBOTO=====");
  Serial.println("Code by Qu4nh");
  Serial.println("-----------------------");

  // Khởi tạo phần cứng
  Serial.println("1. Khởi tạo PCA9685...");
  initPCA9685();
  delay(100);
  
  Serial.println("2. Khởi tạo động cơ di chuyển...");
  initDriveMotors();
  Serial.println("3. Khởi tạo servo thả bóng...");
  initServos();
  Serial.println("4. Khởi tạo động cơ nâng hạ...");
  initLiftMotor();
  
  Serial.println("5. Kết nối tay cầm PS2...");
  setupPS2controller();
  
  Serial.println("\nKhởi tạo hoàn tất! Robot sẵn sàng!");
  
  // Hiển thị hướng dẫn sử dụng
  displayHelp();
}

//========== VÒNG LẶP CHÍNH ==========
void loop() {
  // Xử lý đầu vào từ tay cầm PS2
  handlePS2Input();
  
  // Cập nhật trạng thái các cơ cấu
  updateMechanisms();
  
  // Độ trễ ổn định hệ thống
  delay(20);
}