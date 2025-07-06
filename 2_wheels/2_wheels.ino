/*
 * ROBOT 2 BÁNH - CHƯƠNG TRÌNH CHÍNH
 * 
 * Mô tả: Điều khiển robot 2 bánh bằng tay cầm PS2, hỗ trợ các cơ cấu nâng hạ, servo và chốt hãm.
 * Sử dụng PCA9685 để điều khiển động cơ và servo.
 */

//======================= INCLUDE =======================
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

//======================= THIẾT LẬP =======================
void setup() {
  Serial.begin(115200);
  Serial.println("\n===== ROBOT 2 BÁNH =====");
  Serial.println("Made by Qu4nh");
  Serial.println("-----------------------");

  // 1. Khởi tạo driver PCA9685
  Serial.println("1. Khởi tạo PCA9685...");
  initPCA9685();
  delay(100);
  
  // 2. Khởi tạo động cơ và servo
  Serial.println("2. Khởi tạo động cơ...");
  initDriveMotors();
  Serial.println("3. Khởi tạo servo...");
  initServos();
  Serial.println("4. Khởi tạo động cơ phụ trợ...");
  initAuxDCMotors();
  
  // 3. Khởi tạo tay cầm PS2
  Serial.println("5. Kết nối tay cầm PS2...");
  setupPS2controller();
  
  Serial.println("\nHoàn thành khởi tạo!");
  Serial.println("Robot đã sẵn sàng hoạt động!");
}

//======================= VÒNG LẶP CHÍNH =======================
void loop() {
  // Xử lý tất cả đầu vào từ tay cầm PS2
  handlePS2Input();
  
  // Độ trễ nhỏ để ổn định
  delay(20);
}