/*
 * robot_mechanisms.h - THƯ VIỆN ĐIỀU KHIỂN CƠ CẤU ROBOT
 * 
 * Mô tả: Cung cấp các hàm điều khiển cơ cấu robot bao gồm động cơ DC,
 * servo 180° và 360°, cơ cấu hãm và các thiết bị khác.
 */

#ifndef ROBOT_MECHANISMS_H
#define ROBOT_MECHANISMS_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// ===================== BIẾN TOÀN CỤC =====================
// Driver PWM PCA9685
extern Adafruit_PWMServoDriver pwm;
extern int current_servo360_pulse;

// ===================== HÀM KHỞI TẠO =====================
// Khởi tạo hệ thống
void initPCA9685();         // Khởi tạo driver PCA9685
void initDriveMotors();     // Khởi tạo động cơ chính
void initServos();          // Khởi tạo các servo
void initAuxDCMotors();     // Khởi tạo động cơ phụ trợ

// =================== ĐIỀU KHIỂN DI CHUYỂN =================
// Điều khiển động cơ DC chính (-PWM_MAX đến PWM_MAX)
void setLeftDrivePWM(int pwmValue);    // Điều khiển động cơ trái
void setRightDrivePWM(int pwmValue);   // Điều khiển động cơ phải  
void stopDriveMotors();                // Dừng động cơ chính

// Điều khiển động cơ phụ trợ
void setAuxDCMotor(uint8_t motorNum, int speedVal); // Điều khiển động cơ phụ trợ
void auxMotor3Control(int speed);      // Điều khiển motor 3
void auxMotor4Control(int speed);      // Điều khiển motor 4

// =================== ĐIỀU KHIỂN SERVO ===================
// Điều khiển servo tiêu chuẩn
void setServoAngle(uint8_t servoChannel, uint8_t angle); // Điều khiển servo góc
void setContinuousServoPulse(uint8_t servoChannel, int pulseValue); // Điều khiển servo liên tục

// Điều khiển Servo 180°
void toggleServo180();              // Chuyển đổi trạng thái servo 180°
void setServo180Position(uint8_t position); // Đặt vị trí servo 180°
void setServo180BPosition(uint8_t angle);   // Đặt vị trí servo 180B               

// Hiệu chỉnh Servo 360°
void increaseServo360StopPulse();      // Tăng giá trị dừng servo 360°
void decreaseServo360StopPulse();      // Giảm giá trị dừng servo 360°

// Tiện ích
void showDeviceStatus();               // Hiển thị trạng thái thiết bị

// Điều khiển cơ cấu hãm (Ratchet and Pawl)
void engageLiftPawls();    // Gài cả hai chốt hãm
void releaseLiftPawls();   // Nhả cả hai chốt hãm

#endif