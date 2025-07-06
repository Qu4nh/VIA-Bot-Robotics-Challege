/*
* CƠ CẤU ROBOT 3 BÁNH TOÀN HƯỚNG
* -------------------------------------
* Header file chứa các khai báo hàm cho cơ cấu robot 3 bánh Omni:
*/

#ifndef ROBOT_MECHANISMS_H
#define ROBOT_MECHANISMS_H

#include <Adafruit_PWMServoDriver.h>
#include "config.h"

//========== ĐỊNH NGHĨA TRẠNG THÁI ==========
// Trạng thái servo thả bóng
enum BallReleaseState {
  BALL_RELEASE_IDLE,          // Không hoạt động
  BALL_RELEASE_OPENING,       // Đang mở
  BALL_RELEASE_OPEN,          // Đã mở
  BALL_RELEASE_CLOSING,       // Đang đóng
  BALL_RELEASE_CLOSED         // Đã đóng
};

//========== KHỞI TẠO ==========
// Khởi tạo driver PCA9685
void initPCA9685();

// Khởi tạo động cơ di chuyển
void initDriveMotors();

// Khởi tạo servo
void initServos();

// Khởi tạo động cơ nâng hạ
void initLiftMotor();

//========== CẬP NHẬT TRẠNG THÁI ==========
// Cập nhật trạng thái các cơ cấu
void updateMechanisms();

//========== ĐIỀU KHIỂN ĐỘNG CƠ ==========
// Điều khiển động cơ toàn hướng
void driveOmniMotors(float forwardBackward, float leftRight, float rotation);

// Điều khiển holonomic cấp thấp
void setHolonomicDrive(float x, float y, float rotation);

// Điều khiển động cơ riêng lẻ
void setFrontLeftMotor(int pwmValue);
void setFrontRightMotor(int pwmValue);
void setBackMotor(int pwmValue);

// Dừng động cơ di chuyển
void stopDriveMotors();

//========== ĐIỀU KHIỂN NÂNG HẠ ==========
// Điều khiển động cơ nâng hạ
void moveLiftMotor(float speed);

// Điều khiển tốc độ động cơ nâng hạ
void setLiftMotorSpeed(int speed);

// Dừng động cơ nâng hạ
void stopLiftMotor();

//========== ĐIỀU KHIỂN THẢ BÓNG ==========
// Điều khiển servo
void setServoAngle(uint8_t servoChannel, uint8_t angle);
void setContinuousServoPulse(uint8_t servoChannel, int pulseValue);

// Thả bóng bằng servo
void releaseBall();
void startOpenBallRelease();
void startOpenBallRelease180();

// Thu hồi bóng bằng servo
void retractBall();
void startCloseBallRelease();
void startCloseBallRelease180();

// Cập nhật trạng thái servo
void updateBallReleaseState();
void updateBallRelease180State();

#endif