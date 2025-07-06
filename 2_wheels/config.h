/*
 * config.h - CẤU HÌNH ROBOT 2 BÁNH
 * 
 * Mô tả: Định nghĩa các hằng số và cấu hình cho robot 2 bánh bao gồm 
 * các chân kết nối, thông số điều khiển động cơ, servo và các thiết lập khác.
 */

#ifndef CONFIG_H
#define CONFIG_H

//=============================== DEBUG SETTINGS ===============================
#define DEBUG_PS2_VALUES       // Hiển thị giá trị joystick
#define DEBUG_DRIVE_OUTPUT     // Hiển thị giá trị PWM động cơ
#define DEBUG_SERVO            // Hiển thị thông tin servo
#define DEBUG_MOTOR            // Hiển thị thông tin motor

//========================== PS2 CONTROLLER CONFIG ============================
// Cấu hình chân tay cầm PS2
#define PS2_DAT_PIN 12 
#define PS2_CMD_PIN 13 
#define PS2_SEL_PIN 15 
#define PS2_CLK_PIN 14 

// Giá trị hiệu chỉnh joystick
#define X_JOY_CALIB 127 
#define Y_JOY_CALIB 128 

//========================== MOTOR DRIVER CONFIG ==============================
// Cấu hình PCA9685
#define PWM_FREQ 50       // Tần số 50Hz cho servo
#define PWM_MAX 4095      // Giá trị PWM tối đa (12-bit)

// Cấu hình kênh động cơ DC chính
#define M1_IN1_CHANNEL 8  // Motor trái
#define M1_IN2_CHANNEL 9  
#define M2_IN1_CHANNEL 10 // Motor phải
#define M2_IN2_CHANNEL 11 

// Cấu hình kênh động cơ phụ trợ
#define AUX_M3_IN1_CHANNEL 14
#define AUX_M3_IN2_CHANNEL 15
#define AUX_M4_IN1_CHANNEL 12  // Motor 4 hỗ trợ Motor 3
#define AUX_M4_IN2_CHANNEL 13  

//============================= SERVO CONFIG =================================
// Cấu hình kênh servo
#define SERVO_180_CHANNEL 2   // Servo 180 độ
#define SERVO_360_CHANNEL 3   // Servo 360 độ 
#define SERVO_180B_CHANNEL 4  // Servo 180 độ phụ
#define SERVO_360B_CHANNEL 5  // Servo 360 độ phụ
#define SERVO_PAWL_LEFT_CHANNEL  6  
#define SERVO_PAWL_RIGHT_CHANNEL 7 

// Thông số xung servo
#define SERVO_MIN_PULSE 125   // Xung tối thiểu (~0°)
#define SERVO_MAX_PULSE 575   // Xung tối đa (~180°)

// Cấu hình góc servo
#define SERVO_180_POS_1 0     // Vị trí 1 cho servo 180 độ
#define SERVO_180_POS_2 90    // Vị trí 2 cho servo 180 độ
#define SERVO_180B_ANGLE 45   // Góc cho servo 180B
// Cấu hình góc servo Pawl (hãm)
#define PAWL_TRAVEL_ANGLE   15  // Góc di chuyển là 45 độ để phản hồi nhanh

// Servo trái (kênh 6) - Giả sử quay từ 0 -> 45 độ
#define PAWL_LEFT_ENGAGE_ANGLE   0   // Góc để GÀI (khóa)
#define PAWL_LEFT_RELEASE_ANGLE  PAWL_LEFT_ENGAGE_ANGLE+PAWL_TRAVEL_ANGLE   // Góc để NHẢ (mở)

// Servo phải (kênh 7) - Quay ngược lại để đối xứng, giả sử từ 90 -> 45 độ
#define PAWL_RIGHT_ENGAGE_ANGLE  0 //90  // Góc để GÀI (khóa) - Bắt đầu từ góc khác
#define PAWL_RIGHT_RELEASE_ANGLE PAWL_RIGHT_ENGAGE_ANGLE+PAWL_TRAVEL_ANGLE//(PAWL_RIGHT_ENGAGE_ANGLE - PAWL_TRAVEL_ANGLE) // Góc để NHẢ (mở) là 90 - 45 = 45 độ

//========================== SERVO 360 CONFIG ===============================
// Cấu hình servo 360 (cả kênh 3 và kênh 5)
#define SERVO_360_MIN_PULSE 0
#define SERVO_360_MAX_PULSE 4095
#define SERVO_360_STOP_PULSE 307     // Giá trị dừng (1.5ms @ 50Hz)
#define SERVO_360_MAX_OFFSET (long)(200)  // Offset xung tối đa từ giá trị dừng
#define SERVO_360_FWD_PULSE (SERVO_360_STOP_PULSE + SERVO_360_MAX_OFFSET)
#define SERVO_360_REV_PULSE (SERVO_360_STOP_PULSE - SERVO_360_MAX_OFFSET)
#define PAWL_TIMING_DELAY 60  // Thời gian (ms) chờ cho servo di chuyển trước khi motor chạy

#define SERVO_360_FINE_TUNE_STEP 1   // Bước điều chỉnh
#define SERVO_360_TEST_MODE false     // Chế độ hiệu chỉnh

//=========================== DRIVE CONTROL CONFIG ==========================
// Hệ số tốc độ lái
#define NORM_DRIVE_SPEED_FACTOR 0.5   // 50% tốc độ mặc định
#define TOP_DRIVE_SPEED_FACTOR 0.95    // 95% tốc độ tối đa (R2)
#define PRECISION_DRIVE_SPEED_FACTOR 0.20  // 20% tốc độ chính xác (R1)
#define MAX_ACCELERATION_FACTOR 0.05f // Giới hạn gia tốc. Số càng nhỏ, robot tăng tốc/giảm tốc càng mượt.

// Hệ thống phanh
#define BRAKE_DETECTION_THRESHOLD 0.3f  // Ngưỡng tốc độ để kích hoạt phanh khi nhả ga đột ngột (0.3 = 30%)
#define BRAKE_POWER_FACTOR 0.6f         // Lực phanh mạnh cỡ nào (0.6 = 60% công suất)
#define BRAKE_DURATION_MS 30            // Phanh được giữ trong bao lâu (tính bằng mili giây)

// Cấu hình điều khiển (Arcade Drive)
#define INPUT_DEADBAND 0.01           // Ngưỡng chết joystick
#define THROTTLE_SENSITIVITY 1.0      // Độ nhạy ga
#define STEERING_SENSITIVITY 1.0      // Độ nhạy lái

// Cấu hình xoay nhanh và lái chính xác
#define QUICKTURN_BUTTON PSB_L1       // L1 = chế độ xoay tại chỗ
#define QUICKTURN_SENSITIVITY 0.8     // Tốc độ khi xoay tại chỗ (0.0 đến 1.0)
#define PRECISION_MODE_BUTTON PSB_R1  // R1 = chế độ chạy chậm
#define TURN_GAIN 0.5                 // Giảm độ nhạy rẽ khi chạy nhanh
#define TURN_EXPONENT 1.5             // Mức độ giảm độ nhạy
#define MIN_TURN_SENSITIVITY 0.4f     // Độ nhạy tối thiểu khi chạy nhanh

//========================== AUX MOTOR CONFIG ===============================
// Cấu hình motor phụ
#define AUX_MOTOR_SPEED 3000          // Tốc độ mặc định (0-4095)
#define AUX_MOTOR_ACCELERATION_FACTOR 0.03f  // Giới hạn gia tốc motor phụ
#define LOWERING_SPEED_FACTOR 0.11f    // Hệ số tốc độ khi hạ cơ cấu (20%)
#define NITRO_HANGING_SPEED_FACTOR 1.0f   // Hệ số tốc độ cho chế độ Nitro treo (100%)

#endif