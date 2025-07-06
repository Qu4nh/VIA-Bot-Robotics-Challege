/*
* CẤU HÌNH ROBOT 3 BÁNH TOÀN HƯỚNG
* -------------------------------------
* File cấu hình các thông số cho robot 3 bánh Omni
* Lưu ý: Kiểm tra kỹ thông số trước khi vận hành!
*/

#ifndef CONFIG_H
#define CONFIG_H

//========== CẤU HÌNH DEBUG ==========
#define DEBUG_PS2_VALUES       // Hiển thị giá trị joystick
#define DEBUG_DRIVE_OUTPUT     // Hiển thị giá trị PWM động cơ
#define DEBUG_SERVO            // Hiển thị thông tin servo
#define DEBUG_MOTOR            // Hiển thị thông tin motor

//========== CẤU HÌNH TAY CẦM PS2 ==========
// ------- Chân kết nối -------
#define PS2_DAT_PIN 12 
#define PS2_CMD_PIN 13 
#define PS2_SEL_PIN 15 
#define PS2_CLK_PIN 14 

// ------- Hiệu chỉnh joystick -------
#define X_JOY_CALIB 127 
#define Y_JOY_CALIB 128 

//========== CẤU HÌNH ĐỘNG CƠ ==========
// ------- PCA9685 -------
#define PWM_FREQ 50       // Tần số 50Hz cho servo
#define PWM_MAX 4095      // Giá trị PWM tối đa (12-bit)

// ------- Kênh động cơ di chuyển -------
#define MOTOR_FRONT_LEFT_IN1  8  // Motor bánh trước bên trái
#define MOTOR_FRONT_LEFT_IN2  9
#define MOTOR_FRONT_RIGHT_IN1 10 // Motor bánh trước bên phải
#define MOTOR_FRONT_RIGHT_IN2 11
#define MOTOR_BACK_IN1        14 // Motor bánh sau
#define MOTOR_BACK_IN2        15

// ------- Kênh động cơ nâng hạ -------
#define LIFT_MOTOR_IN1        12 // Motor nâng hạ
#define LIFT_MOTOR_IN2        13

//========== CẤU HÌNH SERVO ==========
// ------- Kênh servo -------
#define BALL_RELEASE_SERVO_CHANNEL 2  // Servo thả bóng (servo 360°)
#define BALL_RELEASE_SERVO_180_CHANNEL 3  // Servo thả bóng dự phòng (servo 180°)

// ------- Thông số servo 360° -------
#define SERVO_360_STOP_PULSE 307      // Giá trị dừng (1.5ms @ 50Hz)
#define SERVO_360_MAX_OFFSET 200      // Offset xung tối đa từ giá trị dừng
#define SERVO_360_FWD_PULSE (SERVO_360_STOP_PULSE + SERVO_360_MAX_OFFSET)  // Xung quay thuận
#define SERVO_360_REV_PULSE (SERVO_360_STOP_PULSE - SERVO_360_MAX_OFFSET)  // Xung quay ngược

// ------- Thông số servo 180° -------
#define SERVO_MIN_PULSE 125   // Xung tối thiểu (~0°)
#define SERVO_MAX_PULSE 575   // Xung tối đa (~180°)

// ------- Góc servo thả bóng -------
#define SERVO_180_START_ANGLE 0   // Góc bắt đầu (có thể thay đổi)
#define SERVO_180_END_ANGLE 90    // Góc kết thúc (có thể thay đổi)

//========== CẤU HÌNH ĐIỀU KHIỂN ==========
// ------- Hệ số tốc độ -------
#define MAX_DRIVE_SPEED_FACTOR 0.5   // 50% tốc độ tối đa cho di chuyển
#define MAX_ROTATION_SPEED_FACTOR 0.35 // 35% tốc độ tối đa cho quay tại chỗ
#define MAX_ACCELERATION_FACTOR 0.025f // Giới hạn gia tốc nhỏ hơn, tăng độ mượt
#define SLOW_MODE_FACTOR 0.4f         // 40% tốc độ tối đa khi ở chế độ chậm (slow mode)

// ------- Điều khiển joystick -------
#define INPUT_DEADBAND 0.2           // Ngưỡng chết joystick cao hơn
#define THROTTLE_SENSITIVITY 0.9      // Độ nhạy ga
#define STEERING_SENSITIVITY 1.0      // Độ nhạy lái tối đa

// ------- Hệ thống giảm tốc -------
#define STOP_DECELERATION_BASE 0.01f   // Hệ số giảm tốc cơ bản (mức tối thiểu)
#define STOP_DECELERATION_SCALE 0.08f  // Hệ số tỷ lệ giảm tốc theo vận tốc hiện tại
#define STOP_MIN_THRESHOLD 50          // Ngưỡng tốc độ nhỏ để dừng hẳn (tránh rung lắc ở tốc độ thấp)
#define LIFT_ACTIVE_THRESHOLD 3000     // Ngưỡng để phát hiện cơ cấu nâng đang hoạt động (PWM)
#define STOP_LIFT_ADJUSTMENT 0.7f      // Hệ số điều chỉnh giảm tốc khi cơ cấu nâng đang hoạt động

//========== CẤU HÌNH NÂNG HẠ ==========
#define LIFT_MOTOR_SPEED 3700         // Tốc độ mặc định (0-4095)
#define LIFT_MOTOR_ACCELERATION_FACTOR 0.03f  // Giới hạn gia tốc motor nâng hạ
#define MAX_LIFT_SPEED_FACTOR 1.0f    // Hệ số tốc độ tối đa cho motor nâng hạ (100%)

#endif