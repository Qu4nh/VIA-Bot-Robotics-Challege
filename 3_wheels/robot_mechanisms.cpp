/*
 * robot_mechanisms.cpp - ĐIỀU KHIỂN CƠ CẤU ROBOT
 *
 * Mô tả: Cung cấp các hàm điều khiển cơ cấu robot bao gồm động cơ DC,
 * servo 180° và 360° và các thiết bị khác.
*/

#include "robot_mechanisms.h"
#include <Wire.h>
#include <math.h>
#include <PS2X_lib.h>

// Khai báo biến từ file khác
extern PS2X ps2x;

//===================== BIẾN TOÀN CỤC =====================
// Driver PWM PCA9685
Adafruit_PWMServoDriver pwm;

// Trạng thái cơ cấu
bool ball_release_state = false;  // false = đóng, true = mở
bool ball_release_180_state = false; // false = đóng, true = mở

// Biến máy trạng thái
BallReleaseState ball_release_servo_state = BALL_RELEASE_CLOSED;
BallReleaseState ball_release_servo_180_state = BALL_RELEASE_CLOSED;
unsigned long ball_release_action_start_time = 0;

// Thời gian chuyển trạng thái servo (ms)
const unsigned long SERVO_ACTION_TIME = 500; // 500ms cho servo hoàn thành chuyển động

// Biến làm mượt chuyển động
float currentFrontLeftPWMSmoothed = 0.0f;
float currentFrontRightPWMSmoothed = 0.0f;
float currentBackPWMSmoothed = 0.0f;

//======================= KHỞI TẠO =======================
void initPCA9685() {
  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);  // Sử dụng tần số 50Hz từ config.h
  Wire.setClock(400000); 
  Serial.println("PCA9685 đã được khởi tạo.");
}

void initDriveMotors() {
  // Đảm bảo các biến đã làm mượt được khởi tạo về 0
  currentFrontLeftPWMSmoothed = 0.0f;
  currentFrontRightPWMSmoothed = 0.0f;
  currentBackPWMSmoothed = 0.0f;
  
  stopDriveMotors();
  Serial.println("Các động cơ di chuyển đã được khởi tạo và dừng lại.");
}

void initServos() {
  // Khởi tạo servo thả bóng ở trạng thái đóng
  setContinuousServoPulse(BALL_RELEASE_SERVO_CHANNEL, SERVO_360_STOP_PULSE);
  setServoAngle(BALL_RELEASE_SERVO_180_CHANNEL, SERVO_180_START_ANGLE);
  
  // Khởi tạo trạng thái
  ball_release_state = false;
  ball_release_180_state = false;
  ball_release_servo_state = BALL_RELEASE_CLOSED;
  ball_release_servo_180_state = BALL_RELEASE_CLOSED;
  ball_release_action_start_time = 0;
  
  Serial.println("Servo thả bóng đã được khởi tạo.");
}

void initLiftMotor() {
  stopLiftMotor();
  Serial.println("Động cơ nâng hạ đã được khởi tạo và dừng lại.");
}

//================= ĐIỀU KHIỂN DI CHUYỂN HOLONOMIC =================
void setHolonomicDrive(float x, float y, float rotation) {
    bool isStoppingCommand = (abs(x) < 0.01f && abs(y) < 0.01f && abs(rotation) < 0.01f);

    x = constrain(x, -1.0, 1.0);
    y = constrain(y, -1.0, 1.0);
    rotation = constrain(rotation, -1.0, 1.0);
  
    float frontLeftPower = 0, frontRightPower = 0, backPower = 0;
  
    if (abs(rotation) > 0.1f) {
        float rotationPower = rotation;
        frontLeftPower = rotationPower;
        frontRightPower = rotationPower;
        backPower = rotationPower;
    }
    else if (abs(x) > 0.1f || abs(y) > 0.1f) {
        // === LOGIC DI CHUYỂN 8 HƯỚNG ===
        float angle = atan2(y, x);
        float magnitude = sqrt(x*x + y*y);
        if (magnitude > 1.0) magnitude = 1.0; // Giới hạn độ lớn

        // Chuyển góc sang độ và chuẩn hóa về khoảng 0-360
        float angleDegrees = angle * 180.0 / PI;
        if (angleDegrees < 0) angleDegrees += 360.0;
    
        // Xác định hướng di chuyển dựa trên góc
        // Đi tiến (North): 90° ± 22.5°
        if (angleDegrees >= 67.5 && angleDegrees < 112.5) {
          frontLeftPower = -magnitude; 
          frontRightPower = magnitude; 
          backPower = 0;
        } 
        // Đi chéo trước-phải (North-East): 45° ± 22.5°
        else if (angleDegrees >= 22.5 && angleDegrees < 67.5) {
          frontLeftPower = -magnitude; 
          frontRightPower = 0; 
          backPower = magnitude;
        } 
        // Đi ngang phải (East): 0° ± 22.5° hoặc 360° ± 22.5°
        else if (angleDegrees >= 337.5 || angleDegrees < 22.5) {
          frontLeftPower = -magnitude*0.5; 
          frontRightPower = -magnitude*0.5; 
          backPower = magnitude;
        } 
        // Đi chéo sau-phải (South-East): 315° ± 22.5°
        else if (angleDegrees >= 292.5 && angleDegrees < 337.5) {
          frontLeftPower = 0; 
          frontRightPower = -magnitude; 
          backPower = magnitude;
        } 
        // Đi lùi (South): 270° ± 22.5°
        else if (angleDegrees >= 247.5 && angleDegrees < 292.5) {
          frontLeftPower = magnitude; 
          frontRightPower = -magnitude; 
          backPower = 0;
        } 
        // Đi chéo sau-trái (South-West): 225° ± 22.5°
        else if (angleDegrees >= 202.5 && angleDegrees < 247.5) {
          frontLeftPower = magnitude; 
          frontRightPower = 0; 
          backPower = -magnitude;
        } 
        // Đi ngang trái (West): 180° ± 22.5°
        else if (angleDegrees >= 157.5 && angleDegrees < 202.5) {
          frontLeftPower = magnitude*0.5; 
          frontRightPower = magnitude*0.5; 
          backPower = -magnitude;
        } 
        // Đi chéo trước-trái (North-West): 135° ± 22.5°
        else if (angleDegrees >= 112.5 && angleDegrees < 157.5) {
          frontLeftPower = 0; 
          frontRightPower = magnitude; 
          backPower = -magnitude;
        }
    }
  
    // Áp dụng hệ số tốc độ
    float speed_factor = MAX_DRIVE_SPEED_FACTOR;
    if (abs(rotation) > 0.1f) {
        speed_factor = MAX_ROTATION_SPEED_FACTOR;
    }
  
    float targetFLPWM = frontLeftPower * PWM_MAX * speed_factor;
    float targetFRPWM = frontRightPower * PWM_MAX * speed_factor;
    float targetBPWM = backPower * PWM_MAX * speed_factor;

    // === LOGIC DỪNG THÍCH ỨNG ===
    if (isStoppingCommand) {
        // Kiểm tra trực tiếp nút bấm để xác định trạng thái nâng hạ
        bool isLiftActive = ps2x.Button(PSB_CIRCLE) || ps2x.Button(PSB_CROSS);
        float liftAdjustment = isLiftActive ? STOP_LIFT_ADJUSTMENT : 1.0f;
        
        // Công thức tính giảm tốc thích ứng
        float maxCurrentSpeed = max(max(abs(currentFrontLeftPWMSmoothed), abs(currentFrontRightPWMSmoothed)), abs(currentBackPWMSmoothed));
        float speedRatio = maxCurrentSpeed / PWM_MAX;
        float adaptiveDecel = (STOP_DECELERATION_BASE + (STOP_DECELERATION_SCALE * speedRatio)) * PWM_MAX * liftAdjustment;
        
        // Áp dụng giảm tốc cho mỗi động cơ
        // Bánh trước bên trái
        if (abs(currentFrontLeftPWMSmoothed) < STOP_MIN_THRESHOLD) {
            currentFrontLeftPWMSmoothed = 0;
        } else {
            currentFrontLeftPWMSmoothed -= adaptiveDecel * (currentFrontLeftPWMSmoothed > 0 ? 1 : -1);
        }
        
        // Bánh trước bên phải
        if (abs(currentFrontRightPWMSmoothed) < STOP_MIN_THRESHOLD) {
            currentFrontRightPWMSmoothed = 0;
        } else {
            currentFrontRightPWMSmoothed -= adaptiveDecel * (currentFrontRightPWMSmoothed > 0 ? 1 : -1);
        }
        
        // Bánh sau
        if (abs(currentBackPWMSmoothed) < STOP_MIN_THRESHOLD) {
            currentBackPWMSmoothed = 0;
        } else {
            currentBackPWMSmoothed -= adaptiveDecel * (currentBackPWMSmoothed > 0 ? 1 : -1);
        }
        
        #ifdef DEBUG_DRIVE_OUTPUT
          static unsigned long last_stop_debug_time = 0;
          if (millis() - last_stop_debug_time > 250) {
            Serial.print("Giảm tốc thích ứng: "); Serial.print(adaptiveDecel);
            Serial.print(" | Nâng hạ hoạt động: "); Serial.print(isLiftActive ? "Có" : "Không");
            Serial.print(" | Tỷ lệ tốc độ: "); Serial.println(speedRatio);
            last_stop_debug_time = millis();
          }
        #endif
    } else {
        // Áp dụng Slew Rate Limiter (làm mượt khi di chuyển)
        float maxDeltaPWM = MAX_ACCELERATION_FACTOR * PWM_MAX;
        currentFrontLeftPWMSmoothed = constrain(targetFLPWM, currentFrontLeftPWMSmoothed - maxDeltaPWM, currentFrontLeftPWMSmoothed + maxDeltaPWM);
        currentFrontRightPWMSmoothed = constrain(targetFRPWM, currentFrontRightPWMSmoothed - maxDeltaPWM, currentFrontRightPWMSmoothed + maxDeltaPWM);
        currentBackPWMSmoothed = constrain(targetBPWM, currentBackPWMSmoothed - maxDeltaPWM, currentBackPWMSmoothed + maxDeltaPWM);
    }
  
    setFrontLeftMotor((int)currentFrontLeftPWMSmoothed);
    setFrontRightMotor((int)currentFrontRightPWMSmoothed);
    setBackMotor((int)currentBackPWMSmoothed);
    
    #ifdef DEBUG_DRIVE_OUTPUT
      static unsigned long last_debug_time = 0;
      if (millis() - last_debug_time > 500) {
              Serial.print("PWM - BT: "); Serial.print((int)currentFrontLeftPWMSmoothed);
      Serial.print(" | BP: "); Serial.print((int)currentFrontRightPWMSmoothed);
      Serial.print(" | BS: "); Serial.print((int)currentBackPWMSmoothed);
        Serial.println("");
        last_debug_time = millis();
      }
    #endif
}

void setFrontLeftMotor(int pwmValue) {
  pwmValue = constrain(pwmValue, -PWM_MAX, PWM_MAX);
  
  // Xác định chiều hiện tại và chiều mới
  static int lastDirection = 0; // 0 = dừng, 1 = tiến, -1 = lùi
  int newDirection = (pwmValue > 0) ? 1 : (pwmValue < 0) ? -1 : 0;
  
  // Kiểm tra xem có đang chuyển từ tiến sang lùi hoặc ngược lại không
  bool isChangingDirection = (lastDirection != 0 && newDirection != 0 && lastDirection != newDirection);
  
  // Nếu đang chuyển hướng, tạo dead time bằng cách tắt động cơ trước
  if (isChangingDirection) {
    pwm.setPWM(MOTOR_FRONT_LEFT_IN1, 0, 0);
    pwm.setPWM(MOTOR_FRONT_LEFT_IN2, 0, 0);
    delay(2); // Dead time 2ms để bảo vệ mạch công suất
  }
  
  // Áp dụng điều khiển động cơ
  if (pwmValue >= 0) { // Tiến
    pwm.setPWM(MOTOR_FRONT_LEFT_IN1, 0, pwmValue);
    pwm.setPWM(MOTOR_FRONT_LEFT_IN2, 0, 0);
  } else { // Lùi
    pwm.setPWM(MOTOR_FRONT_LEFT_IN1, 0, 0);
    pwm.setPWM(MOTOR_FRONT_LEFT_IN2, 0, abs(pwmValue));
  }
  
  // Lưu chiều mới cho lần gọi tiếp theo
  lastDirection = newDirection;
}

void setFrontRightMotor(int pwmValue) {
  pwmValue = constrain(pwmValue, -PWM_MAX, PWM_MAX);
  
  // Xác định chiều hiện tại và chiều mới
  static int lastDirection = 0; // 0 = dừng, 1 = tiến, -1 = lùi
  int newDirection = (pwmValue > 0) ? 1 : (pwmValue < 0) ? -1 : 0;
  
  // Kiểm tra xem có đang chuyển từ tiến sang lùi hoặc ngược lại không
  bool isChangingDirection = (lastDirection != 0 && newDirection != 0 && lastDirection != newDirection);
  
  // Nếu đang chuyển hướng, tạo dead time bằng cách tắt động cơ trước
  if (isChangingDirection) {
    pwm.setPWM(MOTOR_FRONT_RIGHT_IN1, 0, 0);
    pwm.setPWM(MOTOR_FRONT_RIGHT_IN2, 0, 0);
    delay(2); // Dead time 2ms để bảo vệ mạch công suất
  }
  
  // Áp dụng điều khiển động cơ
  if (pwmValue >= 0) { // Tiến
    pwm.setPWM(MOTOR_FRONT_RIGHT_IN1, 0, pwmValue);
    pwm.setPWM(MOTOR_FRONT_RIGHT_IN2, 0, 0);
  } else { // Lùi
    pwm.setPWM(MOTOR_FRONT_RIGHT_IN1, 0, 0);
    pwm.setPWM(MOTOR_FRONT_RIGHT_IN2, 0, abs(pwmValue));
  }
  
  // Lưu chiều mới cho lần gọi tiếp theo
  lastDirection = newDirection;
}

void setBackMotor(int pwmValue) {
  pwmValue = constrain(pwmValue, -PWM_MAX, PWM_MAX);
  
  // Debug để kiểm tra giá trị PWM bánh sau
  #ifdef DEBUG_MOTOR
    static unsigned long last_debug_time = 0;
    if (millis() - last_debug_time > 500) {
      Serial.print("PWM động cơ bánh sau: "); Serial.println(pwmValue);
      last_debug_time = millis();
    }
  #endif
  
  // Xác định chiều hiện tại và chiều mới
  static int lastDirection = 0; // 0 = dừng, 1 = tiến, -1 = lùi
  int newDirection = (pwmValue > 0) ? 1 : (pwmValue < 0) ? -1 : 0;
  
  // Kiểm tra xem có đang chuyển từ tiến sang lùi hoặc ngược lại không
  bool isChangingDirection = (lastDirection != 0 && newDirection != 0 && lastDirection != newDirection);
  
  // Nếu đang chuyển hướng, tạo dead time bằng cách tắt động cơ trước
  if (isChangingDirection) {
    pwm.setPWM(MOTOR_BACK_IN1, 0, 0);
    pwm.setPWM(MOTOR_BACK_IN2, 0, 0);
    delay(2); // Dead time 2ms để bảo vệ mạch công suất
  }
  
  // Áp dụng điều khiển động cơ
  if (pwmValue >= 0) { // Tiến
    pwm.setPWM(MOTOR_BACK_IN1, 0, pwmValue);
    pwm.setPWM(MOTOR_BACK_IN2, 0, 0);
  } else { // Lùi
    pwm.setPWM(MOTOR_BACK_IN1, 0, 0);
    pwm.setPWM(MOTOR_BACK_IN2, 0, abs(pwmValue));
  }
  
  // Lưu chiều mới cho lần gọi tiếp theo
  lastDirection = newDirection;
}

void stopDriveMotors() {
  // Reset các biến làm mượt
  currentFrontLeftPWMSmoothed = 0;
  currentFrontRightPWMSmoothed = 0;
  currentBackPWMSmoothed = 0;
  
  // Dừng động cơ
  setFrontLeftMotor(0);
  setFrontRightMotor(0);
  setBackMotor(0);
}

//================= ĐIỀU KHIỂN CƠ CẤU NÂNG HẠ =================
void setLiftMotorSpeed(int speed) {
  // Giới hạn tốc độ
  speed = constrain(speed, -LIFT_MOTOR_SPEED, LIFT_MOTOR_SPEED);
  
  if (speed > 0) {
    // Nâng lên
    pwm.setPWM(LIFT_MOTOR_IN1, 0, speed);
    pwm.setPWM(LIFT_MOTOR_IN2, 0, 0);
  } else if (speed < 0) {
    // Hạ xuống
    pwm.setPWM(LIFT_MOTOR_IN1, 0, 0);
    pwm.setPWM(LIFT_MOTOR_IN2, 0, -speed);
  } else {
    // Dừng
    pwm.setPWM(LIFT_MOTOR_IN1, 0, 0);
    pwm.setPWM(LIFT_MOTOR_IN2, 0, 0);
  }
  
  #ifdef DEBUG_MOTOR
    static unsigned long lastLiftDebug = 0;
    if (millis() - lastLiftDebug > 500 && speed != 0) {
      Serial.print("Lift motor: ");
      Serial.println(speed);
      lastLiftDebug = millis();
    }
  #endif
}

void stopLiftMotor() {
  setLiftMotorSpeed(0);
}

//================= ĐIỀU KHIỂN SERVO THẢ BÓNG =================
// Đặt góc servo từ 0 đến 180 độ
void setServoAngle(uint8_t servoChannel, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  // Công thức tính toán xung PWM cho servo 180°
  long pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  pwm.setPWM(servoChannel, 0, pulse);
  
  #ifdef DEBUG_SERVO
    Serial.print("Servo "); Serial.print(servoChannel);
    Serial.print(" đặt góc: "); Serial.print(angle);
    Serial.print(" (xung: "); Serial.print(pulse);
    Serial.println(")");
  #endif
}

// Đặt xung PWM trực tiếp cho servo 360°
void setContinuousServoPulse(uint8_t servoChannel, int pulseValue) {
  pulseValue = constrain(pulseValue, 0, PWM_MAX);
  pwm.setPWM(servoChannel, 0, pulseValue);
  
  #ifdef DEBUG_SERVO
    Serial.print("Servo 360° "); Serial.print(servoChannel);
    Serial.print(" đặt xung PWM: "); Serial.println(pulseValue);
  #endif
}

// Bắt đầu mở servo thả bóng
void startOpenBallRelease() {
  // Chỉ bắt đầu quá trình mở nếu đang không hoạt động hoặc đã đóng
  if (ball_release_servo_state == BALL_RELEASE_IDLE || ball_release_servo_state == BALL_RELEASE_CLOSED) {
    // Đặt servo 360° quay theo chiều thuận để mở
    setContinuousServoPulse(BALL_RELEASE_SERVO_CHANNEL, SERVO_360_FWD_PULSE);
    
    // Cập nhật trạng thái và thời điểm bắt đầu
    ball_release_servo_state = BALL_RELEASE_OPENING;
    ball_release_action_start_time = millis();
    
    #ifdef DEBUG_SERVO
      Serial.println("Bắt đầu mở servo thả bóng");
    #endif
  }
}

// Bắt đầu đóng servo thả bóng
void startCloseBallRelease() {
  // Chỉ bắt đầu quá trình đóng nếu đang không hoạt động hoặc đã mở
  if (ball_release_servo_state == BALL_RELEASE_IDLE || ball_release_servo_state == BALL_RELEASE_OPEN) {
    // Đặt servo 360° quay theo chiều ngược để đóng
    setContinuousServoPulse(BALL_RELEASE_SERVO_CHANNEL, SERVO_360_REV_PULSE);
    
    // Cập nhật trạng thái và thời điểm bắt đầu
    ball_release_servo_state = BALL_RELEASE_CLOSING;
    ball_release_action_start_time = millis();
    
    #ifdef DEBUG_SERVO
      Serial.println("Bắt đầu đóng servo thả bóng");
    #endif
  }
}

// Cập nhật trạng thái servo thả bóng
void updateBallReleaseState() {
  // Kiểm tra xem servo có đang trong quá trình chuyển động không
  if (ball_release_servo_state == BALL_RELEASE_OPENING || ball_release_servo_state == BALL_RELEASE_CLOSING) {
    // Kiểm tra xem đã hết thời gian hành động chưa
    if (millis() - ball_release_action_start_time >= SERVO_ACTION_TIME) {
      // Dừng servo khi hoàn thành
      setContinuousServoPulse(BALL_RELEASE_SERVO_CHANNEL, SERVO_360_STOP_PULSE);
      
      // Cập nhật trạng thái
      if (ball_release_servo_state == BALL_RELEASE_OPENING) {
        ball_release_servo_state = BALL_RELEASE_OPEN;
        ball_release_state = true;
        #ifdef DEBUG_SERVO
          Serial.println("Servo thả bóng: ĐÃ MỞ");
        #endif
      } else { // BALL_RELEASE_CLOSING
        ball_release_servo_state = BALL_RELEASE_CLOSED;
        ball_release_state = false;
        #ifdef DEBUG_SERVO
          Serial.println("Servo thả bóng: ĐÃ ĐÓNG");
        #endif
      }
    }
  }
}

// Bắt đầu mở servo 180° thả bóng
void startOpenBallRelease180() {
  // Đặt servo 180° đến góc kết thúc để mở
  setServoAngle(BALL_RELEASE_SERVO_180_CHANNEL, SERVO_180_END_ANGLE);
  ball_release_180_state = true;
  
  #ifdef DEBUG_SERVO
    Serial.println("Servo 180° thả bóng: MỞ");
  #endif
}

// Bắt đầu đóng servo 180° thả bóng
void startCloseBallRelease180() {
  // Đặt servo 180° về góc bắt đầu để đóng
  setServoAngle(BALL_RELEASE_SERVO_180_CHANNEL, SERVO_180_START_ANGLE);
  ball_release_180_state = false;
  
  #ifdef DEBUG_SERVO
    Serial.println("Servo 180° thả bóng: ĐÓNG");
  #endif
}

// Cập nhật trạng thái servo 180° thả bóng
void updateBallRelease180State() {
  // Servo 180° không cần cập nhật trạng thái vì nó không sử dụng delay
}

// Cập nhật tất cả cơ cấu
void updateMechanisms() {
  // Cập nhật trạng thái các servo
  updateBallReleaseState();
  updateBallRelease180State();
}

//=================== TIỆN ÍCH ===================
void showDeviceStatus() {
  Serial.println("\n==== TRẠNG THÁI THIẾT BỊ ====");
  Serial.println("- Servo 360° thả bóng: " + String(ball_release_state ? "MỞ" : "ĐÓNG"));
  Serial.println("- Servo 180° thả bóng: " + String(ball_release_180_state ? "MỞ" : "ĐÓNG"));
  Serial.println("- Góc servo 180°: Bắt đầu=" + String(SERVO_180_START_ANGLE) + "°, Kết thúc=" + String(SERVO_180_END_ANGLE) + "°");
  Serial.println("============================");
}