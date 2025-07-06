/*
 * robot_mechanisms.cpp - ĐIỀU KHIỂN CƠ CẤU ROBOT
 *
 * Mô tả: Cung cấp các hàm điều khiển cơ cấu robot bao gồm động cơ DC,
 * servo 180° và 360°, cơ cấu hãm và các thiết bị khác.
 */

#include "robot_mechanisms.h"
#include <Wire.h>

//===================== BIẾN TOÀN CỤC =====================
// Driver PWM PCA9685
Adafruit_PWMServoDriver pwm;

// Biến trạng thái
bool servo180_position = false;         // false = vị trí 1, true = vị trí 2

// Biến lưu giá trị hiệu chỉnh servo 360
int current_servo360_pulse = SERVO_360_STOP_PULSE;

//======================= KHỞI TẠO =======================
void initPCA9685() {
  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);  // Sử dụng tần số 50Hz từ config.h
  Wire.setClock(400000); 
  Serial.println("PCA9685 đã được khởi tạo.");
}

void initDriveMotors() {
  stopDriveMotors();
  Serial.println("Động cơ di chuyển đã được khởi tạo và dừng.");
}

void initServos() {

  // Servo mới - khởi tạo với giá trị dừng
  setServoAngle(SERVO_180_CHANNEL, SERVO_180_POS_1); // Servo 180 độ - kênh 2
  
  // Khởi tạo các chốt hãm ở trạng thái gài
    engageLiftPawls();
    delay(200); // Chờ một chút để chốt gài hẳn
   
  // Khởi tạo servo 360 độ 
  // Quy trình đảm bảo servo khởi tạo đúng
  // 1. Tắt hoàn toàn servo trước để tránh giật
  
  // 2. Đặt servo lần lượt qua các trạng thái để nó nhận tín hiệu đúng
  // Xung dừng cho servo 360° (1.5ms ở 50Hz)
  Serial.println("Đặt servo 360° về vị trí dừng");
  pwm.setPWM(SERVO_360_CHANNEL, 0, SERVO_360_STOP_PULSE);
  delay(500);

  Serial.println("Đặt servo 360B° về vị trí dừng");
  pwm.setPWM(SERVO_360B_CHANNEL, 0, SERVO_360_STOP_PULSE);
  delay(500);

  
  
  
  // Hiển thị thông tin khởi tạo
  #ifdef DEBUG_SERVO
    Serial.println("Các servo đã được khởi tạo:");
    Serial.println("- Servo 1 (Kẹp): Đóng");
    Serial.println("- Servo 2 (Tay gắp): Hạ xuống");
    Serial.println("- Servo 180° (Kênh 2): Vị trí 1");
    Serial.println("- Servo 360° (Kênh 3): Đã dừng");
    Serial.println("- Servo 360B° (Kênh 5): Đã dừng");
    Serial.print("  + Xung dừng Servo 360°: ");
    Serial.println(SERVO_360_STOP_PULSE);
    
    if (SERVO_360_TEST_MODE) {
      Serial.println("\n*** CHẾ ĐỘ TEST SERVO 360° ĐƯỢC BẬT ***");
      Serial.println("Sử dụng L2/R2 để điều chỉnh giá trị dừng");
      Serial.println("Điều chỉnh cho đến khi servo dừng hoàn toàn");
    }
  #else
  Serial.println("Các servo đã được khởi tạo.");
  #endif
}

void initAuxDCMotors() {
  setAuxDCMotor(3, 0); 
  setAuxDCMotor(4, 0); 
  #ifdef DEBUG_MOTOR
    Serial.println("Động cơ DC phụ trợ đã được khởi tạo và dừng:");
    Serial.println("- Động cơ 3: Đã dừng");
    Serial.println("- Động cơ 4: Đã dừng");
  #else
  Serial.println("Động cơ DC phụ trợ đã được khởi tạo và dừng.");
  #endif
}

//================= ĐIỀU KHIỂN ĐỘNG CƠ DC =================
// --- Động cơ di chuyển chính ---
void setLeftDrivePWM(int pwmValue) {
  pwmValue = constrain(pwmValue, -PWM_MAX, PWM_MAX);
  if (pwmValue >= 0) { // Forward or Stop
    pwm.setPWM(M1_IN1_CHANNEL, 0, pwmValue);
    pwm.setPWM(M1_IN2_CHANNEL, 0, 0);
  } else { // Backward
    pwm.setPWM(M1_IN1_CHANNEL, 0, 0);
    pwm.setPWM(M1_IN2_CHANNEL, 0, abs(pwmValue));
  }
}

void setRightDrivePWM(int pwmValue) {
  pwmValue = constrain(pwmValue, -PWM_MAX, PWM_MAX);
  if (pwmValue >= 0) { // Forward or Stop
    pwm.setPWM(M2_IN1_CHANNEL, 0, pwmValue);
    pwm.setPWM(M2_IN2_CHANNEL, 0, 0);
  } else { // Backward
    pwm.setPWM(M2_IN1_CHANNEL, 0, 0);
    pwm.setPWM(M2_IN2_CHANNEL, 0, abs(pwmValue));
  }
}

void stopDriveMotors() {
  setLeftDrivePWM(0);
  setRightDrivePWM(0);
}

// --- Động cơ phụ trách cơ cấu nâng hạ ---
void setAuxDCMotor(uint8_t motorNum, int speedVal) {
  speedVal = constrain(speedVal, -PWM_MAX, PWM_MAX);
  uint8_t in1_ch, in2_ch;

  if (motorNum == 3) {
    in1_ch = AUX_M3_IN1_CHANNEL;
    in2_ch = AUX_M3_IN2_CHANNEL;
  } else if (motorNum == 4) {
    in1_ch = AUX_M4_IN1_CHANNEL;
    in2_ch = AUX_M4_IN2_CHANNEL;
  } else {
    return; 
  }

  if (speedVal == 0) {
    pwm.setPWM(in1_ch, 0, 0);
    pwm.setPWM(in2_ch, 0, 0);
    #ifdef DEBUG_MOTOR
      Serial.print("Động cơ "); Serial.print(motorNum); Serial.println(" đã dừng.");
    #endif
  } else if (speedVal > 0) { 
    pwm.setPWM(in1_ch, 0, speedVal);
    pwm.setPWM(in2_ch, 0, 0);
    #ifdef DEBUG_MOTOR
      Serial.print("Động cơ "); Serial.print(motorNum); 
      Serial.print(" quay thuận, tốc độ: "); Serial.println(speedVal);
    #endif
  } else { 
    pwm.setPWM(in1_ch, 0, 0);
    pwm.setPWM(in2_ch, 0, abs(speedVal));
    #ifdef DEBUG_MOTOR
      Serial.print("Động cơ "); Serial.print(motorNum); 
      Serial.print(" quay ngược, tốc độ: "); Serial.println(abs(speedVal));
    #endif
  }
}

void auxMotor3Control(int speed) {
    setAuxDCMotor(3, speed);
}

void auxMotor4Control(int speed) {
    setAuxDCMotor(4, speed);
}

//=================== ĐIỀU KHIỂN SERVO ===================
// --- Chức năng servo cơ bản ---
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

// pulseValue: giá trị PWM trực tiếp (0-4095)
void setContinuousServoPulse(uint8_t servoChannel, int pulseValue) {
  pulseValue = constrain(pulseValue, 0, PWM_MAX); // Đảm bảo pulse nằm trong khoảng 0-4095
  
  // Nếu là servo 360 độ và giá trị gần với STOP_PULSE, sử dụng chính xác STOP_PULSE
  // Điều này giúp tránh hiện tượng "trôi" nhẹ ở giá trị gần điểm dừng
  if ((servoChannel == SERVO_360_CHANNEL || servoChannel == SERVO_360B_CHANNEL) && 
      abs(pulseValue - SERVO_360_STOP_PULSE) < 10) {
    pulseValue = SERVO_360_STOP_PULSE;
  }
  
  // Đặt giá trị PWM cho kênh servo
  pwm.setPWM(servoChannel, 0, pulseValue);
  
  #ifdef DEBUG_SERVO
    Serial.print("Servo liên tục "); Serial.print(servoChannel);
    Serial.print(" đặt xung: "); Serial.println(pulseValue);
    
    // Hiển thị thông tin về vị trí dừng, quay thuận hay ngược
    if (servoChannel == SERVO_360_CHANNEL || servoChannel == SERVO_360B_CHANNEL) {
      if (pulseValue == SERVO_360_STOP_PULSE) {
        Serial.println("  - Servo 360° đang ở vị trí DỪNG");
      } else if (pulseValue > SERVO_360_STOP_PULSE) {
        Serial.println("  - Servo 360° đang quay THEO CHIỀU kim đồng hồ");
      } else {
        Serial.println("  - Servo 360° đang quay NGƯỢC CHIỀU kim đồng hồ");
      }
    }
  #endif
}

// --- Servo 180° (Kênh 2) ---
void toggleServo180() {
  servo180_position = !servo180_position;
  if (servo180_position) {
    setServo180Position(SERVO_180_POS_2);
  } else {
    setServo180Position(SERVO_180_POS_1);
  }
  
  #ifdef DEBUG_SERVO
    Serial.print("Chuyển đổi Servo 180° sang vị trí: ");
    Serial.println(servo180_position ? "2" : "1");
  #endif
}

void setServo180Position(uint8_t position) {
  setServoAngle(SERVO_180_CHANNEL, position);
}

// --- Servo 180B° (Kênh 4) ---
void setServo180BPosition(uint8_t angle) {
  setServoAngle(SERVO_180B_CHANNEL, angle);
  Serial.print("Servo 180B (kênh 4) đặt góc: ");
  Serial.println(angle);
}

// --- Cơ cấu hãm (Ratchet and Pawl) ---
void engageLiftPawls() {
    setServoAngle(SERVO_PAWL_LEFT_CHANNEL, PAWL_LEFT_ENGAGE_ANGLE);
    setServoAngle(SERVO_PAWL_RIGHT_CHANNEL, PAWL_RIGHT_ENGAGE_ANGLE);
    #ifdef DEBUG_SERVO
    Serial.println("Chốt hãm ĐÃ GÀI (Khóa)");
    #endif
}
void releaseLiftPawls() {
    setServoAngle(SERVO_PAWL_LEFT_CHANNEL, PAWL_LEFT_RELEASE_ANGLE);
    setServoAngle(SERVO_PAWL_RIGHT_CHANNEL, PAWL_RIGHT_RELEASE_ANGLE);
    #ifdef DEBUG_SERVO
    Serial.println("Chốt hãm ĐÃ MỞ (Không khóa)");
    #endif
}

//===================== HIỆU CHỈNH SERVO =====================
// Tăng giá trị dừng servo 360°
void increaseServo360StopPulse() {
  current_servo360_pulse += SERVO_360_FINE_TUNE_STEP;
  if (current_servo360_pulse > SERVO_360_MAX_PULSE) {
    current_servo360_pulse = SERVO_360_MAX_PULSE;
  }

  setContinuousServoPulse(SERVO_360_CHANNEL, current_servo360_pulse);
  setContinuousServoPulse(SERVO_360B_CHANNEL, current_servo360_pulse);
  
  Serial.print("Xung dừng servo 360°: ");
  Serial.println(current_servo360_pulse);
}

// Giảm giá trị dừng servo 360°
void decreaseServo360StopPulse() {
  current_servo360_pulse -= SERVO_360_FINE_TUNE_STEP;
  if (current_servo360_pulse < SERVO_360_MIN_PULSE) {
    current_servo360_pulse = SERVO_360_MIN_PULSE;
  }

  setContinuousServoPulse(SERVO_360_CHANNEL, current_servo360_pulse);
  setContinuousServoPulse(SERVO_360B_CHANNEL, current_servo360_pulse);
  
  Serial.print("Xung dừng servo 360°: ");
  Serial.println(current_servo360_pulse);
}

//===================== TIỆN ÍCH =====================
void showDeviceStatus() {
  Serial.println("\n----- TRẠNG THÁI THIẾT BỊ -----");
  
  // Trạng thái servo
  Serial.println("SERVO:");
  Serial.print("- Servo 180° (Kênh 2): Vị trí "); Serial.println(servo180_position ? "2" : "1");
  Serial.print("- Servo 360° & 360B° Xung dừng (Hiệu chỉnh): "); Serial.println(current_servo360_pulse);
  
  // Trạng thái motor
  Serial.println("\nĐỘNG CƠ:");
  Serial.println("- Động cơ 3 & 4 (Bộ nâng hạ): Hoạt động đồng bộ");
  
  // Hệ thống phanh
  Serial.println("\nHỆ THỐNG PHANH:");
  Serial.print("- Ngưỡng phát hiện phanh: "); 
  Serial.print(BRAKE_DETECTION_THRESHOLD * 100); Serial.println("% tốc độ");
  Serial.print("- Cường độ phanh: "); 
  Serial.print(BRAKE_POWER_FACTOR * 100); Serial.println("% PWM");
  Serial.print("- Thời lượng phanh: "); 
  Serial.print(BRAKE_DURATION_MS); Serial.println("ms");
  Serial.println("- Hỗ trợ phanh khi rẽ: Có");
  
  Serial.println("-----------------------\n");
}
