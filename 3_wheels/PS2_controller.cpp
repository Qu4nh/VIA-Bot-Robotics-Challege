/*
* XỬ LÝ TAY CẦM PS2
* -------------------------------------
* Cài đặt xử lý tay cầm PS2 cho robot 3 bánh Omni
*/

#include <Arduino.h>
#include <PS2X_lib.h>
#include "config.h"
#include "PS2_controller.h"
#include "robot_mechanisms.h"

//========== KHỞI TẠO ==========
PS2X ps2x;
bool ps2_controller_connected = false;

//========== BIẾN TOÀN CỤC ==========
// Trạng thái điều khiển
float forwardBackward = 0;   // Tiến/lùi
float leftRight = 0;         // Trái/phải
float rotation = 0;          // Xoay
bool slowModeEnabled = false;

//========== THIẾT LẬP TAY CẦM ==========
void setupPS2controller() {
  // Thử kết nối với tay cầm PS2
  int error = -1;
  for (int attempts = 0; attempts < 3; attempts++) {
    delay(300);
    error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, true, true);
    
    if (error == 0) {
      Serial.println("  => Kết nối PS2 thành công!");
      ps2_controller_connected = true;
      break;
    } else {
      Serial.print("  => Lỗi kết nối PS2, mã lỗi: ");
      Serial.println(error);
    }
  }
  
  if (!ps2_controller_connected) {
    Serial.println("  => CẢNH BÁO: Không thể kết nối với tay cầm PS2!");
    Serial.println("     Kiểm tra lại dây kết nối và reset ESP32");
  }
  
  delay(300);
}

//========== ĐIỀU KHIỂN DI CHUYỂN ==========
void processMovementControls() {
  // Đọc giá trị joystick và chuẩn hóa
  float rawY = readJoystickAxis(ps2x, PSS_LY, Y_JOY_CALIB);
  float rawX = readJoystickAxis(ps2x, PSS_LX, X_JOY_CALIB);
  float rawRX = readJoystickAxis(ps2x, PSS_RX, X_JOY_CALIB);
  
  // Áp dụng bộ lọc vùng chết
  float filteredY = applyDeadband(rawY);
  float filteredX = applyDeadband(rawX);
  float filteredRX = applyDeadband(rawRX);
  
  // Kiểm tra chế độ chậm
  slowModeEnabled = ps2x.Button(PSB_R1);
  
  // Tính toán các thành phần di chuyển
  float speedFactor = slowModeEnabled ? SLOW_MODE_FACTOR : MAX_DRIVE_SPEED_FACTOR;
  
  // Đảo chiều Y để tiến là dương
  forwardBackward = -filteredY; 
  leftRight = filteredX;
  rotation = filteredRX * STEERING_SENSITIVITY;
  
  // In debug nếu được kích hoạt
  #ifdef DEBUG_PS2_VALUES
    static unsigned long lastDebugOutput = 0;
    if (millis() - lastDebugOutput > 200) {
      Serial.print("FB=");
      Serial.print(forwardBackward);
      Serial.print(" LR=");
      Serial.print(leftRight);
      Serial.print(" Rot=");
      Serial.println(rotation);
      lastDebugOutput = millis();
    }
  #endif
  
  // Áp dụng điều khiển vào động cơ
  setHolonomicDrive(leftRight, forwardBackward, rotation);
}

//========== ĐIỀU KHIỂN CƠ CẤU ==========
void processMechanismControls() {
  // ------- Nâng hạ -------
  if (ps2x.Button(PSB_CIRCLE)) {
    // Nâng lên
    setLiftMotorSpeed(LIFT_MOTOR_SPEED);
  } else if (ps2x.Button(PSB_CROSS)) {
    // Hạ xuống
    setLiftMotorSpeed(-LIFT_MOTOR_SPEED);
  } else {
    // Dừng cơ cấu nâng hạ
    stopLiftMotor();
  }
  
  // ------- Thả bóng -------
  // Kiểm tra nút mở servo thả bóng
  if (ps2x.ButtonPressed(PSB_PAD_UP)) {
    startOpenBallRelease();
    startOpenBallRelease180();
    Serial.println("Thả bóng");
  }
  
  // Kiểm tra nút đóng servo thả bóng
  if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
    startCloseBallRelease();
    startCloseBallRelease180();
    Serial.println("Thu hồi");
  }
}

//========== XỬ LÝ TÍN HIỆU ==========
void handlePS2Input() {
  if (!ps2_controller_connected) return;
  
  // Đọc tín hiệu từ tay cầm
  ps2x.read_gamepad();
  
  // Xử lý điều khiển di chuyển
  processMovementControls();
  
  // Xử lý điều khiển cơ cấu
  processMechanismControls();
  
  // Cập nhật trạng thái các cơ cấu
  
  updateMechanisms();
}

//========== HÀM TIỆN ÍCH ==========
float applyDeadband(float input) {
  if (fabs(input) < INPUT_DEADBAND) {
    return 0.0f;
  }
  return input;
}

float squareInput(float input) {
  float sign = (input >= 0) ? 1.0f : -1.0f;
  return sign * input * input;
}

bool isPS2Connected() {
  return ps2_controller_connected;
}

float readJoystickAxis(PS2X& ps2x, byte axis, byte calibValue) {
  int rawValue = ps2x.Analog(axis);
  float normalizedValue = (float)(rawValue - calibValue) / 128.0f;
  // Giới hạn giá trị trong khoảng [-1, 1]
  return constrain(normalizedValue, -1.0f, 1.0f);
}