/*
 * PS2_controller.cpp - ĐIỀU KHIỂN TAY CẦM PS2
 * 
 * Mô tả: Xử lý tín hiệu từ tay cầm PS2 và điều khiển các cơ cấu robot tương ứng.
 * Hỗ trợ điều khiển di chuyển, nâng hạ, servo và các chức năng đặc biệt.
 */

#include "PS2_controller.h"

//======================= BIẾN TOÀN CỤC =======================
PS2X ps2x; 
bool ps2_controller_connected = false;

// Trạng thái cơ cấu
bool gripper_is_closed = true; 
bool servo180B_is_45 = false;
bool auto_pawl_enabled = true;       // Trạng thái bật/tắt tính năng tự động mở/cài chốt

// Thời gian cho debounce
unsigned long last_gripper_toggle_time = 0;
unsigned long last_servo180_toggle_time = 0;
unsigned long last_servo360_tuning_time = 0;
unsigned long last_pawl_toggle_time = 0;
const unsigned long debounce_delay = 250; 

// Biến lưu trữ tốc độ động cơ phụ trợ
static float currentMotor3PWMSmoothed = 0.0f;
static float currentMotor4PWMSmoothed = 0.0f;

//======================= TIỆN ÍCH ===========================
float applyDeadband(float value, float deadband) {
if (abs(value) < deadband) {
    return 0.0;
}
// Scale lại giá trị sau deadband để đạt được full range
return (value - copysign(deadband, value)) / (1.0 - deadband);
}

float squareInput(float value) {
return value * abs(value); // Giữ dấu nhưng áp dụng bình phương
}

//===================== KHỞI TẠO PS2 ========================
void setupPS2controller() {
int err = -1;
Serial.println("Cấu hình tay cầm PS2...");
for (int attempt = 0; attempt < 5 && err != 0; attempt++) {
    err = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, true, true);
    if (err == 0) {
    Serial.println("Tay cầm PS2 đã được cấu hình thành công.");
    ps2x.read_gamepad(false, false);
    ps2_controller_connected = true;
    break;
    } else {
    Serial.print("Lỗi cấu hình tay cầm PS2: ");
    switch (err) {
        case 1: Serial.println("Không tìm thấy tay cầm"); break;
        case 2: Serial.println("Tay cầm không chấp nhận lệnh"); break;
        case 3: Serial.println("Tay cầm từ chối chế độ Pressures"); break;
        default: Serial.println("Lỗi không xác định");
    }
    delay(300);
    }
}

if (err != 0) {
    Serial.println("CẢNH BÁO: Không thể cấu hình tay cầm PS2 sau nhiều lần thử!");
}

if (!ps2_controller_connected) {
    Serial.println("CẢNH BÁO: Robot sẽ không phản hồi với tay cầm PS2!");
}
}

//=================== ĐIỀU KHIỂN DI CHUYỂN ===================
void handleAdvancedArcadeDrive() {
    // Biến local cho hàm này
    static float currentLeftPWMSmoothed = 0.0f;
    static float currentRightPWMSmoothed = 0.0f;
    static float prev_raw_throttle = 0.0f;
    static unsigned long last_brake_time = 0;
    static bool brake_applied = false;
    static float prev_steer_for_brake = 0.0f;
    unsigned long currentMillis = millis();

    // Đọc giá trị từ joystick trái
    float rawThrottleInput = (Y_JOY_CALIB - ps2x.Analog(PSS_LY)) / 128.0;
    float rawSteerInput = (X_JOY_CALIB - ps2x.Analog(PSS_LX)) / 127.0;

    // Áp dụng deadband và bình phương giá trị
    float throttle = applyDeadband(rawThrottleInput, INPUT_DEADBAND);
    float steer = applyDeadband(rawSteerInput, INPUT_DEADBAND);
    throttle = squareInput(throttle);
    steer = squareInput(steer);
    
    // Áp dụng độ nhạy và giới hạn giá trị
    throttle *= THROTTLE_SENSITIVITY;
    steer *= STEERING_SENSITIVITY;
    throttle = constrain(throttle, -1.0, 1.0);
    steer = constrain(steer, -1.0, 1.0);

    // Tính toán đầu ra Arcade Drive
    float leftOutput, rightOutput;
    bool quickTurnEngaged = ps2x.Button(QUICKTURN_BUTTON);

    if (quickTurnEngaged) {
        // Chế độ xoay tại chỗ (QuickTurn)
        leftOutput = steer * QUICKTURN_SENSITIVITY;
        rightOutput = -steer * QUICKTURN_SENSITIVITY;
    } else {
        // Chế độ arcade drive thông thường với độ nhạy thay đổi theo tốc độ
        float turnSensitivity = 1.0 - pow(abs(throttle), TURN_EXPONENT) * TURN_GAIN;
        turnSensitivity = max(turnSensitivity, MIN_TURN_SENSITIVITY);
        float adjustedSteer = steer * turnSensitivity;
        leftOutput = throttle + adjustedSteer;
        rightOutput = throttle - adjustedSteer;
        
        // Giới hạn đầu ra tối đa là 1.0
        float maxMagnitude = max(abs(leftOutput), abs(rightOutput));
        if (maxMagnitude > 1.0) {
            leftOutput /= maxMagnitude;
            rightOutput /= maxMagnitude;
        }
    }

    // Chọn hệ số tốc độ dựa trên chế độ lái
    float current_max_speed_factor;
    if (ps2x.Button(PRECISION_MODE_BUTTON)) {
        current_max_speed_factor = PRECISION_DRIVE_SPEED_FACTOR;
    } else if (ps2x.Button(PSB_R2)) {
        current_max_speed_factor = TOP_DRIVE_SPEED_FACTOR;
    } else {
        current_max_speed_factor = NORM_DRIVE_SPEED_FACTOR;
    }

    // Chuyển đổi sang giá trị PWM thực tế
    float targetLeftPWM_float = leftOutput * PWM_MAX * current_max_speed_factor;
    float targetRightPWM_float = rightOutput * PWM_MAX * current_max_speed_factor;

    // Hỗ trợ phanh khi rẽ
    float brake_assist_factor = 0.1f;
    float steer_release_threshold = 0.1f;
    if (abs(steer) < steer_release_threshold && abs(prev_steer_for_brake) >= steer_release_threshold) {
        if (prev_steer_for_brake > 0) {
            targetLeftPWM_float = -(PWM_MAX * current_max_speed_factor * brake_assist_factor);
            targetRightPWM_float = (PWM_MAX * current_max_speed_factor * brake_assist_factor);
        } else {
            targetLeftPWM_float = (PWM_MAX * current_max_speed_factor * brake_assist_factor);
            targetRightPWM_float = -(PWM_MAX * current_max_speed_factor * brake_assist_factor);
        }
    }
    prev_steer_for_brake = steer;

    // Dừng tức thì nếu không có đầu vào
    if ((!quickTurnEngaged && fabs(throttle) < 0.01f && fabs(steer) < 0.01f) ||
        (quickTurnEngaged && fabs(steer) < 0.01f)) {
        currentLeftPWMSmoothed = 0.0f;
        currentRightPWMSmoothed = 0.0f;
        setLeftDrivePWM(0);
        setRightDrivePWM(0);
        brake_applied = false;
        prev_raw_throttle = rawThrottleInput;
        return;
    }

    // Hệ thống phanh khẩn cấp
    if (!brake_applied && abs(throttle) < 0.05f && abs(prev_raw_throttle) >= BRAKE_DETECTION_THRESHOLD) {
        brake_applied = true;
        last_brake_time = currentMillis;
        float brake_intensity = abs(prev_raw_throttle) * BRAKE_POWER_FACTOR;
        if (prev_raw_throttle > 0) {
            targetLeftPWM_float = -PWM_MAX * brake_intensity;
            targetRightPWM_float = -PWM_MAX * brake_intensity;
        } else {
            targetLeftPWM_float = PWM_MAX * brake_intensity;
            targetRightPWM_float = PWM_MAX * brake_intensity;
        }
    } else if (brake_applied) {
        if (currentMillis - last_brake_time < BRAKE_DURATION_MS) {
            // Giữ nguyên phanh trong thời gian cài đặt
        } else {
            brake_applied = false;
        }
    }
    prev_raw_throttle = rawThrottleInput;

    // Áp dụng giới hạn gia tốc (làm mượt chuyển động)
    float maxDeltaPWM = MAX_ACCELERATION_FACTOR * PWM_MAX;
    
    // Làm mượt động cơ bên trái
    currentLeftPWMSmoothed = constrain(targetLeftPWM_float, 
                                    currentLeftPWMSmoothed - maxDeltaPWM, 
                                    currentLeftPWMSmoothed + maxDeltaPWM);
    
    // Làm mượt động cơ bên phải
    currentRightPWMSmoothed = constrain(targetRightPWM_float, 
                                        currentRightPWMSmoothed - maxDeltaPWM, 
                                        currentRightPWMSmoothed + maxDeltaPWM);
    
    // Áp dụng PWM cuối cùng vào động cơ
    int finalLeftPWM = (int)currentLeftPWMSmoothed;
    int finalRightPWM = (int)currentRightPWMSmoothed;
    setLeftDrivePWM(finalLeftPWM);
    setRightDrivePWM(finalRightPWM);
    
    #ifdef DEBUG_DRIVE_OUTPUT
        static unsigned long last_debug_time = 0;
        if (currentMillis - last_debug_time > 500) {
            Serial.print("L: "); Serial.print(finalLeftPWM);
            Serial.print(" | R: "); Serial.println(finalRightPWM);
            last_debug_time = currentMillis;
        }
    #endif
}

void handleAuxMotorControl() {
    // === ĐIỀU KHIỂN CƠ CẤU NÂNG HẠ VỚI NITRO ===

    // Biến static để theo dõi trạng thái và thời gian
    static bool lift_is_moving = false;
    static unsigned long last_move_command_time = 0;

    // 1. Đọc trạng thái joystick và nút Nitro
    float rawRightYInput = (Y_JOY_CALIB - ps2x.Analog(PSS_RY)) / 128.0;
    bool nitro_is_active = ps2x.Button(PSB_R2); 

    // 2. Xác định xem người dùng có đang ra lệnh di chuyển hay không
    bool move_command_active = (abs(rawRightYInput) > INPUT_DEADBAND);

    // 3. XỬ LÝ LOGIC CHO CHỨC NĂNG TỰ ĐỘNG MỞ/CÀI CHỐT
    if (auto_pawl_enabled) {
        // Nếu tính năng tự động mở/cài chốt được bật
        if (move_command_active && !lift_is_moving) {
            // --- CHUỖI HÀNH ĐỘNG: BẮT ĐẦU DI CHUYỂN ---
            // Người dùng vừa mới đẩy joystick.
            // Tự động mở chốt trước khi motor chạy
            releaseLiftPawls();
            
            // Cập nhật trạng thái và thời gian
            lift_is_moving = true;
            last_move_command_time = millis();
            
            #ifdef DEBUG_SERVO
            Serial.println("Tự động mở chốt khi bắt đầu di chuyển");
            #endif
            
            // Chờ một khoảng thời gian ngắn để servo kịp hoạt động
            delay(PAWL_TIMING_DELAY);
        } 
        else if (!move_command_active && lift_is_moving) {
            // --- CHUỖI HÀNH ĐỘNG: DỪNG DI CHUYỂN ---
            // Người dùng vừa mới nhả joystick.
            
            // Dừng motor ngay lập tức
            currentMotor3PWMSmoothed = 0.0f;
            currentMotor4PWMSmoothed = 0.0f;
            auxMotor3Control(0);
            auxMotor4Control(0);
            
            // Chờ một khoảng thời gian ngắn để quán tính motor tắt hẳn
            delay(PAWL_TIMING_DELAY); 
            
            // Tự động cài chốt để khóa vị trí
            engageLiftPawls();
            
            #ifdef DEBUG_SERVO
            Serial.println("Tự động cài chốt khi dừng di chuyển");
            #endif
            
            // Cập nhật trạng thái
            lift_is_moving = false;
            
            // Thoát khỏi hàm sau khi hoàn thành chuỗi dừng
            return;
        }
    }
    else {
        // Nếu tính năng tự động mở/cài chốt bị tắt
        // Chỉ cập nhật trạng thái di chuyển để theo dõi, không điều khiển servo
        if (move_command_active && !lift_is_moving) {
            lift_is_moving = true;
        }
        else if (!move_command_active && lift_is_moving) {
            // Dừng motor
            currentMotor3PWMSmoothed = 0.0f;
            currentMotor4PWMSmoothed = 0.0f;
            auxMotor3Control(0);
            auxMotor4Control(0);
            lift_is_moving = false;
            return;
        }
    }
    
    // 4. NHÁNH THOÁT NẾU KHÔNG CÓ LỆNH DI CHUYỂN
    if (!move_command_active) {
        return;
    }

    // 5. ĐIỀU KHIỂN MOTOR NÂNG HẠ - PHẦN NÀY GIỮ NGUYÊN NHƯ CŨ
    float targetMotorSpeed = 0.0f;
    float current_accel_factor;
    
    // --- XỬ LÝ ĐẶC BIỆT KHI NITRO ĐƯỢC KÍCH HOẠT ---
    if (nitro_is_active) {
        // KHI NITRO (R2) ĐƯỢC KÍCH HOẠT - CHẾ ĐỘ TREO ROBOT
        current_accel_factor = AUX_MOTOR_ACCELERATION_FACTOR * 10.0f;
        
        if (rawRightYInput > 0) {
            // Joystick hướng lên: Motor quay "màu xanh" (cùng chiều nâng)
            const float speed_scale = 0.6;
            targetMotorSpeed = AUX_MOTOR_SPEED * speed_scale * rawRightYInput;
            
            // Áp dụng làm mượt để không giật
            float maxDeltaPWM = current_accel_factor * AUX_MOTOR_SPEED;
            currentMotor3PWMSmoothed = constrain(targetMotorSpeed, 
                                               currentMotor3PWMSmoothed - maxDeltaPWM, 
                                               currentMotor3PWMSmoothed + maxDeltaPWM);
            currentMotor4PWMSmoothed = currentMotor3PWMSmoothed;
            
            // Cấp PWM dương cho motor (màu xanh)
            auxMotor3Control((int)currentMotor3PWMSmoothed);
            auxMotor4Control((int)currentMotor4PWMSmoothed);
        } 
        else if (rawRightYInput < 0) {
            // Joystick hướng xuống: Motor quay "màu đỏ" (ngược chiều nâng)
            // Áp dụng NITRO_HANGING_SPEED_FACTOR để điều chỉnh phần trăm tốc độ
            float speed = PWM_MAX * NITRO_HANGING_SPEED_FACTOR * (-rawRightYInput);
            
            // Áp dụng làm mượt để không giật
            float maxDeltaPWM = current_accel_factor * AUX_MOTOR_SPEED;
            currentMotor3PWMSmoothed = constrain(speed, 
                                               currentMotor3PWMSmoothed - maxDeltaPWM, 
                                               currentMotor3PWMSmoothed + maxDeltaPWM);
            currentMotor4PWMSmoothed = currentMotor3PWMSmoothed;
            
            // Cấp PWM âm cho motor (màu đỏ)
            auxMotor3Control(-(int)currentMotor3PWMSmoothed);
            auxMotor4Control(-(int)currentMotor4PWMSmoothed);
            
            #ifdef DEBUG_MOTOR
            static unsigned long last_debug_time = 0;
            if (millis() - last_debug_time > 500) {
                Serial.print("Nitro Hanging Speed: "); 
                Serial.print(NITRO_HANGING_SPEED_FACTOR * 100.0f);
                Serial.print("% | Joystick: ");
                Serial.print(-rawRightYInput);
                Serial.print(" | Target Speed: "); 
                Serial.println(speed);
                last_debug_time = millis();
            }
            #endif
        }
        else {
            // Joystick ở vị trí trung tâm: Dừng motor
            currentMotor3PWMSmoothed = 0.0f;
            currentMotor4PWMSmoothed = 0.0f;
            auxMotor3Control(0);
            auxMotor4Control(0);
        }
        
        // Đã xử lý xong Nitro, thoát khỏi hàm
        return;
    } 
    // --- XỬ LÝ THÔNG THƯỜNG KHI KHÔNG CÓ NITRO ---
    else {
        // CHẾ ĐỘ THƯỜNG - ĐẢM BẢO KHÔNG RƠI TỰ DO
        current_accel_factor = AUX_MOTOR_ACCELERATION_FACTOR;
        
        if (rawRightYInput > 0) {
            // NÂNG LÊN: Hoạt động bình thường với 60% tốc độ
            const float speed_scale = 0.6;
            targetMotorSpeed = AUX_MOTOR_SPEED * speed_scale * rawRightYInput;
        } else {
            // HẠ XUỐNG: Vẫn cấp điện cùng chiều nâng lên để chống rơi tự do
            // Sử dụng tốc độ cố định được cấu hình trong config.h (mặc định 20%)
            // Giữ nguyên tốc độ LOWERING_SPEED_FACTOR không đổi khi kéo joystick xuống
            targetMotorSpeed = AUX_MOTOR_SPEED * LOWERING_SPEED_FACTOR;
            
            #ifdef DEBUG_MOTOR
            static unsigned long last_debug_time = 0;
            if (millis() - last_debug_time > 500) {
                Serial.print("Lowering speed: "); 
                Serial.print(LOWERING_SPEED_FACTOR * 100.0f);
                Serial.print("% | Target Speed: "); 
                Serial.println(targetMotorSpeed);
                last_debug_time = millis();
            }
            #endif
        }
        
        float maxDeltaPWM = current_accel_factor * AUX_MOTOR_SPEED;

        currentMotor3PWMSmoothed = constrain(targetMotorSpeed, 
                                            currentMotor3PWMSmoothed - maxDeltaPWM, 
                                            currentMotor3PWMSmoothed + maxDeltaPWM);
        currentMotor4PWMSmoothed = currentMotor3PWMSmoothed;
        
        auxMotor3Control((int)currentMotor3PWMSmoothed);
        auxMotor4Control((int)currentMotor4PWMSmoothed);
    }
}

//================= ĐIỀU KHIỂN CƠ CẤU ROBOT =================
void handleServo360Tuning() {
    if (!SERVO_360_TEST_MODE) return;
    
    unsigned long current_time = millis();
    
    // Khi ở chế độ Test, L2/R2 dùng để hiệu chỉnh, không dùng để quay servo 360B
    if (ps2x.ButtonPressed(PSB_L2)) { // Dùng ButtonPressed để mỗi lần nhấn chỉ tăng/giảm 1
        decreaseServo360StopPulse();
    }
    else if (ps2x.ButtonPressed(PSB_R2)) {
        increaseServo360StopPulse();
    }
}

void handleMechanismControl() {
    unsigned long current_time = millis();

    // Servo 180B (kênh 4) - Nút Tam giác (Triangle) - Toggle
    if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Không cần debounce phức tạp với ButtonPressed
        servo180B_is_45 = !servo180B_is_45;
        if (servo180B_is_45) {
            setServo180BPosition(SERVO_180B_ANGLE);
        } else {
            setServo180BPosition(0);
        }
    }

    // Servo 180° (SERVO_180_CHANNEL) - Nút Vuông (Square)
    if (ps2x.ButtonPressed(PSB_SQUARE) && (current_time - last_servo180_toggle_time > debounce_delay)) {
        last_servo180_toggle_time = current_time;
        toggleServo180();
    }
    
    // --- Điều khiển servo 360° (kênh 3) với nút O/X ---
    // Đây là cách làm mới, không cần hàm start/stop
    static int lastPulseSent_360 = SERVO_360_STOP_PULSE;
    int currentPulse_360 = current_servo360_pulse; // Bắt đầu với giá trị dừng đã hiệu chỉnh
    if (ps2x.Button(PSB_CIRCLE)) {
        currentPulse_360 = SERVO_360_FWD_PULSE;
    } else if (ps2x.Button(PSB_CROSS)) {
        currentPulse_360 = SERVO_360_REV_PULSE;
    }
    if (currentPulse_360 != lastPulseSent_360) {
        pwm.setPWM(SERVO_360_CHANNEL, 0, currentPulse_360);
        lastPulseSent_360 = currentPulse_360;
    }

    // --- Điều khiển servo 360B° (kênh 5) với nút D-Pad Up/Down ---
    static int lastPulseSent_360B = SERVO_360_STOP_PULSE;
    int currentPulse_360B = current_servo360_pulse; // Bắt đầu với giá trị dừng đã hiệu chỉnh
    if (ps2x.Button(PSB_PAD_DOWN)) { // D-Pad Up quay thuận
        currentPulse_360B = SERVO_360_FWD_PULSE;
    } else if (ps2x.Button(PSB_PAD_UP)) { // D-Pad Down quay ngược
        currentPulse_360B = SERVO_360_REV_PULSE;
    }
    // Chỉ gửi lệnh PWM nếu trạng thái nút bấm thay đổi
    if (currentPulse_360B != lastPulseSent_360B) {
        pwm.setPWM(SERVO_360B_CHANNEL, 0, currentPulse_360B);
        lastPulseSent_360B = currentPulse_360B;
    }

    // --- Sử dụng nút PAD_RIGHTz để đóng chốt servo về vị trí ban đầu ---
    if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
        // Đặt servo kênh 6 và 7 về vị trí ban đầu (đóng chốt)
        engageLiftPawls();
        
        #ifdef DEBUG_SERVO
        Serial.println("Đã đóng chốt bằng nút PAD_RIGHT");
        #endif
    }
    
    // --- Sử dụng nút PAD_LEFT để mở chốt servo ---
    if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
        // Đặt servo kênh 6 và 7 sang vị trí mở chốt
        releaseLiftPawls();
        
        #ifdef DEBUG_SERVO
        Serial.println("Đã mở chốt bằng nút PAD_LEFT");
        #endif
    }

    // Chế độ hiệu chỉnh servo 360° (dùng L2/R2 khi TEST_MODE=true)
    handleServo360Tuning();
}

//===================== XỬ LÝ CHÍNH ======================
bool isPS2Connected() {
    static int disconnect_count = 0;
    // Kiểm tra tất cả giá trị analog của joystick
    if (ps2x.Analog(PSS_LX) == 0 && ps2x.Analog(PSS_LY) == 0 &&
        ps2x.Analog(PSS_RX) == 0 && ps2x.Analog(PSS_RY) == 0) {
        disconnect_count++;
        if (disconnect_count > 10) return false;
    } else {
        disconnect_count = 0;
    }
    return true;
}

void handlePS2Input() {
    // Đọc trạng thái gamepad một lần mỗi vòng lặp
    ps2x.read_gamepad(false, false);

    // Kiểm tra kết nối tay cầm
    if (!isPS2Connected()) {
        // Dừng tất cả hoạt động nếu mất kết nối
        stopDriveMotors();
        auxMotor3Control(0);
        auxMotor4Control(0);
        return;
    }
    
    // Phát hiện tổ hợp phím L3+R3 để bật/tắt tính năng tự động mở/cài chốt
    unsigned long current_time = millis();
    if (ps2x.Button(PSB_L3) && ps2x.Button(PSB_R3) && (current_time - last_pawl_toggle_time > debounce_delay)) {
        last_pawl_toggle_time = current_time;
        auto_pawl_enabled = !auto_pawl_enabled;
        
        #ifdef DEBUG_SERVO
        Serial.print("Tự động mở/cài chốt: ");
        Serial.println(auto_pawl_enabled ? "BẬT" : "TẮT");
        #endif
    }

    // Xử lý các chức năng chính
    handleAdvancedArcadeDrive();  // Điều khiển robot bằng joystick trái
    handleAuxMotorControl();      // Điều khiển motor phụ bằng joystick phải
    handleMechanismControl();     // Điều khiển các cơ cấu khác bằng nút
}