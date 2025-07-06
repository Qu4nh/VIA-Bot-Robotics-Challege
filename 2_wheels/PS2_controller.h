/*
 * PS2_controller.h - THƯ VIỆN ĐIỀU KHIỂN TAY CẦM PS2
 * 
 * Phiên bản: 1.0
 * Tác giả: Qu4nh
 * 
 * Mô tả: Xử lý tín hiệu từ tay cầm PS2 và điều khiển các cơ cấu robot tương ứng.
 * Hỗ trợ điều khiển di chuyển, nâng hạ, servo và các chức năng đặc biệt.
 */

#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include <PS2X_lib.h>
#include "config.h" 
#include "robot_mechanisms.h" 

// ====================== BIẾN TOÀN CỤC ======================
// Tay cầm PS2
extern PS2X ps2x; 

// Trạng thái kết nối
extern bool ps2_controller_connected;

// Trạng thái cơ cấu
extern bool servo180B_is_45;            // Trạng thái servo 180B
extern bool auto_pawl_enabled;          // Trạng thái bật/tắt tính năng tự động mở/cài chốt

// ================== QUẢN LÝ TAY CẦM PS2 ===================
void setupPS2controller();              // Khởi tạo tay cầm PS2
void handlePS2Input();                  // Xử lý tất cả input từ tay cầm

// =============== ĐIỀU KHIỂN DI CHUYỂN ROBOT ===============
void handleAdvancedArcadeDrive();       // Arcade Drive với các tính năng nâng cao
void handleAuxMotorControl();           // Điều khiển motor phụ trợ

// ================ ĐIỀU KHIỂN CƠ CẤU ROBOT ================
void handleMechanismControl();          // Điều khiển các cơ cấu (gripper, servo,...)
void handleServo360Tuning();            // Hiệu chỉnh servo 360°

// ====================== TIỆN ÍCH ======================
float applyDeadband(float value, float deadband);  // Áp dụng ngưỡng chết cho joystick
float squareInput(float value);                    // Bình phương đầu vào (giữ dấu)
bool isPS2Connected();                             // Kiểm tra tay cầm có kết nối không

#endif