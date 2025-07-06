/*
* XỬ LÝ TAY CẦM PS2
* -------------------------------------
* Header file chứa các khai báo hàm xử lý tay cầm PS2 cho robot 3 bánh Omni
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

// ================== QUẢN LÝ TAY CẦM PS2 ===================
void setupPS2controller();              // Khởi tạo tay cầm PS2
void handlePS2Input();                  // Xử lý tất cả input từ tay cầm

// =============== ĐIỀU KHIỂN DI CHUYỂN ROBOT ===============
void handleHolonomicDrive();            // Điều khiển di chuyển toàn hướng 3 bánh omni
void handleLiftMotorControl();          // Điều khiển motor nâng hạ

// ================ ĐIỀU KHIỂN CƠ CẤU ROBOT ================
void handleBallReleaseControl();        // Điều khiển servo thả bóng

// ====================== TIỆN ÍCH ======================
float squareInput(float value);                    // Bình phương đầu vào (giữ dấu)
bool isPS2Connected();                             // Kiểm tra tay cầm có kết nối không

// Đọc và xử lý giá trị joystick
float readJoystickAxis(PS2X& ps2x, byte axis, byte calibValue);

// Áp dụng vùng chết cho đầu vào
float applyDeadband(float input);

#endif