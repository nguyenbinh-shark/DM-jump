# AI Coding Agent Instructions for DM-jump Project

## Project Overview
This is a control board firmware for a robotic jumping platform using STM32H7 microcontroller. The system implements real-time control for legged locomotion with inertial navigation, motor actuation, and sensor fusion.

## Architecture
- **Microcontroller**: STM32H723 with FreeRTOS
- **Key Components**:
  - BMI088 IMU (SPI interface) for orientation sensing
  - DM4310 brushless motors (CAN bus) for joint actuation
  - FDCAN buses for motor communication
  - UART for debug output and PS2 controller input

## Code Organization
- `Core/`: STM32CubeMX generated HAL and FreeRTOS code
- `User/APP/`: FreeRTOS task implementations
- `User/Devices/`: Hardware drivers (BMI088 IMU, DM4310 motors)
- `User/Algorithm/`: Sensor fusion filters (Mahony AHRS, Kalman, EKF)
- `User/Controller/`: PID controllers with fuzzy logic
- `User/Bsp/`: Board support package utilities
- `MDK-ARM/`: Keil uVision project files

## Task Structure
FreeRTOS tasks run as infinite loops with `osDelay(1)`:
- `INS_Task`: High-priority IMU processing and attitude estimation
- `ChassisL_Task`/`ChassisR_Task`: Motor control for left/right legs
- `OBSERVE_Task`: State observation and estimation
- `PS2_Task`: Input handling from PS2 controller

## Key Patterns
- **Global State Structures**: Use structs like `INS_t`, `BMI088_t` for shared state
- **Task Communication**: Direct global variable access (no message queues in current implementation)
- **Motor Control**: CAN-based with specific ID mappings (e.g., left leg motors: TX 8/RX 4, TX 6/RX 3)
- **Debug Output**: UART prints at 10Hz intervals using `HAL_GetTick()` timing
- **Initialization**: Hardware init in `main.c`, task-specific init at task start

## Build System
- **IDE**: Keil uVision 5 (MDK-ARM project)
- **Generator**: STM32CubeMX for peripheral configuration
- **No CLI builds**: Use IDE for compilation and flashing

## Development Workflow
1. Modify code in User/ folders
2. Build and flash via Keil uVision
3. Debug via UART output (115200 baud)
4. Motor testing requires CAN bus connection

## Common Conventions
- **Coordinate System**: ENU (East-North-Up) for earth frame, FRD (Forward-Right-Down) for body
- **Units**: Radians for angles, m/s² for acceleration, Nm for torques
- **Timing**: DWT cycle counter for microsecond-precision timing
- **Error Handling**: Asserts disabled, use debug prints for issues

## Integration Points
- **IMU Data Flow**: SPI → BMI088 driver → Mahony filter → quaternion output
- **Motor Commands**: PID output → CAN messages → DM4310 drivers
- **State Sharing**: INS struct updated by INS_Task, read by chassis tasks

## Example Patterns
```c
// Task structure
void Some_Task(void const * argument) {
    Init_Function();
    while(1) {
        Process_Data();
        osDelay(1);
    }
}

// Debug output timing
if (HAL_GetTick() - last_print >= 100) {  // 10Hz
    uart_send_str("Debug info\r\n");
    last_print = HAL_GetTick();
}
```

## File References
- [User/APP/INS_task.c](User/APP/INS_task.c): IMU processing and AHRS
- [User/Controller/controller.h](User/Controller/controller.h): PID implementation
- [User/Devices/BMI088/](User/Devices/BMI088/): IMU driver
- [Core/Src/main.c](Core/Src/main.c): Hardware initialization</content>
<parameter name="filePath">c:\Users\shark\Desktop\DM-jump\.github\copilot-instructions.md