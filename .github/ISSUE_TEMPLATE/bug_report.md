---
name: Báo cáo lỗi (Bug Report)
about: Tạo báo cáo để giúp hoàn thiện firmware DM-jump
title: '[BUG] '
labels: 'bug'
assignees: 'nguyenbinh-shark'
---

### Mô tả lỗi (Bug Description)
Mô tả rõ ràng và ngắn gọn về lỗi bạn gặp phải.

### Môi trường phần cứng & phần mềm (Environment)
- **MCU:** STM32H723VGT6
- **Board:** DM-jump CtrlBoard-H7
- **Động cơ:** DM4310 (Firmware version nếu có)
- **Cảm biến IMU:** BMI088 (SPI)
- **Công cụ biên dịch:** Keil MDK-ARM v5.xx / ARMCC v5.06
- **Giao diện kết nối:** UART1 (115200 baud) / CAN Bus / PS2 Controller

### Các bước tái hiện lỗi (Steps to Reproduce)
1. Cấu hình phần cứng: ...
2. Nạp code và khởi động hệ thống ...
3. Gửi lệnh qua UART hoặc điều khiển tay cầm ...
4. Hiện tượng xảy ra: ...

### Hành vi kỳ vọng (Expected Behavior)
Mô tả hành vi đúng mà hệ thống cần thực hiện.

### Log UART / Hình ảnh (Screenshots or Serial Logs)
```text
Dán log UART (115200 baud) tại đây nếu có
```

### Thông tin thêm (Additional Context)
Bất kỳ thông tin bổ sung nào khác về môi trường thử nghiệm.
