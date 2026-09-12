# Hướng Dẫn Đóng Góp (Contributing Guide)

Chào mừng và cảm ơn bạn đã quan tâm đóng góp cho dự án **DM-jump**! 

Dự án này là hệ thống điều khiển firmware thời gian thực dành cho nền tảng robot bánh-chân nhảy (Wheeled-Bipedal Jumping Robot) trên vi điều khiển STM32H723, phát triển bởi **Trần Nguyên Bình** ([@nguyenbinh-shark](https://github.com/nguyenbinh-shark)).

---

## 1. Quy Chuẩn Mã Nguồn (Code Standards)

- **Ngôn ngữ:** C99 / C11 dành cho Embedded Systems.
- **Quy chuẩn file:** Mọi file mã nguồn mới hoặc chỉnh sửa đều phải có header chuẩn Doxygen chứa thông tin tác giả và bản quyền:
  ```c
  /**
    ******************************************************************************
    * @file           : <tên_file>
    * @brief          : <Mô tả chức năng file>
    * @author         : Trần Nguyên Bình (trannguyenbinh.shark@gmail.com)
    * @github         : https://github.com/nguyenbinh-shark/DM-jump
    * @date           : 2024 - 2026
    * @copyright      : Copyright (c) 2024-2026 Trần Nguyên Bình. All rights reserved.
    ******************************************************************************
    */
  ```
- **Quy chuẩn format:** Sử dụng bộ quy tắc trong [.clang-format](.clang-format) (thụt lề 4 dấu cách, không dùng tab).
- **Chú thích:** Chú thích rõ ràng các hàm, tham số đầu vào/ra (`@param[in]`, `@param[out]`, `@retval`), các công thức toán học và giải thuật VMC, Kalman, LQR, PID.
- **Biên dịch:** Đảm bảo dự án biên dịch trên Keil MDK-ARM v5 với **0 Error(s), 0 Warning(s)** trước khi tạo Pull Request.

---

## 2. Quy Trình Làm Việc (Git Workflow)

1. **Fork** repository về tài khoản cá nhân của bạn trên GitHub.
2. Tạo nhánh tính năng mới từ nhánh `main`:
   ```bash
   git checkout -b feature/ten-tinh-nang-moi
   ```
3. Thực hiện thay đổi, kiểm tra biên dịch và test thực tế hoặc mô phỏng.
4. Commit thay đổi tuân thủ [Conventional Commits](https://www.conventionalcommits.org/):
   - `feat:` Tính năng mới
   - `fix:` Sửa lỗi
   - `docs:` Cập nhật tài liệu
   - `refactor:` Tái cấu trúc mã nguồn không thay đổi logic
   - `perf:` Tối ưu hiệu năng
5. Push nhánh lên GitHub và tạo **Pull Request (PR)** vào nhánh `main` của repository gốc kèm mô tả chi tiết theo mẫu `pull_request_template.md`.

---

## 3. Báo Cáo Lỗi & Đề Xuất (Issues)

- Vui lòng sử dụng các mẫu có sẵn trong `.github/ISSUE_TEMPLATE/`:
  - **Bug report:** Báo cáo lỗi chi tiết kèm log UART, phần cứng sử dụng và các bước tái hiện.
  - **Feature request:** Đề xuất tính năng hoặc cải tiến thuật toán mới.

---

## 4. Tác Giả, Kế Thừa & Bản Quyền

- **Author:** Trần Nguyên Bình ([trannguyenbinh.shark@gmail.com](mailto:trannguyenbinh.shark@gmail.com))
- **Website:** [https://nguyenbinh-shark.github.io/](https://nguyenbinh-shark.github.io/)
- **Tham khảo & Kế thừa:** Dự án được phát triển dựa trên việc nghiên cứu và nâng cấp từ repository mã nguồn mở [dmBots/wheel-legged](https://github.com/dmBots/wheel-legged) của Damiao (dmBots).
- **License:** [MIT License](LICENSE)
