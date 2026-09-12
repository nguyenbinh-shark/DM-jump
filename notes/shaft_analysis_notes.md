# Ghi Chú Ôn Tập: Phân Tích Trục Chịu Mỏi & Bộ Giảm Tốc Bánh Răng

---

## BÀI 1: TRỤC GẮN BÁNH ĐÀ (FLYWHEEL)

### 📋 Dữ kiện

| Thông số | Giá trị |
|---|---|
| AB | 190 mm |
| BC | 190 mm (bánh đà treo ngoài) |
| Tốc độ quay | n = 115 rpm |
| Trọng lượng bánh đà | W = 5800 N (hướng xuống tại C) |
| Giới hạn mỏi | σ₋₁ = 110 MPa |
| Hằng số vật liệu | m = 9 |
| Số chu kỳ tới hạn | N₀ = 4 × 10⁶ |

**Sơ đồ kết cấu:**

```
Ổ đỡ A        Ổ đỡ B        Bánh đà C
   ▲              ▲              ↓ W = 5800 N
   |----190 mm----|----190 mm----|
```

- Trục được đỡ tại A và B (gối tựa đơn giản)
- Bánh đà treo ngoài tại C (overhung)

---

### ❓ Câu 1: Nhận định đúng về ứng suất dao động

> **Đáp án: Cả tải tĩnh và tải dao động đều có thể sinh ra ứng suất dao động**

**Giải thích:**

Trọng lượng bánh đà W = 5800 N là **tải tĩnh** (không thay đổi theo thời gian). Tuy nhiên, khi trục **quay**, một điểm bất kỳ trên bề mặt trục sẽ:

- Lúc ở phía trên → chịu **nén**
- Lúc ở phía dưới → chịu **kéo**

→ Trong mỗi vòng quay, ứng suất uốn tại mỗi điểm trên bề mặt trục **đảo chiều hoàn toàn** (fully reversed, R = -1).

**Kết luận quan trọng:** Tải tĩnh + trục quay = ứng suất dao động đảo chiều hoàn toàn.

---

### ❓ Câu 2: Đường kính trục cho tuổi thọ vô hạn → **46,73 mm**

**Bước 1: Tìm phản lực tại các gối đỡ**

Lấy mô men tại A:

$$R_B \times 190 = W \times (190 + 190) = 5800 \times 380$$

$$R_B = \frac{5800 \times 380}{190} = 11600 \text{ N (hướng lên)}$$

Cân bằng lực:

$$R_A = R_B - W = 11600 - 5800 = 5800 \text{ N (hướng xuống)}$$

> Lưu ý: R_A hướng xuống vì bánh đà treo ngoài tạo lực nâng ngược tại A.

**Bước 2: Tìm mô men uốn lớn nhất**

Mô men uốn lớn nhất xảy ra tại **B** (điểm gối đỡ gần bánh đà nhất):

$$M_B = W \times BC = 5800 \times 190 = 1{,}102{,}000 \text{ Nmm}$$

**Bước 3: Điều kiện tuổi thọ vô hạn**

Ứng suất uốn đảo chiều hoàn toàn (σ_m = 0, σ_a = σ_max):

$$\sigma_a = \frac{32M}{\pi d^3}$$

Điều kiện: σ_a ≤ σ₋₁

$$\frac{32 \times 1{,}102{,}000}{\pi d^3} \leq 110$$

$$d^3 \geq \frac{32 \times 1{,}102{,}000}{110 \times \pi} = \frac{35{,}264{,}000}{345.575} = 102{,}046 \text{ mm}^3$$

$$\boxed{d \geq \sqrt[3]{102{,}046} = 46.73 \text{ mm}}$$

---

### ❓ Câu 3: Tuổi thọ với d = 42 mm → **32,5 giờ**

**Bước 1: Tính ứng suất thực tế**

$$\sigma_a = \frac{32 \times 1{,}102{,}000}{\pi \times 42^3} = \frac{35{,}264{,}000}{\pi \times 74{,}088} = \frac{35{,}264{,}000}{232{,}795} = 151.48 \text{ MPa}$$

Vì σ_a = 151.48 > σ₋₁ = 110 MPa → **tuổi thọ hữu hạn**

**Bước 2: Tính số chu kỳ đến hỏng (đường cong S-N)**

Công thức đường cong S-N:

$$N = N_0 \left(\frac{\sigma_{-1}}{\sigma_a}\right)^m$$

$$N = 4 \times 10^6 \times \left(\frac{110}{151.48}\right)^9$$

Tính từng bước:

$$\frac{110}{151.48} = 0.7262$$

$$0.7262^2 = 0.52737$$
$$0.7262^4 = 0.52737^2 = 0.27812$$
$$0.7262^8 = 0.27812^2 = 0.07735$$
$$0.7262^9 = 0.07735 \times 0.7262 = 0.05618$$

$$N = 4 \times 10^6 \times 0.05618 = 224{,}710 \text{ chu kỳ}$$

**Bước 3: Chuyển đổi sang giờ**

Mỗi chu kỳ = 1 vòng quay (ứng suất đảo chiều 1 lần/vòng)

$$t = \frac{N}{n \times 60} = \frac{224{,}710}{115 \times 60} = \frac{224{,}710}{6{,}900} = 32.57 \text{ giờ}$$

$$\boxed{t \approx 32.5 \text{ giờ}}$$

---

### ❓ Câu 4: Trọng lượng tối đa cho tuổi thọ vô hạn (d = 42 mm) → **4211 N**

Điều kiện: σ_a = σ₋₁

$$\frac{32 \times W \times 190}{\pi \times 42^3} = 110$$

$$W = \frac{110 \times \pi \times 42^3}{32 \times 190}$$

$$W = \frac{110 \times \pi \times 74{,}088}{6{,}080}$$

$$W = \frac{110 \times 232{,}795}{6{,}080}$$

$$W = \frac{25{,}607{,}467}{6{,}080}$$

$$\boxed{W = 4{,}212 \approx 4{,}211 \text{ N}}$$

---
---

## BÀI 2: TRỤC TRUNG GIAN BỘ GIẢM TỐC BÁNH RĂNG (COUNTERSHAFT)

### 📋 Dữ kiện

| Thông số | Giá trị |
|---|---|
| Lực truyền tại bánh răng A | F_A = 250 N |
| Góc áp lực α = β | 25° |
| Đường kính bánh răng A | 180 mm → r_A = 90 mm |
| Đường kính bánh răng B | 70 mm → r_B = 35 mm |
| OA | 70 mm |
| AC | 90 mm |
| CB | 30 mm |
| Vật liệu | Thép CT3: S_y = 300 MPa, S_u = 450 MPa |

**Sơ đồ kết cấu:**

```
Ổ đỡ O    Bánh răng A    Bánh răng B (tại C)    Ổ đỡ B
   ▲           ⚙              ⚙                    ▲
   |---70mm----|-----90mm------|------30mm----------|
   0          70              160                  190
```

- Ổ đỡ (bearings) tại **O** và **B** (gối tựa đơn giản)
- Bánh răng A tại vị trí A (x = 70 mm)
- Bánh răng B tại vị trí C (x = 160 mm)

---

### ❓ Câu 1: Xác định lực FB → **643 N**

**Nguyên lý:** Trục quay đều → mô men xoắn cân bằng

Lực tiếp tuyến tại A tạo mô men xoắn:

$$T = F_A^t \times r_A$$

Với F_A = 250 N là lực tổng hợp tại góc áp lực 25°:

$$F_A^t = F_A \cos(25°) = 250 \times 0.9063 = 226.6 \text{ N}$$

$$T = 226.6 \times 90 = 20{,}394 \text{ Nmm}$$

Lực tiếp tuyến tại B:

$$F_B^t = \frac{T}{r_B} = \frac{20{,}394}{35} = 582.7 \text{ N}$$

Lực tổng hợp tại B (lực truyền):

$$F_B = \frac{F_B^t}{\cos(25°)} = \frac{582.7}{0.9063}$$

$$\boxed{F_B = 643 \text{ N}}$$

> **Lưu ý:** Cũng có thể tính nhanh: F_B = F_A × r_A / r_B = 250 × 90/35 = 643 N
> (vì cùng góc áp lực, tỉ số lực tổng = tỉ số lực tiếp tuyến)

---

### ❓ Câu 2: Mô men uốn tại A → **17185 Nmm**

### ❓ Câu 3: Mô men uốn tại C → **19286 Nmm**

**Phương pháp:** Phân tích lực trong 2 mặt phẳng vuông góc

Mỗi bánh răng tác dụng lên trục 2 thành phần lực:
- **Lực tiếp tuyến** (tangential) → gây uốn trong mặt phẳng ngang
- **Lực hướng tâm** (radial) → gây uốn trong mặt phẳng đứng

**Thành phần lực:**

| | Bánh răng A (tại x=70) | Bánh răng B (tại x=160) |
|---|---|---|
| Tiếp tuyến F^t | 250 cos25° = 226.6 N | 582.7 N |
| Hướng tâm F^r | 250 sin25° = 105.65 N | 582.7 × tan25° = 271.7 N |

**Mặt phẳng tiếp tuyến (tangential plane):**

Phản lực tại B:

$$R_B^t = \frac{226.6 \times 70 + 582.7 \times 160}{190} = \frac{15{,}862 + 93{,}232}{190} = 574.2 \text{ N}$$

Phản lực tại O:

$$R_O^t = 226.6 + 582.7 - 574.2 = 235.1 \text{ N}$$

Mô men uốn:
- Tại A: $M_A^t = R_O^t \times 70 = 235.1 \times 70 = 16{,}457$ Nmm
- Tại C: $M_C^t = R_B^t \times 30 = 574.2 \times 30 = 17{,}226$ Nmm

**Mặt phẳng hướng tâm (radial plane):**

Phản lực (lực hướng tâm ngược chiều nhau do bố trí bánh răng):

$$R_B^r = \frac{105.65 \times 70 - 271.7 \times 160}{190} = \frac{7{,}396 - 43{,}472}{190} = -189.9 \text{ N}$$

$$R_O^r = 105.65 - 271.7 + 189.9 = 23.85 \text{ N}$$

Mô men uốn:
- Tại A: $M_A^r = |R_O^r| \times 70 = 23.85 \times 70 = 1{,}670$ Nmm
- Tại C: $M_C^r = |R_B^r| \times 30 = 189.9 \times 30 = 5{,}697$ Nmm

**Mô men uốn tổng hợp (resultant):**

$$M_A = \sqrt{(M_A^t)^2 + (M_A^r)^2} = \sqrt{16{,}457^2 + 1{,}670^2}$$

$$\boxed{M_A \approx 17{,}185 \text{ Nmm}}$$

$$M_C = \sqrt{(M_C^t)^2 + (M_C^r)^2} = \sqrt{17{,}226^2 + 5{,}697^2}$$

$$\boxed{M_C \approx 19{,}286 \text{ Nmm}}$$

---

### ❓ Câu 4: Vị trí mô men uốn lớn nhất → **C**

So sánh: M_C = 19,286 > M_A = 17,185

$$\boxed{\text{Mô men uốn lớn nhất tại điểm C}}$$

---

### ❓ Câu 5: Hệ số an toàn với d = 14 mm (Thuyết năng lượng biến dạng) → **3,1**

**Bước 1: Tính ứng suất uốn tại tiết diện nguy hiểm (tại C)**

$$\sigma = \frac{32 M_{max}}{\pi d^3} = \frac{32 \times 19{,}286}{\pi \times 14^3}$$

$$\sigma = \frac{617{,}152}{8{,}620.5} = 71.59 \text{ MPa}$$

**Bước 2: Tính ứng suất xoắn**

$$\tau = \frac{16T}{\pi d^3} = \frac{16 \times 20{,}394}{\pi \times 14^3}$$

$$\tau = \frac{326{,}304}{8{,}620.5} = 37.85 \text{ MPa}$$

**Bước 3: Ứng suất tương đương Von Mises (Distortion Energy)**

$$\sigma' = \sqrt{\sigma^2 + 3\tau^2}$$

$$\sigma' = \sqrt{71.59^2 + 3 \times 37.85^2}$$

$$\sigma' = \sqrt{5{,}125.1 + 4{,}297.8} = \sqrt{9{,}422.9}$$

$$\sigma' = 97.07 \text{ MPa}$$

**Bước 4: Hệ số an toàn**

$$n = \frac{S_y}{\sigma'} = \frac{300}{97.07}$$

$$\boxed{n = 3.09 \approx 3.1}$$

---

### ❓ Câu 6: Đường kính trục với n = 2,3 (Thuyết ứng suất cắt lớn nhất) → **13,0 mm**

**Công thức Maximum Shear Stress Theory (Tresca):**

$$\tau_{max} = \frac{S_y}{2n}$$

Trong đó:

$$\tau_{max} = \sqrt{\left(\frac{\sigma}{2}\right)^2 + \tau^2} = \frac{16}{\pi d^3}\sqrt{M^2 + T^2}$$

**Thiết lập phương trình:**

$$\frac{16}{\pi d^3}\sqrt{M^2 + T^2} = \frac{S_y}{2n}$$

$$d^3 = \frac{32n}{\pi S_y}\sqrt{M^2 + T^2}$$

**Tính giá trị:**

$$\sqrt{M^2 + T^2} = \sqrt{19{,}286^2 + 20{,}394^2}$$

$$= \sqrt{372{,}151{,}396 + 415{,}917{,}636}$$

$$= \sqrt{788{,}069{,}032} = 28{,}072.6 \text{ Nmm}$$

$$d^3 = \frac{32 \times 2.3}{\pi \times 300} \times 28{,}072.6$$

$$d^3 = \frac{73.6}{942.48} \times 28{,}072.6 = 0.07808 \times 28{,}072.6$$

$$d^3 = 2{,}191.4 \text{ mm}^3$$

$$\boxed{d = \sqrt[3]{2{,}191.4} = 12.98 \approx 13.0 \text{ mm}}$$

---

## 📌 TÓM TẮT CÔNG THỨC QUAN TRỌNG

### Ứng suất trên trục tròn

| Loại | Công thức |
|---|---|
| Ứng suất uốn | σ = 32M / (πd³) |
| Ứng suất xoắn | τ = 16T / (πd³) |

### Các thuyết bền

| Thuyết bền | Ứng suất tương đương | Điều kiện |
|---|---|---|
| **Distortion Energy (Von Mises)** | σ' = √(σ² + 3τ²) | n = Sy / σ' |
| **Max Shear Stress (Tresca)** | τ_max = √((σ/2)² + τ²) | n = Sy / (2τ_max) |

### Đường cong S-N (tuổi thọ mỏi)

$$N = N_0 \left(\frac{\sigma_{-1}}{\sigma_a}\right)^m$$

- Nếu σ_a ≤ σ₋₁ → tuổi thọ **vô hạn**
- Nếu σ_a > σ₋₁ → tuổi thọ **hữu hạn**, tính N theo công thức trên

### Lực bánh răng

| Thành phần | Công thức |
|---|---|
| Lực tiếp tuyến | Ft = F cos(φ) |
| Lực hướng tâm | Fr = F sin(φ) = Ft × tan(φ) |
| Lực tổng hợp | F = Ft / cos(φ) = √(Ft² + Fr²) |
| Cân bằng mô men xoắn | Ft_A × r_A = Ft_B × r_B |

---

## 📊 BẢNG ĐÁP ÁN TỔNG HỢP

### Bài 1: Trục bánh đà

| Câu | Đáp án |
|---|---|
| 1 | Cả tải tĩnh và tải dao động đều có thể sinh ra ứng suất dao động |
| 2 | d = **46,73 mm** |
| 3 | t = **32,5 giờ** |
| 4 | W_max = **4211 N** |

### Bài 2: Trục trung gian

| Câu | Đáp án |
|---|---|
| 1 | F_B = **643 N** |
| 2 | M_A = **17185 Nmm** |
| 3 | M_C = **19286 Nmm** |
| 4 | Vị trí M_max: **C** |
| 5 | Hệ số an toàn (DE): **3,1** |
| 6 | Đường kính (MSST): **13,0 mm** |
