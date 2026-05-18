# Real-time Bilateral Master-Slave Teleoperation System with Haptic Feedback

[![Embedded Platform](https://img.shields.io/badge/Platform-STM32-blue.svg?style=flat-square&logo=stmicroelectronics)](https://www.st.com/)
[![Language](https://img.shields.io/badge/Language-C-green.svg?style=flat-square)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-Bare--metal%20Register-red.svg?style=flat-square)](https://en.wikipedia.org/wiki/Bare_machine)
[![Academic Project](https://img.shields.io/badge/Course-Introduction%20to%20Embedded%20Systems-orange.svg?style=flat-square)](#)

Một hệ thống điều khiển từ xa song phương (Bilateral Teleoperation) 1 Bậc tự do (1-DOF) nâng cao, có khả năng đồng bộ hóa động học và tái tạo chính xác cảm giác chạm vật lý thời gian thực thông qua công nghệ **Phản hồi lực (Haptic Feedback)**.

Toàn bộ hệ thống phần mềm (cho cả hai nút mạng Master và Slave) được xây dựng hoàn toàn bằng phương pháp **lập trình nhúng cấp độ thanh ghi (Bare-metal Register Programming)** trên nền tảng vi xử lý ARM Cortex-M4 (STM32F401RCT6), loại bỏ hoàn toàn các lớp thư viện trừu tượng (như STM32 HAL hoặc SPL) để đạt tới độ trễ tiệm cận không.

---

## 📌 Tính năng cốt lõi & Điểm nhấn Kỹ thuật

### 1. Kiến trúc Lập trình Thanh ghi (Bare-metal) Tất định
* **Triệt tiêu Software Overhead:** Toàn bộ cấu hình cấu trúc ngoại vi được can thiệp trực tiếp thông qua bản đồ bộ nhớ (Memory Map) nhằm tối ưu số chu kỳ CPU cần thiết cho vòng lặp điều khiển khắt khe.
* **Thời gian thực cứng (Hard Real-time):** Khóa cứng chu kỳ lấy mẫu điều khiển vị trí góc tại tần số **500Hz ($T_s = 2ms$)** thông qua ngắt định kỳ phần cứng (`TIM4`), đảm bảo tính tất định cao nhất cho hệ thống haptic vốn cực kỳ nhạy cảm với jitter và trễ vòng lặp.

### 2. Kiến trúc Điều khiển Vị trí - Lực (Position-Force Architecture)
* **Master Node (Haptic Interface):** Hoạt động như một Thiết bị Trở kháng Chủ động (Active Impedance Device). Thiết bị sử dụng bộ đếm trực giao phần cứng (`TIM2` 32-bit) để số hóa quỹ đạo chuyển động tay người vận hành mà không tốn tài nguyên xử lý của CPU. Khi nhận tín hiệu lực cản, thuật toán sẽ băm xung PWM điều khiển mạch công suất để cấp dòng ngược ghì mô-men động cơ Master, tạo ra cảm giác "va chạm" chân thực.
* **Slave Node (Remote Manipulator):** Thực thi vòng kín điều khiển vị trí bám quỹ đạo Master sử dụng thuật toán PID rời rạc nâng cao. Đồng thời, Slave liên tục thực hiện ước lượng mô-men cản gián tiếp qua dòng điện phần ứng thông qua cảm biến hiệu ứng Hall (`ACS712`) và khối chuyển đổi tương tự-số tốc độ cao (2.4 MSPS Polling ADC), loại bỏ sự cần thiết của cảm biến lực đa trục đắt tiền.

### 3. Thuật toán Xử lý Tín hiệu & Điều khiển Bậc cao
* **Hardware-Constrained PID:** Tích hợp bộ lọc số thông thấp hàm mũ (EMA) với hệ số $\beta = 0.6$ đặt trên khâu vi phân nhằm triệt tiêu lỗi lượng tử hóa xung Encoder ($\pm 1$ xung dither).
* **Cơ chế chống bão hòa tích phân (Anti-Windup):** Chiến lược kiểm soát kẹp mềm (Soft-clamping) chỉ kích hoạt khâu $K_i$ trong vùng tuyến tính hẹp ($|e| < 30$ xung) và chủ động xả bộ nhớ tích phân ngay khi sai số đổi dấu ($e_t \cdot e_{t-1} < 0$), loại bỏ hoàn toàn hiện tượng vọt lố (Overshoot).
* **Bù ma sát tĩnh (Coulomb Friction Feedforward):** Tự động bù một mức điện áp xung nền cố định (Baseline PWM = 80) vào ngõ ra điều khiển để triệt tiêu vùng chết cơ học (Mechanical Deadzone) sinh ra bởi hộp số giảm tốc kim loại tỷ số truyền 1/30.
* **Xử lý số thực FPU phần cứng:** Kích hoạt toàn quyền đồng xử lý toán học chuyên biệt (Coprocessor CP10 và CP11) cho phép các phép tính PID số, bộ lọc EMA nhiễu từ trường, và ánh xạ lực va chạm phi tuyến (Haptic Force Mapping) hoàn tất chỉ trong 1-3 chu kỳ máy.
* **Thuật toán Delta chống tràn Encoder:** Tận dụng biểu diễn số bù hai (Two's Complement) của máy tính kết hợp ép kiểu dữ liệu có dấu để tự động sửa lỗi nhảy số khi Encoder đếm tràn ngược (Underflow/Overflow) qua tọa độ gốc.

### 4. Truyền thông Không nghẽn & Cơ chế Phòng vệ Phần cứng
* **Asynchronous USART (Full-duplex):** Đồng bộ hóa liên tục hai chiều ở tốc độ cao 115200 bps thông qua ngắt thu dữ liệu (`RXNE`). Master đẩy tọa độ mục tiêu mỗi 10ms; Slave truyền trả lực phản hồi ước lượng mỗi 20ms dưới dạng khung truyền ASCII tối giản.
* **Bẫy lỗi phần cứng USART:** Tự động phát hiện và thực thi chuỗi xóa cờ lỗi phần cứng bắt buộc (`Overrun Error - ORE`, `Noise Error - NE`, `Framing Error - FE`) theo đúng tài liệu Reference Manual của nhà sản xuất, ngăn chặn trạng thái deadlock của bộ thu.
* **I2C Non-blocking với Timeout & Auto-Recovery:** Toàn bộ tiến trình truyền thông màn hình OLED SSD1306 cấp thanh ghi được trang bị bộ đếm lùi thời gian chờ (Watchdog phần mềm). Nếu xảy ra nhiễu hoặc ngắt kết nối bus vật lý, vi điều khiển tự động thoát vòng lặp chờ bận, reset khối ngoại vi và trả quyền thực thi cho CPU, bảo vệ tuyệt đối chu kỳ hoạt động của bộ điều khiển PID.
* **Thao tác GPIO nguyên tử (Atomic Bit-Banging):** Sử dụng thanh ghi thiết lập/xóa bit (`BSRR`) phần cứng để cấu hình chiều quay động cơ, triệt tiêu hoàn toàn rủi ro cạnh tranh dữ liệu (Race Condition) khi luồng mã bị phân mảnh do ngắt xen ngang.

### 5. Điều chế Cảnh báo Thính giác (Buzzer Frequency Modulation)
* Lập trình điều chế tần số bíp của còi báo động Non-blocking theo thời gian thực tỷ lệ nghịch với cường độ lực cản tải. Lực cản càng lớn, còi bíp càng dồn dập, cảnh báo người vận hành nhả tay điều khiển trước khi xảy ra hiện tượng quá tải dòng (Stall Current) gây hư hỏng hệ cơ khí.

---

## 🛠️ Kiến trúc Hệ thống & Sơ đồ Khối

### 📱 Sơ đồ Kết nối Phần cứng

```text
[ Khối Nguồn Tổ Ong 12V ] ──> Mạch giảm áp Buck LM2596 ──> Nguồn Logic 5V/3.3V
│
┌────────────────────────────────────────────────────────┐     │
│             CỤM MASTER (Haptic Interface)              │     │
│ 🛠️ MCU: STM32F401RCT6 (Cortex-M4, 84MHz)               │<────┤
│ 🔹 Mạch Công Suất: Cầu H BTS7960 (43A Max)             │     │
│ 🔹 Cơ Cấu Chấp Hành: Động cơ DC JGB37-545 + Hall Encoder│    │
└───────────────────────────┬────────────────────────────┘     │
│                           │Chuẩn USART (115200 bps)          │
[Khoảng cách từ xa]         │                                  │
│                           │                                  │
┌───────────────────────────▼────────────────────────────┐     │
│             CỤM SLAVE (Remote Manipulator)                   │
│ 🛠️ MCU: STM32F401RCT6 (Cortex-M4, 84MHz)                │ <─┘
│ 🔹 Mạch Công Suất: Cầu H BTS7960 (43A Max)              │
│ 🔹 Cơ Cấu Chấp Hành: Động cơ DC JGB37-545 + Hall Encoder│
│ 🔹 Cảm Biến Dòng Điện: Module ACS712 (Hiệu ứng Hall 5A) │
│ 🔹 Giao Diện HMI: Màn Hình OLED SSD1306 (I2C 100kHz)    │
│ 🔹 Thiết Bị Cảnh Báo: Active Buzzer (Điều chế tần số)   │
└────────────────────────────────────────────────────────┘
```

---

## 📂 Sơ đồ Thư mục Dự án

```text
.
├── Core
│   ├── Inc
│   │   ├── stm32f4xx.h         # Thanh ghi định nghĩa bản đồ bộ nhớ MCU
│   │   ├── pid.h               # Định nghĩa cấu trúc & tham số PID số rời rạc
│   │   ├── oled_ssd1306.h      # Cấu hình I2C và tệp tiêu đề điều khiển màn hình
│   │   └── ascii_font.h        # Định nghĩa ma trận bộ mã Font 5x8 tĩnh trong Flash
│   └── Src
│       ├── main.c              # Vòng lặp nền & Khởi tạo toàn cục hệ thống
│       ├── stm32f4xx_it.c      # Trình phục vụ ngắt (ISR TIM4 500Hz & USART2)
│       ├── pid.c               # Thuật toán Soft-clamping Anti-windup, EMA Filter
│       └── oled_ssd1306.c      # Trình điều khiển OLED SSD1306 cấp thanh ghi với Timeout
├── Debug/                      # [Bị bỏ qua bởi .gitignore] Thư mục Build local
├── .gitignore                  # Cấu hình bỏ qua các tệp rác khi biên dịch
└── README.md                   # Tài liệu hướng dẫn dự án
```

---

## 🚀 Hướng dẫn Cài đặt & Triển khai

### Điều kiện tiên quyết
* Máy tính cài đặt môi trường phát triển (như **STM32CubeIDE**, **Keil MDK ARM**, hoặc chuỗi công cụ **GNU Arm Embedded Toolchain** biên dịch bằng `Makefile`).
* Mạch nạp **ST-Link V2** hoặc tương đương.
* Hai board phát triển vi điều khiển **WeAct STM32F401RCT6**.

### Các bước nạp code và vận hành
1. **Clone repository về máy local:**
   ```bash
   git clone https://github.com/bangrbt/HAPTIC-FEEDBACK-STM32.git
   cd HAPTIC-FEEDBACK-STM32
   ```
2. Mở dự án bằng IDE của bạn.
3. Cấu hình hằng số định danh trong mã nguồn tùy thuộc vào mạch bạn muốn nạp:
   * Định nghĩa `#define NODE_MASTER` để biên dịch mã nguồn cho cụm điều khiển Master.
   * Định nghĩa `#define NODE_SLAVE` để biên dịch mã nguồn cho cụm chấp hành Slave.
4. Biên dịch hệ thống (Build Project) và tiến hành nạp xuống board mạch thông qua ST-Link.
5. Thực hiện kết nối phần cứng chéo các chân TX/RX giữa hai board MCU để thiết lập kênh truyền thông USART, cấp nguồn 12V tổng và bắt đầu thử nghiệm thực nghiệm.

---

## 📈 Kết quả Thực nghiệm & Tiềm năng Ứng dụng

* **Vận hành ổn định:** Hệ thống đã vận hành thực tế ổn định, đạt độ chính xác bám sai số xác lập cực nhỏ ($\pm 20$ xung trên hành trình dịch chuyển dài hơn 10000 xung).
* **Trải nghiệm chân thực:** Trải nghiệm phản hồi lực (Haptic Rendering) thể hiện rõ nét độ trong suốt (Transparency) khi mô phỏng môi trường "bức tường ảo" với đặc tính không gian pha Lực - Vị trí dốc thẳng đứng cực gắt ($\frac{\Delta F}{\Delta x}$).

Hệ thống lõi này đặt nền móng kỹ thuật vững chắc để mở rộng ứng dụng sang các lĩnh vực mũi nhọn đòi hỏi khắt khe về thời gian thực cứng như:
* **Y tế:** Robot phẫu thuật xâm lấn tối thiểu (Surgical Robotics), tái tạo cảm giác sờ và phân biệt mật độ mô cơ của bệnh nhân cho bác sĩ.
* **Thao tác từ xa môi trường khắc nghiệt:** Robot rà phá bom mìn, bảo trì lõi lò phản ứng hạt nhân hoặc cánh tay không gian trạm vũ trụ.
* **VR/AR Training Simulators:** Hệ thống mô phỏng đào tạo y khoa, giả lập vô-lăng phản hồi lực (Drive-by-wire) cho huấn luyện bay/lái xe.
* **Công nghiệp 4.0:** Hệ thống khuếch đại lực và thu nhỏ vị trí phục vụ dây chuyền lắp ráp vi cơ điện tử, vi mô bán dẫn.

---

## 👥 Tác giả Đồ án

**Thành viên thực hiện:**
* Lê Anh Tuấn Bằng (Lead)
* Đỗ Việt Anh 
* Kiều Minh Dũng 

**Hà Nội, Năm 2026.**
