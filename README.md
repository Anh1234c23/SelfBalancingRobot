# SelfBalancingRobot

**Self-Balancing Two-Wheeled Robot Controlled via Bluetooth**  

### Introduction  
This project is a self-balancing two-wheeled robot built using the STM32F103C8T6 microcontroller. The robot utilizes an MPU6050 sensor to measure tilt angles and maintain balance, combined with an HC-06 module for Bluetooth control via the "Balance_car_Keyes" mobile app. It can execute movement commands such as forward, backward, left turn, and right turn while maintaining balance through signal processing and control algorithms.  

### Theoretical Basis  
The self-balancing two-wheeled robot operates based on the principle of the **Inverted Pendulum**, a naturally unstable system that requires continuous control to maintain an upright position. The main theoretical components include:  

#### **MPU6050 Sensor**  
- Provides accelerometer and gyroscope data on three axes (X, Y, Z).  
- The accelerometer measures static tilt angles, while the gyroscope detects angular velocity, improving accuracy in motion.  

#### **Kalman Filter**  
- A noise-filtering algorithm that combines accelerometer and gyroscope data to estimate tilt angles more accurately, minimizing noise and drift effects.  

#### **PID Controller**  
- A **Proportional-Integral-Derivative (PID) controller** adjusts motor speed based on the error between the current and target angle (typically **0°** to maintain upright balance).  

#### **Bluetooth Control**  
- The **HC-06 module** receives commands from the "Balance_car_Keyes" app via UART, enabling movement control while ensuring balance.  

### **Key Features**  
✅ **Self-balancing** using Kalman Filter and PID algorithms.  
✅ **Bluetooth-controlled** with commands: Forward (F), Backward (B), Left (L), Right (R), Stop (S).  
✅ **PWM control** via TIM2 for motor speed regulation.  
✅ **Sensor updates** at **500 Hz** using TIM3.  

### **Hardware Requirements**  
- **Microcontroller**: STM32F103C8T6  
- **Sensor**: MPU6050 (I2C1 connection)  
- **Bluetooth Module**: HC-06 (UART1 – PA9, PA10)  
- **Motors**: Two DC motors with driver (e.g., L298N)  
- **Power Supply**: Battery or 5V/3.3V power source  

#### **Motor Driver Wiring**  
| Component | STM32 Pin | Device Pin | Description |  
|-----------|----------|------------|-------------|  
| **Motor A (Left)** | PA4 | IN1 | Motor direction control |  
|  | PA5 | IN2 | Motor direction control |  
|  | PA0 (TIM2_CH1) | ENA | PWM speed control |  
| **Motor B (Right)** | PA6 | IN3 | Motor direction control |  
|  | PA7 | IN4 | Motor direction control |  
|  | PA1 (TIM2_CH2) | ENB | PWM speed control |  
| **Power** | Battery | VCC | Motor driver power |  
|  | GND | GND | Ground connection |  

### **Software Requirements**  
- **IDE**: STM32CubeIDE or Keil uVision  
- **Libraries**: STM32 HAL Library  
- **Control App**: "Balance_car_Keyes" (Available on Google Play Store / App Store)  
- **Programming Tools**: ST-Link or UART Bootloader  

### **Setup & Configuration**  

#### **1. Hardware Connection**  
- Follow the wiring table above.  
- Ensure proper power supply (3.3V for MPU6050, 5V/3.3V for HC-06, sufficient power for motors).  

#### **2. Software Setup**  
- Open the project in STM32CubeIDE.  
- Copy the main.c source code into the project.  
- Configure STM32CubeMX settings:  
  - **I2C1**: Fast Mode (400 kHz)  
  - **TIM2**: PWM CH1 & CH2 (Period: 450, Prescaler: 0)  
  - **TIM3**: Internal Clock, Prescaler: 4, Period: 15999  
  - **UART1**: Baud Rate 9600, 8N1  
  - **GPIO**: PA4, PA5, PA6, PA7 set as Output  
- Compile and flash the code into STM32 using **ST-Link**.  

#### **3. Bluetooth Connection**  
- Enable Bluetooth on your phone and pair with **HC-06** (default PIN: "1234" or "0000").  
- Open the "Balance_car_Keyes" app and connect to HC-06.  

### **How to Use**  
1. **Power on the robot.**  
2. **Wait for MPU6050 calibration** (takes a few seconds).  
3. **Use the "Balance_car_Keyes" app to control the robot:**  
   - **Forward**: Press "Forward" (sends 'F').  
   - **Backward**: Press "Backward" (sends 'B').  
   - **Left**: Press "Left" (sends 'L').  
   - **Right**: Press "Right" (sends 'R').  
   - **Stop**: Release buttons (sends 'S').  
4. The robot will automatically balance while executing movement commands.  

### **Fine-Tuning**  
- **PID Parameters** (adjust in `PID_t pid`):  
  - `Kp = 10.0`: Quick response to error.  
  - `Ki = 50.0`: Reduces accumulated error.  
  - `Kd = 1.0`: Improves dynamic response.  
- **Movement speed**: Modify `base_pwm` (default **500**) in `main()`.  
- **Dead zone**: Adjust `DEAD_ZONE` (default **-1.5**) to refine balance thresholds.  

### **Code Structure**  
- **main.c**: Contains the main source code:  
  - Hardware initialization (I2C, TIM, UART, GPIO).  
  - Kalman Filter and PID algorithms.  
  - Bluetooth command processing and motor control.  

### **Notes**  
⚠ Ensure a **stable power supply** to avoid STM32 resets.  
⚠ Verify **HC-06 connection** before running (test via serial terminal if needed).  
⚠ If the robot does not balance well, check **MPU6050 calibration** or **PID settings**.  

### **Contributions**  
If you want to contribute to this project:  
1. **Fork this repository.**  
2. **Create a new branch and make changes.**  
3. **Submit a Pull Request with detailed explanations.**  

### **Author**  
- Created by: **[Nguyen The Anh]**  
- Date: **April 1, 2025**


Robot 2 Bánh Tự Cân Bằng Điều Khiển Qua Bluetooth
Giới thiệu
Dự án này là một robot 2 bánh tự cân bằng được xây dựng dựa trên vi điều khiển STM32F103C8T6. Robot sử dụng cảm biến MPU6050 để đo góc nghiêng và giữ thăng bằng, kết hợp với module HC-06 để điều khiển qua Bluetooth từ ứng dụng "Balance_car_Keyes" trên điện thoại. Robot có thể thực hiện các lệnh di chuyển như tiến, lùi, quay trái, quay phải, đồng thời duy trì trạng thái cân bằng nhờ các thuật toán xử lý tín hiệu và điều khiển.

Cơ sở lý thuyết
Robot 2 bánh tự cân bằng hoạt động dựa trên nguyên lý của bài toán con lắc ngược (Inverted Pendulum), một hệ thống không ổn định tự nhiên cần được điều khiển liên tục để duy trì trạng thái thẳng đứng. Các thành phần lý thuyết chính bao gồm:

Cảm biến MPU6050:
Cung cấp dữ liệu gia tốc (accelerometer) và vận tốc góc (gyroscope) trên 3 trục (X, Y, Z).
Gia tốc được dùng để tính góc nghiêng tĩnh, trong khi gyroscope đo tốc độ thay đổi góc, giúp cải thiện độ chính xác khi có chuyển động.
Kalman Filter:
Thuật toán lọc nhiễu kết hợp dữ liệu từ gia tốc kế và con quay hồi chuyển để ước lượng góc nghiêng chính xác hơn, giảm thiểu ảnh hưởng của nhiễu và drift (trôi số).
PID Controller:
Bộ điều khiển Tỷ lệ - Tích phân - Vi phân (Proportional-Integral-Derivative) điều chỉnh tốc độ động cơ dựa trên sai số giữa góc hiện tại và góc mục tiêu (thường là 0° để giữ thẳng đứng).

Điều khiển Bluetooth:
Module HC-06 nhận lệnh từ ứng dụng "Balance_car_Keyes" qua giao thức UART, cho phép điều khiển hướng di chuyển trong khi vẫn duy trì cân bằng.
Tính năng chính
Tự cân bằng bằng thuật toán Kalman Filter và PID.
Điều khiển qua Bluetooth với các lệnh: Tiến (F), Lùi (B), Trái (L), Phải (R), Dừng (S).
Sử dụng PWM từ TIM2 để điều khiển động cơ.
Cập nhật dữ liệu cảm biến với tần số 500 Hz qua TIM3.
Yêu cầu phần cứng
Vi điều khiển: STM32F103C8T6.
Cảm biến: MPU6050 (kết nối qua I2C1).
Module Bluetooth: HC-06 (kết nối qua UART1 - PA9, PA10).
Động cơ: 2 động cơ DC với driver (ví dụ: L298N).
Động cơ trái: IN1 (PA4), IN2 (PA5), PWM (TIM2_CH1).
Động cơ phải: IN3 (PA6), IN4 (PA7), PWM (TIM2_CH2).
Nguồn điện: Pin hoặc nguồn 5V/3.3V phù hợp.
Yêu cầu phần mềm
IDE: STM32CubeIDE hoặc Keil uVision.
Thư viện: STM32 HAL Library.
Ứng dụng điều khiển: "Balance_car_Keyes" (tải từ Google Play Store hoặc App Store).
Công cụ nạp code: ST-Link hoặc UART Bootloader.
Kết nối phần cứng
Dưới đây là sơ đồ kết nối chi tiết giữa các thành phần:

Thiết bị	Chân STM32	Chân thiết bị	Ghi chú
MPU6050			
- SCL	PB6	SCL	I2C1 Clock
- SDA	PB7	SDA	I2C1 Data
- VCC	3.3V	VCC	Nguồn 3.3V
- GND	GND	GND	Nối đất
HC-06			
- TX	PA10 (RX)	TXD	UART1 RX
- RX	PA9 (TX)	RXD	UART1 TX
- VCC	5V/3.3V	VCC	Tùy module
- GND	GND	GND	Nối đất
Driver động cơ			
- IN1 (Motor A)	PA4	IN1	Điều khiển chiều Motor A
- IN2 (Motor A)	PA5	IN2	Điều khiển chiều Motor A
- ENA (Motor A)	PA0 (TIM2_CH1)	ENA	PWM Motor A
- IN3 (Motor B)	PA6	IN3	Điều khiển chiều Motor B
- IN4 (Motor B)	PA7	IN4	Điều khiển chiều Motor B
- ENB (Motor B)	PA1 (TIM2_CH2)	ENB	PWM Motor B
- VCC	Pin/Nguồn	VCC	Nguồn cho driver
- GND	GND	GND	Nối đất

Cài đặt
1. Kết nối phần cứng
Thực hiện kết nối theo bảng trên.
Đảm bảo cấp nguồn phù hợp (3.3V cho MPU6050, 5V/3.3V cho HC-06, và nguồn đủ mạnh cho động cơ).
2. Cài đặt phần mềm
Mở dự án trong STM32CubeIDE.
Sao chép mã nguồn từ file main.c vào dự án.
Cấu hình STM32CubeMX (nếu cần):
I2C1: Fast Mode (400 kHz).
TIM2: PWM Generation CH1 & CH2 (Period: 450, Prescaler: 0).
TIM3: Internal Clock, Prescaler: 4, Period: 15999.
UART1: Baud Rate 9600, 8N1.
GPIO: PA4, PA5, PA6, PA7 là Output.
Biên dịch và nạp code vào STM32 bằng ST-Link.
3. Kết nối Bluetooth
Bật Bluetooth trên điện thoại, tìm và ghép nối với HC-06 (mật khẩu mặc định thường là "1234" hoặc "0000").
Mở ứng dụng "Balance_car_Keyes", chọn kết nối với HC-06.
Cách sử dụng
Cấp nguồn cho robot.
Chờ robot tự hiệu chỉnh MPU6050 (khoảng vài giây).
Sử dụng ứng dụng "Balance_car_Keyes" để điều khiển:
Tiến: Nhấn nút "Forward" (gửi 'F').
Lùi: Nhấn nút "Backward" (gửi 'B').
Trái: Nhấn nút "Left" (gửi 'L').
Phải: Nhấn nút "Right" (gửi 'R').
Dừng: Nhả các nút (gửi 'S').
Robot sẽ tự động giữ cân bằng trong khi thực hiện các lệnh di chuyển.
Tinh chỉnh
PID: Điều chỉnh các tham số trong PID_t pid:
Kp (10.0): Phản ứng nhanh với sai số.
Ki (50.0): Giảm tích lũy sai số.
Kd (1.0): Cải thiện đáp ứng động.
Tốc độ di chuyển: Thay đổi base_pwm (hiện là 500) trong main() để tăng/giảm tốc độ.
Vùng chết: Điều chỉnh DEAD_ZONE (hiện là -1.5) để thay đổi ngưỡng cân bằng.
Cấu trúc mã nguồn
main.c: Chứa toàn bộ mã nguồn chính.
Khởi tạo phần cứng (I2C, TIM, UART, GPIO).
Thuật toán Kalman Filter và PID.
Xử lý lệnh Bluetooth và điều khiển động cơ.
Lưu ý
Đảm bảo nguồn điện ổn định để tránh reset STM32.
Kiểm tra kết nối HC-06 trước khi chạy (dùng terminal để test nếu cần).
Nếu robot không cân bằng tốt, kiểm tra lại hiệu chỉnh MPU6050 hoặc điều chỉnh PID.
Đóng góp
Nếu bạn muốn đóng góp vào dự án, hãy:

Fork repository này.
Tạo nhánh mới và thực hiện thay đổi.
Gửi Pull Request với mô tả chi tiết.
Tác giả
Tạo bởi: [Nguyễn Thế Anh]
Ngày: 02/04/2025
