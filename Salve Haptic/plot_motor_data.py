import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import sys

# ================= CẤU HÌNH CỔNG COM =================
COM_PORT = 'COM9' 
BAUD_RATE = 115200 
# =====================================================

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"Đã kết nối thành công tới {COM_PORT}")
except Exception as e:
    print(f"Không thể mở cổng {COM_PORT}. Vui lòng kiểm tra lại dây cắm hoặc tắt ứng dụng khác đang dùng cổng COM này!")
    print(f"Chi tiết lỗi: {e}")
    sys.exit()

# Cài đặt giao diện biểu đồ (3 đồ thị)
fig, ax = plt.subplots(3, 1, figsize=(10, 8))
fig.canvas.manager.set_window_title('Biểu Đồ PID & Dòng Điện STM32')

x_data = []
y_tgt = []
y_cur = []
y_err = []
y_ima = []
max_points = 200 # Số điểm hiển thị

def animate(i):
    global x_data, y_tgt, y_cur, y_err, y_ima
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        # Tách chuỗi có chứa ImA: "Tgt: 123 | Cur: 120 | Err: 3 | ImA: 45"
        match = re.search(r'Tgt:\s*(-?\d+)\s*\|\s*Cur:\s*(-?\d+)\s*\|\s*Err:\s*(-?\d+)\s*\|\s*ImA:\s*(-?\d+)', line)
        if match:
            tgt = int(match.group(1))
            cur = int(match.group(2))
            err = int(match.group(3))
            ima = int(match.group(4))
            
            x_data.append(len(x_data) if len(x_data) == 0 else x_data[-1] + 1)
            y_tgt.append(tgt)
            y_cur.append(cur)
            y_err.append(err)
            y_ima.append(ima)
            
            if len(x_data) > max_points:
                x_data.pop(0)
                y_tgt.pop(0)
                y_cur.pop(0)
                y_err.pop(0)
                y_ima.pop(0)
            
            # Đồ thị 1: Vị trí
            ax[0].clear()
            ax[0].plot(x_data, y_tgt, label='Target', color='#1f77b4', linestyle='--')
            ax[0].plot(x_data, y_cur, label='Current', color='#d62728')
            ax[0].set_title('Đáp ứng Vị trí PID [Xung]')
            ax[0].legend(loc='upper left')
            ax[0].grid(True, linestyle=':', alpha=0.6)
            
            # Đồ thị 2: Sai số
            ax[1].clear()
            ax[1].plot(x_data, y_err, label='Error', color='#9467bd')
            ax[1].set_title('Sai số (Error) [Xung]')
            ax[1].legend(loc='upper left')
            ax[1].grid(True, linestyle=':', alpha=0.6)
            ax[1].set_ylim(-300, 300)  # Cố định biên độ sai số (thay đổi giá trị này nếu muốn lớn/nhỏ hơn)

            # Đồ thị 3: Dòng điện (mA)
            ax[2].clear()
            ax[2].plot(x_data, y_ima, label='Current (mA)', color='#ff7f0e', linewidth=2)
            ax[2].set_title('Dòng điện Động cơ (mA) - Theo dõi va chạm')
            ax[2].legend(loc='upper left')
            ax[2].grid(True, linestyle=':', alpha=0.6)
            ax[2].set_ylim(-3000, 3000) # Cố định dòng điện ở mức +- 3.0 Ampe
            
    except Exception as e:
        pass

ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)

plt.tight_layout()
plt.show()
