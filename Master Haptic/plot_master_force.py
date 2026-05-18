import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import sys

# ================= CẤU HÌNH CỔNG COM =================
COM_PORT = 'COM9' # THAY ĐỔI CỔNG COM NÀY THÀNH CỔNG CỦA MẠCH MASTER
BAUD_RATE = 115200 
# =====================================================

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"Đã kết nối thành công tới {COM_PORT}")
except Exception as e:
    print(f"Không thể mở cổng {COM_PORT}. Vui lòng kiểm tra lại dây cắm hoặc tắt ứng dụng khác đang dùng cổng COM này!")
    print(f"Chi tiết lỗi: {e}")
    sys.exit()

# Cài đặt giao diện biểu đồ (Grid Layout)
fig = plt.figure(figsize=(12, 6))
fig.canvas.manager.set_window_title('Đặc tính Haptic Feedback (Master)')

# Phân bổ khung: 2 hàng x 2 cột
# ax1 (Góc trên trái): Vị trí theo thời gian
# ax2 (Góc dưới trái): Lực phản hồi theo thời gian
# ax3 (Cột bên phải): Force vs Position (Phase Plot)
ax1 = fig.add_subplot(2, 2, 1) 
ax2 = fig.add_subplot(2, 2, 3)
ax3 = fig.add_subplot(1, 2, 2)

x_data = []
y_pos = []
y_force = []
max_points = 200 # Số điểm hiển thị tối đa trên đồ thị theo thời gian

def animate(i):
    global x_data, y_pos, y_force
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        # Đọc dữ liệu từ Master: "Master Pos:    123 | Force Rx:  -45"
        match = re.search(r'Master Pos:\s*(-?\d+)\s*\|\s*Force Rx:\s*(-?\d+)', line)
        if match:
            pos = int(match.group(1))
            force = int(match.group(2))
            
            x_data.append(len(x_data) if len(x_data) == 0 else x_data[-1] + 1)
            y_pos.append(pos)
            y_force.append(force)
            
            # Xóa các điểm cũ để đồ thị trôi đi
            if len(x_data) > max_points:
                x_data.pop(0)
                y_pos.pop(0)
                y_force.pop(0)
            
            # ================= ĐỒ THỊ 1: VỊ TRÍ THEO THỜI GIAN =================
            ax1.clear()
            ax1.plot(x_data, y_pos, color='#1f77b4', linewidth=2)
            ax1.set_title('Vị trí tay xoay (Master Pos) [Xung]', fontsize=10)
            ax1.grid(True, linestyle=':', alpha=0.6)
            
            # ================= ĐỒ THỊ 2: LỰC PHẢN HỒI THEO THỜI GIAN =================
            ax2.clear()
            ax2.plot(x_data, y_force, color='#ff7f0e', linewidth=2)
            ax2.set_title('Lực cảm nhận (Force Rx) [mA]', fontsize=10)
            ax2.grid(True, linestyle=':', alpha=0.6)
            ax2.set_ylim(-1000, 1000) # Cố định giới hạn lực
            
            # ================= ĐỒ THỊ 3: ĐẶC TÍNH HAPTIC (FORCE vs POSITION) =================
            ax3.clear()
            # Vẽ quỹ đạo đường đi
            ax3.plot(y_pos, y_force, color='#2ca02c', alpha=0.7, linewidth=1.5, label='Lịch sử')
            
            # Vẽ một điểm tròn đỏ to thể hiện vị trí "hiện tại" của tay cầm
            if len(y_pos) > 0:
                curr_p = y_pos[-1]
                ax3.scatter([curr_p], [y_force[-1]], color='red', s=80, zorder=5, label='Tay người dùng')
                
                # Phóng to (Zoom) và bám theo vị trí hiện tại của tay cầm với khoảng +/- 400
                ax3.set_xlim(curr_p - 400, curr_p + 400)
                
                # Chỉnh lưới (Grid) chi tiết hơn để dễ nhìn (mỗi ô là 100 đơn vị)
                ax3.xaxis.set_major_locator(plt.MultipleLocator(100))

            
            ax3.set_title('Đặc tính Haptic: Lực theo Vị Trí (Force vs Position)', fontsize=12, fontweight='bold')
            ax3.set_xlabel('Vị trí tay cầm [Xung]')
            ax3.set_ylabel('Lực cản [mA]')
            ax3.axhline(0, color='black', linewidth=1, linestyle='--') # Trục 0 của Lực
            ax3.grid(True, linestyle='--', alpha=0.6)
            ax3.legend(loc='upper right')
            
    except Exception as e:
        pass

ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False)

plt.tight_layout()
plt.show()
