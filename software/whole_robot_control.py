import socket
import keyboard
import time
import csv
import datetime
import threading
import matplotlib.pyplot as plt

# ===== WiFi config =====
ESP_IP = "192.168.0.101"
ESP_PORT = 80
UDP_CMD_PORT = 8888
UDP_DATA_PORT = 8889

# ===== Variables =====
is_recording = False
data_buffer = []
last_cmd = ""

# ===== Connections =====
# TCP (control)
tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock.connect((ESP_IP, ESP_PORT))
print(f"✓ TCP port: {ESP_IP}:{ESP_PORT}")

# UDP (measurements)
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind(('0.0.0.0', UDP_DATA_PORT))
udp_sock.settimeout(0.1)
print(f"✓ UDP port: {UDP_DATA_PORT}")

# ===== Measurement functions =====
def send_udp_command(command):
    """Sends recording commend START/STOP via UDP"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(command.encode(), (ESP_IP, UDP_CMD_PORT))
    sock.close()

def start_recording():
    """Starts recording the measurements"""
    global is_recording, data_buffer
    if not is_recording:
        is_recording = True
        data_buffer = []
        send_udp_command("START")
        print("\n Recording has been started")

def stop_recording():
    """Save the data"""
    global is_recording, data_buffer
    if is_recording:
        is_recording = False
        send_udp_command("STOP")
        print("\n Recording has been stopped")
        
        if data_buffer:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f".\\trajectory_{timestamp}.csv"
            
            # Save to CSV
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_ms', 'x_mm', 'y_mm', 'theta_rad', 'enc_left', 'enc_right'])
                writer.writerows(data_buffer)
            
            print(f"Saved {len(data_buffer)} points: {filename}")
            plot_trajectory(data_buffer, timestamp)
            data_buffer = []

def plot_trajectory(data, timestamp):
    """Plot the trajectory"""
    if len(data) < 2:
        print("A few points only")
        return
    
    x_vals = [float(row[1]) for row in data]
    y_vals = [float(row[2]) for row in data]
    
    plt.figure(figsize=(10, 8))
    plt.plot(x_vals, y_vals, 'b-', linewidth=2, label='Trajectory')
    plt.plot(x_vals[0], y_vals[0], 'go', markersize=12, label='Start')
    plt.plot(x_vals[-1], y_vals[-1], 'ro', markersize=12, label='Stop')
    
    plt.xlabel('X [mm]')
    plt.ylabel('Y [mm]')
    plt.title('Trajectory')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()
    
    plot_file = f".\\plot_{timestamp}.png"
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"Plot saved: {plot_file}")
    plt.show()

def receive_measurements():
    """UDP thread"""
    global data_buffer
    
    while True:
        try:
            data, addr = udp_sock.recvfrom(1024)
            message = data.decode().strip()
            
            if message in ["STARTED", "STOPPED"]:
                print(f"ESP32: {message}")
                continue
            
            if is_recording:
                values = message.split(',')
                
                if len(values) >= 5:
                    data_buffer.append(values[:6])  # Six measurement values is being sent [time_ms,x_mm,y_mm,theta_rad,enc_left,enc_right]
                    
                    if len(data_buffer) % 20 == 0:
                        try:
                            x = float(values[1])  
                            y = float(values[2])  
                            print(f"Punktów: {len(data_buffer)} | X: {x:.1f}mm Y: {y:.1f}mm", end='\r')
                        except:
                            pass
        
        except socket.timeout:
            continue
        except Exception as e:
            if is_recording:
                print(f"\nConnection error: {e}")

def main_control():
    """Main control loop"""
    global last_cmd
    
    print("\n" + "="*60)
    print("ROBOT CONTROL")
    print("="*60)
    print("Direction:")
    print("  ↑/↓/←/→ - move")
    print("\nAdhesion mechanism:")
    print("  M - turn ON")
    print("  N - turn OFF")
    print("\nMeasurement recording:")
    print("  R - start")
    print("  E - stop and save")
    print("\n  Q - exit")
    print("="*60 + "\n")
    
    try:
        while True:
            cmd = ""
            
            # Keaybord commands
            if keyboard.is_pressed("up"):
                cmd = "W"
            elif keyboard.is_pressed("down"):
                cmd = "S"
            elif keyboard.is_pressed("left"):
                cmd = "A"
            elif keyboard.is_pressed("right"):
                cmd = "D"
            elif keyboard.is_pressed("m"):
                cmd = "M"
            elif keyboard.is_pressed("n"):
                cmd = "N"
            elif keyboard.is_pressed("r"):
                start_recording()
                time.sleep(0.3)
                continue
            elif keyboard.is_pressed("e"):
                stop_recording()
                time.sleep(0.3)
                continue
            elif keyboard.is_pressed("q"):
                print("\n Robot stopped")
                tcp_sock.sendall(b"STOP\n")
                if is_recording:
                    stop_recording()
                break
            else:
                cmd = "STOP"
            
            # Sends only when the commend changes
            if cmd != last_cmd:
                tcp_sock.sendall((cmd + "\n").encode())
                if cmd != "STOP":
                    print(f"commend: {cmd}")
                last_cmd = cmd
            
            time.sleep(0.05)
    
    except Exception as e:
        print(f"\n Error: {e}")
    finally:
        tcp_sock.close()
        udp_sock.close()
        print("Connection closed")

if __name__ == "__main__":
    # Catching measurements in the background thread
    measurement_thread = threading.Thread(target=receive_measurements, daemon=True)
    measurement_thread.start()
    
    # Main control loop 
    try:
        main_control()
    except KeyboardInterrupt:
        print("\n\nCanceled")
        tcp_sock.close()
        udp_sock.close()