import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

class UAVTelemetryApp:
    def __init__(self, root):
        self.root = root
        self.root.title("1DOF UAV Roll-Axis Telemetry Terminal")
        self.root.geometry("1100x700")
        
        # Data Buffers (Stores last 200 data points for smooth animation)
        self.time_buffer = deque(maxlen=200)
        self.roll_buffer = deque(maxlen=200)
        self.setpoint_buffer = deque(maxlen=200)
        
        self.start_time = time.time()
        self.is_running = False
        self.sock = None
        
        # Default Control Parameters
        self.current_kp = 0.0
        self.current_ki = 0.0
        self.current_kd = 0.0
        self.current_sp = 0.0
        
        self.setup_ui()
        
    def setup_ui(self):
        # Left Panel: Network & Control Settings
        control_frame = ttk.LabelFrame(self.root, text=" Control Panel Link ", padding=15)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        ttk.Label(control_frame, text="ESP32 IP Address:").pack(anchor=tk.W, pady=2)
        self.ip_entry = ttk.Entry(control_frame)
        self.ip_entry.insert(0, "192.168.4.1") # Default ESP32 SoftAP IP
        self.ip_entry.pack(fill=tk.X, pady=5)
        
        ttk.Label(control_frame, text="UDP Port:").pack(anchor=tk.W, pady=2)
        self.port_entry = ttk.Entry(control_frame)
        self.port_entry.insert(0, "8888")
        self.port_entry.pack(fill=tk.X, pady=5)
        
        self.connect_btn = ttk.Button(control_frame, text="Connect to Rig", command=self.toggle_connection)
        self.connect_btn.pack(fill=tk.X, pady=10)
        
        ttk.Separator(control_frame, orient='horizontal').pack(fill=tk.X, pady=10)
        
        # PID Parameter Inputs
        ttk.Label(control_frame, text="Proportional (Kp):").pack(anchor=tk.W, pady=2)
        self.kp_entry = ttk.Entry(control_frame)
        self.kp_entry.insert(0, "1.45")
        self.kp_entry.pack(fill=tk.X, pady=2)
        
        ttk.Label(control_frame, text="Integral (Ki):").pack(anchor=tk.W, pady=2)
        self.ki_entry = ttk.Entry(control_frame)
        self.ki_entry.insert(0, "0.05")
        self.ki_entry.pack(fill=tk.X, pady=2)
        
        ttk.Label(control_frame, text="Derivative (Kd):").pack(anchor=tk.W, pady=2)
        self.kd_entry = ttk.Entry(control_frame)
        self.kd_entry.insert(0, "0.32")
        self.kd_entry.pack(fill=tk.X, pady=2)
        
        ttk.Label(control_frame, text="Target Roll Setpoint (deg):").pack(anchor=tk.W, pady=2)
        self.sp_entry = ttk.Entry(control_frame)
        self.sp_entry.insert(0, "0.0")
        self.sp_entry.pack(fill=tk.X, pady=2)
        
        self.send_btn = ttk.Button(control_frame, text="Transmit Parameters", command=self.send_parameters)
        self.send_btn.pack(fill=tk.X, pady=10)
        
        # Emergency Kill Switch 
        self.kill_btn = tk.Button(control_frame, text="EMERGENCY KILL\n(Stop Motors)", bg="#d9534f", fg="white", 
                                  font=('Helvetica', 12, 'bold'), command=self.emergency_kill)
        self.kill_btn.pack(fill=tk.X, side=tk.BOTTOM, pady=20)
        
        # Right Panel: Real-Time Plotting Area
        self.graph_frame = ttk.Frame(self.root, padding=10)
        self.graph_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_title("Real-Time Roll Attitude Tracking")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Roll Angle (Degrees)")
        self.ax.grid(True, linestyle="--", alpha=0.5)
        
        self.roll_line, = self.ax.plot([], [], label="Current Roll (θ)", color="#0275d8", linewidth=2)
        self.setpoint_line, = self.ax.plot([], [], label="Target Setpoint", color="#d9534f", linestyle="--", linewidth=1.5)
        self.ax.legend(loc="upper right")
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def toggle_connection(self):
        if not self.is_running:
            ip = self.ip_entry.get()
            try:
                port = int(self.port_entry.get())
            except ValueError:
                messagebox.showerror("Port Error", "UDP Port must be an integer.")
                return
                
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(1.0)
            self.is_running = True
            self.connect_btn.config(text="Disconnect")
            
            # Start background listener thread for telemetry streaming
            self.listen_thread = threading.Thread(target=self.receive_telemetry, daemon=True)
            self.listen_thread.start()
            
            # Send initial PING to let ESP32 discover our IP return address
            try:
                self.sock.sendto(b"PING\n", (ip, port))
            except Exception as e:
                print(f"Initial ping failed: {e}")
        else:
            self.is_running = False
            if self.sock:
                self.sock.close()
            self.connect_btn.config(text="Connect to Rig")

    def send_parameters(self):
        if not self.sock or not self.is_running:
            messagebox.showwarning("Connection Status", "Please connect to the flight rig telemetry link first.")
            return
        try:
            self.current_kp = float(self.kp_entry.get())
            self.current_ki = float(self.ki_entry.get())
            self.current_kd = float(self.kd_entry.get())
            self.current_sp = float(self.sp_entry.get())
            
            # Pack string format: "Kp,Ki,Kd,Setpoint\n" matching ESP32 sscanf structure
            packet = f"{self.current_kp},{self.current_ki},{self.current_kd},{self.current_sp}\n".encode('utf-8')
            self.sock.sendto(packet, (self.ip_entry.get(), int(self.port_entry.get())))
        except ValueError:
            messagebox.showerror("Input Error", "All control parameters must be valid numeric values.")

    def receive_telemetry(self):
        while self.is_running:
            try:
                data, _ = self.sock.recvfrom(1024)
                raw_angle = data.decode('utf-8').strip()
                current_angle = float(raw_angle)
                
                elapsed = time.time() - self.start_time
                self.time_buffer.append(elapsed)
                self.roll_buffer.append(current_angle)
                self.setpoint_buffer.append(self.current_sp)
                
                self.update_plot()
            except (socket.timeout, ValueError, OSError):
                continue

    def update_plot(self):
        if not self.time_buffer:
            return
        self.roll_line.set_data(list(self.time_buffer), list(self.roll_buffer))
        self.setpoint_line.set_data(list(self.time_buffer), list(self.setpoint_buffer))
        
        self.ax.set_xlim(min(self.time_buffer), max(self.time_buffer) + 1.0)
        
        all_y = list(self.roll_buffer) + list(self.setpoint_buffer)
        self.ax.set_ylim(min(all_y) - 5, max(all_y) + 5)
        
        self.canvas.draw_idle()

    def emergency_kill(self):
        if self.sock and self.is_running:
            try:
                self.kp_entry.delete(0, tk.END)
                self.ki_entry.delete(0, tk.END)
                self.kd_entry.delete(0, tk.END)
                self.kp_entry.insert(0, "0.0")
                self.ki_entry.insert(0, "0.0")
                self.kd_entry.insert(0, "0.0")
                
                kill_packet = "0.0,0.0,0.0,0.0\n".encode('utf-8')
                self.sock.sendto(kill_packet, (self.ip_entry.get(), int(self.port_entry.get())))
                messagebox.showwarning("HARD MOTOR CUTOFF", "Emergency kill signal sent. Actuators commanded to safe states.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send cutoff signal: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = UAVTelemetryApp(root)
    root.mainloop()
