import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading
import time
import signal
import sys
import numpy as np
import json
import os
from typing import List, Optional
from lidar_sdk import LidarM1, Point
from calibration import LidarCalibrator
from calibration.file_io import load_background_from_file
from detection import TouchDetector, ThresholdConfig
from config import config
from noise_filter.noise_profiler import NoiseProfiler
from noise_filter.noise_filter import NoiseFilter
from persistent_noise_filter.persistent_noise_manager import PersistentNoiseManager
from noise_filter.corner_calibration import CornerCalibrator
#from orientation import get_orientation_manager
import queue
import threading
import traceback

# Добавляем импорт mouse_controller
try:
    from mouse_controller import MouseController
    MOUSE_CONTROL_AVAILABLE = True
except ImportError:
    MOUSE_CONTROL_AVAILABLE = False
    print("⚠️  Mouse controller not available")

# Matplotlib imports for embedding plots in Tkinter
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

def cleanup_files():
    """Удаляет временные файлы при завершении программы"""
    files_to_remove = ["background.npz", "projection_corners.json", "noise_profile.json",
                       "persistent_noise_points.json", "north_calibration.json", "orientation_config.json"]
    
    for filename in files_to_remove:
        try:
            if os.path.exists(filename):
                os.remove(filename)
                print(f"🗑️  Удален файл: {filename}")
        except Exception as e:
            print(f"⚠️  Не удалось удалить {filename}: {e}")

    #Регистрируем функцию для вызова при выходе
    import atexit
    #atexit.register(cleanup_files)

class LiDARApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LiDAR Control Panel")
        self.root.geometry("2000x1200")
        
        # Инициализация менеджера ориентации
        #self.orientation_manager = get_orientation_manager()
        self.plot_queue = queue.Queue()
        self.plot_thread = threading.Thread(target=self.plot_worker, daemon=True)
        self.plot_thread.start()
        
        # LiDAR related attributes
        self.lidar: Optional[LidarM1] = None
        self.detector: Optional[TouchDetector] = None
        self.noise_filter: Optional[NoiseFilter] = None
        self.persistent_noise_manager: Optional[PersistentNoiseManager] = None
        self.background_filename = "background.npz"
        self.projection_corners_file = "projection_corners.json"
        self.mouse_controller = None
        self.background_map = None
        self.corners = None
        
        # Threading control
        self.running = False
        self.lidar_thread = None
        
        # Setup GUI
        self.setup_gui()
        
        # Initialize LiDAR in a separate thread
        self.init_lidar_async()
        
        # Загружаем калибровку севера
        #self.load_north_calibration()

    def plot_worker(self):
        """Отдельный поток для обновления графиков"""
        while True:
            try:
                data = self.plot_queue.get(timeout=0.01)
                if data is None:
                    break
                scan, touch_points = data
                self.update_plot_safe(scan, touch_points)
            except queue.Empty:
                pass

    def update_plot_fast(self, scan=None, touch_points=None):
        """Быстрое обновление графика через очередь"""
        try:
            self.plot_queue.put((scan, touch_points), block=False)
        except:
            pass  # Игнорируем переполнение очереди

    def log_message(self, message):
        """Add a message to the log area"""
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)

    def setup_gui(self):
        # Configure grid weights for resizing
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
    
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=3)
        main_frame.grid_columnconfigure(1, weight=1)
    
        # Plot frame (left side)
        plot_frame = ttk.LabelFrame(main_frame, text="LiDAR Real-time View")
        plot_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
    
        # Create matplotlib figure and axis
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_title("LiDAR Scan (Polar Coordinates)")
    
        # Create canvas for matplotlib plot
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
    
        # Control panel (right side)
        control_frame = ttk.LabelFrame(main_frame, text="Control Panel")
        control_frame.grid(row=0, column=1, sticky="nsew")
        control_frame.grid_columnconfigure(0, weight=1)
    
        # Status indicators
        status_frame = ttk.LabelFrame(control_frame, text="Status")
        status_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        status_frame.grid_columnconfigure(1, weight=1)
    
        ttk.Label(status_frame, text="LiDAR:").grid(row=0, column=0, sticky="w", padx=5, pady=2)
        self.lidar_status = ttk.Label(status_frame, text="Connecting...", foreground="orange")
        self.lidar_status.grid(row=0, column=1, sticky="w", padx=5, pady=2)
    
        ttk.Label(status_frame, text="Background:").grid(row=1, column=0, sticky="w", padx=5, pady=2)
        self.bg_status = ttk.Label(status_frame, text="Not Created", foreground="red")
        self.bg_status.grid(row=1, column=1, sticky="w", padx=5, pady=2)
    
        ttk.Label(status_frame, text="Noise Profile:").grid(row=2, column=0, sticky="w", padx=5, pady=2)
        self.noise_status = ttk.Label(status_frame, text="Not Profiled", foreground="red")
        self.noise_status.grid(row=2, column=1, sticky="w", padx=5, pady=2)
    
        ttk.Label(status_frame, text="Corners:").grid(row=3, column=0, sticky="w", padx=5, pady=2)
        self.corners_status = ttk.Label(status_frame, text="Not Calibrated", foreground="red")
        self.corners_status.grid(row=3, column=1, sticky="w", padx=5, pady=2)
    
        ttk.Label(status_frame, text="Mouse Control:").grid(row=4, column=0, sticky="w", padx=5, pady=2)
        self.mouse_status = ttk.Label(status_frame, text="Not Initialized", foreground="red")
        self.mouse_status.grid(row=4, column=1, sticky="w", padx=5, pady=2)
    
        # Orientation frame
        #orientation_frame = ttk.LabelFrame(control_frame, text="Orientation Control")
        #orientation_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        ##orientation_frame.grid_columnconfigure(0, weight=1)
        #orientation_frame.grid_columnconfigure(1, weight=1)
        #orientation_frame.grid_columnconfigure(2, weight=1)
    
        #ttk.Label(orientation_frame, text="Current Orientation:").grid(row=0, column=0, sticky="w", padx=5)
        #self.orientation_label = ttk.Label(orientation_frame, text="0.0°", foreground="blue")
        #self.orientation_label.grid(row=0, column=1, sticky="w", padx=5)
    
        #self.rotate_ccw_button = ttk.Button(orientation_frame, text="↺ -1°", 
                                       #command=self.rotate_counterclockwise)
        #self.rotate_ccw_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
    #
        #self.reset_orientation_button = ttk.Button(orientation_frame, text="Reset 0°", 
        #                                      command=self.reset_orientation)
        #self.reset_orientation_button.grid(row=1, column=1, sticky="ew", padx=2, pady=2)
    #
        #self.rotate_cw_button = ttk.Button(orientation_frame, text="↻ +1°", 
        #                              command=self.rotate_clockwise)
        #self.rotate_cw_button.grid(row=1, column=2, sticky="ew", padx=2, pady=2)
    
        # North direction calibration frame
        #north_frame = ttk.LabelFrame(control_frame, text="Set North Direction")
        #north_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        #north_frame.grid_columnconfigure(0, weight=1)
        #north_frame.grid_columnconfigure(1, weight=1)
        #north_frame.grid_columnconfigure(2, weight=1)
    
        # Кнопки направлений (как компас)
        #btn_frame = ttk.Frame(north_frame)
        #btn_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=5)
        #btn_frame.grid_columnconfigure(0, weight=1)
        #btn_frame.grid_columnconfigure(1, weight=1)
        #btn_frame.grid_columnconfigure(2, weight=1)
    
        # Верхняя кнопка (Север)
        #self.north_btn = ttk.Button(btn_frame, text="↑\nNorth", 
         #                      command=lambda: self.set_north_direction(0))
        #self.north_btn.grid(row=0, column=1, padx=2, pady=2, sticky="ew")
    
        # Левая кнопка (Запад)
        #self.west_btn = ttk.Button(btn_frame, text="←\nWest", 
        #                          command=lambda: self.set_north_direction(270))
        #self.west_btn.grid(row=1, column=0, padx=2, pady=2, sticky="ew")
    
        # Правая кнопка (Восток)
        #self.east_btn = ttk.Button(btn_frame, text="→\nEast", 
        #                      command=lambda: self.set_north_direction(90))
        #self.east_btn.grid(row=1, column=2, padx=2, pady=2, sticky="ew")
    
        # Нижняя кнопка (Юг)
        #self.south_btn = ttk.Button(btn_frame, text="↓\nSouth", 
        #                       command=lambda: self.set_north_direction(180))
        #self.south_btn.grid(row=2, column=1, padx=2, pady=2, sticky="ew")
    
        # Отображение текущего севера
        #ttk.Label(north_frame, text="Current North:").grid(row=1, column=0, sticky="w", padx=5)
        #self.north_direction_label = ttk.Label(north_frame, text="0°", foreground="blue")
        #self.north_direction_label.grid(row=1, column=1, sticky="w", padx=5)
    
        # Calibration buttons
        calib_frame = ttk.LabelFrame(control_frame, text="Calibration")
        calib_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        calib_frame.grid_columnconfigure(0, weight=1)
    
        self.bg_button = ttk.Button(calib_frame, text="Create Background Map", 
                               command=self.create_background_map)
        self.bg_button.grid(row=0, column=0, sticky="ew", padx=5, pady=2)
    
        self.noise_button = ttk.Button(calib_frame, text="Profile Noise", 
                                  command=self.profile_noise)
        self.noise_button.grid(row=1, column=0, sticky="ew", padx=5, pady=2)
    
        # Corner calibration buttons
        corner_frame = ttk.LabelFrame(calib_frame, text="Corner Calibration")
        corner_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=5)
        corner_frame.grid_columnconfigure(0, weight=1)
        corner_frame.grid_columnconfigure(1, weight=1)
    
        self.top_left_button = ttk.Button(corner_frame, text="Top Left", 
                                     command=lambda: self.calibrate_corner("top_left"))
        self.top_left_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
    
        self.top_right_button = ttk.Button(corner_frame, text="Top Right", 
                                      command=lambda: self.calibrate_corner("top_right"))
        self.top_right_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
    
        self.bottom_left_button = ttk.Button(corner_frame, text="Bottom Left", 
                                        command=lambda: self.calibrate_corner("bottom_left"))
        self.bottom_left_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
    
        self.bottom_right_button = ttk.Button(corner_frame, text="Bottom Right", 
                                         command=lambda: self.calibrate_corner("bottom_right"))
        self.bottom_right_button.grid(row=1, column=1, sticky="ew", padx=2, pady=2)
    
        self.all_corners_button = ttk.Button(calib_frame, text="Calibrate All Corners", 
                                        command=self.calibrate_all_corners)
        self.all_corners_button.grid(row=3, column=0, sticky="ew", padx=5, pady=2)
    
        # Mouse control buttons
        if MOUSE_CONTROL_AVAILABLE:
            mouse_frame = ttk.LabelFrame(control_frame, text="Mouse Control")
            mouse_frame.grid(row=4, column=0, sticky="ew", padx=10, pady=5)
            mouse_frame.grid_columnconfigure(0, weight=1)
            mouse_frame.grid_columnconfigure(1, weight=1)
        
            self.mouse_enable_button = ttk.Button(mouse_frame, text="Enable Mouse", 
                                                 command=self.enable_mouse_control)
            self.mouse_enable_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        
            self.mouse_disable_button = ttk.Button(mouse_frame, text="Disable Mouse", 
                                              command=self.disable_mouse_control, 
                                              state="disabled")
            self.mouse_disable_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
    
        # Countdown label for corner calibration
        self.countdown_label = ttk.Label(calib_frame, text="", foreground="red")
        self.countdown_label.grid(row=4, column=0, pady=5)
    
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=5, column=0, sticky="ew", padx=10, pady=5)
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)
    
        self.start_button = ttk.Button(button_frame, text="Start Detection", 
                                  command=self.start_detection)
        self.start_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
    
        self.stop_button = ttk.Button(button_frame, text="Stop Detection", 
                                 command=self.stop_detection, state="disabled")
        self.stop_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
    
        self.quit_button = ttk.Button(control_frame, text="Quit", command=self.quit_app)
        self.quit_button.grid(row=6, column=0, sticky="ew", padx=10, pady=5)
    
        # Log area
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)
    
        self.log_text = tk.Text(log_frame, height=8)
        self.log_text.grid(row=0, column=0, sticky="nsew")
    
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
    
        # Initialize direction display
        #self.load_north_direction()

    def init_lidar_async(self):
        """Initialize LiDAR in a separate thread"""
        def init():
            try:
                self.log_message("🔌 Connecting to LiDAR...")
                self.lidar = LidarM1()
            
                if not self.lidar.connect():
                    raise ConnectionError("Failed to connect to LiDAR!")
            
                self.lidar.start()
                self.lidar_status.config(text="Connected", foreground="green")
                self.log_message("✅ LiDAR connected and started")
            
                # Initialize persistent noise manager
                self.persistent_noise_manager = PersistentNoiseManager()
                self.log_message("📂 Persistent noise manager initialized")
            
                # Try to load existing background and corners
                self.load_existing_data()
            
                # Initialize mouse controller if available
                if MOUSE_CONTROL_AVAILABLE:
                    try:
                        self.mouse_controller = MouseController()
                        self.mouse_status.config(text="Initialized", foreground="orange")
                        self.log_message("🖱️  Mouse controller initialized")
                    except Exception as e:
                        self.log_message(f"⚠️  Mouse controller initialization failed: {e}")
                        self.mouse_status.config(text="Init Failed", foreground="red")
            
            except Exception as e:
                self.lidar_status.config(text="Connection Failed", foreground="red")
                self.log_message(f"❌ LiDAR connection error: {e}")
    
        threading.Thread(target=init, daemon=True).start()

    def load_existing_data(self):
        """Load existing background map and corners if available"""
        try:
            # Try to load background map
            background_data = load_background_from_file(self.background_filename)
            self.background_map = background_data[0] if isinstance(background_data, tuple) else background_data
            self.bg_status.config(text="Loaded", foreground="green")
            self.log_message(f"💾 Loaded existing background map from '{self.background_filename}'")
            
            # Initialize touch detector if not already done
            if not self.detector and self.background_map:
                self.detector = TouchDetector(self.background_map, ThresholdConfig.sensitive())
                self.log_message("🔍 Touch detector initialized")
        except:
            pass  # No existing background map

        try:
            # Try to load noise profile
            noise_profiler = NoiseProfiler()
            noise_profiler.load_noise_profile()
            if noise_profiler.noise_profile:
                # Create noise filter with loaded profile
                self.noise_filter = NoiseFilter(noise_profiler.noise_profile, self.persistent_noise_manager)
                self.noise_status.config(text="Loaded", foreground="green")
                self.log_message("📊 Noise profile loaded")
        except Exception as e:
            self.log_message(f"⚠️  Could not load noise profile: {e}")
        pass  # No existing noise profile
            
        try:
            # Try to load corners
            with open(self.projection_corners_file, 'r') as f:
                data = json.load(f)
                self.corners = data['corners']
                self.corners_status.config(text="Loaded", foreground="green")
                self.log_message(f"💾 Loaded existing corners from '{self.projection_corners_file}'")
                self.update_plot()  # Update plot with loaded corners
                
                # Update mouse controller with corners if available
                if self.mouse_controller and self.corners:
                    self.mouse_controller.set_projection_corners(self.corners)
                    self.mouse_status.config(text="Ready", foreground="green")
                    self.log_message("🖱️  Mouse controller updated with projection corners")
        except:
            pass  # No existing corners file

    def create_background_map(self):
        """Create background map from LiDAR scans"""
        if not self.lidar:
            self.log_message("⚠️  LiDAR not connected")
            return
            
        def create_bg():
            try:
                self.log_message("📏 Creating background map (10 scans)...")
                self.bg_button.config(state="disabled")
                
                collected_scans = []
                scan_count = 0
                
                def on_scan_received(scan):
                    nonlocal scan_count
                    collected_scans.append(scan)
                    scan_count += 1
                    self.log_message(f"📸 Collected scan {scan_count}/10")
                    
                    if scan_count >= 10:
                        self.lidar.on_scan_complete = None
                
                # Register callback
                self.lidar.on_scan_complete = on_scan_received
                
                # Wait for scans
                timeout = time.time() + 30  # 30 second timeout
                while scan_count < 10 and time.time() < timeout:
                    time.sleep(0.05)
                
                if scan_count < 10:
                    self.log_message("❌ Background creation timed out")
                    self.bg_button.config(state="normal")
                    return
                
                # Process scans
                self.log_message("📊 Processing collected data...")
                calibrator = LidarCalibrator()
                self.background_map = calibrator.process_scans_to_background(collected_scans)
                
                # Save background map
                calibrator.save_background(self.background_filename, self.background_map)
                self.bg_status.config(text="Created", foreground="green")
                self.log_message(f"💾 Background map saved to '{self.background_filename}'")
                
                # Initialize touch detector
                self.detector = TouchDetector(self.background_map, ThresholdConfig.sensitive())
                self.log_message("🔍 Touch detector initialized")
                
                # Update plot with background
                self.update_plot()
                
                self.bg_button.config(state="normal")
                
            except Exception as e:
                self.log_message(f"❌ Background creation error: {e}")
                self.bg_button.config(state="normal")
        
        threading.Thread(target=create_bg, daemon=True).start()

    def profile_noise(self):
        """Profile noise for filtering"""
        if not self.lidar or not self.detector:
            self.log_message("⚠️  LiDAR or detector not ready")
            return
            
        def profile():
            try:
                self.log_message("📊 Profiling noise (10 seconds)...")
                self.log_message("Ensure NO ONE touches the wall!")
                self.noise_button.config(state="disabled")
                
                profiler = NoiseProfiler(duration_seconds=10)
                profiler.start_noise_collection(self.lidar, self.detector)
                
                # Create noise filter
                self.noise_filter = NoiseFilter(profiler.noise_profile, self.persistent_noise_manager)
                self.noise_status.config(text="Profiled", foreground="green")
                self.log_message("✅ Noise profiling completed")
                
                self.noise_button.config(state="normal")
                
            except Exception as e:
                self.log_message(f"❌ Noise profiling error: {e}")
                self.noise_button.config(state="normal")
        
        threading.Thread(target=profile, daemon=True).start()

    def calibrate_corner(self, corner_name):
        """Calibrate a specific corner with 5-second countdown"""
        if not self.lidar or not self.detector or not self.noise_filter:
            self.log_message("⚠️  LiDAR, detector, or noise filter not ready")
            return
        
        def calibrate():
            try:
                self.log_message(f"⏳ Preparing to calibrate {corner_name} corner...")
                self.log_message("Please move to the calibration position within 5 seconds...")
            
                # Disable corresponding button
                btn_map = {
                    "top_left": self.top_left_button,
                    "top_right": self.top_right_button,
                    "bottom_left": self.bottom_left_button,
                    "bottom_right": self.bottom_right_button
                }
                btn = btn_map.get(corner_name)
                if btn:
                    btn.config(state="disabled")
            
                # 5-second countdown
                for i in range(5, 0, -1):
                    self.countdown_label.config(text=f"Starting calibration in {i} seconds...")
                    self.log_message(f"⏳ {i}...")
                    time.sleep(1)
                self.countdown_label.config(text="")
            
                self.log_message(f"🎯 Calibrating {corner_name} corner...")
            
                # Perform corner calibration
                corner_calibrator = CornerCalibrator(self.noise_filter)
                result = corner_calibrator.calibrate_corner(self.lidar, self.detector, corner_name, log_callback=self.log_message)
            
                if result:
                    self.log_message(f"✅ {corner_name} corner calibrated successfully")
                    # Update corners data
                    self.update_corners(result)
                
                    # Update mouse controller with new corners
                    if self.mouse_controller and self.corners:
                        self.mouse_controller.set_projection_corners(self.corners)
                        self.mouse_status.config(text="Updated", foreground="green")
                        self.log_message("🖱️  Mouse controller updated with new corner")
                else:
                    self.log_message(f"❌ {corner_name} corner calibration failed")
            
                # Re-enable button
                if btn:
                    btn.config(state="normal")
                
            except Exception as e:
                self.log_message(f"❌ {corner_name} corner calibration error: {e}")
                # Re-enable button
                btn_map = {
                    "top_left": self.top_left_button,
                    "top_right": self.top_right_button,
                    "bottom_left": self.bottom_left_button,
                    "bottom_right": self.bottom_right_button
                }
                btn = btn_map.get(corner_name)
                if btn:
                    btn.config(state="normal")
                self.countdown_label.config(text="")
    
        threading.Thread(target=calibrate, daemon=True).start()


    def calibrate_all_corners(self):
        """Calibrate all corners with 5-second countdown"""
        if not self.lidar or not self.detector or not self.noise_filter:
            self.log_message("⚠️  LiDAR, detector, or noise filter not ready")
            return
        
        def calibrate_all():
            try:
                self.log_message("⏳ Preparing to calibrate all corners...")
                self.log_message("Please move to the first calibration position within 5 seconds...")
            
                # Disable all corner buttons
                buttons = [self.top_left_button, self.top_right_button, 
                        self.bottom_left_button, self.bottom_right_button,
                        self.all_corners_button]
                for btn in buttons:
                    btn.config(state="disabled")
            
                # 5-second countdown
                for i in range(5, 0, -1):
                    self.countdown_label.config(text=f"Starting calibration in {i} seconds...")
                    self.log_message(f"⏳ {i}...")
                    time.sleep(1)
                self.countdown_label.config(text="")
            
                self.log_message("🎯 Calibrating all corners...")
            
                # Perform calibration of all corners WITH LOG CALLBACK
                corner_calibrator = CornerCalibrator(self.noise_filter)
                corners = corner_calibrator.calibrate_all_corners(self.lidar, self.detector, self.log_message)
            
                if corners:
                    # Save corners to file
                    corners_data = {
                        'timestamp': time.time(),
                        'corners': corners
                    }
                
                    with open(self.projection_corners_file, 'w', encoding='utf-8') as f:
                        json.dump(corners_data, f, ensure_ascii=False, indent=2)
                    
                    self.corners = corners
                    self.corners_status.config(text="Calibrated", foreground="green")
                    self.log_message(f"✅ All corners calibrated and saved to '{self.projection_corners_file}'")
                
                    # Update plot with corners
                    self.update_plot()
                
                    # Update mouse controller with corners
                    if self.mouse_controller:
                        self.mouse_controller.set_projection_corners(corners)
                        self.mouse_status.config(text="Ready", foreground="green")
                        self.log_message("🖱️  Mouse controller updated with all corners")
                else:
                    self.log_message("❌ All corners calibration failed")
            
                # Re-enable buttons
                for btn in buttons:
                    btn.config(state="normal")
                
            except Exception as e:
                self.log_message(f"❌ All corners calibration error: {e}")
                # Re-enable buttons
                buttons = [self.top_left_button, self.top_right_button, 
                        self.bottom_left_button, self.bottom_right_button,
                        self.all_corners_button]
                for btn in buttons:
                    btn.config(state="normal")
                self.countdown_label.config(text="")
    
        threading.Thread(target=calibrate_all, daemon=True).start()


    def update_corners(self, new_corner):
        """Update corners data with a new corner - FIXED VERSION"""
        if not self.corners:
            self.corners = []
    
        # Проверяем, существует ли уже угол с таким именем
        corner_updated = False
        for i, corner in enumerate(self.corners):
            if corner['name'] == new_corner['name']:
                # Заменяем существующий угол
                self.corners[i] = new_corner
                corner_updated = True
                self.log_message(f"🔄 Угол {new_corner['name']} обновлен")
                break
    
        # Если угол с таким именем не найден, добавляем новый
        if not corner_updated:
            self.corners.append(new_corner)
            self.log_message(f"➕ Добавлен новый угол: {new_corner['name']}")
    
        # Убедимся, что у нас максимум 4 угла (удаляем дубликаты по имени)
        unique_corners = []
        seen_names = set()
        for corner in self.corners:
            if corner['name'] not in seen_names:
                unique_corners.append(corner)
                seen_names.add(corner['name'])
    
        self.corners = unique_corners
    
        # Сохраняем в файл
        corners_data = {
            'timestamp': time.time(),
            'corners': self.corners
        }
    
        try:
            with open(self.projection_corners_file, 'w', encoding='utf-8') as f:
                json.dump(corners_data, f, ensure_ascii=False, indent=2)
        
            if len(self.corners) == 4:
                self.corners_status.config(text="Complete", foreground="green")
            else:
                self.corners_status.config(text=f"{len(self.corners)}/4", foreground="orange")
            
            self.log_message(f"💾 Сохранены данные углов ({len(self.corners)} шт.) в '{self.projection_corners_file}'")
        except Exception as e:
            self.log_message(f"❌ Ошибка сохранения углов: {e}")
    
        # Обновляем график
        self.update_plot()


    def enable_mouse_control(self):
        """Enable mouse control"""
        if self.mouse_controller:
            try:
                self.mouse_controller.enable_control()
                self.mouse_status.config(text="Enabled", foreground="green")
                self.mouse_enable_button.config(state="disabled")
                self.mouse_disable_button.config(state="normal")
                self.log_message("🖱️  Mouse control enabled")
            except Exception as e:
                self.log_message(f"❌ Error enabling mouse control: {e}")
        else:
            self.log_message("⚠️  Mouse controller not initialized")

    def disable_mouse_control(self):
        """Disable mouse control"""
        if self.mouse_controller:
            try:
                self.mouse_controller.disable_control()
                self.mouse_status.config(text="Disabled", foreground="orange")
                self.mouse_enable_button.config(state="normal")
                self.mouse_disable_button.config(state="disabled")
                self.log_message("🖱️  Mouse control disabled")
            except Exception as e:
                self.log_message(f"❌ Error disabling mouse control: {e}")
        else:
            self.log_message("⚠️  Mouse controller not initialized")

    def start_detection(self):
        """Start real-time detection"""
        if not self.lidar or not self.detector or not self.noise_filter:
            self.log_message("⚠️  Not all components are ready for detection")
            return
            
        self.running = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.log_message("🔄 Starting real-time detection...")
        
        # Start detection in a separate thread
        self.lidar_thread = threading.Thread(target=self.run_detection, daemon=True)
        self.lidar_thread.start()

    def stop_detection(self):
        """Stop real-time detection"""
        self.running = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.log_message("⏹️ Stopping real-time detection...")

    def run_detection(self):
        """Run the real-time detection loop"""
        try:
            scan_counter = 0
            last_update_time = time.time()

            # Отслеживание касаний для кликов мышью
            tracked_touches = {}
            frame_counter = 0
        
            # Для синхронизации начала скана
            last_sync_angle = None
            sync_tolerance = 10.0  # Градусов для синхронизации
        
            while self.running:
                current_scan = self.lidar.get_last_scan()
    
                if current_scan and len(current_scan) > 0:
                    # Проверяем синхронизацию начала скана (первая точка около 0°)
                    first_point_angle = current_scan[0].angle
                    angle_diff = abs(first_point_angle - 0.0)
                    # Учитываем переход через 0°
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff
                    
                    # Если скан не синхронизирован, пропускаем его
                    if angle_diff > sync_tolerance:
                        # Ждем следующий скан
                        time.sleep(0.005)  # 5ms
                        continue
                    
                    # Синхронизированный скан - обрабатываем его
                    # Применяем коррекцию ориентации если есть
                    corrected_scan = current_scan
        
                    scan_counter += 1
                    frame_counter += 1
        
                    # Детектируем точки касания (оптимизированная версия)
                    all_touch_points = self.detector.detect_touch_points(corrected_scan)
        
                    # Фильтруем точки с минимальной обработкой
                    real_touch_points = []
                    for point in all_touch_points:
                        # Быстрая проверка без обращения к файлам
                        if (not self.persistent_noise_manager or 
                            not self.persistent_noise_manager.is_persistent_noise(point)) and \
                            (not self.noise_filter or 
                            not self.noise_filter.is_false_positive(point)):
                            real_touch_points.append(point)
        
                    # Точки внутри рамки проекции
                    frame_touch_points = []
                    if self.corners:
                        for point in real_touch_points[:10]:  # Ограничиваем количество точек
                            if self.point_in_polygon_fast(point, self.corners):
                                frame_touch_points.append(point)
                    else:
                        frame_touch_points = real_touch_points[:10]  # Ограничиваем количество
        
                    # Обновляем график с новыми данными (опционально, реже)
                    current_time = time.time()
                    if current_time - last_update_time > 0.016:  # ~60 FPS
                        self.update_plot(corrected_scan, frame_touch_points)
                        last_update_time = current_time
        
                    # Управление мышью - оптимизированное отслеживание
                    if (self.mouse_controller and self.mouse_controller.is_active and 
                        frame_touch_points):
            
                        # Используем более агрессивную фильтрацию для мыши
                        stable_touch_points = frame_touch_points[:3]  # Только первые 3 точки
            
                        current_touch_keys = set()
            
                        # Быстрое обновление отслеживания
                        for touch_point in stable_touch_points:
                            touch_key = (round(touch_point.angle, 0), round(touch_point.distance, -1))  # Грубее группировка
                            current_touch_keys.add(touch_key)
                
                            if touch_key not in tracked_touches:
                                tracked_touches[touch_key] = {
                                    'point': touch_point,
                                    'start_frame': frame_counter,
                                    'start_time': time.time(),  # Добавляем время начала
                                    'last_update': frame_counter
                                }
                                # Мгновенное перемещение мыши
                                try:
                                    self.mouse_controller.move_mouse_to_touch(touch_point)
                                except:
                                    pass
            
                        # Проверяем завершение касаний чаще
                        if frame_counter % 2 == 0:  # Каждый второй кадр
                            finished_touches = set(tracked_touches.keys()) - current_touch_keys
                            for touch_key in finished_touches:
                                touch_info = tracked_touches[touch_key]
                                # Проверяем минимальную длительность касания (0.1 секунды)
                                if time.time() - touch_info['start_time'] >= 0.1:
                                    try:
                                        # Выполняем клик при отпускании касания
                                        self.mouse_controller.click_at_touch(touch_info['point'])
                                        self.log_message(f"🖱️  Mouse click executed")
                                    except Exception as e:
                                        self.log_message(f"⚠️  Mouse click error: {e}")
                                # Удаляем отслеживание
                                del tracked_touches[touch_key]
        
                    # Очень редкое логирование
                    if scan_counter % 50 == 0:  # Каждые 50 сканов
                        self.log_message(f"FPS: {1/(time.time()-last_update_time):.0f} | Touches: {len(frame_touch_points)} | Sync angle: {first_point_angle:.1f}°")
    
                # Минимальная задержка
                time.sleep(0.005)  # 5 ms вместо 10 мс для лучшей отзывчивости

        except Exception as e:
            self.log_message(f"❌ Detection error: {e}")
            traceback.print_exc()  # Добавляем трассировку для диагностики
            self.running = False
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")

            if self.mouse_controller:
                self.mouse_controller.clear_all_touches()




    def update_plot(self, scan=None, touch_points=None):
        """Update the matplotlib plot with new scan data"""
        try:
            # Clear the axis
            self.ax.clear()
    
            # Set up the polar plot
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            self.ax.set_title("LiDAR Scan (Polar Coordinates)")
    
            # Plot background if available
            if self.background_map:
                angles = list(self.background_map.keys())
                distances = list(self.background_map.values())
                self.ax.plot(np.radians(angles), distances, 'b-', linewidth=1, label='Background')
    
            # Plot current scan data if provided
            if scan:
                angles = [point.angle for point in scan]
                distances = [point.distance for point in scan]
                # Plot all scan points in blue
                # self.ax.scatter(np.radians(angles), distances, c='blue', s=10, alpha=0.6, label='Scan Points')
    
            # Plot touch points if provided
            if touch_points:
                touch_angles = [point.angle for point in touch_points]
                touch_distances = [point.distance for point in touch_points]
                self.ax.scatter(np.radians(touch_angles), touch_distances, 
                            c='red', s=50, alpha=0.8, marker='o', label='Touch Points')
    
            # Plot corners if available
            if self.corners:
                corner_angles = [corner['angle'] for corner in self.corners]
                corner_distances = [corner['distance'] for corner in self.corners]
                # Отображаем углы без подписей
                self.ax.scatter(np.radians(corner_angles), corner_distances, 
                            c='orange', s=100, alpha=0.9, marker='s', 
                            label='Projection Corners', edgecolors='black', linewidth=2)
    
            # Add legend
            self.ax.legend()
    
            # Refresh the canvas
            self.canvas.draw()
    
        except Exception as e:
            self.log_message(f"Plot update error: {e}")



    def point_in_polygon_fast(self, point: Point, polygon_corners: List[dict]) -> bool:
        """Проверка точки внутри прямоугольной рамки в декартовых координатах"""
        if not polygon_corners or len(polygon_corners) < 4:
            return False

        # Преобразуем точку в декартовы координаты
        def polar_to_cartesian(angle_deg, distance):
            # Учитываем ориентацию если есть
            #if hasattr(self, 'orientation_manager'):
                #angle_deg = self.orientation_manager.adjust_angle_raw(angle_deg)
            angle_rad = np.radians(angle_deg)
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)
            return x, y

        # Преобразуем углы полигона в декартовы координаты
        polygon_points = []
        for corner in polygon_corners:
            x, y = polar_to_cartesian(corner['angle'], corner['distance'])
            polygon_points.append((x, y))

        # Преобразуем проверяемую точку
        px, py = polar_to_cartesian(point.angle, point.distance)

        # Находим границы прямоугольника в декартовых координатах
        x_coords = [p[0] for p in polygon_points]
        y_coords = [p[1] for p in polygon_points]
    
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        # Проверяем, находится ли точка внутри прямоугольника
        return (min_x <= px <= max_x) and (min_y <= py <= max_y)





    def quit_app(self):
        """Quit the application"""
        self.log_message("🛑 Shutting down...")
        self.running = False
        
        # Stop LiDAR if active
        if self.lidar:
            try:
                self.lidar.stop()
                self.log_message("✅ LiDAR stopped")
            except:
                pass
        
        # Disable mouse control if active
        if self.mouse_controller and self.mouse_controller.is_active:
            try:
                self.mouse_controller.disable_control()
                self.log_message("🖱️  Mouse control disabled")
            except:
                pass
        
        # Вызов функции очистки файлов
        #cleanup_files()
        
        self.root.quit()
        self.root.destroy()

    #def rotate_clockwise(self):
        #"""Поворот по часовой стрелке"""
        #new_angle = self.orientation_manager.rotate_clockwise(1.0)
        #self.update_orientation_display()
        #self.log_message(f"🧭 Orientation rotated clockwise: {new_angle:.1f}°")

    #def rotate_counterclockwise(self):
        #"""Поворот против часовой стрелки"""
        #new_angle = self.orientation_manager.rotate_counterclockwise(1.0)
        #self.update_orientation_display()
        #self.log_message(f"🧭 Orientation rotated counterclockwise: {new_angle:.1f}°")

    #def reset_orientation(self):
        #"""Сброс ориентации на 0°"""
        #self.orientation_manager.set_orientation(0.0)
        #self.update_orientation_display()
        #self.log_message("🧭 Orientation reset to 0°")

    #def update_orientation_display(self):
        #"""Обновление отображения ориентации"""
        #current_angle = self.orientation_manager.get_orientation()
        #self.orientation_label.config(text=f"{current_angle:.1f}°")
        
        # Измени цвет в зависимости от угла
       # if abs(current_angle) < 0.1:
           #self.orientation_label.config(foreground="green")
        #elif abs(current_angle) > 90:
            #self.orientation_label.config(foreground="red")
        #else:
            #self.orientation_label.config(foreground="blue")

    #def set_north_direction(self, angle_degrees):
        #"""Устанавливает направление севера"""
        #self.orientation_manager.set_orientation(angle_degrees)
        #self.north_direction_label.config(text=f"{angle_degrees:.0f}°")
        #self.log_message(f"🧭 North direction set to {angle_degrees:.0f}°")
    
        # Обновляем mouse controller если он существует
        #if self.mouse_controller:
            #self.mouse_controller.north_angle = angle_degrees

    #def save_north_calibration(self):
        #"""Сохраняет калибровку севера в файл"""
        #try:
            #current_angle = self.orientation_manager.get_orientation()
            #config_data = {
                #'north_offset': current_angle,
                #'timestamp': time.time()
            #}
            #with open('north_calibration.json', 'w') as f:
                #json.dump(config_data, f, indent=2)
        #except Exception as e:
            #self.log_message(f"⚠️ Error saving north calibration: {e}")

    #def load_north_calibration(self):
        #"""Загружает калибровку севера из файла"""
        #try:
            #if os.path.exists('north_calibration.json'):
                #with open('north_calibration.json', 'r') as f:
                    #config_data = json.load(f)
                    #north_offset = config_data.get('north_offset', 0.0)
                    #self.orientation_manager.set_orientation(north_offset)
                    #self.log_message(f"🧭 Loaded north calibration: {north_offset:.1f}°")
        #except Exception as e:
            #self.log_message(f"⚠️ Error loading north calibration: {e}")

    #def load_north_direction(self):
        #"""Загружает направление севера из файла"""
        #try:
            #if os.path.exists('north_direction.json'):
                #with open('north_direction.json', 'r') as f:
                    #config_data = json.load(f)
                    #north_angle = config_data.get('north_angle', 0.0)
                    #self.orientation_manager.set_orientation(north_angle)
                    #self.north_direction_label.config(text=f"{north_angle:.0f}°")
                    #self.log_message(f"🧭 Loaded north direction: {north_angle:.0f}°")
        #except Exception as e:
            #self.log_message(f"⚠️ Error loading north direction: {e}")
            #self.orientation_manager.set_orientation(0.0)

def main(): 
    root = tk.Tk()
    app = LiDARApp(root)
    
    # Handle window closing
    def on_closing():
        app.quit_app()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
