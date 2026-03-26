# gui/gui_manager.py
"""Менеджер графического интерфейса для LiDAR приложения"""

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

class GUIManger:
    """Менеджер графического интерфейса"""
    
    def __init__(self, root, app_instance):
        self.root = root
        self.app = app_instance
        self.setup_gui()
    
    def setup_gui(self):
        """Настройка графического интерфейса"""
        # Configure grid weights for resizing
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
    
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=3)
        main_frame.grid_columnconfigure(1, weight=1)
    
        self._setup_plot_frame(main_frame)
        self._setup_control_panel(main_frame)
        self._setup_log_area()
    
    def _setup_plot_frame(self, parent):
        """Настройка фрейма с графиком"""
        plot_frame = ttk.LabelFrame(parent, text="LiDAR Real-time View")
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
    
    def _setup_control_panel(self, parent):
        """Настройка панели управления"""
        control_frame = ttk.LabelFrame(parent, text="Control Panel")
        control_frame.grid(row=0, column=1, sticky="nsew")
        control_frame.grid_columnconfigure(0, weight=1)
        
        # Status indicators
        self._setup_status_frame(control_frame)
        
        # Orientation frame
        self._setup_orientation_frame(control_frame)
        
        # North direction calibration frame
        self._setup_north_frame(control_frame)
        
        # Calibration buttons
        self._setup_calibration_frame(control_frame)
        
        # Mouse control buttons
        self._setup_mouse_control_frame(control_frame)
        
        # Control buttons
        self._setup_control_buttons(control_frame)
    
    def _setup_status_frame(self, parent):
        """Настройка фрейма статусов"""
        status_frame = ttk.LabelFrame(parent, text="Status")
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
    
    def _setup_orientation_frame(self, parent):
        """Настройка фрейма ориентации"""
        orientation_frame = ttk.LabelFrame(parent, text="Orientation Control")
        orientation_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        orientation_frame.grid_columnconfigure(0, weight=1)
        orientation_frame.grid_columnconfigure(1, weight=1)
        orientation_frame.grid_columnconfigure(2, weight=1)
    
        ttk.Label(orientation_frame, text="Current Orientation:").grid(row=0, column=0, sticky="w", padx=5)
        self.orientation_label = ttk.Label(orientation_frame, text="0.0°", foreground="blue")
        self.orientation_label.grid(row=0, column=1, sticky="w", padx=5)
    
        self.rotate_ccw_button = ttk.Button(orientation_frame, text="↺ -1°", 
                                       command=self.app.rotate_counterclockwise)
        self.rotate_ccw_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
    
        self.reset_orientation_button = ttk.Button(orientation_frame, text="Reset 0°", 
                                              command=self.app.reset_orientation)
        self.reset_orientation_button.grid(row=1, column=1, sticky="ew", padx=2, pady=2)
    
        self.rotate_cw_button = ttk.Button(orientation_frame, text="↻ +1°", 
                                      command=self.app.rotate_clockwise)
        self.rotate_cw_button.grid(row=1, column=2, sticky="ew", padx=2, pady=2)
    
    def _setup_north_frame(self, parent):
        """Настройка фрейма направления севера"""
        north_frame = ttk.LabelFrame(parent, text="Set North Direction")
        north_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        north_frame.grid_columnconfigure(0, weight=1)
        north_frame.grid_columnconfigure(1, weight=1)
        north_frame.grid_columnconfigure(2, weight=1)
    
        # Кнопки направлений (как компас)
        btn_frame = ttk.Frame(north_frame)
        btn_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=5)
        btn_frame.grid_columnconfigure(0, weight=1)
        btn_frame.grid_columnconfigure(1, weight=1)
        btn_frame.grid_columnconfigure(2, weight=1)
    
        # Верхняя кнопка (Север)
        self.north_btn = ttk.Button(btn_frame, text="↑\nNorth", 
                               command=lambda: self.app.set_north_direction(0))
        self.north_btn.grid(row=0, column=1, padx=2, pady=2, sticky="ew")
    
        # Левая кнопка (Запад)
        self.west_btn = ttk.Button(btn_frame, text="←\nWest", 
                                  command=lambda: self.app.set_north_direction(270))
        self.west_btn.grid(row=1, column=0, padx=2, pady=2, sticky="ew")
    
        # Правая кнопка (Восток)
        self.east_btn = ttk.Button(btn_frame, text="→\nEast", 
                              command=lambda: self.app.set_north_direction(90))
        self.east_btn.grid(row=1, column=2, padx=2, pady=2, sticky="ew")
    
        # Нижняя кнопка (Юг)
        self.south_btn = ttk.Button(btn_frame, text="↓\nSouth", 
                               command=lambda: self.app.set_north_direction(180))
        self.south_btn.grid(row=2, column=1, padx=2, pady=2, sticky="ew")
    
        # Отображение текущего севера
        ttk.Label(north_frame, text="Current North:").grid(row=1, column=0, sticky="w", padx=5)
        self.north_direction_label = ttk.Label(north_frame, text="0°", foreground="blue")
        self.north_direction_label.grid(row=1, column=1, sticky="w", padx=5)
    
    def _setup_calibration_frame(self, parent):
        """Настройка фрейма калибровки"""
        calib_frame = ttk.LabelFrame(parent, text="Calibration")
        calib_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        calib_frame.grid_columnconfigure(0, weight=1)
    
        self.bg_button = ttk.Button(calib_frame, text="Create Background Map", 
                               command=self.app.create_background_map)
        self.bg_button.grid(row=0, column=0, sticky="ew", padx=5, pady=2)
    
        self.noise_button = ttk.Button(calib_frame, text="Profile Noise", 
                                  command=self.app.profile_noise)
        self.noise_button.grid(row=1, column=0, sticky="ew", padx=5, pady=2)
    
        # Corner calibration buttons
        corner_frame = ttk.LabelFrame(calib_frame, text="Corner Calibration")
        corner_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=5)
        corner_frame.grid_columnconfigure(0, weight=1)
        corner_frame.grid_columnconfigure(1, weight=1)
    
        self.top_left_button = ttk.Button(corner_frame, text="Top Left", 
                                     command=lambda: self.app.calibrate_corner("top_left"))
        self.top_left_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
    
        self.top_right_button = ttk.Button(corner_frame, text="Top Right", 
                                      command=lambda: self.app.calibrate_corner("top_right"))
        self.top_right_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
    
        self.bottom_left_button = ttk.Button(corner_frame, text="Bottom Left", 
                                        command=lambda: self.app.calibrate_corner("bottom_left"))
        self.bottom_left_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
    
        self.bottom_right_button = ttk.Button(corner_frame, text="Bottom Right", 
                                         command=lambda: self.app.calibrate_corner("bottom_right"))
        self.bottom_right_button.grid(row=1, column=1, sticky="ew", padx=2, pady=2)
    
        self.all_corners_button = ttk.Button(calib_frame, text="Calibrate All Corners", 
                                        command=self.app.calibrate_all_corners)
        self.all_corners_button.grid(row=3, column=0, sticky="ew", padx=5, pady=2)
        
        # Countdown label for corner calibration
        self.countdown_label = ttk.Label(calib_frame, text="", foreground="red")
        self.countdown_label.grid(row=4, column=0, pady=5)
    
    def _setup_mouse_control_frame(self, parent):
        """Настройка фрейма управления мышью"""
        from app import MOUSE_CONTROL_AVAILABLE
        if MOUSE_CONTROL_AVAILABLE:
            mouse_frame = ttk.LabelFrame(parent, text="Mouse Control")
            mouse_frame.grid(row=4, column=0, sticky="ew", padx=10, pady=5)
            mouse_frame.grid_columnconfigure(0, weight=1)
            mouse_frame.grid_columnconfigure(1, weight=1)
        
            self.mouse_enable_button = ttk.Button(mouse_frame, text="Enable Mouse", 
                                                 command=self.app.enable_mouse_control)
            self.mouse_enable_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        
            self.mouse_disable_button = ttk.Button(mouse_frame, text="Disable Mouse", 
                                              command=self.app.disable_mouse_control, 
                                              state="disabled")
            self.mouse_disable_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
    
    def _setup_control_buttons(self, parent):
        """Настройка кнопок управления"""
        button_frame = ttk.Frame(parent)
        button_frame.grid(row=5, column=0, sticky="ew", padx=10, pady=5)
        button_frame.grid_columnconfigure(0, weight=1)
        button_frame.grid_columnconfigure(1, weight=1)
    
        self.start_button = ttk.Button(button_frame, text="Start Detection", 
                                  command=self.app.start_detection)
        self.start_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
    
        self.stop_button = ttk.Button(button_frame, text="Stop Detection", 
                                 command=self.app.stop_detection, state="disabled")
        self.stop_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
    
        self.quit_button = ttk.Button(parent, text="Quit", command=self.app.quit_app)
        self.quit_button.grid(row=6, column=0, sticky="ew", padx=10, pady=5)
    
    def _setup_log_area(self):
        """Настройка области логов"""
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)
    
        self.log_text = tk.Text(log_frame, height=8)
        self.log_text.grid(row=0, column=0, sticky="nsew")
    
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
    
    def update_plot(self, scan=None, touch_points=None, corners=None):
        """Обновление графика"""
        try:
            # Clear the axis
            self.ax.clear()
        
            # Set up the polar plot
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            self.ax.set_title("LiDAR Scan (Polar Coordinates)")
        
            # Plot background if available
            if hasattr(self.app, 'background_map') and self.app.background_map:
                angles = list(self.app.background_map.keys())
                distances = list(self.app.background_map.values())
                self.ax.plot(np.radians(angles), distances, 'b-', linewidth=1, label='Background')
        
            # Plot current scan data if provided
            if scan:
                angles = [point.angle for point in scan]
                distances = [point.distance for point in scan]
                # Plot all scan points in blue
                #self.ax.scatter(np.radians(angles), distances, c='blue', s=10, alpha=0.6, label='Scan Points')
        
            # Plot touch points if provided (ТОЛЬКО ТЕ, КОТОРЫЕ ВНУТРИ РАМКИ)
            if touch_points and corners:
                # Фильтруем только точки внутри рабочего поля
                filtered_touch_points = []
                for point in touch_points:
                    if self.app.point_in_polygon(point, corners):
                        filtered_touch_points.append(point)
            
                if filtered_touch_points:
                    touch_angles = [point.angle for point in filtered_touch_points]
                    touch_distances = [point.distance for point in filtered_touch_points]
                    self.ax.scatter(np.radians(touch_angles), touch_distances, 
                               c='red', s=50, alpha=0.8, marker='o', label='Touch Points')
            elif touch_points and not corners:
                # Если углы не заданы, показываем все касания
                touch_angles = [point.angle for point in touch_points]
                touch_distances = [point.distance for point in touch_points]
                self.ax.scatter(np.radians(touch_angles), touch_distances, 
                           c='red', s=50, alpha=0.8, marker='o', label='Touch Points')
        
            # Plot corners if available
            if corners:
                corner_angles = [corner['angle'] for corner in corners]
                corner_distances = [corner['distance'] for corner in corners]
                self.ax.scatter(np.radians(corner_angles), corner_distances, 
                           c='orange', s=100, alpha=0.9, marker='s', 
                           label='Projection Corners', edgecolors='black', linewidth=2)
        
            # Add legend
            self.ax.legend()
        
            # Refresh the canvas
            self.canvas.draw()
        
        except Exception as e:
            self.app.log_message(f"Plot update error: {e}")

