"""
Построитель GUI — создание интерфейса LiDAR Touch Controller.

Отвечает за:
- Создание основной структуры окна (фреймы, сетка)
- Размещение статусных индикаторов
- Создание кнопок калибровки
- Создание панели логирования
- Предоставление доступа к GUI элементам для обновления
"""

import tkinter as tk
from tkinter import ttk
from typing import Dict, Any, Optional


class GuiBuilder:
    """Строит GUI компоненты и предоставляет к ним доступ."""

    def __init__(self, root, callbacks: Dict[str, Any]):
        """
        Args:
            root: Root window (tk.Tk)
            callbacks: Словарь callback-функций:
                - on_create_background: вызов при создании фона
                - on_profile_noise: вызов при профилировании шума
                - on_calibrate_corner(corner_name): вызов при калибровке угла
                - on_calibrate_all_corners: вызов при калибровке всех углов
                - on_toggle_mouse: вызов при переключении мыши
                - on_quit: вызов при выходе
        """
        self.root = root
        self.callbacks = callbacks

        # Ссылки на виджеты для обновления
        self.status_labels: Dict[str, ttk.Label] = {}
        self.buttons: Dict[str, ttk.Button] = {}
        self.log_text: Optional[tk.Text] = None
        self.countdown_label: Optional[ttk.Label] = None
        self.mouse_status_label: Optional[ttk.Label] = None
        self.toggle_mouse_button: Optional[ttk.Button] = None

        # Определяем доступность mouse controller
        self.mouse_available = callbacks.get('mouse_available', False)

        self._setup_main_layout()
        self._setup_status_panel()
        self._setup_calibration_panel()
        self._setup_mouse_panel()
        self._setup_log_panel()
        self._setup_quit_button()

    def _setup_main_layout(self):
        """Создаёт основную структуру: plot_frame + control_frame."""
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=0)

        main_frame = ttk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=0)

        self.plot_frame = ttk.LabelFrame(main_frame, text="LiDAR Real-time View")
        self.plot_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.plot_frame.grid_rowconfigure(0, weight=1)
        self.plot_frame.grid_columnconfigure(0, weight=1)

        self.control_frame = ttk.LabelFrame(main_frame, text="Control Panel")
        self.control_frame.grid(row=0, column=1, sticky="nsew")
        self.control_frame.grid_columnconfigure(0, weight=1)

    def _setup_status_panel(self):
        """Создаёт панель статусов."""
        status_frame = ttk.LabelFrame(self.control_frame, text="Status")
        status_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        status_frame.grid_columnconfigure(1, weight=1)

        statuses = [
            ("LiDAR:", "lidar", "Connecting...", "orange"),
            ("Background:", "background", "Not Created", "red"),
            ("Noise Profile:", "noise", "Not Profiled", "red"),
            ("Corners:", "corners", "Not Calibrated", "red"),
            ("Mouse Control:", "mouse", "Not Initialized", "red"),
        ]

        for i, (label_text, key, default_text, default_color) in enumerate(statuses):
            ttk.Label(status_frame, text=label_text).grid(
                row=i, column=0, sticky="w", padx=5, pady=2
            )
            lbl = ttk.Label(status_frame, text=default_text, foreground=default_color)
            lbl.grid(row=i, column=1, sticky="w", padx=5, pady=2)
            self.status_labels[key] = lbl

    def _setup_calibration_panel(self):
        """Создаёт панель калибровки."""
        calib_frame = ttk.LabelFrame(self.control_frame, text="Calibration")
        calib_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        calib_frame.grid_columnconfigure(0, weight=1)

        # Background button
        self.buttons["background"] = ttk.Button(
            calib_frame, text="Create Background Map",
            command=self.callbacks.get('on_create_background', lambda: None)
        )
        self.buttons["background"].grid(row=0, column=0, sticky="ew", padx=5, pady=2)

        # Noise button
        self.buttons["noise"] = ttk.Button(
            calib_frame, text="Profile Noise",
            command=self.callbacks.get('on_profile_noise', lambda: None)
        )
        self.buttons["noise"].grid(row=1, column=0, sticky="ew", padx=5, pady=2)

        # Corner calibration
        corner_frame = ttk.LabelFrame(calib_frame, text="Corner Calibration")
        corner_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=5)
        corner_frame.grid_columnconfigure(0, weight=1)
        corner_frame.grid_columnconfigure(1, weight=1)

        corner_names = ["top_left", "top_right", "bottom_left", "bottom_right"]
        corner_labels = ["Top Left", "Top Right", "Bottom Left", "Bottom Right"]

        for i, (name, label) in enumerate(zip(corner_names, corner_labels)):
            row = i // 2
            col = i % 2
            self.buttons[name] = ttk.Button(
                corner_frame, text=label,
                command=lambda n=name: self.callbacks.get('on_calibrate_corner', lambda _: None)(n)
            )
            self.buttons[name].grid(row=row, column=col, sticky="ew", padx=2, pady=2)

        # Calibrate All button
        self.buttons["all_corners"] = ttk.Button(
            calib_frame, text="Calibrate All Corners",
            command=self.callbacks.get('on_calibrate_all_corners', lambda: None)
        )
        self.buttons["all_corners"].grid(row=3, column=0, sticky="ew", padx=5, pady=2)

        # Countdown label
        self.countdown_label = ttk.Label(calib_frame, text="", foreground="red")
        self.countdown_label.grid(row=4, column=0, pady=5)

    def _setup_mouse_panel(self):
        """Создаёт панель управления мышью."""
        if not self.mouse_available:
            return

        mouse_frame = ttk.LabelFrame(self.control_frame, text="Mouse Control")
        mouse_frame.grid(row=4, column=0, sticky="ew", padx=10, pady=5)
        mouse_frame.grid_columnconfigure(0, weight=1)

        self.toggle_mouse_button = ttk.Button(
            mouse_frame, text="Start Mouse Control",
            command=self.callbacks.get('on_toggle_mouse', lambda: None)
        )
        self.toggle_mouse_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

        self.mouse_status_label = ttk.Label(mouse_frame, text="Stopped", foreground="red")
        self.mouse_status_label.grid(row=1, column=0, sticky="w", padx=5, pady=2)

    def _setup_log_panel(self):
        """Создаёт панель логирования."""
        log_frame = ttk.LabelFrame(self.root, text="Log")
        log_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 10))
        log_frame.grid_rowconfigure(0, weight=1)
        log_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_columnconfigure(1, weight=0)

        self.log_text = tk.Text(log_frame, height=6)
        self.log_text.grid(row=0, column=0, sticky="ew")

        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def _setup_quit_button(self):
        """Создаёт кнопку выхода."""
        self.buttons["quit"] = ttk.Button(
            self.control_frame, text="Quit",
            command=self.callbacks.get('on_quit', lambda: None)
        )
        self.buttons["quit"].grid(row=6, column=0, sticky="ew", padx=10, pady=5)

    # ─── Методы обновления GUI ─────────────────────────────────────────

    def update_status(self, status_name: str, text: str, color: str):
        """Обновляет текст и цвет статусного индикатора."""
        if status_name in self.status_labels:
            self.status_labels[status_name].config(text=text, foreground=color)

    def append_log(self, message: str):
        """Добавляет сообщение в лог."""
        import time
        if self.log_text:
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
            self.log_text.see(tk.END)

    def update_countdown(self, text: str):
        """Обновляет текст обратного отсчёта."""
        if self.countdown_label:
            self.countdown_label.config(text=text)

    def update_mouse_button(self, text: str):
        """Обновляет текст кнопки мыши."""
        if self.toggle_mouse_button:
            self.toggle_mouse_button.config(text=text)

    def update_mouse_status_label(self, text: str, color: str):
        """Обновляет статус мыши."""
        if self.mouse_status_label:
            self.mouse_status_label.config(text=text, foreground=color)

    def set_button_state(self, button_name: str, state: str):
        """Устанавливает состояние кнопки (normal/disabled)."""
        if button_name in self.buttons:
            self.buttons[button_name].config(state=state)

    def get_plot_frame(self):
        """Возвращает plot_frame для размещения canvas."""
        return self.plot_frame
