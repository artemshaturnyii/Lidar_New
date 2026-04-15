"""
LiDAR Touch Controller — главное приложение.

Тонкий фасад: связывает GUI, контроллер и менеджер графиков.
Вся бизнес-логика вынесена в app_controller.py.
"""

import tkinter as tk
import threading
import time
import queue
from typing import List, Optional

from lidar_sdk import Point
from app_controller import AppController, MOUSE_CONTROL_AVAILABLE
from plot_manager import PlotManager
from gui_builder import GuiBuilder


class LiDARApp:
    """Главное приложение — связывает GUI и контроллер."""

    def __init__(self, root):
        self.root = root
        self.root.title("LiDAR Control Panel")
        self.root.geometry("2000x1200")

        # Очередь обновления графиков
        self.plot_queue = queue.Queue()
        self.plot_thread = threading.Thread(target=self._plot_worker, daemon=True)
        self.plot_thread.start()

        # Состояние мыши
        self.mouse_control_active = False

        # Создаём контроллер
        self.controller = AppController(
            on_log=self._on_log,
            on_status_update=self._on_status_update,
            on_scan_ready=self._on_scan_ready,
            on_plot_update=self._on_plot_request,
        )

        # Создаём GUI (callbacks указывают на методы этого класса)
        self.gui = GuiBuilder(
            root,
            callbacks={
                'on_create_background': self._on_create_background,
                'on_profile_noise': self._on_profile_noise,
                'on_calibrate_corner': self._on_calibrate_corner,
                'on_calibrate_all_corners': self._on_calibrate_all_corners,
                'on_toggle_mouse': self._on_toggle_mouse,
                'on_quit': self._on_quit,
                'mouse_available': MOUSE_CONTROL_AVAILABLE,
            }
        )

        # Создаём менеджер графиков
        self.plot_mgr = PlotManager(parent_frame=self.gui.get_plot_frame())

        # Инициализация лидара
        self._init_lidar_async()

    # ─── Plot worker ────────────────────────────────────────────────────

    def _plot_worker(self):
        """Отдельный поток для обновления графиков."""
        while True:
            try:
                data = self.plot_queue.get(timeout=0.01)
                if data is None:
                    break
                scan, touch_points = data
                self.plot_mgr.update_plot(
                    background_map=self.controller.background_map,
                    scan=scan,
                    touch_points=touch_points,
                    corners=self.controller.corners,
                )
            except queue.Empty:
                pass

    def _on_plot_request(self, scan, touch_points):
        """Запрос на обновление графика (из контроллера)."""
        try:
            while not self.plot_queue.empty():
                try:
                    self.plot_queue.get_nowait()
                except queue.Empty:
                    break
            self.plot_queue.put((scan, touch_points), block=False)
        except Exception:
            pass

    # ─── Callbacks от контроллера ───────────────────────────────────────

    def _on_log(self, message: str):
        """Логирование (вызывается из контроллера)."""
        self.gui.append_log(message)

    def _on_status_update(self, status_name: str, text: str, color: str):
        """Обновление статуса (вызывается из контроллера)."""
        self.gui.update_status(status_name, text, color)

    def _on_scan_ready(self, scan: List[Point], touch_points: List[Point]):
        """Скан готов (вызывается из контроллера)."""
        self._on_plot_request(scan, touch_points)

    # ─── Callbacks от GUI (пользователь нажал кнопку) ───────────────────

    def _on_create_background(self):
        """Создание фоновой карты."""
        def task():
            self.gui.set_button_state("background", "disabled")
            success = self.controller.create_background_map(num_scans=10)
            self.gui.set_button_state("background", "normal")
        threading.Thread(target=task, daemon=True).start()

    def _on_profile_noise(self):
        """Профилирование шума."""
        def task():
            self.gui.set_button_state("noise", "disabled")
            success = self.controller.profile_noise(duration=10)
            self.gui.set_button_state("noise", "normal")
        threading.Thread(target=task, daemon=True).start()

    def _on_calibrate_corner(self, corner_name: str):
        """Калибровка одного угла с обратным отсчётом."""
        def task():
            btn_map = {
                "top_left": "top_left",
                "top_right": "top_right",
                "bottom_left": "bottom_left",
                "bottom_right": "bottom_right",
            }
            btn_name = btn_map.get(corner_name)

            self.gui.append_log(f"⏳ Preparing to calibrate {corner_name} corner...")
            self.gui.append_log("Please move to the calibration position within 5 seconds...")

            if btn_name:
                self.gui.set_button_state(btn_name, "disabled")

            for i in range(5, 0, -1):
                self.gui.update_countdown(f"Starting calibration in {i} seconds...")
                self.gui.append_log(f"⏳ {i}...")
                time.sleep(1)
            self.gui.update_countdown("")

            self.gui.append_log(f"🎯 Calibrating {corner_name} corner...")
            success = self.controller.calibrate_corner(corner_name)

            if btn_name:
                self.gui.set_button_state(btn_name, "normal")

        threading.Thread(target=task, daemon=True).start()

    def _on_calibrate_all_corners(self):
        """Калибровка всех углов."""
        def task():
            buttons = ["top_left", "top_right", "bottom_left", "bottom_right", "all_corners"]
            for btn in buttons:
                self.gui.set_button_state(btn, "disabled")

            self.gui.append_log("⏳ Preparing to calibrate all corners. Start with TOP RIGHT corner")
            self.gui.append_log("Please move to the first calibration position within 5 seconds...")

            for i in range(5, 0, -1):
                self.gui.update_countdown(f"Starting calibration in {i} seconds...")
                self.gui.append_log(f"⏳ {i}...")
                time.sleep(1)
            self.gui.update_countdown("")

            self.gui.append_log("🎯 Calibrating all corners...")
            success = self.controller.calibrate_all_corners()

            for btn in buttons:
                self.gui.set_button_state(btn, "normal")

        threading.Thread(target=task, daemon=True).start()

    def _on_toggle_mouse(self):
        """Переключение управления мышью."""
        if not self.mouse_control_active:
            self._start_mouse_control()
        else:
            self._stop_mouse_control()

    def _start_mouse_control(self):
        """Запуск детекции + управление мышью."""
        if not self.controller.check_system_readiness():
            self.gui.append_log("⚠️  System not ready for detection")
            return

        success = self.controller.start_detection()
        if success:
            self.mouse_control_active = True
            self.gui.update_mouse_button("Stop Mouse Control")
            self.gui.update_mouse_status_label("Running", "green")
            self.gui.append_log("🎮 Mouse control started - system is ready!")
        else:
            self.gui.append_log("❌ Failed to start detection")

    def _stop_mouse_control(self):
        """Остановка детекции + управление мышью."""
        self.controller.stop_detection()
        self.mouse_control_active = False
        self.gui.update_mouse_button("Start Mouse Control")
        self.gui.update_mouse_status_label("Stopped", "red")
        self.gui.append_log("⏹️ Mouse control stopped")

    def _on_quit(self):
        """Выход из приложения."""
        self.gui.append_log("🛑 Shutting down...")
        self.controller.stop_detection()
        self.controller.stop_lidar()

        if self.controller.mouse_controller:
            try:
                self.controller.mouse_controller.disable_control()
                self.gui.append_log("🖱️  Mouse control disabled")
            except Exception:
                pass

        self.root.quit()
        self.root.destroy()

    # ─── Инициализация лидара ───────────────────────────────────────────

    def _init_lidar_async(self):
        """Инициализация лидара в отдельном потоке."""
        def init():
            success = self.controller.init_lidar()
            if success:
                # Загрузка существующих данных
                self.controller.load_existing_data()
                # Инициализация мыши
                self.controller.init_mouse()
                # Обновить график с загруженными данными
                self._on_plot_request(None, None)

            if self.controller.mouse_controller:
                if self.controller.corners:
                    self.gui.update_mouse_status_label("Ready", "Blue")

        threading.Thread(target=init, daemon=True).start()


def main():
    root = tk.Tk()
    app = LiDARApp(root)

    def on_closing():
        app._on_quit()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
