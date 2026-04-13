"""
Контроллер приложения — центральная бизнес-логика LiDAR Touch Controller.

Отвечает за:
- Инициализацию и управление лидаром
- Создание фоновой карты
- Профилирование шума
- Калибровку углов
- Запуск/остановку цикла детекции
- Управление мышью (делегирование MouseController)
"""

import threading
import time
import traceback
from typing import List, Optional, Callable, Dict, Any
from lidar_sdk import LidarM1, Point
from calibration import LidarCalibrator
from detection import TouchDetector, ThresholdConfig
from detection.fast_detector import FastTouchDetector
from config import config
from noise_filter.noise_profiler import NoiseProfiler
from noise_filter.noise_filter import NoiseFilter
from persistent_noise_filter.persistent_noise_manager import PersistentNoiseManager
from noise_filter.corner_calibration import CornerCalibrator

try:
    from mouse_controller import MouseController
    MOUSE_CONTROL_AVAILABLE = True
except ImportError:
    MOUSE_CONTROL_AVAILABLE = False


class AppController:
    """
    Центральный контроллер приложения.
    Содержит всю бизнес-логику, не зависит от GUI.
    """

    def __init__(self,
                 on_log: Callable[[str], None],
                 on_status_update: Callable[[str, str, str], None],
                 on_scan_ready: Callable[[List[Point], List[Point]], None],
                 on_plot_update: Callable[[Optional[List[Point]], Optional[List[Point]]], None]):
        """
        Args:
            on_log: Callback для логирования (message)
            on_status_update: Callback для обновления статусов (status_name, text, color)
            on_scan_ready: Callback при готовности скана + точек (scan, touch_points)
            on_plot_update: Callback для обновления графика (scan, touch_points)
        """
        self.on_log = on_log
        self.on_status_update = on_status_update
        self.on_scan_ready = on_scan_ready
        self.on_plot_update = on_plot_update

        # Компоненты
        self.lidar: Optional[LidarM1] = None
        self.detector: Optional[TouchDetector] = None
        self.fast_detector: Optional[FastTouchDetector] = None
        self.noise_filter: Optional[NoiseFilter] = None
        self.persistent_noise_manager: Optional[PersistentNoiseManager] = None
        self.mouse_controller = None
        self.background_map = None
        self.corners = None

        # Состояние
        self.running = False
        self._last_plot_update = time.time()
        self.detection_thread: Optional[threading.Thread] = None

        # Файлы
        self.background_filename = "background.npz"
        self.projection_corners_file = "projection_corners.json"

    # ─── Инициализация лидара ───────────────────────────────────────────

    def init_lidar(self) -> bool:
        """Инициализирует и подключает лидар."""
        try:
            self.on_log("🔌 Connecting to LiDAR...")
            self.lidar = LidarM1()

            if not self.lidar.connect():
                raise ConnectionError("Failed to connect to LiDAR!")

            self.lidar.start()
            self.on_status_update("lidar", "Connected", "green")
            self.on_log("✅ LiDAR connected and started")

            # Persistent noise manager
            self.persistent_noise_manager = PersistentNoiseManager()
            self.on_log("📂 Persistent noise manager initialized")

            return True
        except Exception as e:
            self.on_status_update("lidar", "Connection Failed", "red")
            self.on_log(f"❌ LiDAR connection error: {e}")
            return False

    def stop_lidar(self):
        """Останавливает лидар."""
        if self.lidar:
            try:
                self.lidar.stop()
                self.on_log("✅ LiDAR stopped")
            except Exception:
                pass

    # ─── Фоновая карта ──────────────────────────────────────────────────

    def create_background_map(self, num_scans: int = 10) -> bool:
        """
        Создаёт фоновую карту из num_scans сканов.
        Возвращает True при успехе.
        """
        if not self.lidar:
            self.on_log("⚠️  LiDAR not connected")
            return False

        try:
            self.on_log(f"📏 Creating background map ({num_scans} scans)...")
            collected_scans = []
            scan_count = 0

            def on_scan_received(scan):
                nonlocal scan_count
                collected_scans.append(scan)
                scan_count += 1
                self.on_log(f"📸 Collected scan {scan_count}/{num_scans}")
                if scan_count >= num_scans:
                    self.lidar.on_scan_complete = None

            self.lidar.on_scan_complete = on_scan_received

            timeout = time.time() + 30
            while scan_count < num_scans and time.time() < timeout:
                time.sleep(0.05)

            if scan_count < num_scans:
                self.on_log("❌ Background creation timed out")
                return False

            self.on_log("📊 Processing collected data...")
            calibrator = LidarCalibrator()
            self.background_map = calibrator.process_scans_to_background(collected_scans)
            calibrator.save_background(self.background_filename, self.background_map)

            self.on_status_update("background", "Created", "green")
            self.on_log(f"💾 Background map saved to '{self.background_filename}'")

            self.detector = TouchDetector(self.background_map, ThresholdConfig.sensitive())
            self.fast_detector = FastTouchDetector(self.background_map, threshold_mm=50.0)
            self.on_log("🔍 Touch detector initialized")

            self.on_plot_update(self.background_map, None)
            self.check_system_readiness()
            return True

        except Exception as e:
            self.on_log(f"❌ Background creation error: {e}")
            return False

    # ─── Профилирование шума ────────────────────────────────────────────

    def profile_noise(self, duration: int = 10) -> bool:
        """Профилирует шум в течение duration секунд."""
        if not self.lidar or not self.detector:
            self.on_log("⚠️  LiDAR or detector not ready")
            return False

        try:
            self.on_log(f"📊 Profiling noise ({duration} seconds)...")
            self.on_log("Ensure NO ONE touches the wall!")

            profiler = NoiseProfiler(duration_seconds=duration)
            profiler.start_noise_collection(self.lidar, self.detector)

            self.noise_filter = NoiseFilter(profiler.noise_profile, self.persistent_noise_manager)
            self.on_status_update("noise", "Profiled", "green")
            self.on_log("✅ Noise profiling completed")
            self.check_system_readiness()
            return True

        except Exception as e:
            self.on_log(f"❌ Noise profiling error: {e}")
            return False

    # ─── Калибровка углов ───────────────────────────────────────────────

    def calibrate_corner(self, corner_name: str) -> bool:
        """Калибрует один угол."""
        if not self.lidar or not self.detector or not self.noise_filter:
            self.on_log("⚠️  LiDAR, detector, or noise filter not ready")
            return False

        try:
            corner_calibrator = CornerCalibrator(self.noise_filter)
            result = corner_calibrator.calibrate_corner(
                self.lidar, self.detector, corner_name, log_callback=self.on_log
            )

            if result:
                self.on_log(f"✅ {corner_name} corner calibrated successfully")
                self.update_corners(result)

                if self.mouse_controller and self.corners:
                    self.mouse_controller.set_projection_corners(self.corners)
                    self.on_status_update("mouse", "Updated", "green")
                    self.on_log("🖱️  Mouse controller updated with new corner")
                return True
            else:
                self.on_log(f"❌ {corner_name} corner calibration failed")
                return False

        except Exception as e:
            self.on_log(f"❌ {corner_name} corner calibration error: {e}")
            return False

    def calibrate_all_corners(self) -> bool:
        """Калибрует все 4 угла."""
        if not self.lidar or not self.detector or not self.noise_filter:
            self.on_log("⚠️  LiDAR, detector, or noise filter not ready")
            return False

        try:
            corner_calibrator = CornerCalibrator(self.noise_filter)
            corners = corner_calibrator.calibrate_all_corners(
                self.lidar, self.detector, log_callback=self.on_log
            )

            if corners:
                self.corners = corners
                self.on_status_update("corners", "Calibrated", "green")
                self.on_log(f"✅ All corners calibrated and saved")

                # Сохраняем
                import json
                corners_data = {
                    'timestamp': time.time(),
                    'corners': corners
                }
                with open(self.projection_corners_file, 'w', encoding='utf-8') as f:
                    json.dump(corners_data, f, ensure_ascii=False, indent=2)

                if self.mouse_controller:
                    self.mouse_controller.set_projection_corners(corners)
                    self.on_status_update("mouse", "Ready", "green")
                    self.on_log("🖱️  Mouse controller updated with all corners")

                self.on_plot_update(None, None)  # Обновить график
                return True
            else:
                self.on_log("❌ All corners calibration failed")
                return False

        except Exception as e:
            self.on_log(f"❌ All corners calibration error: {e}")
            return False

    # ─── Управление углами ──────────────────────────────────────────────

    def update_corners(self, new_corner: dict):
        """Обновляет или добавляет угол."""
        if not self.corners:
            self.corners = []

        corner_updated = False
        for i, corner in enumerate(self.corners):
            if corner['name'] == new_corner['name']:
                self.corners[i] = new_corner
                corner_updated = True
                self.on_log(f"🔄 Угол {new_corner['name']} обновлен")
                break

        if not corner_updated:
            self.corners.append(new_corner)
            self.on_log(f"➕ Добавлен новый угол: {new_corner['name']}")

        # Удаляем дубликаты
        unique_corners = []
        seen_names = set()
        for corner in self.corners:
            if corner['name'] not in seen_names:
                unique_corners.append(corner)
                seen_names.add(corner['name'])
        self.corners = unique_corners

        # Сохраняем
        import json
        corners_data = {
            'timestamp': time.time(),
            'corners': self.corners
        }
        try:
            with open(self.projection_corners_file, 'w', encoding='utf-8') as f:
                json.dump(corners_data, f, ensure_ascii=False, indent=2)

            if len(self.corners) == 4:
                self.on_status_update("corners", "Complete", "green")
            else:
                self.on_status_update("corners", f"{len(self.corners)}/4", "orange")

            self.on_log(f"💾 Сохранены данные углов ({len(self.corners)} шт.)")
        except Exception as e:
            self.on_log(f"❌ Ошибка сохранения углов: {e}")

        self.on_plot_update(None, None)
        self.check_system_readiness()

    # ─── Загрузка данных ────────────────────────────────────────────────

    def load_existing_data(self):
        """Загружает background, noise profile, corners если существуют."""
        from calibration.file_io import load_background_from_file

        loaded_anything = False

        # Background
        try:
            background_data = load_background_from_file(self.background_filename)
            self.background_map = background_data[0] if isinstance(background_data, tuple) else background_data
            self.on_status_update("background", "Loaded", "green")
            self.on_log(f"💾 Loaded existing background map from '{self.background_filename}'")
            loaded_anything = True

            if not self.detector and self.background_map:
                self.detector = TouchDetector(self.background_map, ThresholdConfig.sensitive())
                self.fast_detector = FastTouchDetector(self.background_map, threshold_mm=50.0)
                self.on_log("🔍 Touch detector initialized")
        except Exception:
            pass

        # Noise profile
        try:
            noise_profiler = NoiseProfiler()
            noise_profiler.load_noise_profile()
            if noise_profiler.noise_profile:
                self.noise_filter = NoiseFilter(noise_profiler.noise_profile, self.persistent_noise_manager)
                self.on_status_update("noise", "Loaded", "green")
                self.on_log("📊 Noise profile loaded")
                loaded_anything = True
        except Exception as e:
            self.on_log(f"⚠️  Could not load noise profile: {e}")

        # Corners
        try:
            import json
            with open(self.projection_corners_file, 'r') as f:
                data = json.load(f)
                self.corners = data['corners']
                self.on_status_update("corners", "Loaded", "green")
                self.on_log(f"💾 Loaded existing corners from '{self.projection_corners_file}'")
                loaded_anything = True
        except Exception:
            pass

        if loaded_anything:
            self.check_system_readiness()
            self.on_plot_update(None, None)

    # ─── Инициализация мыши ─────────────────────────────────────────────

    def init_mouse(self) -> bool:
        """Инициализирует MouseController."""
        if not MOUSE_CONTROL_AVAILABLE:
            self.on_status_update("mouse", "Not Available", "red")
            return False

        try:
            self.mouse_controller = MouseController()
            self.on_status_update("mouse", "Initialized", "orange")
            self.on_log("🖱️  Mouse controller initialized")

            if self.corners:
                self.mouse_controller.set_projection_corners(self.corners)
                self.on_status_update("mouse", "Ready", "green")
                self.on_log("🖱️  Mouse controller auto-configured with loaded corners")

            return True
        except Exception as e:
            self.on_log(f"⚠️  Mouse controller initialization failed: {e}")
            self.on_status_update("mouse", "Init Failed", "red")
            return False

    # ─── Управление мышью ───────────────────────────────────────────────

    def enable_mouse(self) -> bool:
        """Включает управление мышью."""
        if not self.mouse_controller:
            self.on_log("⚠️  Mouse controller not initialized")
            return False

        try:
            self.mouse_controller.enable_control()
            return True
        except Exception as e:
            self.on_log(f"❌ Error enabling mouse control: {e}")
            return False

    def disable_mouse(self) -> bool:
        """Выключает управление мышью."""
        if not self.mouse_controller:
            return False

        try:
            self.mouse_controller.disable_control()
            return True
        except Exception as e:
            self.on_log(f"❌ Error disabling mouse control: {e}")
            return False

    # ─── Проверка готовности ────────────────────────────────────────────

    def check_system_readiness(self) -> bool:
        """Проверяет готовность системы."""
        if self.lidar and self.detector and self.noise_filter:
            self.on_log("✅ System ready for detection (corners optional)")
            return True
        return False

    # ─── Запуск/остановка детекции ──────────────────────────────────────

    def start_detection(self) -> bool:
        """Запускает цикл детекции в отдельном потоке."""
        if not self.check_system_readiness():
            self.on_log("⚠️  System not ready for detection")
            return False

        if self.running:
            return True

        self.running = True
        self.on_log("🔄 Starting real-time detection...")

        if self.mouse_controller:
            if not self.enable_mouse():
                self.running = False
                return False
            self.on_log("🖱️  Mouse control enabled")

        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()
        return True

    def stop_detection(self):
        """Останавливает цикл детекции."""
        self.running = False

        if self.mouse_controller and getattr(self.mouse_controller, 'is_active', False):
            self.disable_mouse()
            self.on_log("🖱️  Mouse control disabled")

        if self.detection_thread and self.detection_thread.is_alive():
            self.detection_thread.join(timeout=2)

        self.on_log("⏹️ Detection stopped")

    def _detection_loop(self):
        """Основной цикл детекции — минимальная задержка."""
        try:
            last_scan_num = -1

            while self.running:
                current_scan = self.lidar.get_last_scan()
                scan_num = self.lidar.get_scan_counter()

                # Пропускаем если скан тот же самый
                if scan_num == last_scan_num:
                    time.sleep(0.001)  # 1мс — не забиваем CPU
                    continue

                last_scan_num = scan_num

                if current_scan and len(current_scan) >= 5:
                    # Быстрая проверка качества скана
                    angles = [p.angle for p in current_scan]
                    if len(angles) > 1:
                        angle_range = max(angles) - min(angles)
                        if angle_range < 3:
                            continue

                    # Выравнивание скана к 0° + сортировка
                    first_point_angle = current_scan[0].angle
                    shift = first_point_angle
                    corrected_scan = sorted(current_scan, key=lambda p: p.angle)
                    for p in corrected_scan:
                        p.angle = (p.angle - shift) % 360

                    # ─── Быстрый путь для мыши (LUT, без группировок) ───
                    touch_point = None
                    if self.fast_detector:
                        touch_point = self.fast_detector.detect_single_point(corrected_scan)

                        # Фильтрация постоянного шума (только если есть кандидат)
                        if touch_point and self.persistent_noise_manager:
                            if self.persistent_noise_manager.is_persistent_noise(touch_point):
                                touch_point = None

                        # ПРОВЕРКА: точка внутри углов проекции
                        if touch_point and self.corners:
                            if not self._point_in_polygon_fast(touch_point, self.corners):
                                touch_point = None

                    # Обновление графика — каждые 33мс
                    now = time.time()
                    if now - self._last_plot_update > 0.033:
                        self.on_plot_update(None, [touch_point] if touch_point else [])
                        self._last_plot_update = now

                    # Управление мышью — мгновенно, каждый скан
                    if self.mouse_controller and self.mouse_controller.is_active:
                        if touch_point:
                            self.mouse_controller.update_touch_state(True, touch_point)
                        else:
                            self.mouse_controller.update_touch_state(False)

                # БЕЗ sleep — сразу к следующему скану

        except Exception as e:
            self.on_log(f"❌ Detection error: {e}")
            traceback.print_exc()
            self.running = False

            if self.mouse_controller:
                try:
                    self.mouse_controller.disable_control()
                except Exception:
                    pass

    # ─── Утилиты ────────────────────────────────────────────────────────

    @staticmethod
    def _point_in_polygon_fast(point: Point, polygon_corners: List[dict]) -> bool:
        """Проверка точки внутри прямоугольной рамки в декартовых координатах."""
        import numpy as np
        if not polygon_corners or len(polygon_corners) < 4:
            return False

        def polar_to_cartesian(angle_deg, distance):
            angle_rad = np.radians(angle_deg)
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)
            return x, y

        polygon_points = []
        for corner in polygon_corners:
            x, y = polar_to_cartesian(corner['angle'], corner['distance'])
            polygon_points.append((x, y))

        px, py = polar_to_cartesian(point.angle, point.distance)

        x_coords = [p[0] for p in polygon_points]
        y_coords = [p[1] for p in polygon_points]

        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        return (min_x <= px <= max_x) and (min_y <= py <= max_y)
