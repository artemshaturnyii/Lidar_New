"""
Модуль для управления курсором мыши на основе данных LiDAR.
Кроссплатформенный: Win32 API (Windows) / Xlib (Linux).
"""

import platform
import numpy as np
from typing import List, Optional, Tuple
from lidar_sdk import Point
import time


# ─── Платформо-зависимые операции ──────────────────────────────────────

_SYSTEM = platform.system()

if _SYSTEM == "Windows":
    import ctypes
    _user32 = ctypes.windll.user32

    def _get_screen_size() -> Tuple[int, int]:
        return (_user32.GetSystemMetrics(0), _user32.GetSystemMetrics(1))

    def _set_cursor_pos(x: int, y: int) -> None:
        _user32.SetCursorPos(x, y)

    def _mouse_down() -> None:
        _user32.mouse_event(0x0002, 0, 0, 0, 0)  # MOUSEEVENTF_LEFTDOWN

    def _mouse_up() -> None:
        _user32.mouse_event(0x0004, 0, 0, 0, 0)  # MOUSEEVENTF_LEFTUP

elif _SYSTEM == "Linux":
    from Xlib import display, X
    from Xlib.ext.xtest import fake_input

    _dpy = display.Display()
    _root = _dpy.screen().root

    def _get_screen_size() -> Tuple[int, int]:
        return (_root.get_geometry().width, _root.get_geometry().height)

    def _set_cursor_pos(x: int, y: int) -> None:
        _root.warp_pointer(x, y)
        _dpy.sync()

    def _mouse_down() -> None:
        fake_input(_dpy, X.ButtonPress, 1)
        _dpy.sync()

    def _mouse_up() -> None:
        fake_input(_dpy, X.ButtonRelease, 1)
        _dpy.sync()

else:
    raise RuntimeError(f"Неподдерживаемая ОС: {_SYSTEM}")


class MouseController:
    """Контроллер для управления курсором мыши на основе касаний LiDAR.
    Мгновенное перемещение через Win32 SetCursorPos (Win) или Xlib warp_pointer (Linux).
    """

    def __init__(self, screen_width: int = None, screen_height: int = None):
        """Инициализация контроллера мыши"""
        if screen_width is None or screen_height is None:
            self.screen_width, self.screen_height = _get_screen_size()
        else:
            self.screen_width = screen_width
            self.screen_height = screen_height

        self.corners = None
        self.is_active = False
        self.last_mouse_position = None
        self.lidar_corners = None
        self.screen_corners = None
        self.lidar_bbox = None

        # Направление севера
        self.north_angle = 0.0

        # Параметры кликов
        self.click_delay = 0.3
        self.last_click_time = 0

        # Отслеживание состояния касаний
        self.current_touch_state = False
        self.touch_start_time = 0

        # Мёртвая зона — курсор двигается только если смещение > dead_zone_radius px
        self.dead_zone_radius = 50  # пиксели экрана

        # Зафиксированная позиция курсора в пределах мёртвой зоны
        self._anchor_screen_x: Optional[int] = None
        self._anchor_screen_y: Optional[int] = None

        # Гомография для перспективного преобразования (4 угла → экран)
        self._homography: Optional[np.ndarray] = None

        # Предвычисленные константы для map_touch_to_screen
        self._precomputed = {}

    def set_projection_corners(self, corners: List[dict]):
        """Установка углов проекции для калибровки"""
        if corners and len(corners) >= 4:
            self.corners = corners
            self._setup_corner_mapping()
            print(f"✅ Установлены углы проекции для управления мышью ({len(corners)} точек)")
        else:
            print("⚠️  Недостаточно углов для управления мышью")
            self.corners = None
            self.lidar_corners = None
            self.screen_corners = None
            self.lidar_bbox = None
            self._precomputed = {}

    def _setup_corner_mapping(self):
        """Настраивает соответствие углов LiDAR и экрана с гомографией."""
        if not self.corners or len(self.corners) < 4:
            return

        corner_map = {c['name']: c for c in self.corners}

        def polar_to_cartesian(angle_deg, distance):
            corrected_angle = (angle_deg - self.north_angle) % 360
            angle_rad = np.radians(corrected_angle)
            x = distance * np.sin(angle_rad)
            y = distance * np.cos(angle_rad)
            return x, y

        lidar_pts = []
        screen_pts = [
            (0, 0),
            (self.screen_width - 1, 0),
            (self.screen_width - 1, self.screen_height - 1),
            (0, self.screen_height - 1),
        ]

        corner_order = ['top_left', 'top_right', 'bottom_right', 'bottom_left']
        for name in corner_order:
            if name in corner_map:
                c = corner_map[name]
                lidar_pts.append(polar_to_cartesian(c['angle'], c['distance']))

        if len(lidar_pts) < 4:
            print("⚠️  Не все углы найдены — используем упрощённую проекцию")
            self._compute_bbox_fallback(lidar_pts)
            return

        src = np.array(lidar_pts, dtype=np.float64)
        dst = np.array(screen_pts, dtype=np.float64)
        self._homography = self._compute_homography(src, dst)

        x_coords = [p[0] for p in lidar_pts]
        y_coords = [p[1] for p in lidar_pts]
        self.lidar_bbox = {
            'min_x': min(x_coords), 'max_x': max(x_coords),
            'min_y': min(y_coords), 'max_y': max(y_coords),
        }

        self._precomputed['screen_max_x'] = float(self.screen_width - 1)
        self._precomputed['screen_max_y'] = float(self.screen_height - 1)

        print("📏 Гомография вычислена, соответствие углов установлено")

    def _compute_bbox_fallback(self, lidar_pts):
        """Fallback если не все углы найдены — простая проекция."""
        if not lidar_pts:
            return
        x_coords = [p[0] for p in lidar_pts]
        y_coords = [p[1] for p in lidar_pts]
        self.lidar_bbox = {
            'min_x': min(x_coords), 'max_x': max(x_coords),
            'min_y': min(y_coords), 'max_y': max(y_coords),
        }
        self._precomputed['screen_max_x'] = float(self.screen_width - 1)
        self._precomputed['screen_max_y'] = float(self.screen_height - 1)

    @staticmethod
    def _compute_homography(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
        """
        Вычисляет матрицу гомографии 3x3 без OpenCV.
        src: (4, 2) — точки источника (LiDAR в мм)
        dst: (4, 2) — точки назначения (экран в px)
        """
        A = []
        for i in range(4):
            sx, sy = src[i]
            dx, dy = dst[i]
            A.append([sx, sy, 1,  0,  0, 0,  -dx*sx, -dx*sy, -dx])
            A.append([ 0,  0, 0,  sx, sy, 1,  -dy*sx, -dy*sy, -dy])
        A = np.array(A, dtype=np.float64)

        _, _, vh = np.linalg.svd(A)
        h = vh[-1]
        H = h.reshape(3, 3) / h[-1]
        return H

    def map_touch_to_screen(self, touch_point: Point) -> Optional[Tuple[int, int]]:
        """Преобразует точку касания LiDAR в координаты экрана.
        Использует гомографию если доступна, иначе — билинейную проекцию.
        """
        if not self._precomputed:
            return None

        corrected_angle = np.radians(touch_point.angle) - self._precomputed.get('north_rad', 0)
        lx = touch_point.distance * np.sin(corrected_angle)
        ly = touch_point.distance * np.cos(corrected_angle)

        bbox = self.lidar_bbox
        if bbox and not (bbox['min_x'] <= lx <= bbox['max_x'] and bbox['min_y'] <= ly <= bbox['max_y']):
            return None

        if self._homography is not None:
            H = self._homography
            w = H[2, 0] * lx + H[2, 1] * ly + H[2, 2]
            if abs(w) < 1e-10:
                return None
            sx = (H[0, 0] * lx + H[0, 1] * ly + H[0, 2]) / w
            sy = (H[1, 0] * lx + H[1, 1] * ly + H[1, 2]) / w
        else:
            dx = lx - bbox['min_x']
            dy = ly - bbox['min_y']
            w = bbox['max_x'] - bbox['min_x']
            h = bbox['max_y'] - bbox['min_y']
            if w == 0 or h == 0:
                return None
            sx = (dx / w) * self._precomputed['screen_max_x']
            sy = (1.0 - dy / h) * self._precomputed['screen_max_y']

        sx = max(0.0, min(self._precomputed['screen_max_x'], sx))
        sy = max(0.0, min(self._precomputed['screen_max_y'], sy))

        return (int(sx), int(sy))

    def move_mouse_to_touch(self, touch_point: Point) -> bool:
        """Мгновенное перемещение курсора в позицию касания с мёртвой зоной."""
        if not self.is_active:
            return False

        screen_coords = self.map_touch_to_screen(touch_point)
        if screen_coords is None:
            return False

        x, y = screen_coords

        if self._anchor_screen_x is None:
            self._anchor_screen_x = x
            self._anchor_screen_y = y
            _set_cursor_pos(x, y)
            self.last_mouse_position = (x, y)
            return True

        dx = x - self._anchor_screen_x
        dy = y - self._anchor_screen_y
        distance = (dx * dx + dy * dy) ** 0.5

        if distance > self.dead_zone_radius:
            self._anchor_screen_x = x
            self._anchor_screen_y = y
            _set_cursor_pos(x, y)
            self.last_mouse_position = (x, y)
            return True

        return False

    def update_touch_state(self, has_touch: bool, touch_point: Point = None) -> None:
        """
        Обновляет состояние касания.
        Новое касание → ЛКМ DOWN, продолжение → курсор двигается (drag),
        отпускание → ЛКМ UP.
        """
        if has_touch and touch_point:
            if not self.current_touch_state:
                # Начало нового касания — сбрасываем якорь + ЛКМ DOWN
                self._anchor_screen_x = None
                self._anchor_screen_y = None
                self.current_touch_state = True
                self.move_mouse_to_touch(touch_point)
                _mouse_down()
            else:
                # Продолжение касания — обновляем позицию (drag)
                self.move_mouse_to_touch(touch_point)
        else:
            # Нет касания
            if self.current_touch_state:
                # Отпускание — ЛКМ UP
                _mouse_up()
                self.current_touch_state = False
                self._anchor_screen_x = None
                self._anchor_screen_y = None

    def enable_control(self):
        """Включает управление мышью"""
        self.is_active = True
        self.current_touch_state = False
        self.last_mouse_position = None
        self.last_click_time = 0
        self._anchor_screen_x = None
        self._anchor_screen_y = None
        print("✅ Управление мышью включено")

    def disable_control(self):
        """Выключает управление мышью"""
        self.is_active = False
        self.current_touch_state = False
        self.last_mouse_position = None
        self._anchor_screen_x = None
        self._anchor_screen_y = None
        print("🛑 Управление мышью выключено")

    def set_click_delay(self, delay_seconds: float):
        """Устанавливает задержку после клика"""
        self.click_delay = max(0.1, delay_seconds)
        print(f"⏱️  Задержка после клика установлена: {self.click_delay}с")


# Глобальный экземпляр контроллера
mouse_controller = None

def initialize_mouse_control(screen_width: int = None, screen_height: int = None):
    """Инициализация управления мыши"""
    global mouse_controller
    mouse_controller = MouseController(screen_width, screen_height)
    return mouse_controller

def get_mouse_controller() -> MouseController:
    """Возвращает глобальный экземпляр контроллера мыши"""
    global mouse_controller
    return mouse_controller