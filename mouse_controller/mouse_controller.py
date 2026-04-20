"""
Универсальный контроллер мыши для Windows и Linux (X11/Wayland).
Авто-детекция платформы, множественные бэкенды.
"""

import platform
import os
import subprocess
import shutil
from typing import List, Optional, Tuple
from lidar_sdk import Point
import time
import numpy as np


# ─── Детекция платформы и сессии ──────────────────────────────────────

_SYSTEM = platform.system()

def _detect_session_type() -> str:
    """Определяет тип сессии: 'wayland', 'x11', или 'unknown'."""
    if os.environ.get('WAYLAND_DISPLAY'):
        return 'wayland'
    elif os.environ.get('DISPLAY'):
        return 'x11'
    return 'unknown'

def _check_ydotool() -> bool:
    """Проверяет доступность ydotool."""
    return shutil.which('ydotool') is not None

def _check_xlib() -> bool:
    """Проверяет доступность Xlib."""
    try:
        from Xlib import display
        return True
    except ImportError:
        return False

def _check_wayland_automation() -> bool:
    """Проверяет доступность wayland-automation."""
    try:
        import wayland_automation
        return True
    except ImportError:
        return False


# ─── Бэкенд 1: Windows Win32 API ──────────────────────────────────────

class Win32Backend:
    """Бэкенд на основе Win32 API — только Windows."""
    
    def __init__(self):
        import ctypes
        self._user32 = ctypes.windll.user32
    
    def set_cursor_pos(self, x: int, y: int) -> None:
        self._user32.SetCursorPos(x, y)
    
    def mouse_down(self) -> None:
        self._user32.mouse_event(0x0002, 0, 0, 0, 0)
    
    def mouse_up(self) -> None:
        self._user32.mouse_event(0x0004, 0, 0, 0, 0)
    
    def get_screen_size(self) -> Tuple[int, int]:
        return (self._user32.GetSystemMetrics(0), self._user32.GetSystemMetrics(1))


# ─── Бэкенд 2: ydotool (универсальный Linux) ─────────────────────────

class YdotoolBackend:
    """Бэкенд на основе ydotool — работает на X11 и Wayland."""
    
    def __init__(self):
        self.ydotool_path = shutil.which('ydotool')
        if not self.ydotool_path:
            raise RuntimeError("ydotool не найден в PATH. Установите: sudo apt install ydotool")
        
        if not os.access('/dev/uinput', os.W_OK):
            print("⚠️  ПРЕДУПРЕЖДЕНИЕ: Нет доступа к /dev/uinput")
            print("   Решение: sudo chmod 666 /dev/uinput")
    
    def set_cursor_pos(self, x: int, y: int) -> None:
        result = subprocess.run([
            self.ydotool_path, 'mousemove', str(x), str(y)
        ], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"⚠️  ydotool mousemove error: {result.stderr}")
    
    def mouse_down(self) -> None:
        result = subprocess.run([
            self.ydotool_path, 'mousedown', '1'
        ], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"⚠️  ydotool mousedown error: {result.stderr}")
    
    def mouse_up(self) -> None:
        result = subprocess.run([
            self.ydotool_path, 'mouseup', '1'
        ], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"⚠️  ydotool mouseup error: {result.stderr}")
    
    def get_screen_size(self) -> Tuple[int, int]:
        try:
            result = subprocess.run(
                ['xrandr', '--query'], 
                capture_output=True, text=True, timeout=2
            )
            for line in result.stdout.split('\n'):
                if ' connected' in line and 'x' in line:
                    parts = line.split()
                    for p in parts:
                        if 'x' in p and p[0].isdigit():
                            w, h = p.split('x')[0], p.split('x')[1].split('+')[0]
                            return int(w), int(h)
        except Exception as e:
            print(f"⚠️  xrandr error: {e}")
        
        print("⚠️  Не удалось определить разрешение экрана, используем 1920x1080")
        return (1920, 1080)


# ─── Бэкенд 3: Xlib (X11) ────────────────────────────────────────────

class XlibBackend:
    """Бэкенд на основе Xlib — только X11."""
    
    def __init__(self):
        from Xlib import display, X
        self._dpy = display.Display()
        self._root = self._dpy.screen().root
    
    def set_cursor_pos(self, x: int, y: int) -> None:
        self._root.warp_pointer(x, y)
        self._dpy.sync()
    
    def mouse_down(self) -> None:
        from Xlib.ext.xtest import fake_input
        from Xlib import X
        fake_input(self._dpy, X.ButtonPress, 1)
        self._dpy.sync()
    
    def mouse_up(self) -> None:
        from Xlib.ext.xtest import fake_input
        from Xlib import X
        fake_input(self._dpy, X.ButtonRelease, 1)
        self._dpy.sync()
    
    def get_screen_size(self) -> Tuple[int, int]:
        return (self._root.get_geometry().width, self._root.get_geometry().height)


# ─── Бэкенд 4: wayland-automation (Wayland wlroots) ──────────────────

class WaylandAutomationBackend:
    """Бэкенд на основе wayland-automation — только Wayland (wlroots)."""
    
    def __init__(self):
        import wayland_automation as wa
        self.wa = wa
    
    def set_cursor_pos(self, x: int, y: int) -> None:
        self.wa.click(x, y, button=None)
    
    def mouse_down(self) -> None:
        self.wa.click(0, 0, button="left")
    
    def mouse_up(self) -> None:
        pass
    
    def get_screen_size(self) -> Tuple[int, int]:
        try:
            from wayland_automation.utils.screen_resolution import get_screen_resolution
            return get_screen_resolution()
        except Exception:
            return (1920, 1080)


# ─── Фабрика бэкендов ─────────────────────────────────────────────────

class BackendFactory:
    """Создаёт лучший доступный бэкенд для текущей системы."""
    
    @staticmethod
    def create() -> Tuple[object, str]:
        if _SYSTEM == "Windows":
            try:
                return Win32Backend(), "win32"
            except Exception as e:
                raise RuntimeError(f"Win32 backend failed: {e}")
        
        elif _SYSTEM == "Linux":
            session = _detect_session_type()
            
            if _check_ydotool():
                try:
                    return YdotoolBackend(), "ydotool"
                except Exception as e:
                    print(f"⚠️  ydotool backend failed: {e}")
            
            if session == 'x11' and _check_xlib():
                try:
                    return XlibBackend(), "xlib"
                except Exception as e:
                    print(f"⚠️  Xlib backend failed: {e}")
            
            if session == 'wayland' and _check_wayland_automation():
                try:
                    return WaylandAutomationBackend(), "wayland-automation"
                except Exception as e:
                    print(f"⚠️  wayland-automation backend failed: {e}")
            
            raise RuntimeError(
                f"Не найдено доступного бэкенда для мыши.\n"
                f"Сессия: {session}\n"
                f"Установите один из:\n"
                f"  - ydotool (рекомендуется): sudo apt install ydotool\n"
                f"  - Xlib (X11): pip install python-xlib\n"
                f"  - wayland-automation (Wayland): pip install wayland-automation"
            )
        
        else:
            raise RuntimeError(f"Неподдерживаемая ОС: {_SYSTEM}")


# ─── Основной контроллер ──────────────────────────────────────────────

class MouseController:
    """
    Универсальный контроллер мыши с авто-выбором бэкенда.
    """

    def __init__(self, screen_width: int = None, screen_height: int = None):
        self.backend, self.backend_name = BackendFactory.create()
        print(f"✅ Mouse backend: {self.backend_name}")
        
        if screen_width is None or screen_height is None:
            self.screen_width, self.screen_height = self.backend.get_screen_size()
        else:
            self.screen_width = screen_width
            self.screen_height = screen_height

        self.corners = None
        self.is_active = False
        self.last_mouse_position = None
        self.lidar_corners = None
        self.screen_corners = None
        self.lidar_bbox = None
        self.north_angle = 0.0
        self.click_delay = 0.3
        self.last_click_time = 0
        self.current_touch_state = False
        self.touch_start_time = 0
        self.dead_zone_radius = 75
        self._anchor_screen_x: Optional[int] = None
        self._anchor_screen_y: Optional[int] = None
        self._homography: Optional[np.ndarray] = None
        self._precomputed = {}
        
        # Сглаживание позиции курсора
        self._position_buffer: List[Tuple[int, int]] = []
        self._position_buffer_size = 2  # ✅ Уменьшено с 3 до 2 (меньше задержка)
        self._smoothing_enabled = True
        
        # ✅ Гистерезис состояния касания (ОПТИМИЗИРОВАНО)
        self._touch_hold_counter: int = 0
        self._touch_release_counter: int = 0
        self._min_touch_frames: int = 1  # ✅ Уменьшено с 2 до 1 (мгновенный отклик)
        self._min_release_frames: int = 2  # ✅ Уменьшено с 3 до 2 (всё ещё стабильно)

    def set_projection_corners(self, corners: List[dict]):
        """Установка углов проекции для калибровки"""
        if corners and len(corners) >= 4:
            self.corners = corners
            self._setup_corner_mapping()
            print(f"✅ Установлены углы проекции для управления мышью ({len(corners)} точек)")
        else:
            print("⚠️  Недостаточно углов для управления мышью")
            self.corners = None

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
        self._precomputed['north_rad'] = np.radians(self.north_angle)

        print("📏 Гомография вычислена, соответствие углов установлено")

    def _compute_bbox_fallback(self, lidar_pts):
        """Fallback если не все углы найдены."""
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
        """Вычисляет матрицу гомографии 3x3 без OpenCV."""
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
        """Преобразует точку касания LiDAR в координаты экрана."""
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
        """Мгновенное перемещение курсора в позицию касания с мёртвой зоной и сглаживанием."""
        if not self.is_active:
            return False

        screen_coords = self.map_touch_to_screen(touch_point)
        if screen_coords is None:
            return False

        x, y = screen_coords

        # ✅ Применяем сглаживание позиции (2 кадра вместо 3)
        if self._smoothing_enabled:
            self._position_buffer.append((x, y))
            if len(self._position_buffer) > self._position_buffer_size:
                self._position_buffer.pop(0)
            
            x = int(sum(p[0] for p in self._position_buffer) / len(self._position_buffer))
            y = int(sum(p[1] for p in self._position_buffer) / len(self._position_buffer))

        if self._anchor_screen_x is None:
            self._anchor_screen_x = x
            self._anchor_screen_y = y
            self.backend.set_cursor_pos(x, y)
            self.last_mouse_position = (x, y)
            return True

        dx = x - self._anchor_screen_x
        dy = y - self._anchor_screen_y
        distance = (dx * dx + dy * dy) ** 0.5

        if distance > self.dead_zone_radius:
            self._anchor_screen_x = x
            self._anchor_screen_y = y
            self.backend.set_cursor_pos(x, y)
            self.last_mouse_position = (x, y)
            return True

        return False

    def update_touch_state(self, has_touch: bool, touch_point: Point = None) -> None:
        """Обновляет состояние касания с гистерезисом (защита от джиттера)."""
        
        if has_touch and touch_point:
            # ✅ Касание обнаружено
            self._touch_release_counter = 0
            self._touch_hold_counter += 1
            
            # ✅ БЫЛО: >= 2 → СЕЙЧАС: >= 1 (мгновенная реакция)
            if self._touch_hold_counter >= self._min_touch_frames:
                if not self.current_touch_state:
                    self._position_buffer.clear()
                    self._anchor_screen_x = None
                    self._anchor_screen_y = None
                    self.current_touch_state = True
                    self.move_mouse_to_touch(touch_point)
                    self.backend.mouse_down()
                else:
                    self.move_mouse_to_touch(touch_point)
        else:
            # ✅ Касание не обнаружено
            self._touch_hold_counter = 0
            self._touch_release_counter += 1
            
            # ✅ БЫЛО: >= 3 → СЕЙЧАС: >= 2 (всё ещё защита от джиттера)
            if self._touch_release_counter >= self._min_release_frames:
                if self.current_touch_state:
                    self.backend.mouse_up()
                    self.current_touch_state = False
                    self._anchor_screen_x = None
                    self._anchor_screen_y = None
                    self._position_buffer.clear()

    def enable_control(self):
        """Включает управление мышью"""
        self.is_active = True
        self.current_touch_state = False
        self.last_mouse_position = None
        self.last_click_time = 0
        self._anchor_screen_x = None
        self._anchor_screen_y = None
        self._position_buffer.clear()
        self._touch_hold_counter = 0
        self._touch_release_counter = 0
        print("✅ Управление мышью включено")

    def disable_control(self):
        """Выключает управление мышью"""
        self.is_active = False
        self.current_touch_state = False
        self.last_mouse_position = None
        self._anchor_screen_x = None
        self._anchor_screen_y = None
        self._position_buffer.clear()
        self._touch_hold_counter = 0
        self._touch_release_counter = 0
        print("🛑 Управление мышью выключено")

    def set_click_delay(self, delay_seconds: float):
        """Устанавливает задержку после клика"""
        self.click_delay = max(0.1, delay_seconds)


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
