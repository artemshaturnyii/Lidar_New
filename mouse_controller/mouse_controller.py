"""
Модуль для управления курсором мыши на основе данных LiDAR
"""

import pyautogui
import numpy as np
from typing import List, Optional, Tuple
from lidar_sdk import Point

class MouseController:
    """Контроллер для управления курсором мыши на основе касаний LiDAR"""
    
    def __init__(self, screen_width: int = None, screen_height: int = None):
        """
        Инициализация контроллера мыши
        
        Args:
            screen_width: ширина экрана (по умолчанию определяется автоматически)
            screen_height: высота экрана (по умолчанию определяется автоматически)
        """
        # Получаем размеры экрана если не заданы
        if screen_width is None or screen_height is None:
            screen_size = pyautogui.size()
            self.screen_width = screen_width or screen_size.width
            self.screen_height = screen_height or screen_size.height
        else:
            self.screen_width = screen_width
            self.screen_height = screen_height
            
        self.corners = None
        self.is_active = False
        self.smoothing_factor = 0.3  # Фактор сглаживания движения (0.0 - 1.0)
        self.last_mouse_position = None
        self.lidar_corners = None   # Углы LiDAR в декартовых координатах
        self.screen_corners = None  # Соответствующие углы экрана
        self.lidar_bbox = None      # Ограничивающий прямоугольник проекции
        
    def set_projection_corners(self, corners: List[dict]):
        """
        Установка углов проекции для калибровки
        
        Args:
            corners: список углов проекции [{angle, distance, name}, ...]
            Порядок: правый верхний - правый нижний - левый нижний - левый верхний
        """
        if corners and len(corners) >= 4:
            self.corners = corners
            # Вычисляем соответствие углов
            self._setup_corner_mapping()
            print(f"✅ Установлены углы проекции для управления мышью ({len(corners)} точек)")
        else:
            print("⚠️  Недостаточно углов для управления мышью")
            self.corners = None
            self.lidar_corners = None
            self.screen_corners = None
            self.lidar_bbox = None
            
    def _setup_corner_mapping(self):
        """
        Настраивает соответствие углов LiDAR и экрана
        Углы передаются в порядке: правый верхний - правый нижний - левый нижний - левый верхний
        """
        if not self.corners or len(self.corners) < 4:
            return
            
        # Берем первые 4 угла в заданном порядке
        corner_points = self.corners[:4]
        
        # Преобразуем углы LiDAR в декартовы координаты
        # Исправленная функция для правильного преобразования координат LiDAR
        def polar_to_cartesian(angle_deg, distance):
            # Предполагаем, что 0° - это направление "вперёд" (вверх)
            # Поэтому используем sin для X и cos для Y с инверсией
            angle_rad = np.radians(angle_deg)
            x = distance * np.sin(angle_rad)      # Горизонтальное движение
            y = -distance * np.cos(angle_rad)     # Вертикальное движение (инвертировано)
            return x, y
        
        # Преобразуем углы проекции в декартовы координаты
        self.lidar_corners = [polar_to_cartesian(corner['angle'], corner['distance']) 
                             for corner in corner_points]
        
        # Углы экрана в пикселях в том же порядке:
        # правый верхний - правый нижний - левый нижний - левый верхний
        self.screen_corners = [
            (self.screen_width - 1, 0),                    # правый верхний (x=max, y=0)
            (self.screen_width - 1, self.screen_height - 1), # правый нижний (x=max, y=max)
            (0, self.screen_height - 1),                   # левый нижний (x=0, y=max)
            (0, 0)                                         # левый верхний (x=0, y=0)
        ]
        
        # Вычисляем ограничивающий прямоугольник для LiDAR
        if self.lidar_corners:
            x_coords = [p[0] for p in self.lidar_corners]
            y_coords = [p[1] for p in self.lidar_corners]
            self.lidar_bbox = {
                'min_x': min(x_coords),
                'max_x': max(x_coords),
                'min_y': min(y_coords),
                'max_y': max(y_coords)
            }
        
        print("📏 Соответствие углов установлено")
        print(f"   Углы LiDAR: {[f'({x:.1f}, {y:.1f})' for x, y in self.lidar_corners]}")
        print(f"   Углы экрана: {self.screen_corners}")
        print(f"   BBox LiDAR: {self.lidar_bbox}")

    def map_touch_to_screen(self, touch_point: Point) -> Optional[Tuple[int, int]]:
        """
        Преобразует точку касания LiDAR в координаты экрана
        
        Args:
            touch_point: точка касания (Point)
            
        Returns:
            Tuple[int, int]: (x, y) координаты экрана или None
        """
        if not self.corners or not self.lidar_corners or not self.screen_corners or not self.lidar_bbox:
            return None
            
        # Преобразуем точку касания в декартовы координаты
        def polar_to_cartesian(angle_deg, distance):
            angle_rad = np.radians(angle_deg)
            x = distance * np.sin(angle_rad)      # Горизонтальное движение
            y = -distance * np.cos(angle_rad)     # Вертикальное движение (инвертировано)
            return x, y
        
        touch_x, touch_y = polar_to_cartesian(touch_point.angle, touch_point.distance)
        
        # Проверяем, находится ли точка в пределах проекции
        tolerance = 50
        if not (self.lidar_bbox['min_x'] - tolerance <= touch_x <= self.lidar_bbox['max_x'] + tolerance and 
                self.lidar_bbox['min_y'] - tolerance <= touch_y <= self.lidar_bbox['max_y'] + tolerance):
            return None
        
        # Нормализуем координаты точки касания в диапазон 0-1 внутри проекции
        lidar_width = self.lidar_bbox['max_x'] - self.lidar_bbox['min_x']
        lidar_height = self.lidar_bbox['max_y'] - self.lidar_bbox['min_y']
        
        if lidar_width != 0:
            norm_x = (touch_x - self.lidar_bbox['min_x']) / lidar_width
        else:
            norm_x = 0.5
            
        if lidar_height != 0:
            norm_y = (touch_y - self.lidar_bbox['min_y']) / lidar_height
        else:
            norm_y = 0.5
        
        # Преобразуем нормализованные координаты в координаты экрана
        screen_x = norm_x * (self.screen_width - 1)
        screen_y = norm_y * (self.screen_height - 1)
        
        # Ограничиваем координаты границами экрана
        screen_x = max(0, min(self.screen_width - 1, screen_x))
        screen_y = max(0, min(self.screen_height - 1, screen_y))
        
        return (int(screen_x), int(screen_y))
    
    def move_mouse_to_touch(self, touch_point: Point) -> bool:
        """
        Перемещает курсор мыши в позицию касания
        
        Args:
            touch_point: точка касания (Point)
            
        Returns:
            bool: True если перемещение успешно
        """
        if not self.is_active:
            return False
            
        screen_coords = self.map_touch_to_screen(touch_point)
        
        if screen_coords is None:
            return False
            
        x, y = screen_coords
        
        # Применяем сглаживание если есть предыдущая позиция
        if self.last_mouse_position and self.smoothing_factor < 1.0:
            last_x, last_y = self.last_mouse_position
            x = int(last_x * (1 - self.smoothing_factor) + x * self.smoothing_factor)
            y = int(last_y * (1 - self.smoothing_factor) + y * self.smoothing_factor)
        
        # Перемещаем курсор мыши
        try:
            pyautogui.moveTo(x, y)
            self.last_mouse_position = (x, y)
            return True
        except Exception as e:
            print(f"❌ Ошибка перемещения мыши: {e}")
            return False
    
    def click_at_touch(self, touch_point: Point) -> bool:
        """
        Выполняет клик мышью в позиции касания
        
        Args:
            touch_point: точка касания (Point)
            
        Returns:
            bool: True если клик успешен
        """
        if not self.is_active:
            return False
            
        # Сначала перемещаем курсор
        if self.move_mouse_to_touch(touch_point):
            try:
                # Выполняем клик
                pyautogui.click()
                return True
            except Exception as e:
                print(f"❌ Ошибка клика мышью: {e}")
                return False
        return False
    
    def enable_control(self):
        """Включает управление мышью"""
        self.is_active = True
        self.last_mouse_position = None
        print("✅ Управление мышью включено")
        
    def disable_control(self):
        """Выключает управление мышью"""
        self.is_active = False
        self.last_mouse_position = None
        print("🛑 Управление мышью выключено")
        
    def set_smoothing(self, factor: float):
        """
        Устанавливает фактор сглаживания движения мыши
        
        Args:
            factor: фактор сглаживания (0.0 - 1.0, где 1.0 = полное сглаживание)
        """
        self.smoothing_factor = max(0.0, min(1.0, factor))
        print(f"🎚️  Фактор сглаживания установлен: {self.smoothing_factor}")

# Глобальный экземпляр контроллера
mouse_controller = MouseController()

def initialize_mouse_control(screen_width: int = None, screen_height: int = None):
    """
    Инициализация управления мышью
    
    Args:
        screen_width: ширина экрана (опционально)
        screen_height: высота экрана (опционально)
        
    Returns:
        MouseController: экземпляр контроллера мыши
    """
    global mouse_controller
    mouse_controller = MouseController(screen_width, screen_height)
    return mouse_controller

def get_mouse_controller() -> MouseController:
    """Возвращает глобальный экземпляр контроллера мыши"""
    global mouse_controller
    return mouse_controller
