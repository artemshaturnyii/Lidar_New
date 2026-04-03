"""
Модуль для управления курсором мыши на основе данных LiDAR
"""

import pyautogui
import numpy as np
from typing import List, Optional, Tuple
from lidar_sdk import Point
import time

class MouseController:
    """Контроллер для управления курсором мыши на основе касаний LiDAR"""
    
    def __init__(self, screen_width: int = None, screen_height: int = None):
        """
        Инициализация контроллера мыши
        """
        # Отключаем защитные механизмы для скорости
        pyautogui.FAILSAFE = False
        pyautogui.PAUSE = 0
        
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
        
        # Для стабилизации позиции
        self.position_buffer = []
        self.buffer_size = 2
    
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
            
    def _setup_corner_mapping(self):
        """Настраивает соответствие углов LiDAR и экрана"""
        if not self.corners or len(self.corners) < 4:
            return
            
        # Преобразуем углы LiDAR в декартовы координаты (правильная система координат)
        def polar_to_cartesian(angle_deg, distance):
            # Корректируем угол относительно севера
            corrected_angle = (angle_deg - self.north_angle) % 360
            angle_rad = np.radians(corrected_angle)
            # В лидаре: X вправо, Y вперед (как в математике)
            x = distance * np.sin(angle_rad)  # Положительный X вправо
            y = distance * np.cos(angle_rad)  # Положительный Y вперед
            return x, y
        
        # Преобразуем углы проекции в декартовы координаты
        self.lidar_corners = [polar_to_cartesian(corner['angle'], corner['distance']) 
                             for corner in self.corners[:4]]
        
        # Вычисляем ограничивающий прямоугольник
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

    def map_touch_to_screen(self, touch_point: Point) -> Optional[Tuple[int, int]]:
        """Преобразует точку касания LiDAR в координаты экрана"""
        if not self.corners or not self.lidar_corners or not self.lidar_bbox:
            return None
            
        # Преобразуем точку касания в декартовы координаты
        def polar_to_cartesian(angle_deg, distance):
            corrected_angle = (angle_deg - self.north_angle) % 360
            angle_rad = np.radians(corrected_angle)
            x = distance * np.sin(angle_rad)  # Положительный X вправо
            y = distance * np.cos(angle_rad)  # Положительный Y вперед
            return x, y
        
        touch_x, touch_y = polar_to_cartesian(touch_point.angle, touch_point.distance)
        
        # Проверяем, находится ли точка в пределах проекции
        if not (self.lidar_bbox['min_x'] <= touch_x <= self.lidar_bbox['max_x'] and 
                self.lidar_bbox['min_y'] <= touch_y <= self.lidar_bbox['max_y']):
            return None
        
        # Нормализуем координаты в диапазон 0-1
        try:
            lidar_width = self.lidar_bbox['max_x'] - self.lidar_bbox['min_x']
            lidar_height = self.lidar_bbox['max_y'] - self.lidar_bbox['min_y']
            
            if lidar_width == 0 or lidar_height == 0:
                return None
                
            norm_x = (touch_x - self.lidar_bbox['min_x']) / lidar_width
            norm_y = (touch_y - self.lidar_bbox['min_y']) / lidar_height
        except ZeroDivisionError:
            return None
        
        # Преобразуем в координаты экрана
        # Экран: (0,0) в левом верхнем углу, X вправо, Y вниз
        screen_x = norm_x * (self.screen_width - 1)
        screen_y = (1.0 - norm_y) * (self.screen_height - 1)  # Инвертируем Y для экрана
        
        # Ограничиваем границами экрана
        screen_x = max(0, min(self.screen_width - 1, screen_x))
        screen_y = max(0, min(self.screen_height - 1, screen_y))
        
        return (int(screen_x), int(screen_y))
    
    def move_mouse_to_touch(self, touch_point: Point) -> bool:
        """
        Перемещение курсора мыши в позицию касания
        """
        if not self.is_active:
            return False
            
        screen_coords = self.map_touch_to_screen(touch_point)
        
        if screen_coords is None:
            return False
            
        x, y = screen_coords
        
        # Добавляем позицию в буфер для сглаживания
        self.position_buffer.append((x, y))
        if len(self.position_buffer) > self.buffer_size:
            self.position_buffer.pop(0)
        
        # Используем усредненную позицию для стабильности
        if len(self.position_buffer) > 1:
            avg_x = sum(pos[0] for pos in self.position_buffer) / len(self.position_buffer)
            avg_y = sum(pos[1] for pos in self.position_buffer) / len(self.position_buffer)
            x, y = int(avg_x), int(avg_y)
        
        try:
            pyautogui.moveTo(x, y)
            self.last_mouse_position = (x, y)
            return True
        except Exception as e:
            print(f"❌ Ошибка перемещения мыши: {e}")
            return False
    
    def update_touch_state(self, has_touch: bool, touch_point: Point = None) -> bool:
        """
        Обновляет состояние касания и возвращает True если нужно выполнить клик
        """
        current_time = time.time()
        
        if has_touch and touch_point:
            # Есть касание
            if not self.current_touch_state:
                # Начало нового касания
                self.current_touch_state = True
                self.touch_start_time = current_time
                # Очищаем буфер при новом касании
                self.position_buffer = []
                # Перемещаем мышь в позицию касания
                self.move_mouse_to_touch(touch_point)
                return False
            else:
                # Продолжение касания - обновляем позицию мыши
                self.move_mouse_to_touch(touch_point)
                return False
        else:
            # Нет касания
            if self.current_touch_state:
                # Завершение касания - проверяем задержку
                if current_time - self.touch_start_time >= 0.1:
                    if current_time - self.last_click_time >= self.click_delay:
                        self.current_touch_state = False
                        self.last_click_time = current_time
                        # Очищаем буфер
                        self.position_buffer = []
                        return True
                self.current_touch_state = False
        return False
    
    def enable_control(self):
        """Включает управление мышью"""
        self.is_active = True
        self.current_touch_state = False
        self.last_mouse_position = None
        self.last_click_time = 0
        self.position_buffer = []
        print("✅ Управление мышью включено")
        
    def disable_control(self):
        """Выключает управление мышью"""
        self.is_active = False
        self.current_touch_state = False
        self.last_mouse_position = None
        self.position_buffer = []
        print("🛑 Управление мышью выключено")
        
    def set_click_delay(self, delay_seconds: float):
        """Устанавливает задержку после клика"""
        self.click_delay = max(0.1, delay_seconds)
        print(f"⏱️  Задержка после клика установлена: {self.click_delay}с")

# Глобальный экземпляр контроллера
mouse_controller = None

def initialize_mouse_control(screen_width: int = None, screen_height: int = None):
    """
    Инициализация управления мыши
    """
    global mouse_controller
    mouse_controller = MouseController(screen_width, screen_height)
    return mouse_controller

def get_mouse_controller() -> MouseController:
    """Возвращает глобальный экземпляр контроллера мыши"""
    global mouse_controller
    return mouse_controller
