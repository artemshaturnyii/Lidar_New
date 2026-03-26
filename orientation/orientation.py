
# orientation/orientation_manager.py
"""Менеджер ориентации лидара в пространстве"""

import json
from typing import Dict, List
from lidar_sdk import Point
import numpy as np

class OrientationManager:
    """Управление ориентацией лидара в пространстве"""
    
    def __init__(self, config_file: str = "orientation_config.json"):
        self.config_file = config_file
        self.orientation_angle = 0.0  # Угол поворота в градусах (0° = "север" по умолчанию)
        self.load_orientation()
    
    def rotate_clockwise(self, step: float = 1.0) -> float:
        """Поворот по часовой стрелке"""
        self.orientation_angle = (self.orientation_angle + step) % 360
        self.save_orientation()
        return self.orientation_angle
    
    def rotate_counterclockwise(self, step: float = 1.0) -> float:
        """Поворот против часовой стрелки"""
        self.orientation_angle = (self.orientation_angle - step) % 360
        self.save_orientation()
        return self.orientation_angle
    
    def set_orientation(self, angle: float) -> None:
        """Установка конкретного угла ориентации"""
        self.orientation_angle = angle % 360
        self.save_orientation()
    
    def get_orientation(self) -> float:
        """Получение текущего угла ориентации"""
        return self.orientation_angle
    
    def adjust_point_angle(self, point: Point) -> Point:
        """Корректировка угла точки с учетом ориентации лидара"""
        adjusted_angle = (point.angle + self.orientation_angle) % 360
        return Point(
            angle=adjusted_angle,
            distance=point.distance,
            intensity=point.intensity
        )
    
    def adjust_points_batch(self, points: List[Point]) -> List[Point]:
        """Корректировка углов для списка точек"""
        return [self.adjust_point_angle(point) for point in points]
    
    def adjust_angle_raw(self, angle: float) -> float:
        """Корректировка угла без создания точки"""
        return (angle + self.orientation_angle) % 360
    
    def save_orientation(self) -> None:
        """Сохранение ориентации в файл"""
        try:
            config = {
                'orientation_angle': self.orientation_angle,
                'timestamp': time.time() if 'time' in globals() else 0
            }
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            print(f"⚠️ Ошибка сохранения ориентации: {e}")
    
    def load_orientation(self) -> None:
        """Загрузка ориентации из файла"""
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
                self.orientation_angle = config.get('orientation_angle', 0.0)
        except FileNotFoundError:
            self.orientation_angle = 0.0
        except Exception as e:
            print(f"⚠️ Ошибка загрузки ориентации: {e}")
            self.orientation_angle = 0.0

# Глобальный экземпляр
orientation_manager = OrientationManager()

def get_orientation_manager() -> OrientationManager:
    """Получение глобального экземпляра менеджера ориентации"""
    global orientation_manager
    return orientation_manager
