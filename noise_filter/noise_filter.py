"""Фильтр для устранения ложных срабатываний на основе профиля шума"""

from typing import List
from lidar_sdk import Point
import sys
import os

# Добавляем путь к корневой директории для импорта
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class NoiseFilter:
    """Фильтр для устранения ложных срабатываний на основе профиля шума"""
    
    def __init__(self, noise_profile=None, persistent_noise_manager=None):
        self.noise_profile = noise_profile or {}
        # Импортируем здесь чтобы избежать циклических зависимостей
        if persistent_noise_manager is None:
            try:
                from persistent_noise_filter.persistent_noise_manager import PersistentNoiseManager
                self.persistent_noise_manager = PersistentNoiseManager()
            except ImportError:
                # Fallback если модуль не найден
                self.persistent_noise_manager = None
        else:
            self.persistent_noise_manager = persistent_noise_manager
        
    def is_false_positive(self, touch_point, threshold_frequency=2.0):
        """Определяет, является ли точка ложным срабатыванием"""
        # Проверяем постоянный шум если менеджер доступен
        if self.persistent_noise_manager and self.persistent_noise_manager.is_persistent_noise(touch_point):
            return True
            
        # Проверяем статистический профиль шума
        sector_size = 5.0
        sector = int(touch_point.angle / sector_size) * sector_size
        
        if sector in self.noise_profile:
            profile = self.noise_profile[sector]
            # Если частота выше порога - это шум
            if profile['frequency'] > threshold_frequency:
                # Дополнительно проверяем близость по расстоянию
                distance_diff = abs(touch_point.distance - profile['avg_distance'])
                if distance_diff < 100:  # мм
                    return True
        return False
        
    def filter_touch_points(self, touch_points):
        """Фильтрует ложные срабатывания из списка точек"""
        filtered_points = []
        for point in touch_points:
            if not self.is_false_positive(point):
                filtered_points.append(point)
            elif self.persistent_noise_manager:
                # Регистрируем точку как возможный постоянный шум
                self.persistent_noise_manager.register_noise_point(point)
        return filtered_points
        
    def get_noise_statistics(self):
        """Возвращает статистику по шуму"""
        if self.persistent_noise_manager:
            return self.persistent_noise_manager.get_persistent_noise_stats()
        return {}
