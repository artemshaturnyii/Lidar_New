"""Основной класс для детекции касаний"""

from typing import List, Tuple, Optional, Callable, Dict
import numpy as np
from lidar_sdk import Point
from calibration.file_io import load_background_from_file
from .threshold_config import ThresholdConfig
from .comparators import (
    calculate_deviations, 
    filter_by_confidence, 
    filter_by_background_distance,
    find_significant_deviations,
    group_touch_points,
    calculate_adaptive_threshold
)

class TouchDetector:
    """
    Детектор касаний на основе сравнения текущего скана с фоновой картой
    """
    
    def __init__(self, background_source, config: Optional[ThresholdConfig] = None,
                 max_working_distance_ratio: float = 0.9):
        """
        Инициализация детектора касаний
        
        Args:
            background_source: Путь к файлу фоновой карты или словарь {угол: расстояние}
            config: Конфигурация порогов
            max_working_distance_ratio: Максимальное отношение рабочего расстояния (0.9 = 90%)
        """
        self.config = config or ThresholdConfig.default()
        self.max_working_distance_ratio = max_working_distance_ratio
        
        if isinstance(background_source, str):
            self.background_map, self.background_interp = load_background_from_file(background_source)
        elif isinstance(background_source, dict):
            self.background_map = background_source
            self.background_interp = self._create_interpolation_func(background_source)
        else:
            raise ValueError("background_source должен быть путем к файлу или словарем")
            
        if not self.background_map:
            raise ValueError("Фоновая карта пуста или не загружена")
        
        # Сглаживаем загруженную фоновую карту
        self.background_map = self._smooth_loaded_background(self.background_map)
        self.background_interp = self._create_interpolation_func(self.background_map)
    
    def _smooth_loaded_background(self, background_map: dict, window_size: float = 3.0) -> dict:
        """Сглаживает загруженную фоновую карту"""
        if not background_map:
            return background_map
            
        smoothed_map = {}
        angles = sorted(background_map.keys())
        
        for angle in angles:
            window_values = []
            for offset in np.arange(-window_size, window_size + 0.5, 0.5):
                neighbor_angle = (angle + offset) % 360
                if neighbor_angle < 0:
                    neighbor_angle += 360
                elif neighbor_angle >= 360:
                    neighbor_angle -= 360
                    
                if neighbor_angle in background_map:
                    window_values.append(background_map[neighbor_angle])
            
            if window_values:
                # Используем медиану для устойчивости к выбросам
                smoothed_value = np.median(window_values)
                smoothed_map[angle] = smoothed_value
            else:
                smoothed_map[angle] = background_map[angle]
                
        return smoothed_map
    
    def _create_interpolation_func(self, background_map: dict) -> Callable[[float], float]:
        """Создает функцию интерполяции для фоновой карты"""
        if not background_map:
            return lambda angle: 3000.0
            
        angles = list(background_map.keys())
        distances = list(background_map.values())
        
        def background_interp(angle):
            return np.interp(angle, angles, distances, period=360)
            
        return background_interp
    
    def detect_touch_points(self, current_scan: List[Point]) -> List[Point]:
        """
        Обнаруживает точки касания в текущем скане
        """
        if not current_scan:
            return []
            
        # Шаг 1: Вычисляем отклонения от фона с учетом рабочей зоны
        deviations = calculate_deviations(
            current_scan, 
            self.background_interp,
            self.max_working_distance_ratio
        )
        
        # Шаг 2: Фильтруем по минимальной уверенности
        filtered_deviations = filter_by_confidence(
            deviations, self.config.min_confidence
        )
        
        # Шаг 3: Фильтруем по максимальному расстоянию фона
        filtered_deviations = filter_by_background_distance(
            filtered_deviations, self.config.max_background_distance
        )
        
        # Шаг 4: Находим значимые отклонения
        touch_candidates = find_significant_deviations(
            filtered_deviations, self.config.min_distance_deviation
        )
        
        # Шаг 5: Применяем адаптивную фильтрацию
        adaptive_filtered = []
        for point, deviation in touch_candidates:
            expected_distance = self.background_interp(point.angle)
            adaptive_threshold = calculate_adaptive_threshold(
                expected_distance, self.config.adaptive_threshold_multiplier
            )
            if deviation >= adaptive_threshold:
                adaptive_filtered.append((point, deviation))
        
        # Шаг 6: Группируем точки по угловому окну
        groups = group_touch_points(adaptive_filtered, self.config.angle_window)
        
        # Шаг 7: Фильтруем группы по минимальному количеству точек
        valid_groups = [group for group in groups if len(group) >= self.config.min_touch_points]
        
        # Шаг 8: Возвращаем только точки (без отклонений)
        touch_points = []
        for group in valid_groups:
            representative = max(group, key=lambda x: x[1])[0]
            touch_points.append(representative)
            
        return touch_points
    
    def detect_touch_regions(self, current_scan: List[Point]) -> List[List[Point]]:
        """Обнаруживает регионы касания"""
        if not current_scan:
            return []
            
        # Повторяем почти те же шаги, но возвращаем группы
        deviations = calculate_deviations(
            current_scan, 
            self.background_interp,
            self.max_working_distance_ratio
        )
        filtered_deviations = filter_by_confidence(
            deviations, self.config.min_confidence
        )
        filtered_deviations = filter_by_background_distance(
            filtered_deviations, self.config.max_background_distance
        )
        touch_candidates = find_significant_deviations(
            filtered_deviations, self.config.min_distance_deviation
        )
        
        # Применяем адаптивную фильтрацию
        adaptive_filtered = []
        for point, deviation in touch_candidates:
            expected_distance = self.background_interp(point.angle)
            adaptive_threshold = calculate_adaptive_threshold(
                expected_distance, self.config.adaptive_threshold_multiplier
            )
            if deviation >= adaptive_threshold:
                adaptive_filtered.append((point, deviation))
        
        # Группируем точки
        groups = group_touch_points(adaptive_filtered, self.config.angle_window)
        
        # Фильтруем по минимальному количеству точек и возвращаем только точки
        valid_groups = []
        for group in groups:
            if len(group) >= self.config.min_touch_points:
                points_only = [point for point, _ in group]
                valid_groups.append(points_only)
                
        return valid_groups
    
    def get_touch_statistics(self, current_scan: List[Point]) -> Dict:
        """Получает статистику по касаниям"""
        touch_points = self.detect_touch_points(current_scan)
        touch_regions = self.detect_touch_regions(current_scan)
        
        if touch_points:
            avg_distance = np.mean([p.distance for p in touch_points])
            avg_angle = np.mean([p.angle for p in touch_points])
            avg_intensity = np.mean([p.intensity for p in touch_points])
        else:
            avg_distance = avg_angle = avg_intensity = 0.0
            
        return {
            'touch_points_count': len(touch_points),
            'touch_regions_count': len(touch_regions),
            'average_distance': float(avg_distance),
            'average_angle': float(avg_angle),
            'average_intensity': float(avg_intensity),
            'max_region_size': max([len(region) for region in touch_regions], default=0)
        }
    
    def update_background(self, new_background_source):
        """Обновляет фоновую карту"""
        if isinstance(new_background_source, str):
            self.background_map, self.background_interp = load_background_from_file(new_background_source)
        elif isinstance(new_background_source, dict):
            self.background_map = new_background_source
            self.background_interp = self._create_interpolation_func(new_background_source)
        else:
            raise ValueError("new_background_source должен быть путем к файлу или словарем")
            
        if not self.background_map:
            raise ValueError("Новая фоновая карта пуста")
        
        # Сглаживаем новую фоновую карту
        self.background_map = self._smooth_loaded_background(self.background_map)
        self.background_interp = self._create_interpolation_func(self.background_map)
    
    def update_config(self, new_config: ThresholdConfig):
        """Обновляет конфигурацию порогов"""
        self.config = new_config
    
    def update_working_distance_ratio(self, new_ratio: float):
        """Обновляет коэффициент рабочего расстояния"""
        self.max_working_distance_ratio = max(0.1, min(1.0, new_ratio))
