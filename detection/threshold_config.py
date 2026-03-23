"""Конфигурация порогов для детекции касаний"""

from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class ThresholdConfig:
    """
    Конфигурация порогов чувствительности для детекции касаний
    
    Атрибуты:
        min_distance_deviation: Минимальное отклонение расстояния (мм) для считывания касания
        max_background_distance: Максимальное расстояние фона (мм), дальше которого игнорируем
        min_confidence: Минимальная уверенность точки (0-255)
        angle_window: Угловое окно для группировки точек касания (градусы)
        min_touch_points: Минимальное количество точек для подтверждения касания
        adaptive_threshold_multiplier: Множитель для адаптивного порога (% от фона)
    """
    
    # Основные пороги
    min_distance_deviation: float = 100.0  # мм - только объекты, ближе на 100+ мм
    max_background_distance: float = 3000.0  # мм
    min_confidence: float = 150.0  # 0-255
    
    # Группировка точек
    angle_window: float = 10.0  # градусы
    min_touch_points: int = 3  # минимальное количество точек в группе
    
    # Адаптивная чувствительность
    adaptive_threshold_multiplier: float = 0.1  # множитель для адаптивного порога (% от фона)
    
    @classmethod
    def default(cls) -> 'ThresholdConfig':
        """Стандартная конфигурация"""
        return cls()
        
    @classmethod
    def sensitive(cls) -> 'ThresholdConfig':
        """Чувствительная конфигурация для обнаружения слабых касаний"""
        return cls(
            min_distance_deviation=20.0,
            min_confidence=30.0,
            angle_window=5.0,
            min_touch_points=2
        )
        
    @classmethod
    def robust(cls) -> 'ThresholdConfig':
        """Устойчивая конфигурация для фильтрации шума"""
        return cls(
            min_distance_deviation=100.0,
            min_confidence=100.0,
            angle_window=15.0,
            min_touch_points=5
        )
        
    def to_dict(self) -> Dict[str, Any]:
        """Преобразует конфигурацию в словарь"""
        return {
            'min_distance_deviation': self.min_distance_deviation,
            'max_background_distance': self.max_background_distance,
            'min_confidence': self.min_confidence,
            'angle_window': self.angle_window,
            'min_touch_points': self.min_touch_points,
            'adaptive_threshold_multiplier': self.adaptive_threshold_multiplier
        }
