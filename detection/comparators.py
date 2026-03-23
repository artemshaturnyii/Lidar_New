"""Функции сравнения сканов и анализа отклонений"""

from typing import List, Tuple, Callable, Optional
import numpy as np
from lidar_sdk import Point

def calculate_deviations(current_scan: List[Point], 
                        background_interp: Callable[[float], float],
                        max_working_distance_ratio: float = 0.9) -> List[Tuple[Point, float]]:
    """
    Вычисляет отклонения каждой точки текущего скана от фоновой карты
    
    Args:
        current_scan: Текущий скан (список точек)
        background_interp: Функция интерполяции фоновой карты
        max_working_distance_ratio: Максимальное отношение рабочего расстояния (0.9 = 90%)
        
    Returns:
        Список кортежей (точка, отклонение в мм)
        Положительное отклонение = объект ближе, чем фон
        Отрицательное отклонение = объект дальше, чем фон
    """
    # Сначала фильтруем резкие переходы
    filtered_scan = filter_sharp_transitions(current_scan, max_distance_jump=200.0)
    
    deviations = []
    safe_zone = 100  # Безопасная зона в мм для фильтрации ложных срабатываний на границе
    
    for point in filtered_scan:  # <<< Используем отфильтрованный скан
        if point.intensity < 10:  # Игнорируем очень слабые сигналы
            continue
            
        expected_distance = background_interp(point.angle)
        
        # Фильтр по максимальному рабочему расстоянию
        if expected_distance > 3000:  # Игнорируем слишком далекие области
            continue
            
        # Проверяем, не выходит ли точка за пределы рабочей зоны
        max_allowed_distance = expected_distance * max_working_distance_ratio - safe_zone
        if point.distance > max_allowed_distance and expected_distance > 100:
            # Это может быть ложное срабатывание на границе
            continue
            
        deviation = expected_distance - point.distance
        deviations.append((point, deviation))
        
    return deviations


def filter_by_confidence(deviations: List[Tuple[Point, float]], 
                        min_confidence: float) -> List[Tuple[Point, float]]:
    """
    Фильтрует точки по минимальной уверенности
    
    Args:
        deviations: Список (точка, отклонение)
        min_confidence: Минимальная уверенность (0-255)
        
    Returns:
        Отфильтрованный список
    """
    return [(point, dev) for point, dev in deviations if point.intensity >= min_confidence]

def filter_by_background_distance(deviations: List[Tuple[Point, float]], 
                                 max_background_distance: float) -> List[Tuple[Point, float]]:
    """
    Фильтрует точки, где фон слишком далеко (игнорируем бесконечности)
    
    Args:
        deviations: Список (точка, отклонение)
        max_background_distance: Максимальное расстояние фона
        
    Returns:
        Отфильтрованный список
    """
    filtered = []
    for point, dev in deviations:
        expected_distance = point.distance + dev  # Расстояние фона
        if expected_distance <= max_background_distance:
            filtered.append((point, dev))
    return filtered

def find_significant_deviations(deviations: List[Tuple[Point, float]], 
                               threshold: float) -> List[Tuple[Point, float]]:
    """
    Находит точки с отклонением больше порога
    
    Args:
        deviations: Список (точка, отклонение)
        threshold: Порог отклонения (мм)
        
    Returns:
        Список точек с отклонением > threshold
    """
    return [(point, dev) for point, dev in deviations if dev >= threshold]

def group_touch_points(touch_points: List[Tuple[Point, float]], 
                      angle_window: float) -> List[List[Tuple[Point, float]]]:
    """
    Группирует точки касания по угловому окну
    
    Args:
        touch_points: Список (точка, отклонение)
        angle_window: Угловое окно группировки (градусы)
        
    Returns:
        Список групп точек
    """
    if not touch_points:
        return []
        
    # Сортируем по углу
    sorted_points = sorted(touch_points, key=lambda x: x[0].angle)
    
    groups = []
    current_group = [sorted_points[0]]
    
    for point, deviation in sorted_points[1:]:
        last_angle = current_group[-1][0].angle
        
        # Учитываем переход через 0°
        angle_diff = abs(point.angle - last_angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
            
        if angle_diff <= angle_window:
            current_group.append((point, deviation))
        else:
            if current_group:
                groups.append(current_group)
            current_group = [(point, deviation)]
    
    if current_group:
        groups.append(current_group)
        
    return groups

def calculate_adaptive_threshold(expected_distance: float, 
                                multiplier: float = 0.1) -> float:
    """
    Вычисляет адаптивный порог на основе расстояния фона
    
    Args:
        expected_distance: Ожидаемое расстояние (мм)
        multiplier: Множитель (% от расстояния)
        
    Returns:
        Адаптивный порог (мм)
    """
    return max(20.0, expected_distance * multiplier)  # Минимум 20мм

def filter_sharp_transitions(current_scan: List[Point], 
                           max_distance_jump: float = 1.0,
                           angular_window: float = 0.0001) -> List[Point]:
    """
    Фильтрует точки с резкими изменениями расстояния (скачки на краях объектов)
    
    Args:
        current_scan: Текущий скан (список точек)
        max_distance_jump: Максимальный допустимый скачок расстояния (мм)
        angular_window: Угловое окно для поиска соседей (градусы)
        
    Returns:
        Отфильтрованный список точек без резких скачков
    """
    if not current_scan:
        return []
    
    # Создаем словарь для быстрого поиска точек по углу
    angle_to_point = {point.angle: point for point in current_scan}
    filtered_scan = []
    
    for point in current_scan:
        current_angle = point.angle
        current_distance = point.distance
        
        # Ищем соседние точки в угловом окне
        neighbor_distances = []
        for angle_offset in [-angular_window, -angular_window/2, angular_window/2, angular_window]:
            neighbor_angle = (current_angle + angle_offset) % 360
            if neighbor_angle in angle_to_point:
                neighbor_distances.append(angle_to_point[neighbor_angle].distance)
        
        # Если соседи найдены, проверяем скачки
        if neighbor_distances:
            # Считаем среднее расстояние соседей
            avg_neighbor_distance = np.mean(neighbor_distances)
            
            # Если разница больше порога - это резкий переход
            if abs(current_distance - avg_neighbor_distance) > max_distance_jump:
                # Пропускаем эту точку (это край объекта)
                continue
                
        # Точка прошла фильтр
        filtered_scan.append(point)
    
    return filtered_scan


