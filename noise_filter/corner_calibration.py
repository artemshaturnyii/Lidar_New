"""Калибровка углов проекции - поиск точек, встречающихся во всех сканах"""

import time
import numpy as np
from typing import List, Dict, Optional, Callable
from lidar_sdk import Point

class CornerCalibrator:
    """Калибровка углов проекции методом поиска стабильных точек"""
    
    def __init__(self, noise_filter=None):
        self.noise_filter = noise_filter
        
    def calibrate_corner(self, lidar, detector, corner_name: str, scan_count: int = 20, log_callback: Callable = None) -> Dict:
        """
        Калибрует один угол - находит точку, которая встречается во всех сканах
        
        Args:
            lidar: объект LiDAR
            detector: детектор касаний
            corner_name: название угла
            scan_count: количество сканов для сбора
            log_callback: функция для логирования сообщений
            
        Returns:
            Dict с координатами угла
        """
        def log(message):
            if log_callback:
                log_callback(message)
            else:
                print(message)
                
        log(f"🎯 Начинаем калибровку угла: {corner_name}")
        log(f"Сбор {scan_count} сканов...")
        
        scan_collection = []  # Список сканов
        scan_counter = 0
        
        # Собираем заданное количество сканов
        while scan_counter < scan_count:
            current_scan = lidar.get_last_scan()
            if current_scan:
                # Получаем точки касания
                touch_points = detector.detect_touch_points(current_scan)
                
                # Фильтруем шум если фильтр доступен
                if self.noise_filter:
                    filtered_points = self.noise_filter.filter_touch_points(touch_points)
                else:
                    filtered_points = touch_points
                
                if filtered_points:
                    scan_collection.append(filtered_points)
                    scan_counter += 1
                    log(f"  Собран скан {scan_counter}/{scan_count} для угла {corner_name}")
                else:
                    log(f"  Скан {scan_counter + 1}: нет касаний для угла {corner_name}")
                    
            time.sleep(0.1)
        
        if not scan_collection:
            raise Exception(f"Не удалось собрать данные для угла {corner_name}")
        
        # Находим точки, которые встречаются во всех сканах
        common_points = self._find_common_points(scan_collection)
        
        if not common_points:
            log(f"⚠️  Не найдено точек, встречающихся во всех сканах для угла {corner_name}")
            log("Попробуем найти точки, встречающиеся в большинстве сканов...")
            common_points = self._find_majority_points(scan_collection, threshold=0.8)
        
        if not common_points:
            raise Exception(f"Не удалось найти стабильные точки для угла {corner_name}")
        
        # Вычисляем средние координаты
        avg_angle = np.mean([p.angle for p in common_points])
        avg_distance = np.mean([p.distance for p in common_points])
        
        # Вычисляем стандартные отклонения
        angle_std = np.std([p.angle for p in common_points])
        distance_std = np.std([p.distance for p in common_points])
        
        corner_data = {
            'name': corner_name,
            'angle': float(avg_angle),
            'distance': float(avg_distance),
            'angle_std': float(angle_std),
            'distance_std': float(distance_std),
            'sample_count': len(common_points),
            'total_scans': len(scan_collection)
        }
        
        log(f"✅ Угол {corner_name} успешно откалиброван:")
        log(f"   Угол: {avg_angle:.2f}° (±{angle_std:.2f}°)")
        log(f"   Расстояние: {avg_distance:.0f}мм (±{distance_std:.0f}мм)")
        log(f"   Стабильных точек: {len(common_points)} из {len(scan_collection)} сканов")
        
        return corner_data
    
    def _find_common_points(self, scan_collection: List[List[Point]], 
                           angle_tolerance: float = 2.0, 
                           distance_tolerance: float = 30.0) -> List[Point]:
        """
        Находит точки, которые встречаются во ВСЕХ сканах
        
        Args:
            scan_collection: список сканов (каждый скан - список точек)
            angle_tolerance: допуск по углу в градусах
            distance_tolerance: допуск по расстоянию в мм
            
        Returns:
            Список стабильных точек
        """
        if not scan_collection:
            return []
        
        # Берем первый скан как базовый
        reference_scan = scan_collection[0]
        common_points = []
        
        for ref_point in reference_scan:
            # Проверяем, есть ли эта точка во всех остальных сканах
            found_in_all = True
            
            for scan in scan_collection[1:]:
                point_found = False
                for point in scan:
                    # Проверяем близость по углу и расстоянию
                    angle_diff = abs(ref_point.angle - point.angle)
                    distance_diff = abs(ref_point.distance - point.distance)
                    
                    if angle_diff <= angle_tolerance and distance_diff <= distance_tolerance:
                        point_found = True
                        break
                
                if not point_found:
                    found_in_all = False
                    break
            
            if found_in_all:
                common_points.append(ref_point)
        
        return common_points
    
    def _find_majority_points(self, scan_collection: List[List[Point]], 
                             threshold: float = 0.6,
                             angle_tolerance: float = 5.0, 
                             distance_tolerance: float = 30.0) -> List[Point]:
        """
        Находит точки, которые встречаются в большинстве сканов
        
        Args:
            scan_collection: список сканов
            threshold: минимальная доля сканов (0.8 = 80%)
            angle_tolerance: допуск по углу
            distance_tolerance: допуск по расстоянию
            
        Returns:
            Список точек, встречающихся в большинстве сканов
        """
        if not scan_collection:
            return []
        
        min_scans = int(len(scan_collection) * threshold)
        all_points = []
        
        # Собираем все точки из всех сканов
        for scan in scan_collection:
            all_points.extend(scan)
        
        # Группируем близкие точки
        point_groups = []
        for point in all_points:
            # Ищем существующую группу для этой точки
            group_found = False
            for group in point_groups:
                # Берем первую точку группы как представителя
                representative = group[0]
                angle_diff = abs(point.angle - representative.angle)
                distance_diff = abs(point.distance - representative.distance)
                
                if angle_diff <= angle_tolerance and distance_diff <= distance_tolerance:
                    group.append(point)
                    group_found = True
                    break
            
            if not group_found:
                point_groups.append([point])
        
        # Находим группы, которые встречаются в достаточном количестве сканов
        majority_points = []
        for group in point_groups:
            if len(group) >= min_scans:
                # Берем среднюю точку группы
                avg_angle = np.mean([p.angle for p in group])
                avg_distance = np.mean([p.distance for p in group])
                # Создаем представительную точку
                representative_point = Point(
                    angle=avg_angle,
                    distance=avg_distance,
                    intensity=np.mean([p.intensity for p in group])
                )
                majority_points.append(representative_point)
        
        return majority_points
    
    def calibrate_all_corners(self, lidar, detector, log_callback: Callable = None) -> List[Dict]:
        """Калибрует все 4 угла проекции с временными задержками"""
        def log(message):
            if log_callback:
                log_callback(message)
            else:
                print(message)
            
        # Используем только английские названия
        corner_names = [
            "top_right",
            "bottom_right", 
            "bottom_left",
            "top_left"
        ]
    
        log("🔄 Starting calibration of all projection corners")
        log("Please prepare for corner calibration")
        log("⏳ Waiting 10 seconds before starting...")
        time.sleep(10)
    
        corners = []
        for i, corner_name in enumerate(corner_names):
            log(f"\n{'='*60}")
            log(f"🔍 CURRENT STEP: Calibrating corner {i+1}/4")
            log(f"📍 CORNER TO CALIBRATE: {corner_name.upper()}")
            log(f"{'='*60}")
        
            # Перед первым углом дополнительная пауза не нужна (уже была)
            if i > 0:
                log(f"⏳ Waiting 3 seconds to move to corner '{corner_name}'...")
                time.sleep(3)
        
            try:
                corner_data = self.calibrate_corner(lidar, detector, corner_name, scan_count=20, log_callback=log_callback)
                corners.append(corner_data)
            
                # Пауза после калибровки угла (кроме последнего)
                if i < len(corner_names) - 1:
                    log(f"✅ Corner {corner_name} completed. Prepare for next corner...")
                    log(f"⏳ Waiting 3 seconds until next corner...")
                    time.sleep(3)
                
            except Exception as e:
                log(f"❌ ERROR calibrating corner {corner_name}: {e}")
                # Продолжаем с другими углами
                if i < len(corner_names) - 1:
                    log(f"⏳ Moving to next corner in 3 seconds...")
                    time.sleep(3)
                continue
    
        log(f"\n{'='*60}")
        log("🎉 ALL CORNERS SUCCESSFULLY CALIBRATED!")
        log(f"{'='*60}")
    
        return corners
