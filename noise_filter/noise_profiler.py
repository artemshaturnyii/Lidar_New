"""Сбор статистики шума для фильтрации ложных срабатываний"""

import time
import numpy as np
from typing import List, Dict
import json
from lidar_sdk import Point

class NoiseProfiler:
    """Сбор статистики шума для фильтрации ложных срабатываний"""
    
    def __init__(self, duration_seconds=5):
        self.duration = duration_seconds
        self.noise_points = []  # [(angle, distance, frequency)]
        self.is_collecting = False
        self.noise_profile = {}
        
    def start_noise_collection(self, lidar, detector):
        """Начинает сбор шума в течение заданного времени"""
        self.noise_points = []
        self.is_collecting = True
        
        start_time = time.time()
        collection_count = 0
        
        print(f"📊 Сбор шума на протяжении {self.duration} секунд...")
        print("Убедитесь, что НИКТО НЕ КАСАЕТСЯ стены!")
        
        while time.time() - start_time < self.duration:
            current_scan = lidar.get_last_scan()
            if current_scan:
                # Ищем ложные срабатывания (без реального касания)
                touch_points = detector.detect_touch_points(current_scan)
                
                for point in touch_points:
                    self.noise_points.append({
                        'angle': point.angle,
                        'distance': point.distance,
                        'intensity': point.intensity,
                        'timestamp': time.time()
                    })
                    
                collection_count += 1
                if collection_count % 20 == 0:  # Чаще показываем прогресс
                    print(f"  Собрано {collection_count} циклов...")
            
            time.sleep(0.1)
            
        self.is_collecting = False
        self._analyze_noise_profile()
        print("✅ Сбор шума завершен")
        
    def _analyze_noise_profile(self):
        """Анализирует собранные данные шума"""
        if not self.noise_points:
            print("⚠️  Не собрано данных шума")
            return
            
        # Группируем точки по угловым секторам (например, 5° сектора)
        sector_size = 5.0
        sectors = {}
        
        for point in self.noise_points:
            sector = int(point['angle'] / sector_size) * sector_size
            if sector not in sectors:
                sectors[sector] = []
            sectors[sector].append(point)
            
        # Вычисляем частоту срабатываний в каждом секторе
        noise_profile = {}
        for sector, points in sectors.items():
            # Среднее расстояние и частота срабатываний
            if points:
                avg_distance = np.mean([p['distance'] for p in points])
                frequency = len(points) / self.duration  # срабатываний в секунду
                
                noise_profile[sector] = {
                    'avg_distance': float(avg_distance),
                    'frequency': float(frequency),
                    'count': len(points)
                }
            
        self.noise_profile = noise_profile
        self._save_noise_profile()
        print(f"📊 Проанализировано {len(self.noise_points)} точек шума")
        print(f"📁 Создан профиль шума для {len(noise_profile)} секторов")
        
    def _save_noise_profile(self, filename='noise_profile.json'):
        """Сохраняет профиль шума в файл"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.noise_profile, f, indent=2)
            print(f"💾 Профиль шума сохранен в '{filename}'")
        except Exception as e:
            print(f"❌ Ошибка сохранения профиля шума: {e}")
            
    def load_noise_profile(self, filename='noise_profile.json'):
        """Загружает профиль шума из файла"""
        try:
            with open(filename, 'r') as f:
                self.noise_profile = json.load(f)
            print(f"📂 Профиль шума загружен из '{filename}'")
        except FileNotFoundError:
            print(f"⚠️  Файл профиля шума '{filename}' не найден")
            self.noise_profile = {}
        except Exception as e:
            print(f"❌ Ошибка загрузки профиля шума: {e}")
            self.noise_profile = {}
