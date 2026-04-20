"""Конфигурационный файл для приложения"""

# Параметры подключения к лидару
LIDAR_CONFIG = {
    'host': '192.168.0.7',
    'port': 25168,
    'connection_timeout': 5.0,  # секунды
}

# Параметры калибровки
CALIBRATION_CONFIG = {
    'default_scan_count': 1,
    'min_scans': 1,
    'max_scans': 100,
}

# Параметры обработки данных
PROCESSING_CONFIG = {
    'background_grid_step': 1.0,      # градусы
    'angle_tolerance': 0.5,            # градусы для поиска точек
    'use_median_filter': True,         # использовать медиану вместо среднего
}

# Параметры визуализации
DISPLAY_CONFIG = {
    'enable_plot': True,
    'plot_size': (8, 8),
    'plot_title': "Фоновая карта (полярное представление)",
}

# Параметры производительности
PERFORMANCE_CONFIG = {
    'target_fps': 120,               # Повышенная частота
    'max_touch_points': 1,           # Только одна точка для точности
    'scan_point_limit': 50,
    'update_interval': 0.01,        # 1/100 секунды
    'fast_mode': True,
}

# Параметры управления мышью - ОПТИМИЗИРОВАННЫЕ
MOUSE_CONFIG = {
    'stability_threshold': 2.0,       # БОЛЕЕ строгий порог стабильности
    'min_stable_frames': 2,           # Меньше кадров для скорости
    'click_delay': 0.01,              # Минимальная задержка клика
    'position_smoothing': 0.1,        # МИНИМАЛЬНОЕ сглаживание для точности
    'max_touch_points': 1,            # ТОЛЬКО одна точка касания
    'touch_tracking_timeout': 0.5,    # БЫСТРЫЙ таймаут отслеживания
    'touch_average_scans': 3,         # МИНИМУМ усреднений для скорости
    'touch_lock_samples': 5,          # МЕНЬШЕ выборок для скорости
    'touch_relock_samples': 5,        # БЫСТРАЯ смена lock
    'touch_lock_radius_px': 100,      # МЕНЬШЕ мертвая зона
    'touch_relock_confirm_frames': 2, # БЫСТРОЕ подтверждение смены
    'touch_max_relock_step_px': 50,   # МЕНЬШЕ шаг lock
    'touch_relock_proximity_px': 30,  # НОВЫЙ параметр - близость для перезахвата
    'strict_static_touch_lock': False, # БОЛЕЕ отзывчивое позиционирование
    'high_precision_mode': True,      # НОВЫЙ параметр для высокой точности
    'prediction_factor': 0.3,         # НОВЫЙ параметр для предсказания движения (меньше для стабильности)
    'jitter_filter_threshold': 3.0,   # НОВЫЙ параметр - фильтр дрожания
    'movement_amplification': 1.0,    # НОВЫЙ параметр - усиление движения (1.0 = без усиления)
    # Snap-to-grid параметры
    'snap_grid_cell_size': 28,        # Размер ячейки сетки в пикселях (мёртвая зона)
    'snap_enabled': True,             # Включить привязку к сетке
}

class Config:
    """Центральный класс конфигурации"""
    
    def __init__(self):
        self.lidar = LIDAR_CONFIG.copy()
        self.calibration = CALIBRATION_CONFIG.copy()
        self.processing = PROCESSING_CONFIG.copy()
        self.display = DISPLAY_CONFIG.copy()
        self.performance = PERFORMANCE_CONFIG.copy()
        self.mouse = MOUSE_CONFIG.copy()
    
    def update_lidar_config(self, host=None, port=None, timeout=None):
        """Обновление параметров подключения"""
        if host is not None:
            self.lidar['host'] = host
        if port is not None:
            self.lidar['port'] = port
        if timeout is not None:
            self.lidar['connection_timeout'] = timeout
    
    def get_background_processing_params(self):
        """Получение параметров для обработки фона"""
        return {
            'step_deg': self.processing['background_grid_step'],
            'angle_tolerance': self.processing['angle_tolerance'],
            'use_median': self.processing['use_median_filter']
        }

    def get_performance_params(self):
        """Получение параметров производительности"""
        return self.performance.copy()
    
    def get_mouse_params(self):
        """Получение параметров управления мышью"""
        return self.mouse.copy()
    
    def update_mouse_config(self, **kwargs):
        """Обновление параметров управления мышью"""
        for key, value in kwargs.items():
            if key in self.mouse:
                self.mouse[key] = value

# Глобальная конфигурация
config = Config()
