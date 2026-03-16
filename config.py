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

class Config:
    """Центральный класс конфигурации"""
    
    def __init__(self):
        self.lidar = LIDAR_CONFIG.copy()
        self.calibration = CALIBRATION_CONFIG.copy()
        self.processing = PROCESSING_CONFIG.copy()
        self.display = DISPLAY_CONFIG.copy()
    
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

# Глобальная конфигурация
config = Config()

