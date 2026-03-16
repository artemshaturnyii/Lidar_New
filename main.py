import time
import signal
import sys
import numpy as np
from lidar_sdk import LidarM1, Point
from calibration import LidarCalibrator
from config import config

# Попытка импорта matplotlib
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Matplotlib не установлен. График не будет показан. Установите: pip install matplotlib")

# Глобальная переменная для корректного завершения
lidar = None

def signal_handler(sig, frame):
    print("\nПрерывание пользователя. Останавливаем лидар...")
    if lidar:
        lidar.stop()
    sys.exit(0)

def show_calibration_results(background_map: dict):
    """Показывает результаты калибровки"""
    if not background_map:
        return
        
    angles = sorted(background_map.keys())
    distances = [background_map[a] for a in angles]
    print(f"Построена карта на {len(angles)} угловых интервалах.")
    print(f"Диапазон расстояний: {min(distances):.1f} – {max(distances):.1f} мм")
    
    # Пример: выведем значения для нескольких углов
    test_angles = [0, 45, 90, 135, 180, 225, 270, 315]
    print("\nФоновые расстояния для некоторых углов:")
    for a in test_angles:
        if a in background_map:
            bg = background_map[a]
            print(f"  Угол {a:3}°: {bg:.1f} мм")

def show_polar_plot(background_map: dict):
    """Показывает полярный график"""
    if not HAS_MATPLOTLIB or not background_map:
        return
        
    angles = sorted(background_map.keys())
    distances = [background_map[a] for a in angles]
    
    plt.figure(figsize=(8,8))
    ax = plt.subplot(111, projection='polar')
    # Углы в радианах
    angles_rad = np.deg2rad(angles)
    ax.plot(angles_rad, distances, 'b-', linewidth=1)
    ax.set_title(config.display['plot_title'])
    # Настройка: 0° — север, вращение по часовой стрелке
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    plt.show()

def main():
    global lidar

    # 1. Инициализация и подключение лидара
    lidar = LidarM1()
    print("Подключение к лидару...")
    if not lidar.connect():
        print("Не удалось подключиться к лидару!")
        return
    lidar.start()
    print("Лидар запущен.")

    # 2. СБОР СКАНОВ
    num_scans = config.calibration['default_scan_count']
    print(f"Начинаем калибровку. Будет собрано {num_scans} сканов...")
    
    collected_scans = []
    scan_count = 0
    
    def on_scan_received(scan):
        nonlocal scan_count
        collected_scans.append(scan)
        scan_count += 1
        print(f"  Собран скан {scan_count}/{num_scans}")
        
        # Останавливаем сбор если достаточно сканов
        if scan_count >= num_scans:
            lidar.on_scan_complete = None  # Отменяем callback
    
    # App layer регистрирует callback
    lidar.on_scan_complete = on_scan_received
    
    # Ждем завершения сбора
    try:
        while scan_count < num_scans:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nКалибровка прервана пользователем.")
        lidar.stop()
        sys.exit(0)

    # 3. ОБРАБОТКА ДАННЫХ
    print("Обрабатываем собранные данные...")
    calibrator = LidarCalibrator()
    background_map = calibrator.process_scans_to_background(collected_scans)
    
    # Остальной код без изменений...

    
    # 4. ПОКАЗ РЕЗУЛЬТАТОВ (App layer)
    print("\n=== Калибровка завершена ===")
    show_calibration_results(background_map)
    
    # 5. СОХРАНЕНИЕ РЕЗУЛЬТАТОВ (App layer решает что делать с результатами)
    filename = "background.npz"
    calibrator.save_background(filename, background_map)
    print(f"\nКарта сохранена в файл '{filename}'.")
    
    # 6. ВИЗУАЛИЗАЦИЯ (App layer)
    if config.display['enable_plot']:
        show_polar_plot(background_map)
    
    # 7. ЗАВЕРШЕНИЕ
    lidar.stop()
    print("Лидар остановлен.")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
