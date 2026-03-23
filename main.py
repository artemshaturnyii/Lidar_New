"""
Основной модуль для работы с LiDAR M1
1. Подключение к лидару
2. Построение стартовой карты
3. Вывод графика стартовой карты
4. Детекция касаний в реальном времени с визуализацией
5. Фильтрация шума и постоянных ложных срабатываний
6. Калибровка углов проекции
7. Управление курсором мыши
"""

import time
import signal
import sys
import numpy as np
import matplotlib.pyplot as plt
import json
from typing import List, Optional
from lidar_sdk import LidarM1, Point
from calibration import LidarCalibrator
from calibration.file_io import load_background_from_file
from detection import TouchDetector, ThresholdConfig
from config import config
from noise_filter.noise_profiler import NoiseProfiler
from noise_filter.noise_filter import NoiseFilter
from persistent_noise_filter.persistent_noise_manager import PersistentNoiseManager
from noise_filter.corner_calibration import CornerCalibrator

# Глобальные переменные
lidar: Optional[LidarM1] = None
detector: Optional[TouchDetector] = None
noise_filter: Optional[NoiseFilter] = None
persistent_noise_manager: Optional[PersistentNoiseManager] = None
corner_points_scatter = None
touch_points_scatter = None
background_filename = "background.npz"
projection_corners_file = "projection_corners.json"
mouse_controller = None

# Для графика
fig = None
ax = None
background_line = None

def signal_handler(sig, frame):
    """Обработчик прерывания Ctrl+C"""
    print("\n\n🛑 Прерывание пользователя. Останавливаем лидар...")
    if lidar:
        lidar.stop()
    # Закрываем график если открыт
    if fig:
        plt.close(fig)
    print("✅ Завершение работы.")
    sys.exit(0)

def setup_lidar() -> LidarM1:
    """Инициализация и подключение к лидару"""
    print("🔌 Подключение к лидару...")
    lidar_instance = LidarM1()
    
    if not lidar_instance.connect():
        raise ConnectionError("Не удалось подключиться к лидару!")
    
    lidar_instance.start()
    print("✅ Лидар подключен и запущен")
    return lidar_instance

def create_background_map(num_scans: int = 10) -> dict:
    """
    Создание стартовой фоновой карты
    
    Args:
        num_scans: Количество сканов для усреднения
        
    Returns:
        Фоновая карта в формате {угол: расстояние}
    """
    print(f"📏 Создание стартовой карты. Будет собрано {num_scans} сканов...")
    
    collected_scans = []
    scan_count = 0
    
    def on_scan_received(scan):
        nonlocal scan_count
        collected_scans.append(scan)
        scan_count += 1
        print(f"  📸 Собран скан {scan_count}/{num_scans}")
        
        if scan_count >= num_scans:
            lidar.on_scan_complete = None  # Отменяем callback
    
    # Регистрируем callback
    lidar.on_scan_complete = on_scan_received
    
    # Ждем завершения сбора
    try:
        while scan_count < num_scans:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n❌ Создание карты прервано пользователем.")
        raise
    
    # Обрабатываем данные
    print("📊 Обрабатываем собранные данные...")
    calibrator = LidarCalibrator()
    background_map = calibrator.process_scans_to_background(collected_scans)
    
    # Сохраняем фоновую карту
    calibrator.save_background(background_filename, background_map)
    print(f"💾 Карта сохранена в '{background_filename}'")
    
    return background_map

def plot_background_map(background_map: dict):
    """Вывод графика стартовой карты"""
    print("📊 Вывод графика стартовой карты...")
    
    angles = list(background_map.keys())
    distances = list(background_map.values())
    
    # Полярный график
    global fig, ax, background_line
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
    background_line, = ax.plot(np.radians(angles), distances, 'b-', linewidth=1, label='Фон')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_title("Стартовая карта и касания (полярное представление)", fontsize=14)
    ax.grid(True)
    ax.legend()
    
    # Показываем график в неблокирующем режиме
    plt.ion()
    plt.show(block=False)  # Не блокировать выполнение
    plt.pause(0.01)
    
    print("✅ График стартовой карты выведен")

def initialize_touch_detector(background_map: dict) -> TouchDetector:
    """Инициализация детектора касаний"""
    print("🔍 Инициализация детектора касаний...")
    detector = TouchDetector(background_map, ThresholdConfig.sensitive())
    print("✅ Детектор касаний готов")
    return detector

def noise_profiling_phase(duration_seconds: int = 30):
    """Фаза профилирования шума"""
    print(f"\n📊 Фаза профилирования шума ({duration_seconds} секунд)")
    print("Убедитесь, что НИКТО НЕ КАСАЕТСЯ стены!")
    
    profiler = NoiseProfiler(duration_seconds=duration_seconds)
    profiler.start_noise_collection(lidar, detector)
    
    # Создаем фильтр на основе профиля
    global noise_filter
    noise_filter = NoiseFilter(profiler.noise_profile, persistent_noise_manager)
    
    print("✅ Профилирование шума завершено")
    return noise_filter

def update_projection_display(corners: List[dict], touch_points: List[Point] = None):
    """Обновление отображения углов проекции и касаний внутри четырехугольника"""
    global fig, ax, corner_points_scatter, touch_points_scatter
    
    if not fig:
        return
        
    # Удаляем старые точки касаний если они были
    if touch_points_scatter:
        try:
            touch_points_scatter.remove()
        except:
            pass
    touch_points_scatter = None
    
    # Отображаем углы проекции
    if corners:
        corner_angles = [np.radians(corner['angle']) for corner in corners]
        corner_distances = [corner['distance'] for corner in corners]
        
        # Удаляем старые точки углов если они были
        if corner_points_scatter:
            try:
                corner_points_scatter.remove()
            except:
                pass
        
        # Отображаем углы проекции
        corner_points_scatter = ax.scatter(
            corner_angles, corner_distances, 
            c='orange', s=100, alpha=0.9, marker='s', 
            label='Углы проекции', edgecolors='black', linewidth=2
        )
    
    # Отображаем касания внутри четырехугольника
    if touch_points and touch_points:
        touch_angles = [np.radians(p.angle) for p in touch_points]
        touch_distances = [p.distance for p in touch_points]
        
        # Отображаем касания зелеными точками
        touch_points_scatter = ax.scatter(
            touch_angles, touch_distances, 
            c='green', s=50, alpha=0.8, marker='o', 
            label='Касания внутри проекции'
        )
    
    # Обновляем легенду
    ax.legend()
    
    # Обновляем график
    plt.draw()
    plt.pause(0.01)
    plt.show(block=False)

def point_in_polygon(point: Point, polygon_corners: List[dict]) -> bool:
    """
    Проверяет, находится ли точка внутри четырехугольника методом ray casting
    
    Args:
        point: точка касания (Point)
        polygon_corners: список углов проекции (4 точки)
        
    Returns:
        bool: True если точка внутри четырехугольника
    """
    if not polygon_corners or len(polygon_corners) != 4:
        return False
    
    # Преобразуем углы в декартовы координаты для более точной проверки
    def polar_to_cartesian(angle_deg, distance):
        angle_rad = np.radians(angle_deg)
        x = distance * np.cos(angle_rad)
        y = distance * np.sin(angle_rad)
        return x, y
    
    # Преобразуем точку в декартовы координаты
    px, py = polar_to_cartesian(point.angle, point.distance)
    
    # Преобразуем углы четырехугольника в декартовы координаты
    polygon_points = []
    for corner in polygon_corners:
        x, y = polar_to_cartesian(corner['angle'], corner['distance'])
        polygon_points.append((x, y))
    
    # Алгоритм ray casting
    n = len(polygon_points)
    inside = False
    
    p1x, p1y = polygon_points[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon_points[i % n]
        if py > min(p1y, p2y):
            if py <= max(p1y, p2y):
                if px <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (py - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or px <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    
    return inside

def save_projection_corners(corners: List[dict]):
    """Сохраняет координаты углов в JSON файл"""
    corners_data = {
        'timestamp': time.time(),
        'corners': corners
    }
    
    with open(projection_corners_file, 'w', encoding='utf-8') as f:
        json.dump(corners_data, f, ensure_ascii=False, indent=2)
    print(f"💾 Координаты углов сохранены в '{projection_corners_file}'")

def load_projection_corners() -> Optional[List[dict]]:
    """Загружает координаты углов из JSON файла"""
    try:
        with open(projection_corners_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data['corners']
    except FileNotFoundError:
        print(f"⚠️  Файл '{projection_corners_file}' не найден")
        return None
    except Exception as e:
        print(f"❌ Ошибка загрузки координат углов: {e}")
        return None

def corner_calibration_phase():
    """Фаза калибровки углов проекции"""
    print("\n🎯 Фаза калибровки углов проекции")
    print("Будет выполнена калибровка всех 4 углов проекции")
    print("Пожалуйста, следуйте инструкциям на экране")
    
    # Создаем калибратор углов
    corner_calibrator = CornerCalibrator(noise_filter)
    
    try:
        # Выполняем калибровку всех углов
        corners = corner_calibrator.calibrate_all_corners(lidar, detector)
        
        if corners:
            # Сохраняем координаты углов
            save_projection_corners(corners)
            
            # Выводим точки углов на график
            if config.display['enable_plot']:
                update_projection_display(corners)
            
            print(f"\n✅ Калибровка углов завершена успешно!")
            print("Координаты углов:")
            for corner in corners:
                print(f"  {corner['name']}: угол {corner['angle']:.2f}°, расстояние {corner['distance']:.0f}мм")
            
            return corners
        else:
            print("❌ Калибровка углов не удалась")
            return None
            
    except Exception as e:
        print(f"❌ Ошибка калибровки углов: {e}")
        return None

def initialize_mouse_control(corners: List[dict]):
    """Инициализация контроллера мыши"""
    global mouse_controller
    
    try:
        from mouse_controller import MouseController
        mouse_controller = MouseController()
        mouse_controller.enable_control()
        
        if corners:
            mouse_controller.set_projection_corners(corners)
            print("🖱️  Контроллер мыши инициализирован и активирован")
        else:
            print("🖱️  Контроллер мыши инициализирован (ожидание углов)")
            
        return mouse_controller
    except Exception as e:
        print(f"⚠️  Не удалось инициализировать контроллер мыши: {e}")
        mouse_controller = None
        return None

def realtime_detection_with_filtering():
    """Режим реального времени с фильтрацией и визуализацией"""
    print("\n🔄 Переход в режим реального времени с фильтрацией и визуализацией...")
    print("⌨️  Нажмите Ctrl+C для остановки")
    print("-" * 80)
    print("Формат вывода:")
    print("  [TIME] ALL:n | FILTERED:m | REMOVED:k | PERSISTENT:p | RATE:x%")
    print("-" * 80)
    
    # Загружаем углы проекции если они есть
    corners = load_projection_corners()
    if corners:
        print(f"🎯 Загружены координаты углов проекции ({len(corners)} точек)")
    else:
        print("⚠️  Координаты углов проекции не найдены")
    
    # Инициализация контроллера мыши
    mouse_ctrl = initialize_mouse_control(corners)
    
    # Отображаем углы на графике
    if config.display['enable_plot'] and corners:
        update_projection_display(corners)
    
    scan_counter = 0
    total_all = 0
    total_filtered = 0
    persistent_noise_count = 0
    inside_frame_count = 0  # Счетчик касаний внутри рамки
    
    try:
        while True:
            current_scan = lidar.get_last_scan()
            
            if current_scan:
                scan_counter += 1
                
                # Получаем все точки касания
                all_touch_points = detector.detect_touch_points(current_scan)
                total_all += len(all_touch_points)
                
                # Разделяем точки на категории
                persistent_noise_points = []
                regular_noise_points = []
                real_touch_points = []
                
                for point in all_touch_points:
                    if persistent_noise_manager.is_persistent_noise(point):
                        persistent_noise_points.append(point)
                        persistent_noise_count += 1
                    elif noise_filter.is_false_positive(point):
                        regular_noise_points.append(point)
                    else:
                        real_touch_points.append(point)
                
                # Финальный список отфильтрованных точек (только реальные касания)
                filtered_points = real_touch_points
                
                # Фильтруем только точки внутри четырехугольника проекции
                frame_touch_points = []
                if corners:
                    for point in filtered_points:
                        if point_in_polygon(point, corners):
                            frame_touch_points.append(point)
                else:
                    frame_touch_points = filtered_points  # Если нет углов, показываем все
                
                inside_frame_count += len(frame_touch_points)
                
                total_filtered += len(filtered_points)
                removed_count = len(all_touch_points) - len(filtered_points)
                
                if len(all_touch_points) > 0:
                    removal_rate = (removed_count / len(all_touch_points)) * 100
                else:
                    removal_rate = 0
                
                # Обновляем отображение углов и касаний внутри четырехугольника
                if config.display['enable_plot']:
                    update_projection_display(corners, frame_touch_points)
                
                # Управление мышью - перемещаем курсор к первой точке касания
                if mouse_ctrl and frame_touch_points and mouse_ctrl.is_active:
                    # Берем первую точку касания для управления мышью
                    primary_touch = frame_touch_points[0]
                    mouse_ctrl.move_mouse_to_touch(primary_touch)
                
                # Выводим статистику в консоль
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] ALL:{len(all_touch_points):2d} | "
                      f"FILTERED:{len(filtered_points):2d} | "
                      f"REMOVED:{removed_count:2d} | "
                      f"PERSISTENT:{len(persistent_noise_points):2d} | "
                      f"INSIDE:{len(frame_touch_points):2d} | "
                      f"RATE:{removal_rate:5.1f}%")
                
                # Если есть касания внутри рамки, показываем их
                if len(frame_touch_points) > 0:
                    print("    🎯 Касания внутри рамки:")
                    for i, point in enumerate(frame_touch_points[:5]):  # Показываем максимум 5 точек
                        # Получаем экранные координаты
                        screen_coords = None
                        if corners:
                            screen_coords = map_touch_to_screen_coordinates(point, corners)
                        
                        if screen_coords:
                            x, y = screen_coords
                            print(f"      Экран: ({x:.3f}, {y:.3f}) | "
                                  f"Угол: {point.angle:6.1f}° | "
                                  f"Расст.: {point.distance:6.0f}мм | "
                                  f"Инт-сть: {point.intensity:3.0f}")
                        else:
                            print(f"      Угол: {point.angle:6.1f}° | "
                                  f"Расст.: {point.distance:6.0f}мм | "
                                  f"Инт-сть: {point.intensity:3.0f}")
                    
                    if len(frame_touch_points) > 5:
                        print(f"      ... и еще {len(frame_touch_points) - 5} точек")
                
                # Периодически показываем статистику
                if scan_counter % 30 == 0 and scan_counter > 0:  # Каждые 30 сканов
                    print(f"    📊 Статистика: Всего касаний внутри рамки: {inside_frame_count}")
                
                # Пауза чтобы не перегружать вывод и график
                time.sleep(0.05)
            else:
                time.sleep(0.05)
                
    except KeyboardInterrupt:
        print("\n⏹️  Остановка режима реального времени")
        
        # Отключаем управление мышью
        if mouse_ctrl:
            mouse_ctrl.disable_control()
        
        # Финальная статистика
        if total_all > 0:
            final_removal_rate = ((total_all - total_filtered) / total_all) * 100
            print(f"\n📊 Финальная статистика:")
            print(f"   Всего обработано сканов: {scan_counter}")
            print(f"   Общее количество касаний: {total_all}")
            print(f"   После фильтрации: {total_filtered}")
            print(f"   Удалено ложных срабатываний: {total_all - total_filtered}")
            print(f"   Из них постоянный шум: {persistent_noise_count}")
            print(f"   Касаний внутри рамки: {inside_frame_count}")
            print(f"   Эффективность фильтра: {final_removal_rate:.1f}%")

def map_touch_to_screen_coordinates(touch_point: Point, corners: List[dict]):
    """
    Преобразует координаты касания LiDAR в координаты экрана
    
    Args:
        touch_point: Point с координатами касания
        corners: список углов проекции
        
    Returns:
        tuple: (x, y) координаты на экране (0-1) или None
    """
    if not corners or len(corners) != 4:
        return None
        
    # Извлекаем координаты углов
    angles = [c['angle'] for c in corners]
    distances = [c['distance'] for c in corners]
    
    # Находим диапазоны
    min_angle = min(angles)
    max_angle = max(angles)
    min_distance = min(distances)  
    max_distance = max(distances)
    
    if max_angle == min_angle or max_distance == min_distance:
        return None
        
    # Нормализуем координаты в диапазон 0-1
    normalized_x = (touch_point.angle - min_angle) / (max_angle - min_angle)
    normalized_y = 1.0 - (touch_point.distance - min_distance) / (max_distance - min_distance)
    
    # Ограничиваем диапазон
    normalized_x = max(0.0, min(1.0, normalized_x))
    normalized_y = max(0.0, min(1.0, normalized_y))
    
    return (normalized_x, normalized_y)

def main():
    """Основная функция программы"""
    global lidar, detector, persistent_noise_manager
    
    # Регистрируем обработчик сигнала
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 1. Подключение к лидару
        lidar = setup_lidar()
        
        # 2. Инициализация менеджера постоянного шума
        print("📂 Инициализация менеджера постоянного шума...")
        persistent_noise_manager = PersistentNoiseManager()
        
        # 3. Создание стартовой карты
        background_map = create_background_map(num_scans=10)
        
        # 4. Вывод графика стартовой карты
        if config.display['enable_plot']:
            plot_background_map(background_map)
        
        # 5. Инициализация детектора касаний
        detector = initialize_touch_detector(background_map)
        
        # 6. Профилирование шума (этап калибровки)
        noise_filter_instance = noise_profiling_phase(duration_seconds=5)
        
        # 7. Калибровка углов проекции (новая фаза)
        print("\n" + "="*60)
        print("階段 7: Калибровка углов проекции")
        print("="*60)
        
        corners = corner_calibration_phase()
        
        if not corners:
            print("❌ Калибровка углов не удалась. Продолжаем без нее.")
        else:
            print("✅ Калибровка углов успешно завершена!")
        
        # 8. Режим реального времени с фильтрацией и визуализацией
        print("\n" + "="*60)
        print("階段 8: Режим реального времени")
        print("="*60)
        
        realtime_detection_with_filtering()
            
    except Exception as e:
        print(f"❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()
        if lidar:
            lidar.stop()
        sys.exit(1)
        
    finally:
        # Завершение работы
        if lidar:
            print("\n🛑 Остановка лидара...")
            lidar.stop()
            print("✅ Работа завершена")

if __name__ == "__main__":
    main()
