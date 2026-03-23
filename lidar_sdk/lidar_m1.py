import threading
import time
import struct
from typing import List, Optional, Callable
from .connection import LidarConnection
from .protocol import PACKET_FMT, PACKET_SIZE, M1_HEADER, NODES_PER_PACK, crc8
from .models import Point

class LidarM1:
    """Основной класс для работы с лидаром M1"""
    
    def __init__(self):
        """Инициализация драйвера лидара"""
        self.connection = LidarConnection()  # Берет параметры из config
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._current_scan: List[Point] = []  # буфер текущего оборота
        self._last_scan: List[Point] = []     # последний завершённый скан
        self._last_angle: Optional[float] = None  # угол последней обработанной точки
        self.lock = threading.Lock()
        self.on_scan_complete: Optional[Callable[[List[Point]], None]] = None
        self.on_point_received: Optional[Callable[[Point], None]] = None

    def connect(self) -> bool:
        """Подключается к лидару"""
        return self.connection.connect()

    def start(self) -> None:
        """Запускает фоновый поток чтения"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Останавливает поток и закрывает соединение"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.connection.disconnect()

    def _reader(self) -> None:
        """Фоновый поток: читает пакеты и обрабатывает их"""
        print("🔄 Начинаю чтение данных...")
        buffer = bytearray()
    
        while self._running:
            try:
                # Читаем небольшими порциями
                chunk = self.connection.recv_exact(256)
                if not chunk:
                    print("❌ Получены пустые данные")
                    break
                
                buffer.extend(chunk)
            
                # Ищем пакеты в буфере
                i = 0
                while i < len(buffer) - 2:
                    if buffer[i] == 0x54 and buffer[i+1] == 0x2C:
                        # Ищем следующий пакет или конец буфера
                        j = i + 2
                        found_next = False
                        while j < len(buffer) - 1:
                            if buffer[j] == 0x54 and buffer[j+1] == 0x2C:
                                # Нашли следующий пакет
                                packet_data = buffer[i:j]
                                if len(packet_data) >= 10:
                                    self._process_packet(bytes(packet_data))
                                i = j - 1
                                found_next = True
                                break
                            j += 1
                    
                        if not found_next:
                            # Это последний пакет в буфере, ждем больше данных
                            buffer = buffer[i:]
                            break
                    i += 1
                
                # Ограничиваем размер буфера
                if len(buffer) > 2048:
                    buffer = buffer[-1024:]
                
            except Exception as e:
                if self._running:  # Только если не остановка
                    print(f"❌ Reader error: {e}")
                    import traceback
                    traceback.print_exc()
                break
    
        print("⏹️  Чтение остановлено.")

    def _parse_packet(self, data):
        """Распаковывает и проверяет пакет. Возвращает None при ошибке."""
        if len(data) < 10:
            return None
        
        header = data[0]
        if header != M1_HEADER:
         return None
        
        # Пробуем разные размеры пакетов
        for node_count in [0x0C, 0x2C]:  # Наиболее частые значения
            try:
                fmt = '<BBHH' + 'HB' * node_count + 'HHB'
                expected_size = struct.calcsize(fmt)
            
                if len(data) >= expected_size:
                    packet_to_check = data[:expected_size]
                
                    # Проверка CRC
                    calculated_crc = crc8(packet_to_check[:-1])
                    packet_crc = packet_to_check[-1]
                
                    if calculated_crc == packet_crc:
                        # Нашли правильный пакет!
                        fields = struct.unpack(fmt, packet_to_check)
                        speed = fields[2]
                        start_angle = fields[3] * 0.01
                        end_angle = fields[4 + 2*node_count] * 0.01
                    
                        nodes = []
                        for i in range(node_count):
                            dist = fields[4 + i*2]
                            conf = fields[4 + i*2 + 1]
                            nodes.append((dist, conf))
                        
                        return start_angle, end_angle, nodes
                    
            except struct.error:
               print(f"❌ Не удалось распознать пакет размером {len(data)}")

        #print("Первые 10 байт:", ' '.join(f'{b:02X}' for b in data[:10]))
        return None




    def _process_packet(self, data):
        """Обрабатывает один пакет, разбирает точки и определяет завершение оборота."""
        # print(f"🔧 Обработка пакета, размер: {len(data)} байт")
        
        parsed = self._parse_packet(data)
        if parsed is None:
            # print("⚠️  Не удалось распарсить пакет")
            return
            
        start_angle, end_angle, nodes = parsed
        # print(f"📊 Пакет распознан: {start_angle:.1f}° → {end_angle:.1f}° ({len(nodes)} точек)")

        # Вычисляем шаг угла между точками в пакете
        if end_angle < start_angle:
            step = (end_angle + 360 - start_angle) / (NODES_PER_PACK - 1)
        else:
            step = (end_angle - start_angle) / (NODES_PER_PACK - 1)

        # Обрабатываем каждую точку пакета
        for i, (dist, conf) in enumerate(nodes):
            angle = (start_angle + step * i) % 360
            pt = Point(angle=angle, distance=float(dist), intensity=float(conf))

            with self.lock:
                # Если есть предыдущая точка и угол уменьшился (переход через 0°)
                if self._last_angle is not None and angle < self._last_angle - 300:
                    # Завершаем текущий оборот
                    if self._current_scan:
                        self._last_scan = self._current_scan.copy()
                        if self.on_scan_complete:
                            self.on_scan_complete(self._last_scan)
                    self._current_scan = []

                # Добавляем точку в текущий оборот
                self._current_scan.append(pt)
                self._last_angle = angle

                if self.on_point_received:
                    self.on_point_received(pt)

    def get_last_scan(self) -> List[Point]:
        """Возвращает последний завершённый скан (копию)."""
        with self.lock:
            return self._last_scan.copy()
