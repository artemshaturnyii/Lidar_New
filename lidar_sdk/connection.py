import socket
import time
from typing import Optional
from config import config  # Импортируем конфиг

# Константы команд (можно тоже в config)
CMD_PW_ON = b'\xA5\x30'
CMD_START = b'\xA5\x20'
CMD_STOP = b'\xA5\x25'

class LidarConnection:
    """Управление сетевым подключением к лидару"""
    
    def __init__(self):
        # Берем параметры из глобального конфига
        self.host = config.lidar['host']
        self.port = config.lidar['port']
        self.timeout = config.lidar['connection_timeout']
        self.sock: Optional[socket.socket] = None
    
    def connect(self) -> bool:
        """Устанавливает соединение с лидаром"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            # Включение питания
            self.sock.send(CMD_PW_ON)
            time.sleep(2)
            # Запуск сканирования
            self.sock.send(CMD_START)
            time.sleep(1)
            return True
        except Exception as e:
            print(f"Ошибка подключения: {e}")
            return False
    
    def disconnect(self) -> None:
        """Закрывает соединение"""
        if self.sock:
            try:
                self.sock.send(CMD_STOP)
            except:
                pass
            self.sock.close()
            self.sock = None
    
    def recv_exact(self, n: int) -> Optional[bytes]:
        """Получает ровно n байт из сокета"""
        if not self.sock:
         return None
    
        data = bytearray()
        while len(data) < n:
            try:
                chunk = self.sock.recv(n - len(data))
                if not chunk:
                    print(f"⚠️ Получен пустой чанк при ожидании {n - len(data)} байт")
                    return None
                data.extend(chunk)
            except Exception as e:
                print(f"⚠️ Ошибка получения данных: {e}")
                return None
        # print(f"📥 Получено {len(data)} байт")  # Можно временно включить для отладки
        return bytes(data)
