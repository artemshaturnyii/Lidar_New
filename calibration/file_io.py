import numpy as np

def save_background_to_file(filename: str, background_map: dict) -> None:
    """Сохраняет фоновую карту в файл (например, numpy .npz)."""
    if background_map is None:
        return
    np.savez(filename, angles=list(background_map.keys()), distances=list(background_map.values()))

def load_background_from_file(filename: str) -> tuple:
    """Загружает фоновую карту из файла.
    
    Returns:
        tuple: (background_map_dict, interpolation_function)
    """
    data = np.load(filename)
    angles = data['angles']
    distances = data['distances']
    background_map = dict(zip(angles, distances))
    
    def background_interp(angle):
        return np.interp(angle, angles, distances, period=360)
    
    return background_map, background_interp

