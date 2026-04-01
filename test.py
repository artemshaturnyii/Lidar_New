
try:
    print("Testing mouse controller import...")
    from mouse_controller import MouseController
    print("✅ Import successful")
    
    # Попробуем создать экземпляр
    mc = MouseController()
    print("✅ MouseController created successfully")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
