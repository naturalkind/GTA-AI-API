import cv2
import numpy as np
import time
import x11capture_v4l2

WIN_NUM = 0

def main():
    capture = x11capture_v4l2.X11CaptureV4L2(enable_v4l2=True, format=x11capture_v4l2.FORMAT_RGB24)
    windows = capture.get_window_list()
    
    print("Доступные окна:")
    for i, (win_id, win_name) in enumerate(windows):
        print(f"{i}: {win_name} (ID: {win_id})")
    
    if windows:
        # Выбираем первое окно
        window_id, window_name = windows[WIN_NUM]
        print(f"\nЗахватываем: {window_name}")
        
        # Запускаем захват
        result = capture.start_capture(
            window_id=window_id,
            output_width=640,
            output_height=480,
            fps=120
        )
        
        if result["success"]:
            print(f"V4L2 устройство: {result['v4l2_device']}")
            print(f"V4L2 включен: {result['v4l2_enabled']}")
            
            print("\nНажмите Ctrl+C для остановки...")
            
            try:
                while True:
                    # Получаем кадр
                    frame = capture.get_frame(format=x11capture_v4l2.FORMAT_RGB24)  # RGB формат
                    if frame is not None and frame[0] is not None:
                        data, w, h, fmt = frame
                        
                        # Конвертируем в numpy массив
                        img = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
                        
                        # Отображаем информацию
                        print(f"Frame: {w}x{h}, FPS: {capture.get_fps():.1f}, Format: {fmt}", end='\r')
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        # Показываем кадр
                        cv2.imshow('X11 Capture', img)
                        
                        # Обработка клавиши 'q' для выхода
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    else:
                        print("Нет данных кадра")
                        time.sleep(0.1)
                        
            except KeyboardInterrupt:
                print("\nОстановлено пользователем")
            finally:
                capture.stop()
                cv2.destroyAllWindows()
        else:
            print("Не удалось запустить захват")
    else:
        print("Нет доступных окон")

if __name__ == "__main__":
    main()

