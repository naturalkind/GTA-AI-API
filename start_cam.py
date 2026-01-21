#!/usr/bin/env python3
"""
Пример использования x11capture_v4l2 для захвата окна и трансляции в виртуальную камеру
"""
import x11capture_v4l2
import cv2
import numpy as np
import time
import sys

WIN_NUM = 0

def main():
    capture = x11capture_v4l2.X11CaptureV4L2(enable_v4l2=True, format=x11capture_v4l2.FORMAT_RGB24)
    windows = capture.get_window_list()
    print (f"Список окон ({len(windows)}):", windows, f"Выброно #{WIN_NUM}")
    if windows:
        window_id, window_name = windows[WIN_NUM]
        print(f"Захватываем: {window_name}")
        
        # Запускаем захват только для V4L2
        result = capture.start_capture(
            window_id=window_id,
            #capture_width=1150,     # Захватываем в Full HD
            #capture_height=933,
            output_width=640,
            output_height=480,
            fps=140,                 # 30 FPS
            #scale=1                 # Без дополнительного масштабирования
        )

        print (result)
        print(f"Трансляция запущена на /dev/video10")
        print(f"V4L2 device: {result['v4l2_device']}")
        print(f"Format: {result['format_string']}")
        print("Нажмите Ctrl+C для остановки...")
        
        try:
            while True:
                time.sleep(1)
                fps = capture.get_fps()
                print(f"FPS: {fps:.1f}", end='\r')
        except KeyboardInterrupt:
            capture.stop()
            print("\nОстановлено")

if __name__ == "__main__":
    main()
