# GTA-AI-API

Захват окна GTA5 c++

# Зпуск:

#### версия с opencv
компиляция

```
g++ cap_app_lib.cpp -o getcap -lX11 -lXext -Ofast -mfpmath=both -march=native -m64 -funroll-loops -mavx2 `pkg-config opencv --cflags --libs`
```

старт

```
./getcap

```
#### версия с стандартными библиотеками X11 и GTK, работает быстрей!!!
компиляция

```
g++ -std=c++17 xlib_capture.cpp -lX11 -lXext -O3 -o xlib_capture
```

старт 

```
./xlib_capture
```
#### тестовая версия с интеграцией python как виртуальная вэб камера
скомпилировать расширение в текущей директории env

```
python setup.py build_ext --inplace
```

очистить скомпилированные файлы

```
python setup.py clean --all
```

запуск виртуальной вэб камеры

```
python start_cam.py
```

получение данных

```
python raw_data.py
```

#### для запуска отдельной библиотекой как виртуальная вэб камера

```
g++ -O3 -std=c++17 -DSTANDALONE_APP=1 -mavx2 x11capture_v4l2.cpp -o x11capture -lX11 -lXext -lv4l2 -lpthread
./x11capture --format rgb24 --fps 140
```

#### настройки камеры
список устройст:
```
v4l2-ctl --list-devices
ls -la /dev/video10
```
создать:
```
sudo modprobe v4l2loopback video_nr=10 card_label="X11_Capture" exclusive_caps=1
```
удалить:
```
sudo rmmod v4l2loopback
```
все данные:
```
v4l2-ctl --device=/dev/video10 --all
v4l2-ctl -d /dev/video10 --info
```
форматы:
```
v4l2-ctl --device=/dev/video10 --list-formats
v4l2-ctl --device=/dev/video10 --get-fmt-video
```
смотреть тестовый стрим:
```
vlc v4l2:///dev/video10
ffplay -f v4l2 -i /dev/video10
```
### Пример работы:
![Иллюстрация к проекту](https://github.com/evilsadko/GTA-AI-API/blob/master/media/example.png)

### Захват окна windows
https://stackoverflow.com/questions/38535809/how-can-i-access-a-graphics-cards-output-directly    
https://docs-microsoft-com.translate.goog/en-us/windows/win32/gdi/capturing-an-image?_x_tr_sl=auto&_x_tr_tl=ru&_x_tr_hl=ru    
https://superkogito.github.io/blog/CaptureScreenUsingOpenCv.html    
https://stackoverflow.com/questions/5069104/fastest-method-of-screen-capturing-on-windows    
https://stackoverflow.com/questions/34176795/any-efficient-way-of-converting-ximage-data-to-pixel-map-e-g-array-of-rgb-quad   
