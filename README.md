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

### Пример работы:
![Иллюстрация к проекту](https://github.com/evilsadko/GTA-AI-API/blob/master/media/example.png)

### Захват окна windows
https://stackoverflow.com/questions/38535809/how-can-i-access-a-graphics-cards-output-directly    
https://docs-microsoft-com.translate.goog/en-us/windows/win32/gdi/capturing-an-image?_x_tr_sl=auto&_x_tr_tl=ru&_x_tr_hl=ru    
https://superkogito.github.io/blog/CaptureScreenUsingOpenCv.html    
https://stackoverflow.com/questions/5069104/fastest-method-of-screen-capturing-on-windows    
https://stackoverflow.com/questions/34176795/any-efficient-way-of-converting-ximage-data-to-pixel-map-e-g-array-of-rgb-quad   
