# GTA-AI-API

Захват окна GTA5 c++

# Зпуск:

компиляция

```
g++ cap_app_lib.cpp -o getcap -lX11 -lXext -Ofast -mfpmath=both -march=native -m64 -funroll-loops -mavx2 `pkg-config opencv --cflags --libs`
```

старт

```
./getcap

```
### Пример работы:
![Иллюстрация к проекту](https://github.com/evilsadko/GTA-AI-API/blob/master/media/example.png)

### Захват окна windows
https://stackoverflow-com.translate.goog/questions/38535809/how-can-i-access-a-graphics-cards-output-directly?_x_tr_sl=auto&_x_tr_tl=ru&_x_tr_hl=ru
