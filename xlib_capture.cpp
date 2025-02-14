#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <memory>
#include <string>
#include <unistd.h>

class X11Capture {
private:
    Display* display;
    Window captureWindow;
    Window displayWindow;
    XWindowAttributes windowAttrs;
    XShmSegmentInfo shminfo;
    XImage* ximg;
    GC gc;
    bool running;

public:
    X11Capture() : display(nullptr), ximg(nullptr), gc(nullptr), running(true) {
        display = XOpenDisplay(nullptr);
        if (!display) {
            throw std::runtime_error("Cannot open display");
        }
    }

    ~X11Capture() {
        cleanup();
    }

    Window* findWindows(ulong* winCount) {
        Atom actualType;
        int format;
        ulong bytesAfter;
        unsigned char* list = nullptr;

        Status status = XGetWindowProperty(
            display,
            DefaultRootWindow(display),
            XInternAtom(display, "_NET_CLIENT_LIST", False),
            0L,
            ~0L,
            False,
            XA_WINDOW,
            &actualType,
            &format,
            winCount,
            &bytesAfter,
            &list
        );

        if (status != Success) {
            *winCount = 0;
            return nullptr;
        }

        return reinterpret_cast<Window*>(list);
    }

    char* getWindowName(Window win) {
        Atom actualType;
        int format;
        ulong count, bytesAfter;
        unsigned char* name = nullptr;

        Status status = XGetWindowProperty(
            display,
            win,
            XInternAtom(display, "_NET_WM_NAME", False),
            0L,
            ~0L,
            False,
            XInternAtom(display, "UTF8_STRING", False),
            &actualType,
            &format,
            &count,
            &bytesAfter,
            &name
        );

        if (status != Success) {
            return nullptr;
        }

        if (name == nullptr) {
            status = XGetWindowProperty(
                display,
                win,
                XInternAtom(display, "WM_NAME", False),
                0L,
                ~0L,
                False,
                AnyPropertyType,
                &actualType,
                &format,
                &count,
                &bytesAfter,
                &name
            );

            if (status != Success) {
                return nullptr;
            }
        }

        return reinterpret_cast<char*>(name);
    }

    void initializeCapture(Window window) {
        captureWindow = window;
        XGetWindowAttributes(display, captureWindow, &windowAttrs);
        if (windowAttrs.height <= 0) {
            throw std::runtime_error("Invalid window attributes");
        }

        // Создаем окно для отображения
        int screen = DefaultScreen(display);
        displayWindow = XCreateSimpleWindow(
            display,
            RootWindow(display, screen),
            0, 0,
            windowAttrs.width, windowAttrs.height,
            1,
            BlackPixel(display, screen),
            WhitePixel(display, screen)
        );

        // Устанавливаем заголовок окна
        XStoreName(display, displayWindow, "X11 Capture");

        // Устанавливаем обработку событий
        XSelectInput(display, displayWindow, ExposureMask | KeyPressMask);

        // Создаем графический контекст
        gc = XCreateGC(display, displayWindow, 0, nullptr);

        // Показываем окно
        XMapWindow(display, displayWindow);

        // Инициализация разделяемой памяти
        Screen* screen_ptr = windowAttrs.screen;
        XMapWindow(display, captureWindow);

        ximg = XShmCreateImage(
            display,
            DefaultVisualOfScreen(screen_ptr),
            DefaultDepthOfScreen(screen_ptr),
            ZPixmap,
            nullptr,
            &shminfo,
            windowAttrs.width,
            windowAttrs.height
        );

        shminfo.shmid = shmget(
            IPC_PRIVATE,
            ximg->bytes_per_line * ximg->height,
            IPC_CREAT | 0777
        );

        if (shminfo.shmid < 0) {
            throw std::runtime_error("Failed to allocate shared memory");
        }

        shminfo.shmaddr = ximg->data = static_cast<char*>(shmat(shminfo.shmid, 0, 0));
        shminfo.readOnly = False;

        if (!XShmAttach(display, &shminfo)) {
            throw std::runtime_error("Failed to attach shared memory");
        }
    }

    void captureLoop() {
        auto prevTime = std::chrono::system_clock::now();
        XEvent event;

        while (running) {
            while (XPending(display)) {
                XNextEvent(display, &event);
                if (event.type == KeyPress) {
                    running = false;
                    break;
                }
            }

            // Захват изображения
            XShmGetImage(display, captureWindow, ximg, 0, 0, AllPlanes);

            // Отображение изображения
            XPutImage(
                display,
                displayWindow,
                gc,
                ximg,
                0, 0,
                0, 0,
                windowAttrs.width,
                windowAttrs.height
            );

            XFlush(display);

            // Вычисление FPS
            auto now = std::chrono::system_clock::now();
            std::chrono::duration<double> period = (now - prevTime);
            prevTime = now;
            std::cout << "FPS: " << 1 / period.count() << std::endl;

            // Небольшая задержка для снижения нагрузки на CPU
            usleep(1000);
        }
    }

    void cleanup() {
        if (display) {
            if (ximg) {
                XShmDetach(display, &shminfo);
                XDestroyImage(ximg);
                shmdt(shminfo.shmaddr);
                shmctl(shminfo.shmid, IPC_RMID, nullptr);
            }
            if (gc) {
                XFreeGC(display, gc);
            }
            XCloseDisplay(display);
        }
    }
};

int main() {
    try {
        X11Capture capture;
        
        // Получение списка окон
        ulong count = 0;
        Window* wins = capture.findWindows(&count);
        if (!wins) {
            std::cerr << "No windows found" << std::endl;
            return 1;
        }

        // Вывод списка окон
        for (ulong i = 0; i < count; ++i) {
            if (char* name = capture.getWindowName(wins[i])) {
                std::cout << i << " : " << name << std::endl;
                XFree(name);
            }
        }

        // Выбор окна
        int selection;
        std::cout << "Select window number: ";
        std::cin >> selection;

        if (selection < 0 || static_cast<ulong>(selection) >= count) {
            std::cerr << "Invalid selection" << std::endl;
            XFree(wins);
            return 1;
        }

        Window selectedWindow = wins[selection];
        XFree(wins);

        // Инициализация захвата
        capture.initializeCapture(selectedWindow);

        std::cout << "Capture started. Press any key to exit." << std::endl;

        // Основной цикл захвата
        capture.captureLoop();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
