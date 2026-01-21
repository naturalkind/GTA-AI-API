//g++ -std=c++17 -O2 x11_to_v4l2_single.cpp -o x11_to_v4l2 -lX11 -lXext -pthread -lm
//./x11_to_v4l2 --list
//sudo ./x11_to_v4l2 --window 0x2400040
// sudo modprobe v4l2loopback video_nr=10 card_label="X11_Capture" exclusive_caps=1
// sudo rmmod v4l2loopback - удалить

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <chrono>
#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <atomic>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <cmath>

// Добавляем определение M_PI если его нет
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Вместо TurboJPEG - используем простую реализацию
class SimpleJPEGCompressor {
private:
    // Таблицы для квантования (упрощенные)
    std::vector<unsigned char> quant_table;
    
public:
    SimpleJPEGCompressor() {
        // Простая таблица квантования
        quant_table.resize(64);
        for (int i = 0; i < 64; i++) {
            quant_table[i] = 16 + (i % 8) * 3;
        }
    }
    
    // Упрощенное сжатие - на самом деле просто сохраняем как RAW RGB
    // В реальном приложении нужно использовать libjpeg или аналогичную библиотеку
    size_t compressRGBtoJPEG(const unsigned char* rgb, int width, int height, 
                           std::vector<unsigned char>& output, int quality = 85) {
        // Для простоты просто копируем RGB данные
        // В реальном приложении здесь должно быть JPEG сжатие
        
        // Создаем "фальшивый" JPEG заголовок
        std::string header = "FAKEJPEG";  // Заглушка вместо настоящего JPEG заголовка
        output.resize(header.size() + width * height * 3);
        
        // Копируем заголовок
        memcpy(output.data(), header.c_str(), header.size());
        
        // Копируем RGB данные
        memcpy(output.data() + header.size(), rgb, width * height * 3);
        
        return output.size();
    }
};

class V4L2Output {
private:
    int fd;
    int width;
    int height;
    uint32_t current_format;
    
    SimpleJPEGCompressor jpegCompressor;
    std::vector<unsigned char> jpegBuffer;
    std::vector<unsigned char> conversionBuffer;
    
    // Статистика
    std::atomic<size_t> totalFrames;
    std::atomic<size_t> droppedFrames;
    std::chrono::steady_clock::time_point lastStatsTime;

public:
    V4L2Output() : fd(-1), width(0), height(0), current_format(0),
                   totalFrames(0), droppedFrames(0) {
        lastStatsTime = std::chrono::steady_clock::now();
    }
    
    ~V4L2Output() {
        cleanup();
    }
    
    bool initialize(const char* device, int w, int h, 
                    uint32_t preferred_format = V4L2_PIX_FMT_YUYV) {
        width = w;
        height = h;
        
        // Открываем устройство в неблокирующем режиме
        fd = open(device, O_WRONLY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "Cannot open " << device << ": " << strerror(errno) << std::endl;
            return false;
        }
        
        // Пробуем разные форматы
        uint32_t formats_to_try[] = {
            preferred_format,
            V4L2_PIX_FMT_YUYV,
            V4L2_PIX_FMT_RGB24,
            V4L2_PIX_FMT_BGR24,
        };
        
        bool format_set = false;
        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        
        for (auto fmt_try : formats_to_try) {
            fmt.fmt.pix.width = width;
            fmt.fmt.pix.height = height;
            fmt.fmt.pix.pixelformat = fmt_try;
            fmt.fmt.pix.field = V4L2_FIELD_NONE;
            
            // Устанавливаем параметры в зависимости от формата
            switch (fmt_try) {
                case V4L2_PIX_FMT_YUYV:
                    fmt.fmt.pix.bytesperline = width * 2;
                    fmt.fmt.pix.sizeimage = width * height * 2;
                    break;
                case V4L2_PIX_FMT_RGB24:
                case V4L2_PIX_FMT_BGR24:
                    fmt.fmt.pix.bytesperline = width * 3;
                    fmt.fmt.pix.sizeimage = width * height * 3;
                    break;
                default:
                    continue;
            }
            
            fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
            
            if (ioctl(fd, VIDIOC_S_FMT, &fmt) >= 0) {
                current_format = fmt_try;
                format_set = true;
                std::cout << "Successfully set format: ";
                printFourCC(current_format);
                
                // Выделяем буфер для конвертации
                switch (current_format) {
                    case V4L2_PIX_FMT_YUYV:
                        conversionBuffer.resize(width * height * 2);
                        break;
                    case V4L2_PIX_FMT_RGB24:
                    case V4L2_PIX_FMT_BGR24:
                        conversionBuffer.resize(width * height * 3);
                        break;
                }
                break;
            }
        }
        
        if (!format_set) {
            std::cerr << "Failed to set any supported format" << std::endl;
            close(fd);
            fd = -1;
            return false;
        }
        
        // Устанавливаем FPS
        struct v4l2_streamparm streamparm = {};
        streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (ioctl(fd, VIDIOC_G_PARM, &streamparm) == 0) {
            streamparm.parm.output.timeperframe.numerator = 1;
            streamparm.parm.output.timeperframe.denominator = 30;
            ioctl(fd, VIDIOC_S_PARM, &streamparm);
        }
        
        return true;
    }
    
    void printFourCC(uint32_t fourcc) {
        char fourcc_str[5];
        fourcc_str[0] = (fourcc >> 0) & 0xFF;
        fourcc_str[1] = (fourcc >> 8) & 0xFF;
        fourcc_str[2] = (fourcc >> 16) & 0xFF;
        fourcc_str[3] = (fourcc >> 24) & 0xFF;
        fourcc_str[4] = '\0';
        
        if (fourcc == V4L2_PIX_FMT_YUYV) {
            std::cout << "YUYV" << std::endl;
        } else if (fourcc == V4L2_PIX_FMT_RGB24) {
            std::cout << "RGB24" << std::endl;
        } else if (fourcc == V4L2_PIX_FMT_BGR24) {
            std::cout << "BGR24" << std::endl;
        } else {
            std::cout << fourcc_str << std::endl;
        }
    }
    
    // Конвертация RGB в YUYV
    void convertRGBtoYUYV(const unsigned char* rgb, unsigned char* yuyv) {
        // Оптимизированная версия
        for (int i = 0; i < width * height; i += 2) {
            int idx1 = i * 3;
            int idx2 = (i + 1) * 3;
            
            int r1 = rgb[idx1], g1 = rgb[idx1 + 1], b1 = rgb[idx1 + 2];
            int r2 = rgb[idx2], g2 = rgb[idx2 + 1], b2 = rgb[idx2 + 2];
            
            // Формулы ITU-R BT.601
            int y0 = ((66 * r1 + 129 * g1 + 25 * b1 + 128) >> 8) + 16;
            int y1 = ((66 * r2 + 129 * g2 + 25 * b2 + 128) >> 8) + 16;
            int u = ((-38 * r1 - 74 * g1 + 112 * b1 + 128) >> 8) + 128;
            int v = ((112 * r1 - 94 * g1 - 18 * b1 + 128) >> 8) + 128;
            
            // Ограничение значений
            y0 = std::max(0, std::min(255, y0));
            y1 = std::max(0, std::min(255, y1));
            u = std::max(0, std::min(255, u));
            v = std::max(0, std::min(255, v));
            
            int yuyv_idx = i * 2;
            yuyv[yuyv_idx] = static_cast<uint8_t>(y0);
            yuyv[yuyv_idx + 1] = static_cast<uint8_t>(u);
            yuyv[yuyv_idx + 2] = static_cast<uint8_t>(y1);
            yuyv[yuyv_idx + 3] = static_cast<uint8_t>(v);
        }
    }
    
    // Конвертация RGB в BGR24
    void convertRGBtoBGR24(const unsigned char* rgb, unsigned char* bgr) {
        for (int i = 0; i < width * height; i++) {
            int src_idx = i * 3;
            int dst_idx = i * 3;
            bgr[dst_idx] = rgb[src_idx + 2];
            bgr[dst_idx + 1] = rgb[src_idx + 1];
            bgr[dst_idx + 2] = rgb[src_idx];
        }
    }
    
    void writeFrame(const unsigned char* rgb_data, size_t size) {
        if (fd < 0) return;
        
        totalFrames++;
        
        const unsigned char* write_data = rgb_data;
        size_t write_size = size;
        
        // Конвертируем при необходимости
        switch (current_format) {
            case V4L2_PIX_FMT_YUYV:
                convertRGBtoYUYV(rgb_data, conversionBuffer.data());
                write_data = conversionBuffer.data();
                write_size = conversionBuffer.size();
                break;
                
            case V4L2_PIX_FMT_BGR24:
                convertRGBtoBGR24(rgb_data, conversionBuffer.data());
                write_data = conversionBuffer.data();
                write_size = conversionBuffer.size();
                break;
                
            case V4L2_PIX_FMT_RGB24:
                // Уже в правильном формате
                break;
        }
        
        // Запись кадра
        ssize_t written = write(fd, write_data, write_size);
        
        if (written != static_cast<ssize_t>(write_size)) {
            if (errno == EAGAIN) {
                droppedFrames++;
            } else {
                std::cerr << "Write error: " << strerror(errno) << std::endl;
            }
        }
        
        // Периодический вывод статистики
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - lastStatsTime).count();
        
        if (elapsed >= 5) {
            double fps = totalFrames / static_cast<double>(elapsed);
            double drop_percent = (totalFrames > 0) ? 
                (droppedFrames * 100.0 / totalFrames) : 0;
            
            std::cout << "\rStats: " << std::fixed << std::setprecision(1) 
                      << fps << " FPS, " << droppedFrames 
                      << " dropped (" << drop_percent << "%)" << std::string(20, ' ') << std::flush;
            
            lastStatsTime = now;
            totalFrames = 0;
            droppedFrames = 0;
        }
    }
    
    uint32_t getCurrentFormat() const {
        return current_format;
    }
    
    void cleanup() {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }
};

class X11Capture {
private:
    Display* display;
    Window captureWindow;
    XWindowAttributes windowAttrs;
    XShmSegmentInfo shminfo;
    XImage* ximg;
    V4L2Output v4l2Output;
    std::atomic<bool> running;
    std::string v4l2Device;
    
    // Информация о формате XImage
    int bitsPerPixel;
    int bytesPerPixel;
    unsigned long redMask, greenMask, blueMask;
    int redShift, greenShift, blueShift;
    bool isLittleEndian;
    
    // Буферы
    std::vector<unsigned char> rgbBuffer;
    
    // Поток для захвата
    std::thread captureThread;
    
    // Статистика
    std::chrono::steady_clock::time_point captureStart;
    std::atomic<size_t> capturedFrames;
    
    // Размеры окна для Python кода
    int windowWidth;
    int windowHeight;

public:
    X11Capture(const std::string& v4l2_dev = "/dev/video10") 
        : display(nullptr), ximg(nullptr), running(true), 
          v4l2Device(v4l2_dev), bitsPerPixel(0), bytesPerPixel(0),
          redMask(0), greenMask(0), blueMask(0),
          redShift(0), greenShift(0), blueShift(0),
          isLittleEndian(true), capturedFrames(0),
          windowWidth(0), windowHeight(0) {
        
        display = XOpenDisplay(nullptr);
        if (!display) {
            throw std::runtime_error("Cannot open display");
        }
        
        // Определяем порядок байтов
        int test = 1;
        isLittleEndian = (*((char*)&test) == 1);
        
        captureStart = std::chrono::steady_clock::now();
    }
    
    ~X11Capture() {
        stop();
        cleanup();
    }
    
    void printWindowInfo(Window win) {
        XWindowAttributes attrs;
        if (XGetWindowAttributes(display, win, &attrs)) {
            char* name = getWindowName(win);
            std::cout << "Window 0x" << std::hex << win << std::dec << ": ";
            if (name) {
                std::cout << name;
                XFree(name);
            }
            std::cout << " (" << attrs.width << "x" << attrs.height << ")" << std::endl;
        }
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
            0L, ~0L, False, XA_WINDOW,
            &actualType, &format, winCount, &bytesAfter, &list
        );
        
        return (status == Success) ? reinterpret_cast<Window*>(list) : nullptr;
    }
    
    char* getWindowName(Window win) {
        Atom actualType;
        int format;
        ulong count, bytesAfter;
        unsigned char* name = nullptr;
        
        Status status = XGetWindowProperty(
            display, win,
            XInternAtom(display, "_NET_WM_NAME", False),
            0L, ~0L, False,
            XInternAtom(display, "UTF8_STRING", False),
            &actualType, &format, &count, &bytesAfter, &name
        );
        
        if (status == Success && name) {
            return reinterpret_cast<char*>(name);
        }
        
        return nullptr;
    }
    
    void initializeCapture(Window window) {
        captureWindow = window;
        
        if (!XGetWindowAttributes(display, captureWindow, &windowAttrs)) {
            throw std::runtime_error("Cannot get window attributes");
        }
        
        if (windowAttrs.width <= 0 || windowAttrs.height <= 0) {
            throw std::runtime_error("Invalid window size");
        }
        
        windowWidth = windowAttrs.width;
        windowHeight = windowAttrs.height;
        
        std::cout << "Window size: " << windowWidth << "x" << windowHeight << std::endl;
        
        // Создаем XImage с разделяемой памятью
        ximg = XShmCreateImage(
            display,
            DefaultVisualOfScreen(DefaultScreenOfDisplay(display)),
            DefaultDepthOfScreen(DefaultScreenOfDisplay(display)),
            ZPixmap,
            nullptr,
            &shminfo,
            windowWidth,
            windowHeight
        );
        
        if (!ximg) {
            throw std::runtime_error("Failed to create XImage");
        }
        
        // Анализируем формат
        analyzeImageFormat();
        
        // Создаем shared memory
        shminfo.shmid = shmget(
            IPC_PRIVATE,
            ximg->bytes_per_line * ximg->height,
            IPC_CREAT | 0666
        );
        
        if (shminfo.shmid < 0) {
            XDestroyImage(ximg);
            throw std::runtime_error("Failed to allocate shared memory");
        }
        
        shminfo.shmaddr = ximg->data = static_cast<char*>(shmat(shminfo.shmid, 0, 0));
        shminfo.readOnly = False;
        
        if (!XShmAttach(display, &shminfo)) {
            XDestroyImage(ximg);
            shmdt(shminfo.shmaddr);
            throw std::runtime_error("Failed to attach shared memory");
        }
        
        XSync(display, False);
        
        // Инициализируем V4L2 с предпочтением YUYV (так как нет настоящего MJPEG)
        if (!v4l2Output.initialize(v4l2Device.c_str(), 
                                  windowWidth, 
                                  windowHeight,
                                  V4L2_PIX_FMT_YUYV)) {
            std::cerr << "Failed to initialize V4L2 output" << std::endl;
            throw std::runtime_error("V4L2 initialization failed");
        }
        
        // Создаем буфер RGB
        rgbBuffer.resize(windowWidth * windowHeight * 3);
        
        std::cout << "Capture initialized successfully" << std::endl;
        std::cout << "Using format: ";
        switch (v4l2Output.getCurrentFormat()) {
            case V4L2_PIX_FMT_YUYV:
                std::cout << "YUYV" << std::endl;
                break;
            case V4L2_PIX_FMT_RGB24:
                std::cout << "RGB24" << std::endl;
                break;
            case V4L2_PIX_FMT_BGR24:
                std::cout << "BGR24" << std::endl;
                break;
            default:
                std::cout << "Unknown" << std::endl;
        }
    }
    
    void analyzeImageFormat() {
        bitsPerPixel = ximg->bits_per_pixel;
        bytesPerPixel = bitsPerPixel / 8;
        redMask = ximg->red_mask;
        greenMask = ximg->green_mask;
        blueMask = ximg->blue_mask;
        
        calculateMaskShifts();
        
        std::cout << "XImage format analysis:" << std::endl;
        std::cout << "  Bits per pixel: " << bitsPerPixel << std::endl;
        std::cout << "  Bytes per pixel: " << bytesPerPixel << std::endl;
        std::cout << "  Bytes per line: " << ximg->bytes_per_line << std::endl;
        std::cout << "  Red mask: 0x" << std::hex << redMask << std::dec
                  << ", shift: " << redShift << std::endl;
        std::cout << "  Green mask: 0x" << std::hex << greenMask << std::dec
                  << ", shift: " << greenShift << std::endl;
        std::cout << "  Blue mask: 0x" << std::hex << blueMask << std::dec
                  << ", shift: " << blueShift << std::endl;
        std::cout << "  Byte order: " << (isLittleEndian ? "Little Endian" : "Big Endian") << std::endl;
        
        printFormatInfo();
    }
    
    void calculateMaskShifts() {
        auto calcShift = [](unsigned long mask) -> int {
            if (mask == 0) return 0;
            int shift = 0;
            while ((mask & 1) == 0) {
                mask >>= 1;
                shift++;
            }
            return shift;
        };
        
        redShift = calcShift(redMask);
        greenShift = calcShift(greenMask);
        blueShift = calcShift(blueMask);
    }
    
    void printFormatInfo() {
        if (bytesPerPixel == 4) {
            if (isLittleEndian) {
                if (redMask == 0xff0000 && greenMask == 0xff00 && blueMask == 0xff) {
                    std::cout << "  Format: BGRx (B,G,R,x in memory)" << std::endl;
                } else if (redMask == 0xff && greenMask == 0xff00 && blueMask == 0xff0000) {
                    std::cout << "  Format: xRGB (x,R,G,B in memory)" << std::endl;
                } else if (redMask == 0xff000000 && greenMask == 0xff0000 && blueMask == 0xff00) {
                    std::cout << "  Format: RGBx (R,G,B,x in memory)" << std::endl;
                } else {
                    std::cout << "  Format: Unknown 32-bit" << std::endl;
                }
            }
        } else if (bytesPerPixel == 3) {
            if (redMask == 0xff && greenMask == 0xff00 && blueMask == 0xff0000) {
                std::cout << "  Format: BGR" << std::endl;
            } else if (redMask == 0xff0000 && greenMask == 0xff00 && blueMask == 0xff) {
                std::cout << "  Format: RGB" << std::endl;
            } else {
                std::cout << "  Format: Unknown 24-bit" << std::endl;
            }
        } else if (bytesPerPixel == 2) {
            std::cout << "  Format: 16-bit (likely RGB565 or similar)" << std::endl;
        } else {
            std::cout << "  Format: Unusual (" << bytesPerPixel << " bytes per pixel)" << std::endl;
        }
    }
    
    void convertToRGB24() {
        int width = ximg->width;
        int height = ximg->height;
        int srcStride = ximg->bytes_per_line;
        
        const unsigned char* src = reinterpret_cast<const unsigned char*>(ximg->data);
        unsigned char* dst = rgbBuffer.data();
        
        // Определяем наиболее эффективный метод конвертации
        if (bytesPerPixel == 4 && isLittleEndian) {
            if (redMask == 0xff0000 && greenMask == 0xff00 && blueMask == 0xff) {
                // BGRx -> RGB (самый распространенный)
                for (int y = 0; y < height; y++) {
                    const unsigned char* src_line = src + y * srcStride;
                    unsigned char* dst_line = dst + y * width * 3;
                    
                    for (int x = 0; x < width; x++) {
                        dst_line[0] = src_line[2]; // R
                        dst_line[1] = src_line[1]; // G
                        dst_line[2] = src_line[0]; // B
                        src_line += 4;
                        dst_line += 3;
                    }
                }
                return;
            } else if (redMask == 0xff && greenMask == 0xff00 && blueMask == 0xff0000) {
                // xRGB -> RGB
                for (int y = 0; y < height; y++) {
                    const unsigned char* src_line = src + y * srcStride;
                    unsigned char* dst_line = dst + y * width * 3;
                    
                    for (int x = 0; x < width; x++) {
                        dst_line[0] = src_line[1]; // R
                        dst_line[1] = src_line[2]; // G
                        dst_line[2] = src_line[3]; // B
                        src_line += 4;
                        dst_line += 3;
                    }
                }
                return;
            }
        } else if (bytesPerPixel == 3) {
            if (redMask == 0xff && greenMask == 0xff00 && blueMask == 0xff0000) {
                // BGR -> RGB
                for (int y = 0; y < height; y++) {
                    const unsigned char* src_line = src + y * srcStride;
                    unsigned char* dst_line = dst + y * width * 3;
                    
                    for (int x = 0; x < width; x++) {
                        dst_line[0] = src_line[2]; // R
                        dst_line[1] = src_line[1]; // G
                        dst_line[2] = src_line[0]; // B
                        src_line += 3;
                        dst_line += 3;
                    }
                }
                return;
            } else if (redMask == 0xff0000 && greenMask == 0xff00 && blueMask == 0xff) {
                // RGB -> RGB (просто копирование)
                if (srcStride == width * 3) {
                    memcpy(dst, src, width * height * 3);
                } else {
                    for (int y = 0; y < height; y++) {
                        memcpy(dst + y * width * 3,
                               src + y * srcStride,
                               width * 3);
                    }
                }
                return;
            }
        }
        
        // Универсальный метод для остальных форматов
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // Читаем пиксель
                unsigned long pixel = 0;
                for (int b = 0; b < bytesPerPixel; b++) {
                    int byte_pos = isLittleEndian ? b : (bytesPerPixel - 1 - b);
                    pixel |= static_cast<unsigned long>(
                        src[y * srcStride + x * bytesPerPixel + byte_pos]) << (b * 8);
                }
                
                // Извлекаем компоненты
                unsigned char r = (pixel & redMask) >> redShift;
                unsigned char g = (pixel & greenMask) >> greenShift;
                unsigned char b = (pixel & blueMask) >> blueShift;
                
                int dst_idx = (y * width + x) * 3;
                dst[dst_idx] = r;
                dst[dst_idx + 1] = g;
                dst[dst_idx + 2] = b;
            }
        }
    }
    
    void captureFrame() {
        if (!XShmGetImage(display, captureWindow, ximg, 0, 0, AllPlanes)) {
            std::cerr << "XShmGetImage failed" << std::endl;
            return;
        }
        
        // Конвертируем в RGB24
        convertToRGB24();
        
        // Отправляем в V4L2
        v4l2Output.writeFrame(rgbBuffer.data(), rgbBuffer.size());
        
        capturedFrames++;
    }
    
    void captureLoop() {
        std::cout << "Starting capture loop..." << std::endl;
        std::cout << "Press Ctrl+C to stop." << std::endl;
        
        auto prevTime = std::chrono::steady_clock::now();
        int frameCount = 0;
        
        // Даем X11 время для обновления
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        while (running) {
            auto frameStart = std::chrono::steady_clock::now();
            
            try {
                captureFrame();
            } catch (const std::exception& e) {
                std::cerr << "Capture error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            frameCount++;
            
            // Расчет FPS каждую секунду
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(now - prevTime);
            
            if (elapsed.count() >= 1.0) {
                double fps = frameCount / elapsed.count();
                auto totalElapsed = std::chrono::duration<double>(
                    now - captureStart).count();
                double avgFps = capturedFrames / totalElapsed;
                
                std::cout << "\rFPS: " << std::fixed << std::setprecision(1) 
                          << fps << " | Avg: " << avgFps 
                          << " | Total: " << capturedFrames 
                          << std::string(20, ' ') << std::flush;
                
                frameCount = 0;
                prevTime = now;
            }
            
            // Поддержание целевого FPS (~30 FPS)
            auto frameTime = std::chrono::duration_cast<std::chrono::microseconds>(
                now - frameStart);
            
            int targetFrameTime = 33333; // микросекунд для 30 FPS
            if (frameTime.count() < targetFrameTime) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(targetFrameTime - frameTime.count()));
            }
        }
    }
    
    void start() {
        if (running) {
            captureThread = std::thread(&X11Capture::captureLoop, this);
        }
    }
    
    void stop() {
        running = false;
        if (captureThread.joinable()) {
            captureThread.join();
        }
        std::cout << "\nCapture stopped." << std::endl;
    }
    
    void cleanup() {
        stop();
        
        if (display) {
            if (ximg) {
                XShmDetach(display, &shminfo);
                XDestroyImage(ximg);
                ximg = nullptr;
                
                if (shminfo.shmaddr) {
                    shmdt(shminfo.shmaddr);
                }
                if (shminfo.shmid >= 0) {
                    shmctl(shminfo.shmid, IPC_RMID, nullptr);
                }
            }
            XCloseDisplay(display);
            display = nullptr;
        }
        
        v4l2Output.cleanup();
    }
    
    int getWindowWidth() const { return windowWidth; }
    int getWindowHeight() const { return windowHeight; }
};

// Глобальные переменные для обработки сигналов
std::atomic<bool> g_signal_received(false);
X11Capture* g_capture_ptr = nullptr;

void signalHandler(int signum) {
    std::cout << "\n\nSignal " << signum << " received. Stopping..." << std::endl;
    g_signal_received = true;
    if (g_capture_ptr) {
        g_capture_ptr->stop();
    }
}

void registerSignalHandlers() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGQUIT, signalHandler);
    
    // Игнорируем SIGPIPE чтобы не падать при разрыве соединения
    signal(SIGPIPE, SIG_IGN);
}

void printUsage(const char* program_name) {
    std::cout << "X11 to V4L2 Screen Capture Tool" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --device N      V4L2 device number (default: 10 = /dev/video10)" << std::endl;
    std::cout << "  --list          List available windows and exit" << std::endl;
    std::cout << "  --window ID     Window ID to capture (from --list)" << std::endl;
    std::cout << "  --help          Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " --list" << std::endl;
    std::cout << "  " << program_name << " --window 0x12345678" << std::endl;
    std::cout << "  " << program_name << " --device 11 --window 0x12345678" << std::endl;
}

void printBanner() {
    std::cout << "==========================================" << std::endl;
    std::cout << "     X11 to V4L2 Screen Capture" << std::endl;
    std::cout << "     Single File Version" << std::endl;
    std::cout << "==========================================" << std::endl;
}

int main(int argc, char* argv[]) {
    printBanner();
    
    // Регистрируем обработчики сигналов
    registerSignalHandlers();
    
    // Парсим аргументы командной строки
    std::string v4l2_device = "/dev/video10";
    bool list_windows = false;
    Window selected_window = 0;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--device" && i + 1 < argc) {
            int device_num = std::stoi(argv[++i]);
            v4l2_device = "/dev/video" + std::to_string(device_num);
        } else if (arg == "--list") {
            list_windows = true;
        } else if (arg == "--window" && i + 1 < argc) {
            std::string window_str = argv[++i];
            if (window_str.substr(0, 2) == "0x") {
                selected_window = std::stoul(window_str.substr(2), nullptr, 16);
            } else {
                selected_window = std::stoul(window_str, nullptr, 0);
            }
        } else if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
    try {
        if (list_windows) {
            // Только список окон без захвата
            std::unique_ptr<X11Capture> temp_capture;
            try {
                temp_capture = std::make_unique<X11Capture>();
            } catch (const std::exception& e) {
                std::cerr << "Error: " << e.what() << std::endl;
                return 1;
            }
            
            ulong count = 0;
            Window* wins = temp_capture->findWindows(&count);
            
            if (!wins || count == 0) {
                std::cerr << "No windows found" << std::endl;
                return 1;
            }
            
            std::cout << "\nAvailable windows (" << count << " found):" << std::endl;
            std::cout << "==========================================" << std::endl;
            for (ulong i = 0; i < std::min(count, ulong(20)); i++) {
                std::cout << i << ": ";
                temp_capture->printWindowInfo(wins[i]);
            }
            
            if (count > 20) {
                std::cout << "... and " << (count - 20) << " more windows." << std::endl;
            }
            
            XFree(wins);
            return 0;
        }
        
        // Создаем основной объект захвата
        std::unique_ptr<X11Capture> capture;
        try {
            capture = std::make_unique<X11Capture>(v4l2_device);
        } catch (const std::exception& e) {
            std::cerr << "Error creating capture: " << e.what() << std::endl;
            return 1;
        }
        
        g_capture_ptr = capture.get();
        
        // Если окно не выбрано, показываем список и просим выбрать
        if (selected_window == 0) {
            ulong count = 0;
            Window* wins = capture->findWindows(&count);
            
            if (!wins || count == 0) {
                std::cerr << "No windows found" << std::endl;
                return 1;
            }
            
            std::cout << "\nAvailable windows (" << count << " found):" << std::endl;
            std::cout << "==========================================" << std::endl;
            for (ulong i = 0; i < std::min(count, ulong(10)); i++) {
                std::cout << i << ": ";
                capture->printWindowInfo(wins[i]);
            }
            
            std::cout << "\nSelect window number (0-" << std::min(count, ulong(10))-1 << "): ";
            int selection;
            std::cin >> selection;
            
            if (selection < 0 || selection >= int(count)) {
                std::cerr << "Invalid selection" << std::endl;
                XFree(wins);
                return 1;
            }
            
            selected_window = wins[selection];
            XFree(wins);
        }
        
        std::cout << "\nInitializing capture for window 0x" 
                  << std::hex << selected_window << std::dec << "..." << std::endl;
        
        try {
            capture->initializeCapture(selected_window);
        } catch (const std::exception& e) {
            std::cerr << "Error initializing capture: " << e.what() << std::endl;
            return 1;
        }
        
        std::cout << "\nInitialization complete!" << std::endl;
        std::cout << "Device: " << v4l2_device << std::endl;
        std::cout << "Window size: " << capture->getWindowWidth() << "x" << capture->getWindowHeight() << std::endl;
        std::cout << "OpenCV can now access this virtual camera." << std::endl;
        std::cout << "\nExample Python OpenCV code:" << std::endl;
        std::cout << "```python" << std::endl;
        std::cout << "import cv2" << std::endl;
        std::cout << "import time" << std::endl;
        std::cout << std::endl;
        std::cout << "# Wait for capture to start" << std::endl;
        std::cout << "time.sleep(1)" << std::endl;
        std::cout << std::endl;
        std::cout << "# Open the virtual camera" << std::endl;
        std::cout << "device_id = " << v4l2_device.substr(10) << std::endl;
        std::cout << "camera = cv2.VideoCapture(device_id, cv2.CAP_V4L2)" << std::endl;
        std::cout << std::endl;
        std::cout << "# Try YUYV format (what we're using)" << std::endl;
        std::cout << "camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))" << std::endl;
        std::cout << "camera.set(cv2.CAP_PROP_FRAME_WIDTH, " << capture->getWindowWidth() << ")" << std::endl;
        std::cout << "camera.set(cv2.CAP_PROP_FRAME_HEIGHT, " << capture->getWindowHeight() << ")" << std::endl;
        std::cout << std::endl;
        std::cout << "while True:" << std::endl;
        std::cout << "    ret, frame = camera.read()" << std::endl;
        std::cout << "    if ret:" << std::endl;
        std::cout << "        cv2.imshow('Virtual Camera', frame)" << std::endl;
        std::cout << "    if cv2.waitKey(1) & 0xFF == ord('q'):" << std::endl;
        std::cout << "        break" << std::endl;
        std::cout << std::endl;
        std::cout << "camera.release()" << std::endl;
        std::cout << "cv2.destroyAllWindows()" << std::endl;
        std::cout << "```" << std::endl;
        
        std::cout << "\nStarting capture in 3 seconds..." << std::endl;
        std::cout << "You can now run your OpenCV/Python code." << std::endl;
        
        // Даем время для запуска клиентов
        for (int i = 3; i > 0; i--) {
            std::cout << i << "... " << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "GO!" << std::endl;
        
        // Запускаем захват
        capture->start();
        
        // Ожидание завершения
        while (!g_signal_received) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        capture->stop();
        
    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }
    
    g_capture_ptr = nullptr;
    std::cout << "\nCapture program terminated." << std::endl;
    return 0;
}
