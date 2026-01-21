#include <Python.h>
#include <structmember.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <memory>
#include <string>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <algorithm>
#include <immintrin.h>

// ============================================================================
// КОНСТАНТЫ И НАСТРОЙКИ ПРОИЗВОДИТЕЛЬНОСТИ
// ============================================================================
#define V4L2_LOOPBACK_DEVICE "/dev/video10"  // Виртуальная камера
#define MAX_BUFFER_SIZE (1920 * 1080 * 4)    // Full HD BGRA (32 бита)
#define FRAME_BUFFER_COUNT 3                 // Тройная буферизация
#define DEFAULT_FPS 60                       // Частота кадров
#define USE_SIMD 1                           // Использовать SIMD оптимизации

// Поддерживаемые форматы V4L2
enum V4L2Format {
    FORMAT_YUYV = V4L2_PIX_FMT_YUYV,
    FORMAT_RGB24 = V4L2_PIX_FMT_RGB24,
    FORMAT_BGR24 = V4L2_PIX_FMT_BGR24,
    FORMAT_MJPEG = V4L2_PIX_FMT_MJPEG
};

// ============================================================================
// СТРУКТУРЫ ДАННЫХ
// ============================================================================
struct FrameData {
    uint8_t* data;
    int width;
    int height;
    int format; // V4L2 format code
    size_t size;
    std::chrono::steady_clock::time_point timestamp;
    
    FrameData() : data(nullptr), width(0), height(0), format(0), size(0) {}
    
    FrameData(const FrameData& other) : data(nullptr), width(0), height(0), format(0), size(0) {
        copyFrom(other);
    }
    
    FrameData& operator=(const FrameData& other) {
        if (this != &other) {
            copyFrom(other);
        }
        return *this;
    }
    
    void copyFrom(const FrameData& other) {
        if (data) {
            free(data);
            data = nullptr;
        }
        
        width = other.width;
        height = other.height;
        format = other.format;
        size = other.size;
        timestamp = other.timestamp;
        
        if (other.data && other.size > 0) {
            if (posix_memalign((void**)&data, 32, size) == 0) {
                memcpy(data, other.data, size);
            } else {
                data = nullptr;
                throw std::runtime_error("Failed to allocate aligned memory for copy");
            }
        }
    }
    
    void allocate(int w, int h, int fmt = FORMAT_BGR24) {
        width = w;
        height = h;
        format = fmt;
        
        switch (format) {
            case FORMAT_RGB24:
            case FORMAT_BGR24:
                size = w * h * 3; // 24-bit RGB/BGR
                break;
            case FORMAT_YUYV:
                size = w * h * 2; // 16-bit YUYV
                break;
            case FORMAT_MJPEG:
                size = w * h * 3; // Начальный размер для JPEG
                break;
            default:
                throw std::runtime_error("Unsupported format");
        }
        
        if (data) {
            free(data);
        }
        
        // Выровненная память для SIMD (32 байта для AVX)
        if (posix_memalign((void**)&data, 32, size) != 0) {
            throw std::runtime_error("Failed to allocate aligned memory");
        }
    }
    
    bool isValid() const {
        return data != nullptr && size > 0 && width > 0 && height > 0;
    }
    
    ~FrameData() {
        if (data) {
            free(data);
            data = nullptr;
        }
    }
};

// ============================================================================
// КЛАСС V4L2 LOOPBACK С ПОДДЕРЖКОЙ РАЗНЫХ ФОРМАТОВ
// ============================================================================
class V4L2Loopback {
private:
    int fd;
    bool initialized;
    int width;
    int height;
    unsigned int buffer_size;
    int pixel_format;
    
    // Memory-mapped буферы для нулевого копирования
    struct MMapBuffer {
        void* start;
        size_t length;
    };
    std::vector<MMapBuffer> buffers;
    unsigned int n_buffers;
    
public:
    V4L2Loopback(int format = FORMAT_BGR24) 
        : fd(-1), initialized(false), width(0), height(0), 
          buffer_size(0), pixel_format(format), n_buffers(FRAME_BUFFER_COUNT) {}
    
    ~V4L2Loopback() {
        close();
    }
    
    bool open_device(const std::string& device_path, int w, int h) {
        width = w;
        height = h;
        
        // Открываем устройство
        fd = v4l2_open(device_path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "V4L2: Cannot open device: " << device_path 
                     << " (error: " << strerror(errno) << ")" << std::endl;
            return false;
        }
        
        // Проверяем возможности
        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        if (v4l2_ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            std::cerr << "V4L2: QUERYCAP failed: " << strerror(errno) << std::endl;
            close();
            return false;
        }
        
        if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)) {
            std::cerr << "V4L2 device does not support video output" << std::endl;
            close();
            return false;
        }
        
        // Проверяем поддержку формата
        if (!check_format_support()) {
            std::cerr << "V4L2: Format not supported" << std::endl;
            close();
            return false;
        }
        
        // Устанавливаем формат
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixel_format;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
        
        // Устанавливаем параметры в зависимости от формата
        switch (pixel_format) {
            case FORMAT_RGB24:
            case FORMAT_BGR24:
                fmt.fmt.pix.bytesperline = width * 3;
                fmt.fmt.pix.sizeimage = width * height * 3;
                break;
            case FORMAT_YUYV:
                fmt.fmt.pix.bytesperline = width * 2;
                fmt.fmt.pix.sizeimage = width * height * 2;
                break;
            default:
                std::cerr << "V4L2: Unsupported pixel format: " << pixel_format << std::endl;
                close();
                return false;
        }
        
        if (v4l2_ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            std::cerr << "V4L2: S_FMT failed: " << strerror(errno) << std::endl;
            close();
            return false;
        }
        
        buffer_size = fmt.fmt.pix.sizeimage;
        std::cout << "V4L2 buffer size: " << buffer_size 
                  << " (format: " << format_to_string(pixel_format) 
                  << ")" << std::endl;
        
        // Настраиваем буферы
        if (!init_buffers()) {
            close();
            return false;
        }
        
        // Запускаем поток вывода
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (v4l2_ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            std::cerr << "V4L2: STREAMON failed: " << strerror(errno) << std::endl;
            close();
            return false;
        }
        
        initialized = true;
        std::cout << "V4L2 loopback initialized: " << device_path 
                  << " (" << width << "x" << height << ", " 
                  << format_to_string(pixel_format) << ")" << std::endl;
        return true;
    }
    
    bool check_format_support() {
        struct v4l2_fmtdesc fmtdesc;
        memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmtdesc.index = 0;
        
        std::cout << "Supported V4L2 formats:" << std::endl;
        bool format_supported = false;
        
        while (v4l2_ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
            std::cout << "  " << std::hex << fmtdesc.pixelformat << std::dec 
                      << " (" << (char*)fmtdesc.description << ")" << std::endl;
            
            if (fmtdesc.pixelformat == pixel_format) {
                format_supported = true;
            }
            
            fmtdesc.index++;
        }
        
        return format_supported;
    }
    
    static std::string format_to_string(int format) {
        switch (format) {
            case FORMAT_YUYV: return "YUYV";
            case FORMAT_RGB24: return "RGB24";
            case FORMAT_BGR24: return "BGR24";
            case FORMAT_MJPEG: return "MJPEG";
            default: return "UNKNOWN";
        }
    }
    
    bool init_buffers() {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = n_buffers;
        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_MMAP;
        
        if (v4l2_ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            std::cerr << "V4L2: REQBUFS failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        if (req.count < 2) {
            std::cerr << "V4L2: Insufficient buffer memory: " << req.count << " buffers" << std::endl;
            return false;
        }
        
        buffers.resize(req.count);
        std::cout << "V4L2 allocated " << req.count << " buffers" << std::endl;
        
        for (unsigned int i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            
            if (v4l2_ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                std::cerr << "V4L2: QUERYBUF failed for buffer " << i << std::endl;
                return false;
            }
            
            buffers[i].length = buf.length;
            buffers[i].start = v4l2_mmap(NULL, buf.length,
                                        PROT_READ | PROT_WRITE, MAP_SHARED,
                                        fd, buf.m.offset);
            
            if (buffers[i].start == MAP_FAILED) {
                std::cerr << "V4L2: mmap failed for buffer " << i << ": " << strerror(errno) << std::endl;
                return false;
            }
            
            // Запоминаем, что буфер готов к записи
            if (v4l2_ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                std::cerr << "V4L2: QBUF failed for buffer " << i << std::endl;
                return false;
            }
        }
        
        return true;
    }
    
    bool write_frame(const uint8_t* data, size_t size) {
        if (!initialized || fd < 0) {
            return false;
        }
        
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        
        // Получаем свободный буфер
        if (v4l2_ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) {
                // No buffer available, try again
                return false;
            }
            std::cerr << "V4L2: DQBUF failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Проверяем индекс буфера
        if (buf.index >= buffers.size()) {
            std::cerr << "V4L2: Invalid buffer index: " << buf.index << std::endl;
            return false;
        }
        
        // Копируем данные в memory-mapped буфер
        size_t copy_size = std::min<size_t>(buffers[buf.index].length, size);
        memcpy(buffers[buf.index].start, data, copy_size);
        buf.bytesused = copy_size;
        
        // Возвращаем буфер в очередь
        if (v4l2_ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "V4L2: QBUF failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        return true;
    }
    
    void close() {
        if (fd >= 0) {
            // Останавливаем поток
            if (initialized) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                v4l2_ioctl(fd, VIDIOC_STREAMOFF, &type);
            }
            
            // Освобождаем memory-mapped буферы
            for (auto& buffer : buffers) {
                if (buffer.start != MAP_FAILED && buffer.start != nullptr) {
                    v4l2_munmap(buffer.start, buffer.length);
                }
            }
            buffers.clear();
            
            v4l2_close(fd);
            fd = -1;
        }
        initialized = false;
    }
    
    bool is_initialized() const {
        return initialized;
    }
    
    int get_format() const { return pixel_format; }
    int get_width() const { return width; }
    int get_height() const { return height; }
    size_t get_buffer_size() const { return buffer_size; }
};

// ============================================================================
// SIMD ОПТИМИЗАЦИИ И КОНВЕРТЕРЫ ФОРМАТОВ
// ============================================================================
#ifdef USE_SIMD
class SIMDConverter {
private:
    // Маска для перестановки RGB -> BGR
    alignas(32) uint8_t shuffle_mask_32[32] = {
        2,1,0, 5,4,3, 8,7,6, 11,10,9, 14,13,12, 17,16,15,
        20,19,18, 23,22,21, 26,25,24, 29,28,27, 30,31
    };
    
public:
    // Быстрая конвертация RGB -> BGR с использованием AVX2
    void rgb_to_bgr_avx2(const uint8_t* rgb, uint8_t* bgr, int pixel_count) {
        const __m256i mask = _mm256_loadu_si256((__m256i*)shuffle_mask_32);
        
        int i = 0;
        for (; i <= pixel_count - 32; i += 32) {
            // Загружаем 32 пикселя (96 байт)
            __m256i chunk1 = _mm256_loadu_si256((__m256i*)(rgb + i * 3));
            __m256i chunk2 = _mm256_loadu_si256((__m256i*)(rgb + i * 3 + 32));
            __m256i chunk3 = _mm256_loadu_si256((__m256i*)(rgb + i * 3 + 64));
            
            // Переставляем байты (RGB -> BGR)
            chunk1 = _mm256_shuffle_epi8(chunk1, mask);
            chunk2 = _mm256_shuffle_epi8(chunk2, mask);
            chunk3 = _mm256_shuffle_epi8(chunk3, mask);
            
            // Сохраняем
            _mm256_storeu_si256((__m256i*)(bgr + i * 3), chunk1);
            _mm256_storeu_si256((__m256i*)(bgr + i * 3 + 32), chunk2);
            _mm256_storeu_si256((__m256i*)(bgr + i * 3 + 64), chunk3);
        }
        
        // Обработка остатка
        for (; i < pixel_count; i++) {
            bgr[i * 3] = rgb[i * 3 + 2];
            bgr[i * 3 + 1] = rgb[i * 3 + 1];
            bgr[i * 3 + 2] = rgb[i * 3];
        }
    }
    
    // Конвертация XImage (32-bit) в RGB24
    void ximage_to_rgb24(XImage* image, uint8_t* rgb, int target_width, int target_height) {
        int src_width = image->width;
        int src_height = image->height;
        char* src_data = image->data;
        int src_stride = image->bytes_per_line;
        
        // Определяем порядок байтов
        bool is_lsb_first = (image->byte_order == LSBFirst);
        
        // Для каждого целевого пикселя
        for (int y = 0; y < target_height; y++) {
            int src_y = (y * src_height) / target_height;
            char* src_row = src_data + src_y * src_stride;
            uint8_t* dst_row = rgb + y * target_width * 3;
            
            for (int x = 0; x < target_width; x++) {
                int src_x = (x * src_width) / target_width;
                char* src_pixel = src_row + src_x * 4;
                uint8_t* dst_pixel = dst_row + x * 3;
                
                unsigned long pixel;
                memcpy(&pixel, src_pixel, 4);
                
                // Извлекаем компоненты в RGB порядке
                if (is_lsb_first) {
                    // BGRA (little-endian) -> RGB
                    dst_pixel[0] = pixel & 0xFF;          // R
                    dst_pixel[1] = (pixel >> 8) & 0xFF;   // G
                    dst_pixel[2] = (pixel >> 16) & 0xFF;  // B
                } else {
                    // ARGB (big-endian) -> RGB
                    dst_pixel[0] = (pixel >> 24) & 0xFF;  // R
                    dst_pixel[1] = (pixel >> 16) & 0xFF;  // G
                    dst_pixel[2] = (pixel >> 8) & 0xFF;   // B
                }
            }
        }
    }
    
    // Конвертация XImage (32-bit) в BGR24
    void ximage_to_bgr24(XImage* image, uint8_t* bgr, int target_width, int target_height) {
        int src_width = image->width;
        int src_height = image->height;
        char* src_data = image->data;
        int src_stride = image->bytes_per_line;
        
        // Определяем порядок байтов
        bool is_lsb_first = (image->byte_order == LSBFirst);
        
        // Для каждого целевого пикселя
        for (int y = 0; y < target_height; y++) {
            int src_y = (y * src_height) / target_height;
            char* src_row = src_data + src_y * src_stride;
            uint8_t* dst_row = bgr + y * target_width * 3;
            
            for (int x = 0; x < target_width; x++) {
                int src_x = (x * src_width) / target_width;
                char* src_pixel = src_row + src_x * 4;
                uint8_t* dst_pixel = dst_row + x * 3;
                
                unsigned long pixel;
                memcpy(&pixel, src_pixel, 4);
                
                // Извлекаем компоненты в BGR порядке
                if (is_lsb_first) {
                    // BGRA (little-endian) -> BGR (уже почти правильно)
                    dst_pixel[0] = (pixel >> 16) & 0xFF;  // B
                    dst_pixel[1] = (pixel >> 8) & 0xFF;   // G
                    dst_pixel[2] = pixel & 0xFF;          // R
                } else {
                    // ARGB (big-endian) -> BGR
                    dst_pixel[0] = (pixel >> 8) & 0xFF;   // B
                    dst_pixel[1] = (pixel >> 16) & 0xFF;  // G
                    dst_pixel[2] = (pixel >> 24) & 0xFF;  // R
                }
            }
        }
    }
    
    // Конвертация RGB24 в YUYV с использованием целочисленной арифметики
    void rgb_to_yuyv(const uint8_t* rgb, uint8_t* yuyv, int width, int height) {
        // Коэффициенты для конвертации RGB в YUV (фиксированная точка 16.16)
        const int y_r = 19595;   // 0.299 * 65536
        const int y_g = 38470;   // 0.587 * 65536
        const int y_b = 7471;    // 0.114 * 65536
        
        const int u_r = -11058;  // -0.168736 * 65536
        const int u_g = -21710;  // -0.331264 * 65536
        const int u_b = 32768;   // 0.5 * 65536
        
        const int v_r = 32768;   // 0.5 * 65536
        const int v_g = -27439;  // -0.418688 * 65536
        const int v_b = -5329;   // -0.081312 * 65536
        
        for (int y = 0; y < height; y++) {
            const uint8_t* rgb_row = rgb + y * width * 3;
            uint8_t* yuyv_row = yuyv + y * width * 2;
            
            for (int x = 0; x < width; x += 2) {
                // Пиксель 1
                int r1 = rgb_row[x * 3];
                int g1 = rgb_row[x * 3 + 1];
                int b1 = rgb_row[x * 3 + 2];
                
                // Пиксель 2
                int r2 = rgb_row[(x + 1) * 3];
                int g2 = rgb_row[(x + 1) * 3 + 1];
                int b2 = rgb_row[(x + 1) * 3 + 2];
                
                // Вычисляем Y для каждого пикселя
                int y1 = (y_r * r1 + y_g * g1 + y_b * b1 + 32768) >> 16;
                int y2 = (y_r * r2 + y_g * g2 + y_b * b2 + 32768) >> 16;
                
                // Вычисляем U и V из усредненных значений двух пикселей
                int r_avg = (r1 + r2) / 2;
                int g_avg = (g1 + g2) / 2;
                int b_avg = (b1 + b2) / 2;
                
                int u = 128 + ((u_r * r_avg + u_g * g_avg + u_b * b_avg + 32768) >> 16);
                int v = 128 + ((v_r * r_avg + v_g * g_avg + v_b * b_avg + 32768) >> 16);
                
                // Ограничиваем значения 0-255
                y1 = std::max(0, std::min(255, y1));
                y2 = std::max(0, std::min(255, y2));
                u = std::max(0, std::min(255, u));
                v = std::max(0, std::min(255, v));
                
                // Записываем YUYV (Y0, U, Y1, V)
                int idx = x * 2;
                yuyv_row[idx] = static_cast<uint8_t>(y1);
                yuyv_row[idx + 1] = static_cast<uint8_t>(u);
                yuyv_row[idx + 2] = static_cast<uint8_t>(y2);
                yuyv_row[idx + 3] = static_cast<uint8_t>(v);
            }
        }
    }
};
#endif

// ============================================================================
// КЛАСС X11 CAPTURE С V4L2 LOOPBACK И ПОДДЕРЖКОЙ ФОРМАТОВ
// ============================================================================
class X11CaptureV4L2 {
private:
    // X11 компоненты
    Display* display;
    Window captureWindow;
    XWindowAttributes windowAttrs;
    XShmSegmentInfo shminfo;
    XImage* ximg;
    bool shm_available;
    
    // Поток захвата
    std::atomic<bool> running;
    std::thread captureThread;
    std::mutex frameMutex;
    std::mutex currentFrameMutex;
    
    // Буферизация кадров
    FrameData frameBuffers[FRAME_BUFFER_COUNT];
    FrameData currentFrame;
    std::atomic<int> writeIndex;
    std::atomic<int> readIndex;
    
    // V4L2 loopback
    V4L2Loopback v4l2;
    std::string v4l2_device_path;
    bool v4l2_enabled;
    int v4l2_format;
    
    // SIMD конвертер
#ifdef USE_SIMD
    SIMDConverter simd_converter;
#endif
    
    // Производительность
    std::chrono::steady_clock::time_point lastFrameTime;
    int targetFPS;
    std::atomic<float> actualFPS;
    std::atomic<int> frameCounter;
    std::thread fpsCounterThread;
    
    // Размеры
    int capture_width;
    int capture_height;
    int output_width;
    int output_height;
    int scaleFactor;
    
    // Временный буфер для конвертации
    std::vector<uint8_t> tempBuffer;
    
public:
    X11CaptureV4L2(bool enable_v4l2 = true, 
                   const std::string& v4l2_path = V4L2_LOOPBACK_DEVICE,
                   int format = FORMAT_BGR24)
        : display(nullptr), ximg(nullptr), shm_available(false), running(false),
          writeIndex(0), readIndex(0), v4l2_device_path(v4l2_path), 
          v4l2_enabled(enable_v4l2), v4l2_format(format),
          targetFPS(DEFAULT_FPS), actualFPS(0), frameCounter(0),
          capture_width(0), capture_height(0),
          output_width(0), output_height(0), scaleFactor(1),
          v4l2(format) {
        
        // Инициализируем X11 display
        display = XOpenDisplay(nullptr);
        if (!display) {
            throw std::runtime_error("Cannot open X11 display");
        }
        
        // Проверяем поддержку XShm
        int major, minor;
        Bool pixmaps;
        shm_available = XShmQueryVersion(display, &major, &minor, &pixmaps);
        if (!shm_available) {
            std::cout << "XShm not available, using slower XGetImage" << std::endl;
        } else {
            std::cout << "XShm version " << major << "." << minor << " available" << std::endl;
        }
        
        std::cout << "X11CaptureV4L2 initialized with format: " 
                  << V4L2Loopback::format_to_string(format) << std::endl;
    }
    
    ~X11CaptureV4L2() {
        stop();
        cleanup();
    }
    
    // ============================================================================
    // ОСНОВНЫЕ МЕТОДЫ
    // ============================================================================
    std::vector<std::pair<Window, std::string>> getWindowList() {
        std::vector<std::pair<Window, std::string>> windows;
        
        Atom actualType;
        int format;
        unsigned long count, bytesAfter;
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
            &count,
            &bytesAfter,
            &list
        );
        
        if (status == Success && list) {
            Window* wins = reinterpret_cast<Window*>(list);
            
            for (unsigned long i = 0; i < count; ++i) {
                std::string name = getWindowName(wins[i]);
                if (!name.empty()) {
                    windows.push_back({wins[i], name});
                }
            }
            
            XFree(list);
        }
        
        return windows;
    }
    
    bool startCapture(Window window, int cap_width = -1, int cap_height = -1,
                     int out_width = -1, int out_height = -1, 
                     int fps = DEFAULT_FPS, int scale = 1) {
        
        captureWindow = window;
        if (!XGetWindowAttributes(display, captureWindow, &windowAttrs)) {
            throw std::runtime_error("Cannot get window attributes");
        }
        
        // Определяем размер захвата
        capture_width = (cap_width > 0) ? cap_width : windowAttrs.width;
        capture_height = (cap_height > 0) ? cap_height : windowAttrs.height;
        
        if (capture_width <= 0 || capture_height <= 0) {
            throw std::runtime_error("Invalid capture dimensions");
        }
        
        // Определяем выходной размер
        if (out_width > 0 && out_height > 0) {
            output_width = out_width;
            output_height = out_height;
        } else if (scale > 1) {
            output_width = capture_width / scale;
            output_height = capture_height / scale;
            scaleFactor = scale;
        } else {
            output_width = capture_width;
            output_height = capture_height;
        }
        
        targetFPS = fps;
        
        // Инициализируем X11
        if (shm_available) {
            if (!initX11Shm()) {
                std::cout << "Falling back to XGetImage" << std::endl;
                shm_available = false;
            }
        }
        
        // Инициализируем V4L2 loopback если включен
        if (v4l2_enabled) {
            std::cout << "Initializing V4L2 with size " << output_width << "x" 
                      << output_height << " format: " 
                      << V4L2Loopback::format_to_string(v4l2_format) << std::endl;
            
            if (!v4l2.open_device(v4l2_device_path, output_width, output_height)) {
                std::cerr << "Failed to initialize V4L2, continuing without it" << std::endl;
                v4l2_enabled = false;
            } else {
                std::cout << "V4L2 initialized successfully" << std::endl;
            }
        }
        
        // Инициализируем буферы в RGB формате (для внутреннего использования)
        for (int i = 0; i < FRAME_BUFFER_COUNT; i++) {
            frameBuffers[i].allocate(output_width, output_height, FORMAT_RGB24);
        }
        
        // Инициализируем текущий кадр
        currentFrame.allocate(output_width, output_height, FORMAT_RGB24);
        
        // Инициализируем временный буфер для конвертации
        tempBuffer.resize(output_width * output_height * 3);
        
        // Запускаем поток захвата
        running = true;
        captureThread = std::thread(&X11CaptureV4L2::captureLoop, this);
        
        // Запускаем счетчик FPS
        frameCounter = 0;
        fpsCounterThread = std::thread(&X11CaptureV4L2::fpsCounterLoop, this);
        
        std::cout << "Capture started: " << capture_width << "x" << capture_height 
                  << " -> " << output_width << "x" << output_height 
                  << " @ " << fps << " FPS" 
                  << " (V4L2 format: " << V4L2Loopback::format_to_string(v4l2_format) 
                  << ")" << std::endl;
        
        return true;
    }
    
    void stop() {
        running = false;
        
        if (captureThread.joinable()) {
            captureThread.join();
        }
        
        if (fpsCounterThread.joinable()) {
            fpsCounterThread.join();
        }
        
        v4l2.close();
    }
    
    FrameData getLatestFrame(int format = FORMAT_RGB24) {
        std::lock_guard<std::mutex> lock(currentFrameMutex);
        
        if (!currentFrame.isValid()) {
            return FrameData();
        }
        
        // Если запрошенный формат совпадает с текущим, возвращаем как есть
        if (format == currentFrame.format) {
            FrameData copy;
            try {
                copy = currentFrame;
                return copy;
            } catch (const std::exception& e) {
                std::cerr << "Failed to copy frame: " << e.what() << std::endl;
                return FrameData();
            }
        }
        
        // Конвертируем в запрошенный формат
        return convertFrame(currentFrame, format);
    }
    
    // ============================================================================
    // ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
    // ============================================================================
private:
    std::string getWindowName(Window win) {
        Atom actualType;
        int format;
        unsigned long count, bytesAfter;
        unsigned char* name = nullptr;
        std::string result;
        
        // Пробуем UTF8_STRING
        Status status = XGetWindowProperty(
            display,
            win,
            XInternAtom(display, "_NET_WM_NAME", False),
            0L,
            1024L,
            False,
            XInternAtom(display, "UTF8_STRING", False),
            &actualType,
            &format,
            &count,
            &bytesAfter,
            &name
        );
        
        if (status == Success && name && count > 0) {
            result = std::string(reinterpret_cast<char*>(name), count);
            XFree(name);
            return result;
        }
        
        if (name) XFree(name);
        
        // Пробуем WM_NAME
        status = XGetWindowProperty(
            display,
            win,
            XInternAtom(display, "WM_NAME", False),
            0L,
            1024L,
            False,
            AnyPropertyType,
            &actualType,
            &format,
            &count,
            &bytesAfter,
            &name
        );
        
        if (status == Success && name && count > 0) {
            if (format == 8) {
                result = std::string(reinterpret_cast<char*>(name), count);
            }
            XFree(name);
        }
        
        return result;
    }
    
    bool initX11Shm() {
        Screen* screen = DefaultScreenOfDisplay(display);
        int depth = DefaultDepthOfScreen(screen);
        
        if (depth != 24 && depth != 32) {
            std::cout << "Unsupported depth: " << depth << ", falling back to XGetImage" << std::endl;
            return false;
        }
        
        ximg = XShmCreateImage(
            display,
            DefaultVisualOfScreen(screen),
            depth,
            ZPixmap,
            nullptr,
            &shminfo,
            capture_width,
            capture_height
        );
        
        if (!ximg) {
            std::cerr << "XShmCreateImage failed" << std::endl;
            return false;
        }
        
        shminfo.shmid = shmget(
            IPC_PRIVATE,
            ximg->bytes_per_line * ximg->height,
            IPC_CREAT | 0777
        );
        
        if (shminfo.shmid < 0) {
            XDestroyImage(ximg);
            ximg = nullptr;
            std::cerr << "Failed to allocate shared memory" << std::endl;
            return false;
        }
        
        shminfo.shmaddr = ximg->data = static_cast<char*>(shmat(shminfo.shmid, 0, 0));
        shminfo.readOnly = False;
        
        if (!XShmAttach(display, &shminfo)) {
            shmdt(shminfo.shmaddr);
            shmctl(shminfo.shmid, IPC_RMID, nullptr);
            XDestroyImage(ximg);
            ximg = nullptr;
            std::cerr << "Failed to attach shared memory" << std::endl;
            return false;
        }
        
        XSync(display, False);
        return true;
    }
    
    void captureLoop() {
        using namespace std::chrono;
        auto frameDuration = microseconds(1000000 / targetFPS);
        lastFrameTime = steady_clock::now();
        
        XImage* local_ximg = nullptr;
        
        while (running) {
            auto startTime = steady_clock::now();
            
            // Обработка X11 событий
            processX11Events();
            
            // Обновляем атрибуты окна
            XGetWindowAttributes(display, captureWindow, &windowAttrs);
            
            // Захват кадра из X11
            if (shm_available && ximg) {
                if (!XShmGetImage(display, captureWindow, ximg, 0, 0, AllPlanes)) {
                    std::cerr << "XShmGetImage failed, falling back to XGetImage" << std::endl;
                    shm_available = false;
                    if (ximg) {
                        XShmDetach(display, &shminfo);
                        XDestroyImage(ximg);
                        ximg = nullptr;
                    }
                }
                local_ximg = ximg;
            } else {
                local_ximg = XGetImage(display, captureWindow, 0, 0,
                                      capture_width, capture_height,
                                      AllPlanes, ZPixmap);
                if (!local_ximg) {
                    std::this_thread::sleep_for(milliseconds(10));
                    continue;
                }
            }
            
            // Получаем индекс для записи
            int write_idx;
            FrameData* frame;
            {
                std::lock_guard<std::mutex> lock(frameMutex);
                write_idx = writeIndex.load();
                frame = &frameBuffers[write_idx];
            }
            
            // Захватываем в RGB (внутренний формат)
            if (local_ximg) {
#ifdef USE_SIMD
                simd_converter.ximage_to_rgb24(local_ximg, frame->data,
                                              output_width, output_height);
#else
                ximage_to_rgb24_basic(local_ximg, frame->data,
                                     output_width, output_height);
#endif
                
                frame->timestamp = steady_clock::now();
                
                // Обновляем текущий кадр
                {
                    std::lock_guard<std::mutex> lock(currentFrameMutex);
                    try {
                        currentFrame = *frame;
                    } catch (const std::exception& e) {
                        std::cerr << "Failed to update current frame: " << e.what() << std::endl;
                    }
                }
                
                frameCounter++;
                
                // Записываем в V4L2 если включено
                if (v4l2_enabled) {
                    sendToV4L2(*frame);
                }
                
                // Обновляем индексы буферов
                {
                    std::lock_guard<std::mutex> lock(frameMutex);
                    int new_write = (write_idx + 1) % FRAME_BUFFER_COUNT;
                    writeIndex.store(new_write);
                    readIndex.store(write_idx);
                }
                
                // Освобождаем XGetImage если использовали его
                if (!shm_available) {
                    XDestroyImage(local_ximg);
                }
            }
            
            // Контроль FPS
            auto endTime = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(endTime - startTime);
            
            if (elapsed < frameDuration) {
                std::this_thread::sleep_for(frameDuration - elapsed);
            }
            
            lastFrameTime = endTime;
        }
        
        if (!shm_available && local_ximg) {
            XDestroyImage(local_ximg);
        }
    }
    
    void ximage_to_rgb24_basic(XImage* image, uint8_t* rgb, int target_width, int target_height) {
        int src_width = image->width;
        int src_height = image->height;
        
        for (int y = 0; y < target_height; y++) {
            int src_y = (y * src_height) / target_height;
            for (int x = 0; x < target_width; x++) {
                int src_x = (x * src_width) / target_width;
                
                unsigned long pixel = XGetPixel(image, src_x, src_y);
                
                int dst_idx = (y * target_width + x) * 3;
                
                // Извлекаем RGB
                rgb[dst_idx] = pixel & 0xFF;         // R
                rgb[dst_idx + 1] = (pixel >> 8) & 0xFF;  // G
                rgb[dst_idx + 2] = (pixel >> 16) & 0xFF; // B
            }
        }
    }
    
    void sendToV4L2(const FrameData& frame) {
        // Конвертируем из RGB (внутренний формат) в формат V4L2
        switch (v4l2_format) {
            case FORMAT_RGB24:
                // Уже в RGB, отправляем как есть
                v4l2.write_frame(frame.data, frame.size);
                break;
                
            case FORMAT_BGR24:
                // Конвертируем RGB -> BGR
                tempBuffer.resize(frame.size);
#ifdef USE_SIMD
                simd_converter.rgb_to_bgr_avx2(frame.data, tempBuffer.data(), 
                                              frame.width * frame.height);
#else
                rgb_to_bgr_basic(frame.data, tempBuffer.data(), 
                                frame.width * frame.height);
#endif
                v4l2.write_frame(tempBuffer.data(), frame.size);
                break;
                
            case FORMAT_YUYV:
                // Конвертируем RGB -> YUYV
                tempBuffer.resize(frame.width * frame.height * 2);
#ifdef USE_SIMD
                simd_converter.rgb_to_yuyv(frame.data, tempBuffer.data(),
                                          frame.width, frame.height);
#else
                rgb_to_yuyv_basic(frame.data, tempBuffer.data(),
                                 frame.width, frame.height);
#endif
                v4l2.write_frame(tempBuffer.data(), frame.width * frame.height * 2);
                break;
                
            default:
                std::cerr << "Unsupported V4L2 format: " << v4l2_format << std::endl;
                break;
        }
    }
    
    void rgb_to_bgr_basic(const uint8_t* rgb, uint8_t* bgr, int pixel_count) {
        for (int i = 0; i < pixel_count; i++) {
            bgr[i * 3] = rgb[i * 3 + 2];
            bgr[i * 3 + 1] = rgb[i * 3 + 1];
            bgr[i * 3 + 2] = rgb[i * 3];
        }
    }
    
    void rgb_to_yuyv_basic(const uint8_t* rgb, uint8_t* yuyv, int width, int height) {
        for (int y = 0; y < height; y++) {
            const uint8_t* rgb_row = rgb + y * width * 3;
            uint8_t* yuyv_row = yuyv + y * width * 2;
            
            for (int x = 0; x < width; x += 2) {
                // Пиксель 1
                int r1 = rgb_row[x * 3];
                int g1 = rgb_row[x * 3 + 1];
                int b1 = rgb_row[x * 3 + 2];
                
                // Пиксель 2
                int r2 = rgb_row[(x + 1) * 3];
                int g2 = rgb_row[(x + 1) * 3 + 1];
                int b2 = rgb_row[(x + 1) * 3 + 2];
                
                // Формулы для конвертации RGB в YUV
                int y1 = 0.299 * r1 + 0.587 * g1 + 0.114 * b1;
                int y2 = 0.299 * r2 + 0.587 * g2 + 0.114 * b2;
                
                // Усредненные значения для U и V
                int r_avg = (r1 + r2) / 2;
                int g_avg = (g1 + g2) / 2;
                int b_avg = (b1 + b2) / 2;
                
                int u = 128 + (-0.168736 * r_avg - 0.331264 * g_avg + 0.5 * b_avg);
                int v = 128 + (0.5 * r_avg - 0.418688 * g_avg - 0.081312 * b_avg);
                
                // Ограничиваем значения
                y1 = std::max(0, std::min(255, y1));
                y2 = std::max(0, std::min(255, y2));
                u = std::max(0, std::min(255, u));
                v = std::max(0, std::min(255, v));
                
                // Записываем YUYV
                int idx = x * 2;
                yuyv_row[idx] = static_cast<uint8_t>(y1);
                yuyv_row[idx + 1] = static_cast<uint8_t>(u);
                yuyv_row[idx + 2] = static_cast<uint8_t>(y2);
                yuyv_row[idx + 3] = static_cast<uint8_t>(v);
            }
        }
    }
    
    FrameData convertFrame(const FrameData& src, int dst_format) {
        FrameData dst;
        dst.allocate(src.width, src.height, dst_format);
        dst.timestamp = src.timestamp;
        
        // Конвертация из RGB (внутренний формат) в целевой формат
        switch (dst_format) {
            case FORMAT_RGB24:
                // Уже в RGB, просто копируем
                memcpy(dst.data, src.data, src.size);
                break;
                
            case FORMAT_BGR24:
                // RGB -> BGR
#ifdef USE_SIMD
                simd_converter.rgb_to_bgr_avx2(src.data, dst.data, 
                                              src.width * src.height);
#else
                rgb_to_bgr_basic(src.data, dst.data, src.width * src.height);
#endif
                break;
                
            case FORMAT_YUYV:
                // RGB -> YUYV
#ifdef USE_SIMD
                simd_converter.rgb_to_yuyv(src.data, dst.data,
                                          src.width, src.height);
#else
                rgb_to_yuyv_basic(src.data, dst.data,
                                 src.width, src.height);
#endif
                break;
                
            default:
                std::cerr << "Unsupported conversion format: " << dst_format << std::endl;
                break;
        }
        
        return dst;
    }
    
    void processX11Events() {
        while (XPending(display)) {
            XEvent event;
            XNextEvent(display, &event);
        }
    }
    
    void fpsCounterLoop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            int frames = frameCounter.exchange(0);
            actualFPS = static_cast<float>(frames);
            
            if (actualFPS > 0 && frames < targetFPS - 5) {
                std::cout << "FPS: " << actualFPS << " (target: " << targetFPS << ")" << std::endl;
            }
        }
    }
    
    void cleanup() {
        stop();
        
        if (display) {
            if (ximg) {
                XShmDetach(display, &shminfo);
                XDestroyImage(ximg);
                if (shminfo.shmaddr) {
                    shmdt(shminfo.shmaddr);
                }
                if (shminfo.shmid >= 0) {
                    shmctl(shminfo.shmid, IPC_RMID, nullptr);
                }
                ximg = nullptr;
            }
            XCloseDisplay(display);
            display = nullptr;
        }
    }
    
public:
    bool isV4L2Enabled() const { return v4l2_enabled && v4l2.is_initialized(); }
    float getFPS() const { return actualFPS.load(); }
    std::string getV4L2Device() const { return v4l2_device_path; }
    int getV4L2Format() const { return v4l2_format; }
    std::string getFormatString() const { return V4L2Loopback::format_to_string(v4l2_format); }
};

// ============================================================================
// PYTHON ОБЕРТКА
// ============================================================================
typedef struct {
    PyObject_HEAD
    X11CaptureV4L2* capture;
    bool is_capturing;
} PyX11CaptureV4L2;

// Прототипы функций
static void PyX11CaptureV4L2_dealloc(PyX11CaptureV4L2* self);
static PyObject* PyX11CaptureV4L2_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
static int PyX11CaptureV4L2_init(PyX11CaptureV4L2* self, PyObject* args, PyObject* kwds);
static PyObject* PyX11CaptureV4L2_get_window_list(PyX11CaptureV4L2* self);
static PyObject* PyX11CaptureV4L2_start_capture(PyX11CaptureV4L2* self, PyObject* args, PyObject* kwds);
static PyObject* PyX11CaptureV4L2_get_frame(PyX11CaptureV4L2* self, PyObject* args, PyObject* kwds);
static PyObject* PyX11CaptureV4L2_stop(PyX11CaptureV4L2* self);
static PyObject* PyX11CaptureV4L2_get_fps(PyX11CaptureV4L2* self);
static PyObject* PyX11CaptureV4L2_is_v4l2_enabled(PyX11CaptureV4L2* self);
static PyObject* PyX11CaptureV4L2_get_format(PyX11CaptureV4L2* self);
static PyObject* PyX11CaptureV4L2_get_format_string(PyX11CaptureV4L2* self);

// Методы класса
static PyMethodDef PyX11CaptureV4L2_methods[] = {
    {"get_window_list", (PyCFunction)PyX11CaptureV4L2_get_window_list, METH_NOARGS,
     "Get list of available windows"},
    {"start_capture", (PyCFunction)PyX11CaptureV4L2_start_capture, 
     METH_VARARGS | METH_KEYWORDS,
     "Start capturing a window and streaming to V4L2"},
    {"get_frame", (PyCFunction)PyX11CaptureV4L2_get_frame, 
     METH_VARARGS | METH_KEYWORDS,
     "Get current frame as bytes"},
    {"stop", (PyCFunction)PyX11CaptureV4L2_stop, METH_NOARGS,
     "Stop capturing"},
    {"get_fps", (PyCFunction)PyX11CaptureV4L2_get_fps, METH_NOARGS,
     "Get current FPS"},
    {"is_v4l2_enabled", (PyCFunction)PyX11CaptureV4L2_is_v4l2_enabled, METH_NOARGS,
     "Check if V4L2 streaming is enabled"},
    {"get_format", (PyCFunction)PyX11CaptureV4L2_get_format, METH_NOARGS,
     "Get current V4L2 format code"},
    {"get_format_string", (PyCFunction)PyX11CaptureV4L2_get_format_string, METH_NOARGS,
     "Get current V4L2 format as string"},
    {nullptr, nullptr, 0, nullptr}
};

// Тип объекта
static PyTypeObject PyX11CaptureV4L2Type = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    .tp_name = "x11capture_v4l2.X11CaptureV4L2",
    .tp_basicsize = sizeof(PyX11CaptureV4L2),
    .tp_itemsize = 0,
    .tp_dealloc = (destructor)PyX11CaptureV4L2_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = "X11 Window Capture with V4L2 Loopback Streaming",
    .tp_methods = PyX11CaptureV4L2_methods,
    .tp_init = (initproc)PyX11CaptureV4L2_init,
    .tp_new = PyX11CaptureV4L2_new,
};

// Реализация методов
static void PyX11CaptureV4L2_dealloc(PyX11CaptureV4L2* self) {
    if (self->capture) {
        self->capture->stop();
        delete self->capture;
        self->capture = nullptr;
    }
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* PyX11CaptureV4L2_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    PyX11CaptureV4L2* self = (PyX11CaptureV4L2*)type->tp_alloc(type, 0);
    if (self) {
        self->capture = nullptr;
        self->is_capturing = false;
    }
    return (PyObject*)self;
}

static int PyX11CaptureV4L2_init(PyX11CaptureV4L2* self, PyObject* args, PyObject* kwds) {
    int enable_v4l2 = 1;
    const char* v4l2_device = V4L2_LOOPBACK_DEVICE;
    int format = FORMAT_BGR24;  // По умолчанию BGR24
    
    static char* kwlist[] = {"enable_v4l2", "v4l2_device", "format", nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|psi", kwlist, 
                                     &enable_v4l2, &v4l2_device, &format)) {
        return -1;
    }
    
    try {
        self->capture = new X11CaptureV4L2(enable_v4l2 != 0, v4l2_device, format);
        return 0;
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return -1;
    }
}

static PyObject* PyX11CaptureV4L2_get_window_list(PyX11CaptureV4L2* self) {
    try {
        auto windows = self->capture->getWindowList();
        PyObject* list = PyList_New(windows.size());
        
        for (size_t i = 0; i < windows.size(); i++) {
            PyObject* tuple = PyTuple_New(2);
            PyTuple_SetItem(tuple, 0, PyLong_FromUnsignedLong(windows[i].first));
            PyTuple_SetItem(tuple, 1, PyUnicode_FromString(windows[i].second.c_str()));
            PyList_SetItem(list, i, tuple);
        }
        
        return list;
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* PyX11CaptureV4L2_start_capture(PyX11CaptureV4L2* self, 
                                                PyObject* args, PyObject* kwds) {
    unsigned long window_id = 0;
    int capture_width = -1;
    int capture_height = -1;
    int output_width = -1;
    int output_height = -1;
    int fps = DEFAULT_FPS;
    int scale = 1;
    
    static char* kwlist[] = {"window_id", "capture_width", "capture_height",
                            "output_width", "output_height", "fps", "scale",
                            nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "k|iiiiii", kwlist,
                                     &window_id, &capture_width, &capture_height,
                                     &output_width, &output_height, &fps, &scale)) {
        return nullptr;
    }
    
    try {
        if (self->capture->startCapture(static_cast<Window>(window_id),
                                       capture_width, capture_height,
                                       output_width, output_height,
                                       fps, scale)) {
            self->is_capturing = true;
            
            // Возвращаем информацию о V4L2 устройстве
            PyObject* result = PyDict_New();
            PyDict_SetItemString(result, "success", Py_True);
            PyDict_SetItemString(result, "v4l2_device", 
                               PyUnicode_FromString(self->capture->getV4L2Device().c_str()));
            PyDict_SetItemString(result, "v4l2_enabled", 
                               self->capture->isV4L2Enabled() ? Py_True : Py_False);
            PyDict_SetItemString(result, "format", 
                               PyLong_FromLong(self->capture->getV4L2Format()));
            PyDict_SetItemString(result, "format_string", 
                               PyUnicode_FromString(self->capture->getFormatString().c_str()));
            
            return result;
        } else {
            Py_RETURN_FALSE;
        }
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* PyX11CaptureV4L2_get_frame(PyX11CaptureV4L2* self, 
                                           PyObject* args, PyObject* kwds) {
    if (!self->is_capturing) {
        PyErr_SetString(PyExc_RuntimeError, "Not capturing");
        return nullptr;
    }
    
    int format = FORMAT_RGB24; // По умолчанию RGB
    static char* kwlist[] = {"format", nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|i", kwlist, &format)) {
        return nullptr;
    }
    
    try {
        auto frame = self->capture->getLatestFrame(format);
        
        if (!frame.isValid()) {
            Py_RETURN_NONE;
        }
        
        // Создаем bytes объект с данными кадра
        PyObject* frame_bytes = PyBytes_FromStringAndSize(
            reinterpret_cast<const char*>(frame.data), 
            frame.size
        );
        
        if (!frame_bytes) {
            PyErr_SetString(PyExc_MemoryError, "Failed to create bytes object");
            return nullptr;
        }
        
        // Возвращаем кортеж (data, width, height, format)
        PyObject* result = PyTuple_New(4);
        if (!result) {
            Py_DECREF(frame_bytes);
            return nullptr;
        }
        
        PyTuple_SetItem(result, 0, frame_bytes);
        PyTuple_SetItem(result, 1, PyLong_FromLong(frame.width));
        PyTuple_SetItem(result, 2, PyLong_FromLong(frame.height));
        PyTuple_SetItem(result, 3, PyLong_FromLong(frame.format));
        
        return result;
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* PyX11CaptureV4L2_stop(PyX11CaptureV4L2* self) {
    if (self->is_capturing) {
        self->capture->stop();
        self->is_capturing = false;
    }
    Py_RETURN_NONE;
}

static PyObject* PyX11CaptureV4L2_get_fps(PyX11CaptureV4L2* self) {
    try {
        float fps = self->capture->getFPS();
        return PyFloat_FromDouble(fps);
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* PyX11CaptureV4L2_is_v4l2_enabled(PyX11CaptureV4L2* self) {
    try {
        if (self->capture->isV4L2Enabled()) {
            Py_RETURN_TRUE;
        } else {
            Py_RETURN_FALSE;
        }
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* PyX11CaptureV4L2_get_format(PyX11CaptureV4L2* self) {
    try {
        int format = self->capture->getV4L2Format();
        return PyLong_FromLong(format);
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* PyX11CaptureV4L2_get_format_string(PyX11CaptureV4L2* self) {
    try {
        std::string format_str = self->capture->getFormatString();
        return PyUnicode_FromString(format_str.c_str());
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ МОДУЛЯ
// ============================================================================
static PyModuleDef x11capture_v4l2_module = {
    PyModuleDef_HEAD_INIT,
    .m_name = "x11capture_v4l2",
    .m_doc = "High-performance X11 window capture with V4L2 loopback streaming",
    .m_size = -1,
};

PyMODINIT_FUNC PyInit_x11capture_v4l2(void) {
    PyObject* module;
    
    if (PyType_Ready(&PyX11CaptureV4L2Type) < 0) {
        return nullptr;
    }
    
    module = PyModule_Create(&x11capture_v4l2_module);
    if (module == nullptr) {
        return nullptr;
    }
    
    Py_INCREF(&PyX11CaptureV4L2Type);
    if (PyModule_AddObject(module, "X11CaptureV4L2", 
                          (PyObject*)&PyX11CaptureV4L2Type) < 0) {
        Py_DECREF(&PyX11CaptureV4L2Type);
        Py_DECREF(module);
        return nullptr;
    }
    
    // Добавляем константы форматов
    PyModule_AddIntConstant(module, "FORMAT_YUYV", FORMAT_YUYV);
    PyModule_AddIntConstant(module, "FORMAT_RGB24", FORMAT_RGB24);
    PyModule_AddIntConstant(module, "FORMAT_BGR24", FORMAT_BGR24);
    PyModule_AddIntConstant(module, "FORMAT_MJPEG", FORMAT_MJPEG);
    
    PyModule_AddStringConstant(module, "DEFAULT_V4L2_DEVICE", V4L2_LOOPBACK_DEVICE);
    
    return module;
}
