// x11capture_fixed.cpp
#ifdef STANDALONE_APP
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <signal.h>
#else
#include <Python.h>
#include <structmember.h>
#endif

// ============================================================================
// СТАНДАРТНЫЕ ЗАГОЛОВКИ
// ============================================================================
#include <stdexcept>
#include <vector>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <memory>
#include <utility>
#include <functional>
#include <queue>
#include <map>

// ============================================================================
// СИСТЕМНЫЕ ЗАГОЛОВКИ
// ============================================================================
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <immintrin.h>

// ============================================================================
// КОНСТАНТЫ И НАСТРОЙКИ
// ============================================================================
#define V4L2_LOOPBACK_DEVICE "/dev/video10"
#define MAX_BUFFER_SIZE (1920 * 1080 * 4)
#define FRAME_BUFFER_COUNT 3
#define DEFAULT_FPS 60

// Для standalone используем write() вместо mmap для надежности
#ifdef STANDALONE_APP
#define USE_SIMPLE_WRITE 1
#else
#define USE_SIMPLE_WRITE 0
#endif

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
    int format;
    size_t size;
    std::chrono::steady_clock::time_point timestamp;
    
    FrameData() : data(nullptr), width(0), height(0), format(FORMAT_RGB24), size(0) {}
    
    FrameData(const FrameData& other) : data(nullptr), width(0), height(0), format(FORMAT_RGB24), size(0) {
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
            data = static_cast<uint8_t*>(malloc(size));
            if (data) {
                memcpy(data, other.data, size);
            } else {
                throw std::runtime_error("Failed to allocate memory for copy");
            }
        }
    }
    
    void allocate(int w, int h, int fmt = FORMAT_RGB24) {
        width = w;
        height = h;
        format = fmt;
        
        switch (format) {
            case FORMAT_RGB24:
            case FORMAT_BGR24:
                size = w * h * 3;
                break;
            case FORMAT_YUYV:
                size = w * h * 2;
                break;
            case FORMAT_MJPEG:
                size = w * h * 3;
                break;
            default:
                throw std::runtime_error("Unsupported format");
        }
        
        if (data) {
            free(data);
        }
        
        data = static_cast<uint8_t*>(malloc(size));
        if (!data) {
            throw std::runtime_error("Failed to allocate memory");
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
// КЛАСС V4L2 LOOPBACK - УПРОЩЕННАЯ ВЕРСИЯ ДЛЯ STANDALONE
// ============================================================================
class V4L2Loopback {
private:
    int fd;
    bool initialized;
    int width;
    int height;
    unsigned int buffer_size;
    int pixel_format;
    
#if !USE_SIMPLE_WRITE
    struct MMapBuffer {
        void* start;
        size_t length;
    };
    std::vector<MMapBuffer> buffers;
    unsigned int n_buffers;
#endif
    
public:
    V4L2Loopback(int format = FORMAT_BGR24) 
        : fd(-1), initialized(false), width(0), height(0), 
          buffer_size(0), pixel_format(format)
#if !USE_SIMPLE_WRITE
        , n_buffers(FRAME_BUFFER_COUNT)
#endif
    {}
    
    ~V4L2Loopback() {
        close();
    }
    
    bool open_device(const std::string& device_path, int w, int h) {
        width = w;
        height = h;
        
#if USE_SIMPLE_WRITE
        fd = open(device_path.c_str(), O_WRONLY | O_NONBLOCK);
#else
        fd = v4l2_open(device_path.c_str(), O_RDWR | O_NONBLOCK);
#endif
        
        if (fd < 0) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: Cannot open device: " << device_path 
                     << " (error: " << strerror(errno) << ")" << std::endl;
#endif
            return false;
        }
        
        struct v4l2_capability cap;
        memset(&cap, 0, sizeof(cap));
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: QUERYCAP failed: " << strerror(errno) << std::endl;
#endif
            close();
            return false;
        }
        
        if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2 device does not support video output" << std::endl;
#endif
            close();
            return false;
        }
        
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixel_format;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
        
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
#ifdef STANDALONE_APP
                std::cerr << "V4L2: Unsupported pixel format: " << pixel_format << std::endl;
#endif
                close();
                return false;
        }
        
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: S_FMT failed: " << strerror(errno) << std::endl;
#endif
            close();
            return false;
        }
        
        buffer_size = fmt.fmt.pix.sizeimage;
#ifdef STANDALONE_APP
        std::cout << "V4L2: " << width << "x" << height 
                  << ", format: " << format_to_string(pixel_format)
                  << ", buffer: " << buffer_size << " bytes" << std::endl;
#endif
        
#if !USE_SIMPLE_WRITE
        if (!init_buffers()) {
            close();
            return false;
        }
        
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: STREAMON failed: " << strerror(errno) << std::endl;
#endif
            close();
            return false;
        }
#endif
        
        initialized = true;
#ifdef STANDALONE_APP
        std::cout << "V4L2 initialized: " << device_path << std::endl;
#endif
        return true;
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
    
#if !USE_SIMPLE_WRITE
    bool init_buffers() {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = n_buffers;
        req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        req.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: REQBUFS failed: " << strerror(errno) << std::endl;
#endif
            return false;
        }
        
        if (req.count < 2) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: Insufficient buffer memory: " << req.count << " buffers" << std::endl;
#endif
            return false;
        }
        
        buffers.resize(req.count);
#ifdef STANDALONE_APP
        std::cout << "V4L2 allocated " << req.count << " buffers" << std::endl;
#endif
        
        for (unsigned int i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            
            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
#ifdef STANDALONE_APP
                std::cerr << "V4L2: QUERYBUF failed for buffer " << i << std::endl;
#endif
                return false;
            }
            
            buffers[i].length = buf.length;
            buffers[i].start = mmap(NULL, buf.length,
                                   PROT_READ | PROT_WRITE, MAP_SHARED,
                                   fd, buf.m.offset);
            
            if (buffers[i].start == MAP_FAILED) {
#ifdef STANDALONE_APP
                std::cerr << "V4L2: mmap failed for buffer " << i << ": " << strerror(errno) << std::endl;
#endif
                return false;
            }
            
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
#ifdef STANDALONE_APP
                std::cerr << "V4L2: QBUF failed for buffer " << i << std::endl;
#endif
                return false;
            }
        }
        
        return true;
    }
#endif
    
    bool write_frame(const uint8_t* data, size_t size) {
        if (!initialized || fd < 0) {
            return false;
        }
        
#if USE_SIMPLE_WRITE
        // Простой write() для standalone - стабильнее
        ssize_t written = write(fd, data, size);
        if (written != static_cast<ssize_t>(size)) {
            if (errno == EAGAIN) {
                // Буфер полон, пропускаем кадр
                return false;
            }
#ifdef STANDALONE_APP
            std::cerr << "V4L2: write failed: " << strerror(errno) << std::endl;
#endif
            return false;
        }
        return true;
#else
        // Оригинальный mmap код для Python
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) {
                return false;
            }
#ifdef STANDALONE_APP
            std::cerr << "V4L2: DQBUF failed: " << strerror(errno) << std::endl;
#endif
            return false;
        }
        
        if (buf.index >= buffers.size()) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: Invalid buffer index: " << buf.index << std::endl;
#endif
            return false;
        }
        
        size_t copy_size = std::min<size_t>(buffers[buf.index].length, size);
        memcpy(buffers[buf.index].start, data, copy_size);
        buf.bytesused = copy_size;
        
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
#ifdef STANDALONE_APP
            std::cerr << "V4L2: QBUF failed: " << strerror(errno) << std::endl;
#endif
            return false;
        }
        
        return true;
#endif
    }
    
    void close() {
        if (fd >= 0) {
#if !USE_SIMPLE_WRITE
            if (initialized) {
                enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                ioctl(fd, VIDIOC_STREAMOFF, &type);
            }
            
            for (auto& buffer : buffers) {
                if (buffer.start != MAP_FAILED && buffer.start != nullptr) {
                    munmap(buffer.start, buffer.length);
                }
            }
            buffers.clear();
#endif
            
            ::close(fd);
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
// КЛАСС АНАЛИЗА ФОРМАТА X11
// ============================================================================
class X11FormatAnalyzer {
private:
    int bitsPerPixel;
    int bytesPerPixel;
    unsigned long redMask, greenMask, blueMask;
    int redShift, greenShift, blueShift;
    bool isLittleEndian;
    
public:
    X11FormatAnalyzer() : bitsPerPixel(0), bytesPerPixel(0), 
                          redMask(0), greenMask(0), blueMask(0),
                          redShift(0), greenShift(0), blueShift(0),
                          isLittleEndian(true) {}
    
    void analyze(XImage* image) {
        bitsPerPixel = image->bits_per_pixel;
        bytesPerPixel = bitsPerPixel / 8;
        redMask = image->red_mask;
        greenMask = image->green_mask;
        blueMask = image->blue_mask;
        isLittleEndian = (image->byte_order == LSBFirst);
        
        calculateMaskShifts();
        
#ifdef STANDALONE_APP
        std::cout << "=== X11 FORMAT ANALYSIS ===" << std::endl;
        std::cout << "Bits per pixel: " << bitsPerPixel << std::endl;
        std::cout << "Bytes per pixel: " << bytesPerPixel << std::endl;
        std::cout << "Bytes per line: " << image->bytes_per_line << std::endl;
        std::cout << "Red mask: 0x" << std::hex << redMask << std::dec
                  << ", shift: " << redShift << std::endl;
        std::cout << "Green mask: 0x" << std::hex << greenMask << std::dec
                  << ", shift: " << greenShift << std::endl;
        std::cout << "Blue mask: 0x" << std::hex << blueMask << std::dec
                  << ", shift: " << blueShift << std::endl;
        std::cout << "Byte order: " << (isLittleEndian ? "Little Endian" : "Big Endian") << std::endl;
        std::cout << "Format: " << getFormatName() << std::endl;
#endif
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
    
    std::string getFormatName() const {
        if (bytesPerPixel == 4) {
            if (isLittleEndian) {
                if (redMask == 0xff0000 && greenMask == 0xff00 && blueMask == 0xff) {
                    return "BGRx";
                } else if (redMask == 0xff && greenMask == 0xff00 && blueMask == 0xff0000) {
                    return "xRGB";
                } else if (redMask == 0xff000000 && greenMask == 0xff0000 && blueMask == 0xff00) {
                    return "RGBx";
                }
            }
        } else if (bytesPerPixel == 3) {
            if (redMask == 0xff && greenMask == 0xff00 && blueMask == 0xff0000) {
                return "BGR";
            } else if (redMask == 0xff0000 && greenMask == 0xff00 && blueMask == 0xff) {
                return "RGB";
            }
        } else if (bytesPerPixel == 2) {
            return "16-bit";
        } else if (bytesPerPixel == 1) {
            return "8-bit";
        }
        return "UNKNOWN";
    }
    
    bool isOptimizedFormat() const {
        std::string format = getFormatName();
        return format == "BGRx" || format == "xRGB" || format == "BGR" || format == "RGB";
    }
    
    int getBytesPerPixel() const { return bytesPerPixel; }
    bool isLittleEndianFormat() const { return isLittleEndian; }
    bool isFormatAnalyzed() const { return bitsPerPixel > 0; }
    
    // Получить компоненты цвета из пикселя
    void getRGBComponents(unsigned long pixel, uint8_t& r, uint8_t& g, uint8_t& b) const {
        r = static_cast<uint8_t>((pixel & redMask) >> redShift);
        g = static_cast<uint8_t>((pixel & greenMask) >> greenShift);
        b = static_cast<uint8_t>((pixel & blueMask) >> blueShift);
    }
};

// ============================================================================
// КЛАСС X11 CAPTURE С V4L2
// ============================================================================
class X11CaptureV4L2 {
private:
    Display* display;
    Window captureWindow;
    XWindowAttributes windowAttrs;
    XShmSegmentInfo shminfo;
    XImage* ximg;
    bool shm_available;
    
    X11FormatAnalyzer formatAnalyzer;
    bool formatAnalyzed;
    
    std::atomic<bool> running;
    std::thread captureThread;
    std::mutex frameMutex;
    std::mutex currentFrameMutex;
    
    FrameData frameBuffers[FRAME_BUFFER_COUNT];
    FrameData currentFrame;
    std::atomic<int> writeIndex;
    std::atomic<int> readIndex;
    
    V4L2Loopback v4l2;
    std::string v4l2_device_path;
    bool v4l2_enabled;
    int v4l2_format;
    
    std::chrono::steady_clock::time_point lastFrameTime;
    int targetFPS;
    std::atomic<float> actualFPS;
    std::atomic<int> frameCounter;
    std::thread fpsCounterThread;
    
    int capture_width;
    int capture_height;
    int output_width;
    int output_height;
    
    std::vector<uint8_t> tempBuffer;
    
public:
    X11CaptureV4L2(bool enable_v4l2 = true, 
                   const std::string& v4l2_path = V4L2_LOOPBACK_DEVICE,
                   int format = FORMAT_BGR24)
        : display(nullptr), ximg(nullptr), shm_available(false), 
          formatAnalyzed(false), running(false), writeIndex(0), readIndex(0),
          v4l2_device_path(v4l2_path), v4l2_enabled(enable_v4l2), 
          v4l2_format(format), v4l2(format), targetFPS(DEFAULT_FPS), 
          actualFPS(0), frameCounter(0), capture_width(0), capture_height(0),
          output_width(0), output_height(0) {
        
        display = XOpenDisplay(nullptr);
        if (!display) {
            throw std::runtime_error("Cannot open X11 display");
        }
        
        int major, minor;
        Bool pixmaps;
        shm_available = XShmQueryVersion(display, &major, &minor, &pixmaps);
        if (!shm_available) {
#ifdef STANDALONE_APP
            std::cout << "XShm not available, using XGetImage" << std::endl;
#endif
        }
        
#ifdef STANDALONE_APP
        std::cout << "X11CaptureV4L2 initialized with format: " 
                  << V4L2Loopback::format_to_string(format) << std::endl;
#endif
    }
    
    ~X11CaptureV4L2() {
        stop();
        cleanup();
    }
    
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
        
        capture_width = (cap_width > 0) ? cap_width : windowAttrs.width;
        capture_height = (cap_height > 0) ? cap_height : windowAttrs.height;
        
        if (capture_width <= 0 || capture_height <= 0) {
            throw std::runtime_error("Invalid capture dimensions");
        }
        
        if (out_width > 0 && out_height > 0) {
            output_width = out_width;
            output_height = out_height;
        } else if (scale > 1) {
            output_width = capture_width / scale;
            output_height = capture_height / scale;
        } else {
            output_width = capture_width;
            output_height = capture_height;
        }
        
        targetFPS = fps;
        
        if (shm_available) {
            if (!initX11Shm()) {
#ifdef STANDALONE_APP
                std::cout << "Falling back to XGetImage" << std::endl;
#endif
                shm_available = false;
            }
        }
        
        if (v4l2_enabled) {
#ifdef STANDALONE_APP
            std::cout << "Initializing V4L2: " << output_width << "x" 
                      << output_height << " @ " << fps << " FPS" << std::endl;
#endif
            
            if (!v4l2.open_device(v4l2_device_path, output_width, output_height)) {
#ifdef STANDALONE_APP
                std::cerr << "Failed to initialize V4L2, continuing without it" << std::endl;
#endif
                v4l2_enabled = false;
            }
        }
        
        for (int i = 0; i < FRAME_BUFFER_COUNT; i++) {
            frameBuffers[i].allocate(output_width, output_height, FORMAT_RGB24);
        }
        
        currentFrame.allocate(output_width, output_height, FORMAT_RGB24);
        
        tempBuffer.resize(output_width * output_height * 3);
        
        running = true;
        captureThread = std::thread(&X11CaptureV4L2::captureLoop, this);
        
        frameCounter = 0;
        fpsCounterThread = std::thread(&X11CaptureV4L2::fpsCounterLoop, this);
        
#ifdef STANDALONE_APP
        std::cout << "Capture started: " << capture_width << "x" << capture_height 
                  << " -> " << output_width << "x" << output_height 
                  << " @ " << fps << " FPS" << std::endl;
#endif
        
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
        
        if (format == FORMAT_RGB24) {
            FrameData copy;
            try {
                copy = currentFrame;
                return copy;
            } catch (const std::exception& e) {
#ifdef STANDALONE_APP
                std::cerr << "Failed to copy frame: " << e.what() << std::endl;
#endif
                return FrameData();
            }
        }
        
        return convertFrame(currentFrame, format);
    }
    
private:
    std::string getWindowName(Window win) {
        Atom actualType;
        int format;
        unsigned long count, bytesAfter;
        unsigned char* name = nullptr;
        std::string result;
        
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
#ifdef STANDALONE_APP
            std::cout << "Unsupported depth: " << depth << ", falling back to XGetImage" << std::endl;
#endif
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
#ifdef STANDALONE_APP
            std::cerr << "XShmCreateImage failed" << std::endl;
#endif
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
#ifdef STANDALONE_APP
            std::cerr << "Failed to allocate shared memory" << std::endl;
#endif
            return false;
        }
        
        shminfo.shmaddr = ximg->data = static_cast<char*>(shmat(shminfo.shmid, 0, 0));
        shminfo.readOnly = False;
        
        if (!XShmAttach(display, &shminfo)) {
            shmdt(shminfo.shmaddr);
            shmctl(shminfo.shmid, IPC_RMID, nullptr);
            XDestroyImage(ximg);
            ximg = nullptr;
#ifdef STANDALONE_APP
            std::cerr << "Failed to attach shared memory" << std::endl;
#endif
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
            
            processX11Events();
            XGetWindowAttributes(display, captureWindow, &windowAttrs);
            
            if (shm_available && ximg) {
                if (!XShmGetImage(display, captureWindow, ximg, 0, 0, AllPlanes)) {
#ifdef STANDALONE_APP
                    std::cerr << "XShmGetImage failed, falling back to XGetImage" << std::endl;
#endif
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
            
            if (!formatAnalyzed) {
                formatAnalyzer.analyze(local_ximg);
                formatAnalyzed = true;
            }
            
            int write_idx;
            FrameData* frame;
            {
                std::lock_guard<std::mutex> lock(frameMutex);
                write_idx = writeIndex.load();
                frame = &frameBuffers[write_idx];
            }
            
            if (local_ximg) {
                // ВАЖНО: Для standalone используем простую конвертацию
                convertXImageSimple(local_ximg, frame->data);
                
                // Отладочный вывод первого кадра
                static bool first_frame = true;
                if (first_frame) {
                    first_frame = false;
#ifdef STANDALONE_APP
                    std::cout << "First frame captured: " 
                              << static_cast<int>(frame->data[0]) << " "
                              << static_cast<int>(frame->data[1]) << " "
                              << static_cast<int>(frame->data[2]) << std::endl;
#endif
                }
                
                frame->timestamp = steady_clock::now();
                
                {
                    std::lock_guard<std::mutex> lock(currentFrameMutex);
                    try {
                        currentFrame = *frame;
                    } catch (const std::exception& e) {
#ifdef STANDALONE_APP
                        std::cerr << "Failed to update current frame: " << e.what() << std::endl;
#endif
                    }
                }
                
                frameCounter++;
                
                if (v4l2_enabled) {
                    sendToV4L2(*frame);
                }
                
                {
                    std::lock_guard<std::mutex> lock(frameMutex);
                    int new_write = (write_idx + 1) % FRAME_BUFFER_COUNT;
                    writeIndex.store(new_write);
                    readIndex.store(write_idx);
                }
                
                if (!shm_available) {
                    XDestroyImage(local_ximg);
                }
            }
            
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
    
    // Простая и надежная конвертация XImage -> RGB24
    void convertXImageSimple(XImage* image, uint8_t* rgb) {
        int src_width = image->width;
        int src_height = image->height;
        
        for (int y = 0; y < output_height; y++) {
            int src_y = (y * src_height) / output_height;
            for (int x = 0; x < output_width; x++) {
                int src_x = (x * src_width) / output_width;
                
                unsigned long pixel = XGetPixel(image, src_x, src_y);
                
                int dst_idx = (y * output_width + x) * 3;
                
                // ВАЖНО: Правильный порядок байт для RGB24
                rgb[dst_idx] = (pixel >> 16) & 0xFF;     // Red
                rgb[dst_idx + 1] = (pixel >> 8) & 0xFF;  // Green
                rgb[dst_idx + 2] = pixel & 0xFF;         // Blue
            }
        }
    }
    
    void sendToV4L2(const FrameData& frame) {
        switch (v4l2_format) {
            case FORMAT_RGB24:
                v4l2.write_frame(frame.data, frame.size);
                break;
                
            case FORMAT_BGR24:
                tempBuffer.resize(frame.size);
                rgb_to_bgr_simple(frame.data, tempBuffer.data(), 
                                frame.width * frame.height);
                v4l2.write_frame(tempBuffer.data(), frame.size);
                break;
                
            case FORMAT_YUYV:
                tempBuffer.resize(frame.width * frame.height * 2);
                rgb_to_yuyv_simple(frame.data, tempBuffer.data(),
                                  frame.width, frame.height);
                v4l2.write_frame(tempBuffer.data(), frame.width * frame.height * 2);
                break;
                
            default:
#ifdef STANDALONE_APP
                std::cerr << "Unsupported V4L2 format: " << v4l2_format << std::endl;
#endif
                break;
        }
    }
    
    void rgb_to_bgr_simple(const uint8_t* rgb, uint8_t* bgr, int pixel_count) {
        for (int i = 0; i < pixel_count; i++) {
            bgr[i * 3] = rgb[i * 3 + 2];     // B
            bgr[i * 3 + 1] = rgb[i * 3 + 1]; // G
            bgr[i * 3 + 2] = rgb[i * 3];     // R
        }
    }
    
    void rgb_to_yuyv_simple(const uint8_t* rgb, uint8_t* yuyv, int width, int height) {
        for (int y = 0; y < height; y++) {
            const uint8_t* rgb_row = rgb + y * width * 3;
            uint8_t* yuyv_row = yuyv + y * width * 2;
            
            for (int x = 0; x < width; x += 2) {
                int r1 = rgb_row[x * 3];
                int g1 = rgb_row[x * 3 + 1];
                int b1 = rgb_row[x * 3 + 2];
                
                int r2 = rgb_row[(x + 1) * 3];
                int g2 = rgb_row[(x + 1) * 3 + 1];
                int b2 = rgb_row[(x + 1) * 3 + 2];
                
                // Стандартные коэффициенты для YUV
                int y1 = 0.299 * r1 + 0.587 * g1 + 0.114 * b1;
                int y2 = 0.299 * r2 + 0.587 * g2 + 0.114 * b2;
                
                int r_avg = (r1 + r2) / 2;
                int g_avg = (g1 + g2) / 2;
                int b_avg = (b1 + b2) / 2;
                
                int u = 128 + (-0.168736 * r_avg - 0.331264 * g_avg + 0.5 * b_avg);
                int v = 128 + (0.5 * r_avg - 0.418688 * g_avg - 0.081312 * b_avg);
                
                y1 = std::max(0, std::min(255, y1));
                y2 = std::max(0, std::min(255, y2));
                u = std::max(0, std::min(255, u));
                v = std::max(0, std::min(255, v));
                
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
        
        switch (dst_format) {
            case FORMAT_RGB24:
                memcpy(dst.data, src.data, src.size);
                break;
                
            case FORMAT_BGR24:
                rgb_to_bgr_simple(src.data, dst.data, src.width * src.height);
                break;
                
            case FORMAT_YUYV:
                rgb_to_yuyv_simple(src.data, dst.data, src.width, src.height);
                break;
                
            default:
#ifdef STANDALONE_APP
                std::cerr << "Unsupported conversion format: " << dst_format << std::endl;
#endif
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
            
#ifdef STANDALONE_APP
            if (frames > 0) {
                std::cout << "FPS: " << actualFPS << std::endl;
            }
#endif
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
    std::string getX11Format() const { return formatAnalyzer.getFormatName(); }
};

// ============================================================================
// АВТОНОМНОЕ ПРИЛОЖЕНИЕ
// ============================================================================
#ifdef STANDALONE_APP

std::atomic<bool> g_signal_received(false);
X11CaptureV4L2* g_capture_ptr = nullptr;

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
    signal(SIGPIPE, SIG_IGN);
}

void printUsage(const char* program_name) {
    std::cout << "X11 to V4L2 Screen Capture Tool" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --device N      V4L2 device number (default: 10 = /dev/video10)" << std::endl;
    std::cout << "  --format FMT    Output format: yuyv, rgb24, bgr24 (default: bgr24)" << std::endl;
    std::cout << "  --list          List available windows and exit" << std::endl;
    std::cout << "  --window ID     Window ID to capture (from --list)" << std::endl;
    std::cout << "  --fps N         Target FPS (default: 60)" << std::endl;
    std::cout << "  --scale N       Scaling factor (default: 1)" << std::endl;
    std::cout << "  --help          Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " --list" << std::endl;
    std::cout << "  " << program_name << " --window 0x12345678" << std::endl;
    std::cout << "  " << program_name << " --device 11 --window 0x12345678 --format yuyv --fps 30 --scale 2" << std::endl;
}

void printBanner() {
    std::cout << "==========================================" << std::endl;
    std::cout << "     X11 to V4L2 Screen Capture" << std::endl;
    std::cout << "     Fixed Version - No Stripes" << std::endl;
    std::cout << "==========================================" << std::endl;
}

int main(int argc, char* argv[]) {
    printBanner();
    registerSignalHandlers();
    
    std::string v4l2_device = V4L2_LOOPBACK_DEVICE;
    bool list_windows = false;
    Window selected_window = 0;
    int fps = DEFAULT_FPS;
    int scale = 1;
    int format = FORMAT_BGR24;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--device" && i + 1 < argc) {
            int device_num = std::stoi(argv[++i]);
            v4l2_device = "/dev/video" + std::to_string(device_num);
        } else if (arg == "--format" && i + 1 < argc) {
            std::string fmt = argv[++i];
            if (fmt == "yuyv") format = FORMAT_YUYV;
            else if (fmt == "rgb24") format = FORMAT_RGB24;
            else if (fmt == "bgr24") format = FORMAT_BGR24;
            else {
                std::cerr << "Unknown format: " << fmt << std::endl;
                return 1;
            }
        } else if (arg == "--list") {
            list_windows = true;
        } else if (arg == "--window" && i + 1 < argc) {
            std::string window_str = argv[++i];
            if (window_str.substr(0, 2) == "0x") {
                selected_window = std::stoul(window_str.substr(2), nullptr, 16);
            } else {
                selected_window = std::stoul(window_str, nullptr, 0);
            }
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = std::stoi(argv[++i]);
        } else if (arg == "--scale" && i + 1 < argc) {
            scale = std::stoi(argv[++i]);
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
        std::unique_ptr<X11CaptureV4L2> capture;
        try {
            capture = std::make_unique<X11CaptureV4L2>(true, v4l2_device, format);
        } catch (const std::exception& e) {
            std::cerr << "Error creating capture: " << e.what() << std::endl;
            return 1;
        }
        
        g_capture_ptr = capture.get();
        
        if (list_windows) {
            auto windows = capture->getWindowList();
            std::cout << "\nAvailable windows (" << windows.size() << " found):" << std::endl;
            std::cout << "==========================================" << std::endl;
            for (size_t i = 0; i < std::min(windows.size(), size_t(20)); i++) {
                std::cout << i << ": 0x" << std::hex << windows[i].first << std::dec 
                          << " - " << windows[i].second << std::endl;
            }
            
            if (windows.size() > 20) {
                std::cout << "... and " << (windows.size() - 20) << " more windows." << std::endl;
            }
            return 0;
        }
        
        if (selected_window == 0) {
            auto windows = capture->getWindowList();
            if (windows.empty()) {
                std::cerr << "No windows found" << std::endl;
                return 1;
            }
            
            std::cout << "\nAvailable windows (" << windows.size() << " found):" << std::endl;
            for (size_t i = 0; i < std::min(windows.size(), size_t(10)); i++) {
                std::cout << i << ": 0x" << std::hex << windows[i].first << std::dec 
                          << " - " << windows[i].second << std::endl;
            }
            
            std::cout << "\nSelect window number (0-" << std::min(windows.size(), size_t(10))-1 << "): ";
            int selection;
            std::cin >> selection;
            
            if (selection < 0 || selection >= int(windows.size())) {
                std::cerr << "Invalid selection" << std::endl;
                return 1;
            }
            
            selected_window = windows[selection].first;
        }
        
        std::cout << "\nInitializing capture for window 0x" 
                  << std::hex << selected_window << std::dec << "..." << std::endl;
        
        if (!capture->startCapture(selected_window, -1, -1, -1, -1, fps, scale)) {
            std::cerr << "Failed to start capture" << std::endl;
            return 1;
        }
        
        std::cout << "\nInitialization complete!" << std::endl;
        std::cout << "Device: " << v4l2_device << std::endl;
        std::cout << "Format: " << V4L2Loopback::format_to_string(format) << std::endl;
        std::cout << "Target FPS: " << fps << std::endl;
        std::cout << "V4L2 streaming: " << (capture->isV4L2Enabled() ? "ENABLED" : "DISABLED") << std::endl;
        
        std::cout << "\nOpenCV can now access this virtual camera." << std::endl;
        std::cout << "\nPress Ctrl+C to stop." << std::endl;
        
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

// ============================================================================
// PYTHON МОДУЛЬ
// ============================================================================
#else

typedef struct {
    PyObject_HEAD
    X11CaptureV4L2* capture;
    bool is_capturing;
} PyX11CaptureV4L2;

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
static PyObject* PyX11CaptureV4L2_get_x11_format(PyX11CaptureV4L2* self);

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
    {"get_x11_format", (PyCFunction)PyX11CaptureV4L2_get_x11_format, METH_NOARGS,
     "Get detected X11 format name"},
    {nullptr, nullptr, 0, nullptr}
};

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
    int format = FORMAT_BGR24;
    
    static const char* kwlist[] = {"enable_v4l2", "v4l2_device", "format", nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|isi", const_cast<char**>(kwlist), 
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
    
    static const char* kwlist[] = {"window_id", "capture_width", "capture_height",
                            "output_width", "output_height", "fps", "scale",
                            nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "k|iiiiii", const_cast<char**>(kwlist),
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
            PyDict_SetItemString(result, "x11_format", 
                               PyUnicode_FromString(self->capture->getX11Format().c_str()));
            
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
    
    int format = FORMAT_RGB24;
    static const char* kwlist[] = {"format", nullptr};
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|i", const_cast<char**>(kwlist), &format)) {
        return nullptr;
    }
    
    try {
        auto frame = self->capture->getLatestFrame(format);
        
        if (!frame.isValid()) {
            Py_RETURN_NONE;
        }
        
        PyObject* frame_bytes = PyBytes_FromStringAndSize(
            reinterpret_cast<const char*>(frame.data), 
            frame.size
        );
        
        if (!frame_bytes) {
            PyErr_SetString(PyExc_MemoryError, "Failed to create bytes object");
            return nullptr;
        }
        
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

static PyObject* PyX11CaptureV4L2_get_x11_format(PyX11CaptureV4L2* self) {
    try {
        std::string format_str = self->capture->getX11Format();
        return PyUnicode_FromString(format_str.c_str());
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

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
    
    PyModule_AddIntConstant(module, "FORMAT_YUYV", FORMAT_YUYV);
    PyModule_AddIntConstant(module, "FORMAT_RGB24", FORMAT_RGB24);
    PyModule_AddIntConstant(module, "FORMAT_BGR24", FORMAT_BGR24);
    PyModule_AddIntConstant(module, "FORMAT_MJPEG", FORMAT_MJPEG);
    
    PyModule_AddStringConstant(module, "DEFAULT_V4L2_DEVICE", V4L2_LOOPBACK_DEVICE);
    
    return module;
}

#endif
