# setup.py
from setuptools import setup, Extension
import sys
import os

# Проверка зависимостей
def check_dependencies():
    import subprocess
    import pkgconfig
    
    required = ['x11', 'xext', 'xshm', 'v4l2']
    missing = []
    
    for pkg in required:
        try:
            if pkg == 'v4l2':
                # Для v4l2 проверяем через pkg-config
                subprocess.run(['pkg-config', '--exists', 'libv4l2'], 
                             check=True, capture_output=True)
            else:
                pkgconfig.exists(pkg)
        except:
            missing.append(pkg)
    
    if missing:
        print(f"Не найдены библиотеки: {missing}")
        print("Установите их с помощью:")
        print("  Ubuntu: sudo apt-get install libx11-dev libxext-dev libv4l-dev")
        print("  Fedora: sudo dnf install libX11-devel libXext-devel v4l-utils-devel")
        sys.exit(1)

# Проверяем зависимости перед компиляцией
check_dependencies()

# Флаги компиляции
extra_compile_args = [
    '-O3', '-march=native', '-mtune=native',  # Оптимизации
    '-ffast-math', '-funroll-loops',          # Дополнительные оптимизации
    '-fPIC',                                  # Position Independent Code
    '-std=c++17',                             # Стандарт C++17
]

# Проверяем поддержку AVX2
import subprocess
result = subprocess.run(['gcc', '-mavx2', '-dM', '-E', '-'],
                       input='', text=True, capture_output=True)
if 'AVX2' in result.stdout:
    extra_compile_args.append('-mavx2')
    print("AVX2 поддержка обнаружена, включаем оптимизации")

# Определение модуля
x11capture_module = Extension(
    'x11capture_v4l2',
    sources=['x11capture_v4l2_test.cpp'],  # Ваш C++ файл
    libraries=['X11', 'Xext', 'v4l2'],
    include_dirs=[
        '/usr/include',
        '/usr/local/include',
        '/usr/include/libv4l2'
    ],
    library_dirs=['/usr/lib', '/usr/local/lib'],
    extra_compile_args=extra_compile_args,
    language='c++',
    define_macros=[('USE_SIMD', '1')]
)

setup(
    name='x11capture-v4l2',
    version='2.0.0',
    description='High-performance X11 window capture with V4L2 loopback streaming',
    author='Your Name',
    ext_modules=[x11capture_module],
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: Multimedia :: Video :: Capture',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
    install_requires=[
        'numpy>=1.18.0',
        'opencv-python>=4.5.0',
    ],
)
