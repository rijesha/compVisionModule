compVisionModule

A collection of code for stereo vision plane detection

Basic Steps to install openCV
```
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE  -DBUILD_opencv_java=OFF -D WITH_CUDA=OFF -D INSTALL_C_EXAMPLES=OFF -D BUILD_opencv_python=OFF -DBUILD_opencv_cnn_3dobj=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -DBUILD_opencv_dnn_modern=OFF -D BUILD_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules/ -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_FAT_JAVA_LIB=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -DOpenCV_DIR=OpenCVConfig.cmake ..
make
make install
LD_LIBRARY_PATH=/usr/local/lib
export LD_LIBRARY_PATH
```

After cloning
```
git submodule update --init --recursive
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/libs/aruco/lib
```
