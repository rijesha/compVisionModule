compVisionModule

A collection of code for stereo vision plane detection

Basic Steps to install openCV (use v2.4 to v3.3)
```
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE  -DBUILD_opencv_java=OFF -D WITH_CUDA=OFF -D INSTALL_C_EXAMPLES=OFF -DBUILD_opencv_cnn_3dobj=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -DBUILD_opencv_dnn_modern=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_FAT_JAVA_LIB=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_opencv_python=OFF -DOpenCV_DIR=OpenCVConfig.cmake ..
make
sudo make install
```

sudo apt-get install libboost-filesystem-dev

then you can build like any other cmake project from the main directory.
