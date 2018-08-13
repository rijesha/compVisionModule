compVisionModule

A collection of code for stereo vision plane detection

Basic Steps to install openCV (use v2.4 to v3.3)
```
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE  -DBUILD_opencv_java=OFF -D WITH_CUDA=OFF -D INSTALL_C_EXAMPLES=OFF -DBUILD_opencv_cnn_3dobj=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -DBUILD_opencv_dnn_modern=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_FAT_JAVA_LIB=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -DOpenCV_DIR=OpenCVConfig.cmake ..
make
make install
```

After building openCV. build aruco. Modify the location for the opencv release directory in build_aruco.
```
git submodule update --init --recursive
./build_aruco
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/libs/aruco/release/src/
```
