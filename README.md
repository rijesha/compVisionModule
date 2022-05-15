compVisionModule

A collection of code for stereo vision plane detection

Basic Steps to install openCV (use v2.4 to v3.3)
```
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE  -DBUILD_opencv_java=OFF -D WITH_CUDA=OFF -D INSTALL_C_EXAMPLES=OFF -DBUILD_opencv_cnn_3dobj=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -DBUILD_opencv_dnn_modern=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_FAT_JAVA_LIB=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_opencv_python=OFF -DOpenCV_DIR=OpenCVConfig.cmake ..
make -j
sudo make install
```

sudo apt-get install libboost-filesystem-dev

then you can build like any other cmake project from the main directory.

install mavlink:
cd into third_party/mavlink/pymavlink
sudo MDEF=`pwd`/../message_definitions pip install .


To start mavproxy on ground station

mavproxy.py --master=udpin:192.168.1.68:14561 --out=udpout:127.0.0.1:14562 --out=udpout:127.0.0.1:14563

the ip adress is the ipaddress of the ground station computer. 14562 is used for QGC and 14563 is used for ground_component

on the rover
mavproxy.py --master=udpout:127.0.0.1:14560 --out=udpout:192.168.1.68:14561

the ip adress is the ipaddress of the ground station computer. 14560 is used for uav sensor deploy and 14561 is used for mavproxy on the ground station computer


 make uav_sensor_deploy -j && ./uav_sensor_deploy/uav_sensor_deploy --calib_file ../data/realsense_calibration/realsense_cam_1.yml