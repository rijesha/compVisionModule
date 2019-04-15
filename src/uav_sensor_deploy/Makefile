# General
CC = g++
MAKEFLAGS += -j

#-Imavlink-interface
# Names and flags
INC_DIR = -I. -I../../libs/mavlink-interface/mavlink_c_library_v1/common -I.. -I../../libs -I../../libs/camera-v4l2 -I../../libs/attitude_controller
CPPSOURCES += $(wildcard *.cpp) $(wildcard ../../libs/mavlink-interface/*.cpp) $(wildcard ../common/*.cpp) $(wildcard ../../libs/camera-v4l2/camera.cpp) $(wildcard ../../libs/attitude_controller/*.cpp)

OBJECTS = $(CPPSOURCES:.cpp=.o)
EXECUTABLE = deploy
CFLAGS = -c -Wall -O2 $(INC_DIR) 
OPENCV = `pkg-config opencv --cflags --libs `


LIBS = $(OPENCV)
LIBS += -L../../libs/aruco/lib -laruco


# Targets
all: $(CPPSOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) states.hpp
	$(CC) -pthread $(OBJECTS) -o $@ $(LIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -std=c++14 $(INCLUDES) $< -o $@

clean:
	@echo $OBJECTS = $(OBJECTS)
	rm -rf $(OBJECTS) $(EXECUTABLE)
