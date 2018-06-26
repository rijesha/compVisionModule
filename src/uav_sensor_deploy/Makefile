# General
CC = g++
MAKEFLAGS += -j

#-Imavlink-interface
# Names and flags
INC_DIR = -I. -I../../libs/mavlink-interface/mavlink_c_library_v1/common -I.. -I../../libs -I../../libs/camera-v4l2
CPPSOURCES += $(wildcard *.cpp) $(wildcard ../../libs/mavlink-interface/*.cpp) $(wildcard ../common/*.cpp) $(wildcard ../../libs/camera-v4l2/camera.cpp)

OBJECTS = $(CPPSOURCES:.cpp=.o)
EXECUTABLE = deploy
CFLAGS = -c -Wall -g $(INC_DIR)  
OPENCV = `pkg-config opencv --cflags --libs `


LIBS = $(OPENCV)
LIBS += -L../../libs/aruco/lib -laruco


# Targets
all: $(CPPSOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -pthread $(OBJECTS) -o $@ $(LIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -std=c++14 $(INCLUDES) $< -o $@

clean:
	@echo $OBJECTS = $(OBJECTS)
	rm -rf $(OBJECTS) $(EXECUTABLE)
