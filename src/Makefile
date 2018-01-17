# General
CC = g++
MAKEFLAGS += -j

# Names and flags
INC_DIR = -I. -I../libs/mavlink/
CPPSOURCES += $(wildcard *.cpp) $(wildcard mavlink_control/*.cpp)
CPPSOURCES += aruco/aruco_processor.cpp aruco/undistort_image.cpp 

OBJECTS = $(CPPSOURCES:.cpp=.o)
EXECUTABLE = deploy
CFLAGS = -c -Wall -g $(INC_DIR)  
OPENCV = `pkg-config opencv --cflags --libs `
LIBS = $(OPENCV)


# Targets
all: $(CPPSOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -pthread $(OBJECTS) -o $@ $(LIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -std=c++11 $(INCLUDES) $< -o $@

clean:
	@echo $OBJECTS = $(OBJECTS)
	rm -rf $(OBJECTS) $(EXECUTABLE)
