# General
CC = g++
MAKEFLAGS += -j

#-Imavlink-interface
# Names and flags
INC_DIR = -I. -I../../libs/mavlink-interface/mavlink_c_library_v1/common -I.. -I../../libs -I../../libs/camera-v4l2
CPPSOURCES += $(wildcard ../../libs/mavlink-interface/*.cpp) $(wildcard ../../libs/camera-v4l2/camera.cpp) $(wildcard ../common/*.cpp)

COMMON_OBJECTS = $(CPPSOURCES:.cpp=.o)

IMAGECOLLECTOR_SOURCES = test_data_collection.cpp
FOLDER_ANALYZER_SOURCES = folder_analyzer.cpp
EXECUTABLES = test_data_collection folderAnalyzer

CFLAGS = -c -Wall -g $(INC_DIR)  
OPENCV = `pkg-config opencv --cflags --libs `


LIBS = $(OPENCV)
LIBS += -L../../libs/aruco/lib -laruco  -L../../libs/camera-v4l2
LIBS += -lboost_system -lboost_filesystem


# Targets
all: $(CPPSOURCES) $(EXECUTABLES)

folderAnalyzer: $(COMMON_OBJECTS) $(FOLDER_ANALYZER_SOURCES)
	$(CC) -pthread -std=c++11  $(COMMON_OBJECTS) $(FOLDER_ANALYZER_SOURCES) -o $@ $(LIBS) $(INC_DIR)

test_data_collection: $(COMMON_OBJECTS) $(IMAGECOLLECTOR_SOURCES)
	$(CC) -pthread -std=c++11 $(COMMON_OBJECTS) $(IMAGECOLLECTOR_SOURCES) -o $@ $(LIBS) $(INC_DIR)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -std=c++14 $(INCLUDES) $< -o $@

clean:
	@echo COMMON_OBJECTS = $(COMMON_OBJECTS)
	rm -rf $(COMMON_OBJECTS) $(EXECUTABLES)
