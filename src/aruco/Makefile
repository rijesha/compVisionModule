# General
CC = g++
MAKEFLAGS += -j

#t slotted framing
# Names and flags
INC_DIR = .
CSOURCES = $(wildcard *.c )
CPPSOURCES += aruco_processor.cpp undistort_image.cpp ../cvm_argument_parser.cpp
COMMON_OBJECTS = $(CSOURCES:.c=.o) $(CPPSOURCES:.cpp=.o)
IMAGECOLLECTOR_SOURCES = test_data_collection.cpp
FOLDER_ANALYZER_SOURCES = folder_analyzer.cpp
EXECUTABLES = test_data_collection folderAnalyzer
CFLAGS = -c -Wall -g -I$(INC_DIR) -g -DSTANDALONE
OPENCV = `pkg-config opencv --cflags --libs `
LIBS = $(OPENCV)
LIBS +=  -lboost_system -lboost_filesystem


# Targets
all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLES)

folderAnalyzer: $(COMMON_OBJECTS) $(FOLDER_ANALYZER_SOURCES)
	$(CC) -pthread -std=c++11  $(COMMON_OBJECTS) $(FOLDER_ANALYZER_SOURCES) -o $@ $(LIBS) 

test_data_collection: $(COMMON_OBJECTS) $(IMAGECOLLECTOR_SOURCES)
	$(CC) -pthread -std=c++11 $(COMMON_OBJECTS) $(IMAGECOLLECTOR_SOURCES) -o $@ $(LIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -std=c++11 $(INCLUDES) $< -o $@

clean:
	@echo $COMMON_OBJECTS = $(COMMON_OBJECTS)
	rm -rf $(COMMON_OBJECTS) $(EXECUTABLE)