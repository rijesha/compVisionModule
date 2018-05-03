# General
CC = g++
MAKEFLAGS += -j

# Names and flags
INC_DIR = -I. -I../../libs/argparse
CSOURCES = $(wildcard *.c )
CPPSOURCES += $(wildcard *.cpp)
CPPSOURCES +=  ../common/cvm_argument_parser.cpp

OBJECTS = $(CSOURCES:.c=.o) $(CPPSOURCES:.cpp=.o)
EXECUTABLE = calibrate
CFLAGS = -c -Wall -g $(INC_DIR) 
OPENCV = `pkg-config opencv --cflags --libs `
LIBS = $(OPENCV)


# Targets
all: $(CSOURCES) $(CPPSOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -pthread $(OBJECTS) -o $@ $(LIBS)

.c.o:
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

.cpp.o:
	$(CC) $(CFLAGS) -std=c++11 $(INCLUDES) $< -o $@

clean:
	@echo $OBJECTS = $(OBJECTS)
	rm -rf $(OBJECTS) $(EXECUTABLE)
