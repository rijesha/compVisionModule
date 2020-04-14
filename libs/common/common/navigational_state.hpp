#ifndef NAVIGATIONAL_STATE_H
#define NAVIGATIONAL_STATE_H

#include <iostream>
#include <stdio.h>
#include <ctime>
#include <chrono>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include "aruco_processor.h"
#include <position_controller.h>

using namespace std;

template <class E> 
class NavigationalState
{
public:
    virtual E currentState() const = 0;
    virtual Position computeDesiredPosition(Position p) = 0;
    virtual NavigationalState * returnNextState(Position p) = 0;
};

#endif /* NAVIGATIONAL_STATE_H */