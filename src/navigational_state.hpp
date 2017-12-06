#ifndef NAVIGATIONAL_STATE_H
#define NAVIGATIONAL_STATE_H

#include <iostream>
#include <stdio.h>
#include "configuration.h"
#include <ctime>
#include <chrono>
#include <cstring>
//#include <iostream.h>
#include <time.h>
#include <unistd.h>

using namespace std;


class NavigationalState
{
public:
    bool A(void);
    bool B(void);
    bool C(void);
    bool D(void);
    bool E(void);
    bool F(void);
    static int zzz;
    virtual bool ComputeDesiredPosition(bool);
    virtual bool returnNextState();
};

class AutoPilotState : NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState returnNextState();
}

class InitialApproach : NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState returnNextState();
}

class FinalApproach : NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState returnNextState();
}

class DataAcquisitionState : NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState returnNextState();
}

class PullOutState : NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState returnNextState();
}

#endif /* NAVIGATIONAL_STATE_H */