#ifndef NAVIGATIONAL_STATE_H
#define NAVIGATIONAL_STATE_H

#include <iostream>
#include <stdio.h>
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
    virtual bool ComputeDesiredPosition(bool) = 0;
    virtual NavigationalState * returnNextState() = 0;
};

class AutoPilotState : public NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class InitialApproach : public NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class FinalApproach : public NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class DataAcquisitionState : public NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class PullOutState : public NavigationalState
{
public:
    bool ComputeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

#endif /* NAVIGATIONAL_STATE_H */