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
    enum State {BASE, AP, IP, FP, DA, PO};
    bool A(void);
    bool B(void);
    bool C(void);
    bool D(void);
    bool E(void);
    bool F(void);
    static int zzz;
    virtual State currentState() const = 0;
    virtual bool computeDesiredPosition(bool) = 0;
    virtual NavigationalState * returnNextState() = 0;
    static NavigationalState * ap;
    static NavigationalState * ip;
    static NavigationalState * fp;
    static NavigationalState * da;
    static NavigationalState * po;
};

class AutoPilotState : public NavigationalState
{
public:
    State currentState() const override {return AP;}
    bool computeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class InitialApproach : public NavigationalState
{
public:
    State currentState() const override {return IP;}
    bool computeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class FinalApproach : public NavigationalState
{
public:
    State currentState() const override {return FP;}
    bool computeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class DataAcquisitionState : public NavigationalState
{
public:
    State currentState() const override {return DA;}
    bool computeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

class PullOutState : public NavigationalState
{
public:
    State currentState() const override {return PO;}
    bool computeDesiredPosition(bool);
    NavigationalState * returnNextState();
};

#endif /* NAVIGATIONAL_STATE_H */