#ifndef NAVIGATIONAL_STATE_H
#define NAVIGATIONAL_STATE_H

#include <iostream>
#include <stdio.h>
#include <ctime>
#include <chrono>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include "aruco/aruco_processor.h"

using namespace std;

class NavigationalState
{
public:
    enum State {BASE, AP, IA, FA, DA, PO};
    virtual State currentState() const = 0;
    virtual Position computeDesiredPosition(Position p) = 0;
    virtual NavigationalState * returnNextState(Position p) = 0;
    static NavigationalState * ap;
    static NavigationalState * ia;
    static NavigationalState * fa;
    static NavigationalState * da;
    static NavigationalState * po;
};

class AutoPilotState : public NavigationalState
{
public:
    State currentState() const override {return AP;}
    Position computeDesiredPosition(Position);
    NavigationalState * returnNextState(Position);
};

class InitialApproach : public NavigationalState
{
public:
    State currentState() const override {return IA;}
    Position computeDesiredPosition(Position);
    NavigationalState * returnNextState(Position);
};

class FinalApproach : public NavigationalState
{
public:
    State currentState() const override {return FA;}
    Position computeDesiredPosition(Position);
    NavigationalState * returnNextState(Position);
};

class DataAcquisitionState : public NavigationalState
{
public:
    State currentState() const override {return DA;}
    Position computeDesiredPosition(Position);
    NavigationalState * returnNextState(Position);
};

class PullOutState : public NavigationalState
{
public:
    State currentState() const override {return PO;}
    Position computeDesiredPosition(Position);
    NavigationalState * returnNextState(Position);
};

#endif /* NAVIGATIONAL_STATE_H */