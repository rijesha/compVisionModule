#ifndef TEST_STATE_H
#define TEST_STATE_H

#include <iostream>
#include <stdio.h>
#include <ctime>
#include <chrono>
#include <cstring>
#include <time.h>
#include <unistd.h>
#include <common/position.hpp>
#include <mavlink-interface/position_controller.h>
#include <common/navigational_state.hpp>

using namespace std;

enum State {BASE, AP, IA, FA, DA, PO};

extern Position_Controller * pc;
extern NavigationalState<State> * ap;
extern NavigationalState<State> * ia;
extern NavigationalState<State> * fa;
extern NavigationalState<State> * da;
extern NavigationalState<State> * po;

class AutoPilotState : public NavigationalState<State>
{
public:
    State currentState() const override {return AP;}

    NavigationalState * returnNextState(Position cp){
    if (cp.A() || !cp.B() || cp.F())
        return ap;

    if (!cp.A() && cp.B() && cp.C() && !cp.F()){
        pc->toggle_offboard_control(true);
        return ia;
    }

        return ap;
    }

    Position computeDesiredPosition(Position cp){
    /*If (current state is offboard)
        pc->toggle_offboard_control(false);
    */
    return Position(0,0,6,0);
    }
    
};

class InitialApproach : public NavigationalState<State>
{
public:
    State currentState() const override {return IA;}

    Position computeDesiredPosition(Position cp){
    /*If (current state is not offboard)
        pc->toggle_offboard_control(true);
    */
    //Yaw Controller
    double angle_in_frame = cp.angle_in_frame(); 
    if (angle_in_frame > 15 && cp.azi < 60){

    } else {

    }

    //Position Controller
    //double x = cp.x;
    //double y = cp.y;

    return Position();
    }

    NavigationalState * returnNextState(Position cp){
        if (!cp.A() && cp.B() && cp.C() && !cp.D())
            return ia;
        if (!cp.A() && cp.D())
            return fa;
        if (!cp.A() || !cp.B() || !cp.C())
            return ap;
        return ia;
    }
};

class FinalApproach : public NavigationalState<State>
{
public:
    State currentState() const override {return FA;}
    
    Position computeDesiredPosition(Position cp){
        return Position();
    }

    NavigationalState * returnNextState(Position cp){
    if (!cp.A() && cp.D() && !cp.E())
        return fa;
    if (!cp.A() && cp.D() && cp.E())
        return da;
    if (!cp.A() && !cp.D() && !cp.E())
        return ia;
    if (cp.A())
        return ap;
    return fa;
    }
};

class DataAcquisitionState : public NavigationalState<State>
{
public:
    State currentState() const override {return DA;}
    

    NavigationalState * returnNextState(Position cp){
        if (!cp.A() && !cp.F())
            return da;
        if (cp.A() || !cp.E() || cp.F())
            return po;
        return po;
    }

    Position computeDesiredPosition(Position cp){
        return Position(cp.x, cp.y, cp.z, cp.azi);
    }

};

class PullOutState : public NavigationalState<State>
{
public:
    State currentState() const override {return PO;}
    NavigationalState * returnNextState(Position cp){
        if (!cp.A() && cp.B())
            return po;
        if (cp.A() || !cp.B())
            return ap;
        return ap;
    }
    Position computeDesiredPosition(Position cp){
        return Position(0,0,6.5,0);
    }
};




#endif /* TEST_STATE_H */