#include "navigational_state.hpp"


bool NavigationalState::A(void){
    //Target Detection Failed for X seconds
    return false;
}
bool NavigationalState::B(void){
    //Target Within 6 m depth
    return false;
}
bool NavigationalState::C(void){
    //Target Azimuthal Angle <60 deg
    return false;
}
bool NavigationalState::D(void){
    //Within 2m depth and Target zimuthal angle < 10 deg
    return false;
}
bool NavigationalState::E(void){
    //Sensor Contact
    return false;
}
bool NavigationalState::F(void){
    //Data Acquisition Complete
    return false;
}

int NavigationalState::zzz = 21;// = 21;
NavigationalState * NavigationalState::ap = new AutoPilotState();
NavigationalState * NavigationalState::ip = new InitialApproach();
NavigationalState * NavigationalState::fp = new FinalApproach();
NavigationalState * NavigationalState::da = new DataAcquisitionState();
NavigationalState * NavigationalState::po = new PullOutState();

NavigationalState * AutoPilotState::returnNextState(){
    if (A() || !B() || F())
        return ap;

    if (!A() && B() && C() && !F())
        return ip;
}

NavigationalState * InitialApproach::returnNextState(){
    if (!A() && B() && C() && !D())
        return ip;
    if (!A() && D())
        return fp;
    if (!A() || !B() || !C())
        return ap;
}

NavigationalState * FinalApproach::returnNextState(){
    if (!A() && D() && !E())
        return fp;
    if (!A() && D() && E())
        return da;
    if (!A() && !D() && !E())
        return ip;
    if (A())
        return ap;
}

NavigationalState * DataAcquisitionState::returnNextState(){
    if (!A() && !F())
        return da;
    if (A() || !E() || F())
        return po;
}

NavigationalState * PullOutState::returnNextState(){
    if (!A() && B())
        return po;
    if (A() || !B())
        return ap;
}

bool AutoPilotState::computeDesiredPosition(bool){
    return false;
}

bool InitialApproach::computeDesiredPosition(bool){
return false;
}

bool FinalApproach::computeDesiredPosition(bool){
return false;
}

bool DataAcquisitionState::computeDesiredPosition(bool){
return false;
}

bool PullOutState::computeDesiredPosition(bool){
return false;
}
