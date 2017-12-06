#include "navigational_state.hpp"


bool NavigationalState::A(void){
    return false;
}
bool NavigationalState::B(void){
    return false;
}
bool NavigationalState::C(void){
    return false;
}
bool NavigationalState::D(void){
    return false;
}
bool NavigationalState::E(void){
    return false;
}
bool NavigationalState::F(void){
    return false;
}

NavigationalState * AutoPilotState::returnNextState(){
    return new AutoPilotState();
}

NavigationalState * InitialApproach::returnNextState(){
        return new InitialApproach();
}

NavigationalState * FinalApproach::returnNextState(){
        return new FinalApproach();
}

NavigationalState * DataAcquisitionState::returnNextState(){
        return new DataAcquisitionState();
}

NavigationalState * PullOutState::returnNextState(){
    return new FinalApproach();
}

bool AutoPilotState::ComputeDesiredPosition(bool){
    return false;
}

bool InitialApproach::ComputeDesiredPosition(bool){
return false;
}

bool FinalApproach::ComputeDesiredPosition(bool){
return false;
}

bool DataAcquisitionState::ComputeDesiredPosition(bool){
return false;
}

bool PullOutState::ComputeDesiredPosition(bool){
return false;
}
