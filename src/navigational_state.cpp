#include "navigational_state.hpp"

NavigationalState * NavigationalState::ap = new AutoPilotState();
NavigationalState * NavigationalState::ia = new InitialApproach();
NavigationalState * NavigationalState::fa = new FinalApproach();
NavigationalState * NavigationalState::da = new DataAcquisitionState();
NavigationalState * NavigationalState::po = new PullOutState();

NavigationalState * AutoPilotState::returnNextState(Position p){
    if (p.A() || !p.B() || p.F())
        return ap;

    if (!p.A() && p.B() && p.C() && !p.F())
        return ia;
}

NavigationalState * InitialApproach::returnNextState(Position p){
    if (!p.A() && p.B() && p.C() && !p.D())
        return ia;
    if (!p.A() && p.D())
        return fa;
    if (!p.A() || !p.B() || !p.C())
        return ap;
}

NavigationalState * FinalApproach::returnNextState(Position p){
    if (!p.A() && p.D() && !p.E())
        return fa;
    if (!p.A() && p.D() && p.E())
        return da;
    if (!p.A() && !p.D() && !p.E())
        return ia;
    if (p.A())
        return ap;
}

NavigationalState * DataAcquisitionState::returnNextState(Position p){
    if (!p.A() && !p.F())
        return da;
    if (p.A() || !p.E() || p.F())
        return po;
}

NavigationalState * PullOutState::returnNextState(Position p){
    if (!p.A() && p.B())
        return po;
    if (p.A() || !p.B())
        return ap;
}

Position AutoPilotState::computeDesiredPosition(Position p){
    return Position();
}

Position InitialApproach::computeDesiredPosition(Position p){
    double x = p.x;
    double y = p.y;

return Position();
}

Position FinalApproach::computeDesiredPosition(Position p){
return Position();
}

Position DataAcquisitionState::computeDesiredPosition(Position p){
    return Position(p.x, p.y, p.depth, p.azi);
}

Position PullOutState::computeDesiredPosition(Position p){
    return Position(0,0,6.5,0);
}
