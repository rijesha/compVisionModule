#include "navigational_state.hpp"

NavigationalState * NavigationalState::ap = new AutoPilotState();
NavigationalState * NavigationalState::ia = new InitialApproach();
NavigationalState * NavigationalState::fa = new FinalApproach();
NavigationalState * NavigationalState::da = new DataAcquisitionState();
NavigationalState * NavigationalState::po = new PullOutState();
Position_Controller * NavigationalState::pc = NULL;

NavigationalState * AutoPilotState::returnNextState(Position cp){
    if (cp.A() || !cp.B() || cp.F())
        return ap;

    if (!cp.A() && cp.B() && cp.C() && !cp.F()){
        pc->toggle_offboard_control(true);
        return ia;
    }

    return ap;
}

NavigationalState * InitialApproach::returnNextState(Position cp){
    if (!cp.A() && cp.B() && cp.C() && !cp.D())
        return ia;
    if (!cp.A() && cp.D())
        return fa;
    if (!cp.A() || !cp.B() || !cp.C())
        return ap;
    return ia;
}

NavigationalState * FinalApproach::returnNextState(Position cp){
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

NavigationalState * DataAcquisitionState::returnNextState(Position cp){
    if (!cp.A() && !cp.F())
        return da;
    if (cp.A() || !cp.E() || cp.F())
        return po;
    return po;
}

NavigationalState * PullOutState::returnNextState(Position cp){
    if (!cp.A() && cp.B())
        return po;
    if (cp.A() || !cp.B())
        return ap;
    return ap;
}

Position AutoPilotState::computeDesiredPosition(Position cp){
    /*If (current state is offboard)
        pc->toggle_offboard_control(false);
    */
    return Position(0,0,6,0);
}

Position InitialApproach::computeDesiredPosition(Position cp){
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

Position FinalApproach::computeDesiredPosition(Position cp){
return Position();
}

Position DataAcquisitionState::computeDesiredPosition(Position cp){
    return Position(cp.x, cp.y, cp.z, cp.azi);
}

Position PullOutState::computeDesiredPosition(Position cp){
    return Position(0,0,6.5,0);
}
