#pragma once

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <common/navigational_state.hpp>
#include <common/position.hpp>
#include <cstring>
#include <ctime>
#include <iostream>

using namespace std;

enum State { BASE, AP, NA, DA, PO, XT, CT, PH };

extern clock_t lastDetectedTime;
extern clock_t startedDataAcquisition;

extern NavigationalState<State> *ap;
extern NavigationalState<State> *na;
extern NavigationalState<State> *da;
extern NavigationalState<State> *po;

extern NavigationalState<State> *xt;
extern NavigationalState<State> *ct;
extern NavigationalState<State> *ph;

bool detectionFailedFor(float seconds) {
  return ((clock() - lastDetectedTime) / CLOCKS_PER_SEC) > seconds;
};

bool acquiredDataFor(float seconds) {
  return ((clock() - startedDataAcquisition) / CLOCKS_PER_SEC) > seconds;
};

class AutoPilotState : public NavigationalState<State> {
 public:
  State current_state() const override { return AP; }

  NavigationalState *return_next_state(Position cp) {
    if (!cp.empty_position) {
      if ((cp.z < 6) && (abs(cp.azi) < 60) && (cp.x < 6)) {
        // return xt;
        return ph;
      }
    }
    return this;
  }

  Position compute_desired_position(Position cp) { return Position(0, 0, 2, 0); }
};

class NormalApproach : public NavigationalState<State> {
 public:
  State current_state() const override { return NA; }

  Position compute_desired_position(Position cp) {
    double angle_in_frame = cp.angle_in_frame();

    float aruco_z =
        ((abs(cp.w_x) * 2 + (cp.w_z - FINAL_Z_ARUCO)) * .3) + FINAL_Z_ARUCO;

    return Position(0, FINAL_Y_ARUCO, aruco_z, angle_in_frame);
  }

  NavigationalState *return_next_state(Position cp) {
    if (!cp.empty_position) {
      if (cp.w_z < (FINAL_Z_ARUCO + .05)) {
        startedDataAcquisition = clock();
        return da;
      }
    } else {
      if (detectionFailedFor(0.3)) {
        return ap;
      }
    }
    return this;
  }
};

class DataAcquisitionState : public NavigationalState<State> {
 public:
  State current_state() const override { return DA; }

  NavigationalState *return_next_state(Position cp) {
    if (acquiredDataFor(10)) {
      return po;
    } else if (cp.empty_position && detectionFailedFor(.3)) {
      return po;
    }
    return da;
  }

  Position compute_desired_position(Position cp) {
    return Position(0, FINAL_Y_ARUCO, FINAL_Z_ARUCO, 0);
  }
};

class PullOutState : public NavigationalState<State> {
 public:
  State current_state() const override { return PO; }

  NavigationalState *return_next_state(Position cp) {
    if (cp.empty_position && detectionFailedFor(0.5)) return ap;

    return this;
  }

  Position compute_desired_position(Position cp) { return Position(0, 0, 4, 0); }
};

class CrossTest : public NavigationalState<State> {
 public:
  State current_state() const override { return XT; }

  int state = 0;
  clock_t crossTime = clock();

  Position stateA = Position(0, 0, 4, 0);
  // Position stateC = Position(2, 0, 3, 0);
  // Position stateB = Position(0, 0, 2, 0);
  // Position stateD = Position(-2, 0, 3, 0);

  Position stateC = Position(0, 0, 4, 0);
  Position stateB = Position(0, 0, 2, 0);
  Position stateD = Position(0, 0, 2, 0);

  Position *currentPosition = &stateA;

  bool greaterThanCrossTime(float seconds) {
    return ((clock() - crossTime) / CLOCKS_PER_SEC) > seconds;
  };

  NavigationalState *return_next_state(Position cp) {
    if (cp.empty_position && detectionFailedFor(0.3)) {
      return ap;
    }

    return this;
  }

  Position compute_desired_position(Position cp) {
    if (greaterThanCrossTime(10)) {
      crossTime = clock();

      switch (state) {
        case 0:
          currentPosition = &stateB;
          state = 1;
          break;
        case 1:
          currentPosition = &stateC;
          state = 2;
          break;
        case 2:
          currentPosition = &stateD;
          state = 3;
          break;
        case 3:
          currentPosition = &stateA;
          state = 0;
          break;

        default:
          break;
      }
    }
    return *currentPosition;
  }
};

class CircleTest : public NavigationalState<State> {
 public:
  State current_state() const override { return CT; }

  NavigationalState *return_next_state(Position cp) { return this; }

  Position compute_desired_position(Position cp) {
    return Position(0, 0, 6.5, 0);
  }
};

class PositionHold : public NavigationalState<State> {
 public:
  State current_state() const override { return PH; }

  NavigationalState *return_next_state(Position cp) { return this; }

  Position compute_desired_position(Position cp) {
    return Position(0, 0, 2.0, 0);
  }
};
