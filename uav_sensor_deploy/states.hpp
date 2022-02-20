#pragma once

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <common/navigational_state.hpp>
#include <aruco_processor/aruco_position.h>
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

  NavigationalState *return_next_state(Vector3f cp) {
    // if ((cp.z < 6) && (abs(cp.azi) < 60) && (cp.x < 6)) {
    //  // return xt;
    //  return ph;
    //}

    return this;
  }

  Vector3f compute_desired_position(Vector3f cp) { return Vector3f(0, 0, 2); }
};

class NormalApproach : public NavigationalState<State> {
 public:
  State current_state() const override { return NA; }

  Vector3f compute_desired_position(Vector3f cp) {
    // double angle_in_frame = cp.angle_in_frame();

    // float aruco_z =
    //    ((abs(cp.w_x) * 2 + (cp.w_z - FINAL_Z_ARUCO)) * .3) + FINAL_Z_ARUCO;

    float aruco_z = 0;
    return Vector3f(0, FINAL_Y_ARUCO, aruco_z);
  }

  NavigationalState *return_next_state(Vector3f cp) {
    // if (cp.w_z < (FINAL_Z_ARUCO + .05)) {
    //  startedDataAcquisition = clock();
    //  return da;
    //}

    if (detectionFailedFor(0.3)) {
      return ap;
    }

    return this;
  }
};

class DataAcquisitionState : public NavigationalState<State> {
 public:
  State current_state() const override { return DA; }

  NavigationalState *return_next_state(Vector3f cp) {
    if (acquiredDataFor(10)) {
      return po;
    } else if (detectionFailedFor(.3)) {
      return po;
    }
    return da;
  }

  Vector3f compute_desired_position(Vector3f cp) {
    return Vector3f(0, FINAL_Y_ARUCO, FINAL_Z_ARUCO);
  }
};

class PullOutState : public NavigationalState<State> {
 public:
  State current_state() const override { return PO; }

  NavigationalState *return_next_state(Vector3f cp) {
    if (detectionFailedFor(0.5)) return ap;

    return this;
  }

  Vector3f compute_desired_position(Vector3f cp) { return Vector3f(0, 0, 4); }
};

class CrossTest : public NavigationalState<State> {
 public:
  State current_state() const override { return XT; }

  int state = 0;
  clock_t crossTime = clock();

  Vector3f stateA = Vector3f(0, 0, 4);
  // Vector3f stateC = Vector3f(2, 0, 3, 0);
  // Vector3f stateB = Vector3f(0, 0, 2, 0);
  // Vector3f stateD = Vector3f(-2, 0, 3, 0);

  Vector3f stateC = Vector3f(0, 0, 4);
  Vector3f stateB = Vector3f(0, 0, 2);
  Vector3f stateD = Vector3f(0, 0, 2);

  Vector3f *currentVector3f = &stateA;

  bool greaterThanCrossTime(float seconds) {
    return ((clock() - crossTime) / CLOCKS_PER_SEC) > seconds;
  };

  NavigationalState *return_next_state(Vector3f cp) {
    if (detectionFailedFor(0.3)) {
      return ap;
    }

    return this;
  }

  Vector3f compute_desired_position(Vector3f cp) {
    if (greaterThanCrossTime(10)) {
      crossTime = clock();

      switch (state) {
        case 0:
          // currentPosition = &stateB;
          state = 1;
          break;
        case 1:
          // currentPosition = &stateC;
          state = 2;
          break;
        case 2:
          // currentPosition = &stateD;
          state = 3;
          break;
        case 3:
          // currentPosition = &stateA;
          state = 0;
          break;

        default:
          break;
      }
    }
    return {0, 0, 0};
  }
};

class CircleTest : public NavigationalState<State> {
 public:
  State current_state() const override { return CT; }

  NavigationalState *return_next_state(Vector3f cp) { return this; }

  Vector3f compute_desired_position(Vector3f cp) { return Vector3f(0, 0, 6.5); }
};

class PositionHold : public NavigationalState<State> {
 public:
  State current_state() const override { return PH; }

  NavigationalState *return_next_state(Vector3f cp) { return this; }

  Vector3f compute_desired_Vector3f(Vector3f cp) { return Vector3f(0, 0, 2.0); }
};
