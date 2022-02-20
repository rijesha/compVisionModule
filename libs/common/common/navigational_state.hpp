#pragma once

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>

using namespace std;

template <class E>
class NavigationalState {
 public:
  virtual E current_state() const = 0;
  virtual Vector3f compute_desired_position(Vector3f p) = 0;
  virtual NavigationalState* return_next_state(Vector3f p) = 0;
};
