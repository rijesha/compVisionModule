#pragma once

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iostream>
#include "position.hpp"

using namespace std;

template <class E>
class NavigationalState {
 public:
  virtual E current_state() const = 0;
  virtual Position compute_desired_position(Position p) = 0;
  virtual NavigationalState* return_next_state(Position p) = 0;
};
