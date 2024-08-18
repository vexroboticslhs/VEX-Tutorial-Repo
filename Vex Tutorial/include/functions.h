#pragma once
#include "main.h"

using namespace pros;

void stop_motors();
void drive_time(int time);
int sign(double num);

std::tuple<double, double, double> turn_lookup(double degrees);
std::tuple<double, double, double> drive_lookup(double degrees);

void turn(double angle);
void drive_dist(double dist);