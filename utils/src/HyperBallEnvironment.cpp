#include "environment.h"
#include <cmath>
#include <algorithm> // for std::min
#include <iostream>

HyperBallEnvironment::HyperBallEnvironment(pt lows, pt highs, pt start, pt goal, int N)
{
    lows_  = lows;
    highs_ = highs;
    start_ = start;
    goal_  = goal;
    N_     = N;
    delta_ = 0;

    inner_radius_ = 1.0;
    outer_radius_ = 2.0;
}

HyperBallEnvironment::~HyperBallEnvironment()
{
    // nothing special, e.g. no dynamic allocations to free
}

bool HyperBallEnvironment::isStateValid(pt point)
{
    // E.g. distance from origin or from 'start_'
    double sum = 0.0;
    for (int i=0; i < N_; i++)
    {
        // Or from 'start_[i]' if you want
        sum += point[i]*point[i]; 
    }
    double res = std::sqrt(sum);
    // if res is outside [inner_radius_, outer_radius_], we say "valid"
    if (res < inner_radius_ || res > outer_radius_)
        return true;
    return false;
}

double HyperBallEnvironment::penetrationDist(pt point, pt &close_point)
{
    // e.g. return how far from the ring [inner_radius_, outer_radius_]
    double sum = 0.0;
    for (int i=0; i < N_; i++)
        sum += point[i]*point[i];
    double res = std::sqrt(sum);
    // If res < inner_radius_, distance is (res - inner_radius_)
    // If res > outer_radius_, distance is (outer_radius_ - res)
    return std::min(res - inner_radius_, outer_radius_ - res);
}

