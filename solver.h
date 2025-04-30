#pragma once
#include "environment.h"

// Simplest signature – returns true if a path was found
bool solver(Environment* env,
            double duration            = 1.0,
            bool   use_training        = false,
            bool   use_GaussianSampler = false);
