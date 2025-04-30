#include "solver.h"
#include "SDCL.h"                 // planner
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <thread>
#include <chrono>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool solver(Environment* env,
            double duration,
            bool   use_training,
            bool   use_Gaussian)
{
    if (!env) {
        std::cerr << "solver(): null environment pointer\n";
        return false;
    }

    const int N = env->get_dim();                    // dimension
    auto space = std::make_shared<ob::RealVectorStateSpace>(N);

    // --- bounds ----------------------------------------------------------------
    ob::RealVectorBounds bounds(N);
    pt lows  = env->get_lows();
    pt highs = env->get_highs();
    double   delta = env->get_delta();

    for (int i = 0; i < N; ++i) {
        bounds.setLow (i, lows[i]  - 2 * delta);
        bounds.setHigh(i, highs[i] + 2 * delta);
    }
    space->setBounds(bounds);

    // --- space information & validity checker ----------------------------------
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(
        [env, N](const ob::State* s) {
            pt p(N);
            for (int i = 0; i < N; ++i)
                p[i] = s->as<ob::RealVectorStateSpace::StateType>()->values[i];
            return env->isStateValid(p);
        });
    si->setup();

    // --- start & goal -----------------------------------------------------------
    ob::ScopedState<> start(space), goal(space);
    pt p_start = env->get_start();
    pt p_goal  = env->get_goal();
    for (int i = 0; i < N; ++i) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = p_start[i];
        goal ->as<ob::RealVectorStateSpace::StateType>()->values[i] = p_goal [i];
    }

    // --- problem definition -----------------------------------------------------
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    // --- planner (your SDCL wrapper) -------------------------------------------
    auto planner = std::make_shared<SDCL>(si, /*starStrategy*/false,
                                              use_training,
                                              use_Gaussian);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // --- threads doing auxiliary work (example) --------------------------------
    std::thread threadA([env]{ std::cout << "[T-A] dim=" << env->get_dim() << '\n'; });
    std::thread threadB([env]{ std::cout << "[T-B] dim=" << env->get_dim() << '\n'; });

    // --- solve ------------------------------------------------------------------
    ob::PlannerStatus solved = planner->solve(duration);

    // join helper threads before we touch env again
    threadA.join();
    threadB.join();

    if (solved && pdef->hasExactSolution()) {
        std::cout << "Found solution:\n";
        pdef->getSolutionPath()->print(std::cout);
        return true;
    }
    std::cout << "No solution found\n";
    return false;
}
