#include <thread>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include "environment.h"
#include "scenes.h"
#include "parser.h"
#include "SDCL.h"

#include "solver.h"
#include <chrono>
#include <cstdlib>
#include <iostream>


namespace ob = ompl::base;
namespace og = ompl::geometric;



bool solver(Environment* env, double duration, bool use_training, bool use_Gaussian) {

    int N = env->get_dim();

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(N));

    // set the bounds
    ob::RealVectorBounds bounds(N);
    pt lows = env->get_lows();
    pt highs = env->get_highs();
    double delta = env->get_delta();
    for (int i = 0; i < N; i++) {
        bounds.setLow(i, lows[i] - delta * 2);
        bounds.setHigh(i, highs[i] + delta * 2);
    }
    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker([=](const ob::State *state) {
                                                                pt point(N);
                                                                for (int i = 0; i < N; i++) {
                                                                    point[i] = (float)state->as<ob::RealVectorStateSpace::StateType>()->values[i];
                                                                }
                                                                if (!env->isStateValid(point)) {
                                                                    return false;
                                                                } else {
                                                                    return true;
                                                                }});
    si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    pt p1 = env->get_start();
    pt p2 = env->get_goal();
    for (int i = 0; i < N; i++) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = p1[i];
        goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = p2[i];
    }

    // create a problem instance
    auto pdef_ = std::make_shared<ob::ProblemDefinition>(si);

    // set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner_ = std::make_shared<SDCL>(si, false, use_training, use_Gaussian);

    // set the problem we are trying to solve for the planner
    planner_->setProblemDefinition(pdef_);

    // perform setup steps for the planner
    planner_->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef_->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner_->ob::Planner::solve(duration);

    if (solved && pdef_->hasExactSolution())
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef_->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
        return true;
    } else {
        std::cout << "No solution found" << std::endl;
        return false;
    }
}



int main(int argc, char** argv)
{
    auto t0 = std::chrono::steady_clock::now();
    std::cout << "SDCL planner start!\n";

    //1  Parse CLI
    int    scene        = 0;
    double duration     = 1.0;
    bool   use_training = false;
    bool   use_Gaussian = false;
    parse(argc, argv, &scene, &duration, &use_training, &use_Gaussian);

    std::cout << "Parsed  scene=" << scene
              << "  duration=" << duration
              << "  use_training=" << use_training
              << "  use_Gaussian=" << use_Gaussian << '\n';

    //  2  Build environment
    Environment* env = get_scene_env(scene);
    if (!env) {
        std::cerr << "Failed to create environment for scene " << scene << '\n';
        return 1;
    }

    //3  Call the solver
    //bool found = solver(env, duration, use_training, use_Gaussian);

    //  4  Clean-up
    //delete env;            // safe: solver has joined its threads

    // 5  Plotting (requires the same SpaceInformation)
    // Build a tiny dummy space just so SDCL's plot helpers compile
    //auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    //ompl::base::SpaceInformationPtr si = std::make_shared<ompl::base::SpaceInformation>(space);
    //SDCL sdcl_dummy(si, false, false, false);
    //sdcl_dummy.plotRoadmapScatter("roadmapScatter.svg");
    //sdcl_dummy.plotGMMClusters("clusters.svg");

    //auto dt = std::chrono::steady_clock::now() - t0;
    //std::cout << "Total run time: "
    //          << std::chrono::duration<double>(dt).count() << " s\n";
    //return found ? 0 : 2;
        // 3) Create the state space
        int N = env->get_dim();  // e.g. 2 for 2D
        auto space = std::make_shared<ob::RealVectorStateSpace>(N);
    
        // 4) Set up the bounds
        ob::RealVectorBounds bounds(N);
        pt lows  = env->get_lows();
        pt highs = env->get_highs();
        double delta = env->get_delta();
        for (int i = 0; i < N; i++)
        {
            bounds.setLow(i,  lows[i]  - delta*2);
            bounds.setHigh(i, highs[i] + delta*2);
        }
        space->setBounds(bounds);
    
        // 5) Create SpaceInformation
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker([=](const ob::State *state) {
            pt point(N);
            for (int i = 0; i < N; i++)
                point[i] = (float)state->as<ob::RealVectorStateSpace::StateType>()->values[i];
            return env->isStateValid(point);
        });
        si->setup();
    
        // 6) Create start & goal
        pt p1 = env->get_start();
        pt p2 = env->get_goal();
        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);
        for (int i = 0; i < N; i++)
        {
            start->as<ob::RealVectorStateSpace::StateType>()->values[i] = p1[i];
            goal->as<ob::RealVectorStateSpace::StateType>()->values[i]  = p2[i];
        }
    
        // 7) Create a ProblemDefinition
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal);
    
        // 8) Create an SDCL planner (which presumably builds/holds a roadmap)
        auto planner = std::make_shared<SDCL>(si, /* verbose */ false,
                                              use_training,
                                              use_Gaussian);
        planner->setProblemDefinition(pdef);
        planner->setup();
    
        // Print out the settings
        si->printSettings(std::cout);
        pdef->print(std::cout);
    
        // 9) Attempt to solve
        //ob::PlannerStatus solved = planner->solve(duration);
        ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(duration));

        bool foundSolution = false;
        if (solved && pdef->hasExactSolution())
        {
            std::cout << "Found solution!\n";
            ob::PathPtr path = pdef->getSolutionPath();
            path->print(std::cout);
            foundSolution = true;
        }
        else
        {
            std::cout << "No solution found.\n";
        }
        

        // 1) Gather all states and short/long components from the roadmap
        auto info = planner->reportShortAndLongComponents(30); 
    // This returns a ShortLongComponentsAndStates struct with:
    //   info.shortCompIDs, info.longCompIDs, info.allRoadmapStates

    // the entire roadmap data for GMM or analysis
        auto data = info.allRoadmapStates; // same type: vector<vector<double>>


    // 3) Plot the roadmap “before”
        planner->plotRoadmapScatter("roadmapBefore.png");

    // 4) Build single‐cluster diagonal GMM per connected component
        planner->buildPerComponentGMMsDiag(1, 1);

    // 5) Compute pairwise Bhattacharyya Distances
        planner->computePairwiseBD();

    // 6) Sample from short and long components
        planner->sampleFromProductOfShortComps(100);
        planner->sampleFromProductOfLongComps(100);

    // 7) Plot the roadmap “after”
        planner->plotRoadmapScatter("roadmapAfter.png");

    // 8) Plot each per‐component single‐cluster GMM in 2D (color short vs. long)
        planner->plotPerComponentGMMClusters("clustersPerComp.png");

    // 9) Clean up
        delete env;  
        auto dt = std::chrono::steady_clock::now() - t0;
        std::cout << "Total run time: "
                  << std::chrono::duration<double>(dt).count() << " s\n";

        return foundSolution ? 0 : 2;

}
