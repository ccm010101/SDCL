#include "SDCL.h"
#include <boost/bind.hpp>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#define foreach BOOST_FOREACH



// ----objfunc, objfunc2, myconstraint, findClosetPoint removed ---


namespace ompl
{
    namespace magic
    {
        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }  // namespace magic
}  // namespace ompl


SDCL::SDCL(const base::SpaceInformationPtr &si, bool starStrategy, bool use_training, bool use_Gaussian)
  : base::Planner(si, "SDCL")
  , starStrategy_(starStrategy)
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
  , stddev_(si->getMaximumExtent() * magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
  , use_training_(use_training)
  , use_Gaussian_(use_Gaussian)
  , lower_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low)
  , upper_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &SDCL::setMaxNearestNeighbors,
                                            &SDCL::getMaxNearestNeighbors, std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationCount(); });
    addPlannerProgressProperty("best cost REAL", [this] { return getBestCost(); });
    addPlannerProgressProperty("milestone count INTEGER", [this] { return getMilestoneCountString(); });
    addPlannerProgressProperty("edge count INTEGER", [this] { return getEdgeCountString(); });

}

SDCL::~SDCL()
{
    freeMemory();
}

void SDCL::setup()
{
    Planner::setup();
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!connectionStrategy_)
        setDefaultConnectionStrategy();
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &) { return true; };

    // Setup optimization objective
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void SDCL::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = ompl::geometric::KStrategy<Vertex>(k, nn_);
    if (isSetup())
        setup();
}

unsigned int SDCL::getMaxNearestNeighbors() const
{
    const auto strategy = connectionStrategy_.target<ompl::geometric::KStrategy<Vertex>>();
    return strategy ? strategy->getNumNeighbors() : 0u;
}

void SDCL::setDefaultConnectionStrategy()
{
    if (starStrategy_)
        connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>([this] { return milestoneCount(); }, nn_, si_->getStateDimension());
    else
        connectionStrategy_ = ompl::geometric::KStrategy<Vertex>(ompl::magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
}

void SDCL::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void SDCL::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void SDCL::clear()
{
    OMPL_INFORM("Calling clear()...");
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

    // // SVM model data cleanup
    //if (!savedModelData_.vectors) 
    //{
    //    delete [] savedModelData_.vectors;
    //    savedModelData_.vectors = NULL;
    //}
    //if (!savedModelData_.coef) 
    //{
    //    delete [] savedModelData_.coef;
    //    savedModelData_.coef = nullptr;
    //}

    collisionPoints_.reset(new pvec());
    stats_ = proof_stats();
    addedToTraining_ = 0;
}

void SDCL::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();

    // // SVM model data cleanup
    //if (!savedModelData_.vectors) delete [] savedModelData_.vectors;
    //if (!savedModelData_.coef) delete [] savedModelData_.coef;

    // saved2dPoints_.close();
}

void SDCL::expandRoadmap(double expandTime)
{
    expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
}

void SDCL::expandRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> states(ompl::magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(ptc, states);
    si_->freeStates(states);
}

void SDCL::expandRoadmap(const base::PlannerTerminationCondition &ptc,
                                         std::vector<base::State *> &workStates)
{
    // construct a probability distribution over the vertices in the roadmap
    // as indicated in
    //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
    //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars
    
    PDF<Vertex> pdf;

    graphMutex_.lock();
    foreach (Vertex v, boost::vertices(g_))
    {
        const unsigned long int t = totalConnectionAttemptsProperty_[v];
        pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
    }
    graphMutex_.unlock();

    if (pdf.empty())
        return;

    while (!ptc)
    {
        iterations_++;
        Vertex v = pdf.sample(rng_.uniform01());
        unsigned int s =
            si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
        if (s > 0)
        {
            s--;
            Vertex last = addMilestone(si_->cloneState(workStates[s]));

            graphMutex_.lock();
            for (unsigned int i = 0; i < s; ++i)
            {
                // add the vertex along the bouncing motion
                Vertex m = boost::add_vertex(g_);
                stateProperty_[m] = si_->cloneState(workStates[i]);
                totalConnectionAttemptsProperty_[m] = 1;
                successfulConnectionAttemptsProperty_[m] = 0;
                disjointSets_.make_set(m);

		// add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, m, properties, g_);
                uniteComponents(v, m);

                // add the vertex to the nearest neighbors data structure
                nn_->add(m);
                v = m;
            }

            // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
            // we add an edge
            if (s > 0 || !sameComponent(v, last))
            {
                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, last, properties, g_);
                uniteComponents(v, last);
            }
            graphMutex_.unlock();
        }
    }
}

void SDCL::growRoadmap(double growTime)
{
    growRoadmap(base::timedPlannerTerminationCondition(growTime));
}

void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    base::State *workState = si_->allocState();
    growRoadmap(ptc, workState);
    si_->freeState(workState);
}

//void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState)
//{
//    while (!ptc)
//    {
//        iterations_++;
        // search for a valid state
//        bool found = false;
        
//        while (!found && !ptc)
//        {
//            unsigned int attempts = 0;
//            do
//           {
//                found = sampleAndSaveCollisionPoints(workState, use_Gaussian_);
//                attempts++;
//            } while (attempts < ompl::magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
//        }
//       // add it as a milestone
//       if (found)
//            addMilestone(si_->cloneState(workState));
//    }
//}
void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState)
{
    // Keep generating valid states until time is up (ptc).
    // Then add each valid state to the roadmap.
    while (!ptc)
    {
        iterations_++;

        // 1) Ask the valid-state sampler for a guaranteed collision-free state.
        //    Because sampler_ is a ValidStateSampler, it returns 'true' only when
        //    it successfully finds a valid state inside some internal #attempts.
        bool gotValid = sampler_->sample(workState);

        // 2) If we found a valid state (which we should, unless the sampler gave up),
        //    then add it to the roadmap as a milestone:
        if (gotValid)
            addMilestone(si_->cloneState(workState));
        else
        {
            // If for some reason sampler_ fails, we might just break or continue.
            // For example:
            OMPL_WARN("ValidStateSampler failed to find a valid sample. Stopping growth.");
            break;
        }
    }
}



void SDCL::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
    auto *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    while (!ptc && !addedNewSolution_)
    {
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st != nullptr)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
        if (!addedNewSolution_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool SDCL::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                  base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    base::Cost sol_cost(opt_->infiniteCost());
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                base::PathPtr p = constructSolution(start, goal);
                if (p)
                {
                    base::Cost pathCost = p->cost(opt_);
                    if (opt_->isCostBetterThan(pathCost, bestCost_))
                        bestCost_ = pathCost;

                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        return true;
                    }
                    if (opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                    }
                }
            }
        }
    }

    return false;
}

bool SDCL::addedNewSolution() const
{
    return addedNewSolution_;
}

ompl::base::PlannerStatus SDCL::solve(const base::PlannerTerminationCondition &ptc)
{
    auto start_time = std::chrono::steady_clock::now();
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    // Reset addedNewSolution_ member and create solution checking thread
    addedNewSolution_ = false;
    base::PathPtr sol;
    //std::thread slnThread([this, &ptc, &sol] { checkForSolution(ptc, sol); });
    
    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || addedNewSolution(); });

    // create SDCL thread to generate "useful" samples
    //std::thread infThread([this, &ptcOrSolutionFound]
    //{
        // generateSamples(ptcOrSolutionFound);
    //});

    constructRoadmap(ptcOrSolutionFound);

    // Ensure slnThread and SDCLThread is ceased before exiting solve
    //slnThread.join();
    //infThread.join();
    checkForSolution(ptc, sol);
    constructRoadmap(ptcOrSolutionFound);
    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    // // SVM stats (commented out)
    OMPL_INFORM("Total training time is %f, total sampling time is %f.", stats_.training_time, stats_.sampling_time);

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);

        stats_.solved = 1;
    }
    else
    {
        // Return an approximate solution.
        ompl::base::Cost diff = constructApproximateSolution(startM_, goalM_, sol);
        if (!opt_->isFinite(diff))
        {
            OMPL_INFORM("Closest path is still start and goal");
            return base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("Using approximate solution, heuristic cost-to-go is %f", diff.value());
        pdef_->addSolutionPath(sol, true, diff.value(), getName());

        std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
        stats_.total_time += tot.count();

        //printStat();
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
    stats_.total_time += tot.count();

    //printStat();

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void SDCL::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> xstates(ompl::magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    bestCost_ = opt_->infiniteCost();
    while (!ptc())
    {
        // maintain a 2:1 ratio for growing/expansion of roadmap
        // call growRoadmap() twice as long for every call of expandRoadmap()
        if (grow)
            growRoadmap(base::plannerOrTerminationCondition(
                            ptc, base::timedPlannerTerminationCondition(2.0 * ompl::magic::ROADMAP_BUILD_TIME)),
                        xstates[0]);
        else
            expandRoadmap(base::plannerOrTerminationCondition(
                              ptc, base::timedPlannerTerminationCondition(ompl::magic::ROADMAP_BUILD_TIME)),
                          xstates);
        grow = !grow;
    }

    si_->freeStates(xstates);
}


SDCL::Vertex SDCL::addMilestone(base::State *state)
{
    //std::lock_guard<std::mutex> _(graphMutex_); deadlock with expandRoadmap()

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(n, m))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(n, m, properties, g_);
                uniteComponents(n, m);
            }
        }

    nn_->add(m);
    return m;
}

void SDCL::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool SDCL::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

ompl::base::Cost SDCL::constructApproximateSolution(const std::vector<Vertex> &starts,
                                                    const std::vector<Vertex> &goals,
                                                    base::PathPtr &solution)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    base::Goal *g = pdef_->getGoal().get();
    base::Cost closestVal(opt_->infiniteCost());
    bool approxPathJustStart = true;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            base::Cost heuristicCost(costHeuristic(start, goal));
            if (opt_->isCostBetterThan(heuristicCost, closestVal))
            {
                closestVal = heuristicCost;
                approxPathJustStart = true;
            }
            if (!g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                continue;
            }
            base::PathPtr p;
            boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> dist(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> rank(boost::num_vertices(g_));

            try
            {
                boost::astar_search(
                    g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
                    boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare(
                            [this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(goal)));
            }
            catch (AStarFoundGoal &)
            {
            }

            Vertex closeToGoal = start;
            for (auto vp = vertices(g_); vp.first != vp.second; vp.first++)
            {
                ompl::base::Cost dist_to_goal(costHeuristic(*vp.first, goal));
                if (opt_->isFinite(rank[*vp.first]) && opt_->isCostBetterThan(dist_to_goal, closestVal))
                {
                    closeToGoal = *vp.first;
                    closestVal = dist_to_goal;
                    approxPathJustStart = false;
                }
            }
            if (closeToGoal != start)
            {
                auto p(std::make_shared<ompl::geometric::PathGeometric>(si_));
                for (Vertex pos = closeToGoal; prev[pos] != pos; pos = prev[pos])
                    p->append(stateProperty_[pos]);
                p->append(stateProperty_[start]);
                p->reverse();

                solution = p;
            }
        }
    }
    if (approxPathJustStart)
    {
        return opt_->infiniteCost();
    }
    return closestVal;
}

ompl::base::PathPtr SDCL::constructSolution(const Vertex &start, const Vertex &goal)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
            g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev)
                .distance_compare([this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                .distance_inf(opt_->infiniteCost())
                .distance_zero(opt_->identityCost())
                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    auto p(std::make_shared<ompl::geometric::PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
        p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    return p;
}

void SDCL::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    
    // Explicitly add start and goal states:
    for (unsigned long i : startM_)
        data.addStartVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<SDCL *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : goalM_)
        data.addGoalVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<SDCL *>(this)->disjointSets_.find_set(i)));
   
    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));
        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]), base::PlannerDataVertex(stateProperty_[v1]));
        
        // Add tags for the newly added vertices
        data.tagState(stateProperty_[v1], const_cast<SDCL *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2], const_cast<SDCL *>(this)->disjointSets_.find_set(v2));
    }
}

ompl::base::Cost SDCL::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}


// -----------SDCL part removed---------------------------------------
// ----------GMM Part------------------------
#include <iostream>
#include <armadillo>
#include "SDCL.h"



void SDCL::buildPerComponentGMMsDiag(int minK, int maxK)
{
    // Clear any previously stored GMMs
    compGMMsDiag_.clear();

    // 1) Group vertices by connected component
    std::unordered_map<unsigned long, std::vector<Vertex>> compToVertices;
    {
        // (optional) lock the roadmap if using threads
        // std::lock_guard<std::mutex> lock(graphMutex_);

        for (auto v : boost::make_iterator_range(boost::vertices(g_)))
        {
            // which connected component is this vertex in?
            Vertex rep    = disjointSets_.find_set(v);
            unsigned long compID = static_cast<unsigned long>(rep);
            compToVertices[compID].push_back(v);
        }
    }

    // 2) For each connected component, gather states into an arma::mat, do BIC loop
    for (auto &kv : compToVertices)
    {
        unsigned long compID    = kv.first;
        const auto   &vertexVec = kv.second;
        size_t N = vertexVec.size();

        // If trivial size, skip or store a 1-cluster if you want
        if (N < 2)
        {
            OMPL_INFORM("Component %lu has only %zu states => skipping GMM or storing trivial model.",
                        compID, N);
            continue;
        }

        // dimension = state dimension
        unsigned int dim = si_->getStateDimension();
        arma::mat dataset(dim, N);

        // Fill dataset columns from the roadmap states
        for (size_t i = 0; i < N; ++i)
        {   
            //auto vv = vertices[i];
            Vertex vv = vertexVec[i];
            const base::State* st = stateProperty_[vv];
            auto *rv = st->as<base::RealVectorStateSpace::StateType>();
            for (unsigned int d = 0; d < dim; d++)
                dataset(d, i) = rv->values[d];

        }

        // BIC loop
        arma::gmm_diag bestModel;
        double bestBIC = std::numeric_limits<double>::infinity();
        int bestK      = 0;

        for (int k = minK; k <= maxK; k++)
        {
            arma::gmm_diag candidate;
            // Built-in K-means + EM for diagonal Gaussians:
            bool ok = candidate.learn(
                dataset,            // data
                k,                  // # components
                arma::eucl_dist,           // dist_mode
                //arma::gaussian_init,// init method
                arma::static_spread,  // or arma::random_subset
                10,                 // max EM iters
                10,                 // max k-means iters
                1e-10,              // EM tolerance
                false               // no verbose
            );

            if (!ok) 
            {
                // E-M can fail for degenerate or tiny data
                continue;
            }

            // compute log-likelihood
            arma::rowvec lp = candidate.log_p(dataset);
            double ll = arma::accu(lp);

            // for diagonal-cov GMM with k comps, dimension=dim:
            //   paramCount = (k - 1) + k*dim + k*dim = 2*k*dim + (k - 1)
            double paramCount = 2.0 * k * dim + (k - 1);
            double bic = -2.0 * ll + paramCount * std::log((double)N);

            if (bic < bestBIC)
            {
                bestBIC  = bic;
                bestModel= candidate;
                bestK    = k;
            }
        }

        compGMMsDiag_[compID] = bestModel; // store final
        unsigned int finalComps = bestModel.means.n_cols;
        OMPL_INFORM("Component %lu => best BIC=%.2f with %d mixture comps",
                    compID, bestBIC,  bestK, finalComps); //bestModel.n_gaus);
    }

    OMPL_INFORM("Done building per component diagonal GMMs => compGMMsDiag_");
}


arma::gmm_diag SDCL::buildSingleCovDiagGMM(const arma::mat& dataset, int k)
{
    using namespace arma;

    size_t d = dataset.n_rows;
    size_t N = dataset.n_cols;

    // 1) K-means to get cluster centroids
    mat centroids(d, k);
    bool kmSuccess = kmeans(centroids, dataset, k, static_spread, 10, false);
    if(!kmSuccess)
    {
        OMPL_WARN("K-means failed for k=%d", k);
        return gmm_diag(); // empty
    }

    // 2) Assign each point, gather membership
    rowvec assignments(N, fill::zeros);
    vec clusterCounts(k, fill::zeros);

    for(size_t i=0; i < N; i++)
    {
        double bestDist = DBL_MAX;
        uword bestC = 0;
        for(int c=0; c < k; c++)
        {
            double dsq = accu(square(dataset.col(i) - centroids.col(c)));
            if(dsq < bestDist)
            {
                bestDist = dsq;
                bestC = c;
            }
        }
        assignments[i] = bestC;
        clusterCounts[bestC]++;
    }

    // 3) Refine means
    centroids.zeros();
    for(size_t i=0; i < N; i++)
    {
        uword c = (uword) assignments[i];
        centroids.col(c) += dataset.col(i);
    }
    for(int c=0; c < k; c++)
    {
        if(clusterCounts[c] > 0)
            centroids.col(c) /= clusterCounts[c];
    }

    // 4) Weights
    vec weights = clusterCounts / double(N);

    // 5) single diag => compute overall variance from entire dataset
    vec globalMean = mean(dataset, 1);
    vec globalVar(d, fill::zeros);

    for(size_t i=0; i < N; i++)
    {
        vec diff = dataset.col(i) - globalMean;
        globalVar += diff % diff;
    }
    globalVar /= double(N);
    // clamp or fix tiny variance
    globalVar.for_each( [](double &val) { if(val<1e-12) val=1e-12; } );

    // Construct the gmm_diag
    //gmm_diag model;
    //model.means.set_size(d, k);
    //model.dcovs.set_size(d, k);
    //model.hefts.set_size(k);
    // Prepare local structures for means, dcovs, hefts
    //  Then use model.set_params(...) to avoid direct writes to const fields

    arma::mat outMeans(d, k, fill::zeros);
    arma::mat outDcovs(d, k, fill::zeros);
    arma::rowvec outHefts(k, fill::zeros);

    for(int c=0; c < k; c++)
    {
        //model.means.col(c) = centroids.col(c);
        //model.hefts[c]     = weights[c];
        //model.dcovs.col(c) = globalVar;  // same diagonal for all
        outMeans.col(c) = centroids.col(c); // each column is the mean
        outHefts(c)     = weights[c];       // store mixture weight
        outDcovs.col(c) = globalVar;        // single diagonal for all
    
    }

    // ensure sum of weights = 1
    double sumW = accu(outHefts);
    //if(sumW > 0) model.hefts /= sumW;


    if (sumW > 0)
    outHefts = outHefts * (1.0 / sumW);

    // 7) Build the gmm_diag via set_params
    arma::gmm_diag model;
    // bool set_params(...)
    //   set_params( means, dcovs, heaps, normalise_hefts, check_empty )
    model.set_params(outMeans, outDcovs, outHefts);
                                    
    //if(!success)
    //{
    //    OMPL_WARN("set_params() failed for single-cov GMM k=%d", k);
    //    return gmm_diag(); // empty
    //}

    return model;  // 'model' now has your single diag
}



void SDCL::trainGMMArmadilloBIC(const std::vector<std::vector<double>> &data,
                                int minK, int maxK)
{
    if (data.empty() || minK <= 0 || maxK < minK)
    {
        OMPL_INFORM("No data or invalid range for single diag GMM BIC training.");
        return;
    }
    size_t dim = data[0].size();
    size_t N   = data.size();
    arma::mat dataset(dim, N);
    for (size_t i = 0; i < N; ++i)
    {
        for (size_t d = 0; d < dim; d++)
            dataset(d, i) = data[i][d];
    }

    double bestBIC = std::numeric_limits<double>::infinity();
    arma::gmm_diag bestModel;

    for(int k = minK; k <= maxK; k++)
    {
        arma::gmm_diag model = buildSingleCovDiagGMM(dataset, k);
        if(model.means.n_cols == 0)
        {
            // means it failed or is empty
            continue;
        }

        // compute log-likelihood
        // Use log_p(...) for the entire dataset
        arma::rowvec logpVec = model.log_p(dataset); // rowvec of length N
        double ll = arma::accu(logpVec);
        // single diag => p = (k - 1) + k*dim + dim
        double p = (k - 1) + double(k*dim) + double(dim);

        double bic = -2.0 * ll + p * std::log((double)N);

        OMPL_INFORM("k=%d => logLik=%.2f, BIC=%.2f", k, ll, bic);

        if(bic < bestBIC)
        {
            bestBIC  = bic;
            bestModel= model;
        }

    }

    OMPL_INFORM("Single diag => best BIC=%.2f", bestBIC);

    // store it for later
    armadilloDiagGMM_ = bestModel;
}



// Build a single full-covariance Gaussian for each connected component
//void SDCL::buildPerComponentGMMs()
//{
    // 1) Clear out the map in case we've built GMMs before
//    perComponentGMMs_.clear();

    // 2) Gather each component's states
    //    We'll collect them in a map: componentID -> (arma::mat of points)
 //   std::unordered_map<unsigned long, arma::mat> compData;

    // First pass: figure out how many states are in each component,
    // so we can pre-allocate properly. We'll also track the dimension.
//    unsigned int dim = si_->getStateDimension();
//    std::unordered_map<unsigned long, size_t> compSizes;

//    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
//    {
        // find representative
//        Vertex c = disjointSets_.find_set(v);
 //       unsigned long compID = static_cast<unsigned long>(c);
  //      compSizes[compID] += 1;
    //}

    // Now actually allocate each arma::mat with (dim rows) x (N columns)
    //for (auto &kv : compSizes)
    //{
    //    unsigned long cID = kv.first;
    //    size_t N         = kv.second;
    //    compData[cID].set_size(dim, N); // (dim x N)
    //}

    // 3) Fill each matrix with that component's states
    //    We maintain a small "current column index" per component
    //std::unordered_map<unsigned long, size_t> idxPerComp;
    //for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    //{
    //    Vertex c       = disjointSets_.find_set(v);
    //    unsigned long compID = static_cast<unsigned long>(c);

    //    auto *rv = stateProperty_[v]->as<base::RealVectorStateSpace::StateType>();
    //    size_t col = idxPerComp[compID];  // 0-based
    //    for (unsigned int d=0; d<dim; d++)
    //        compData[compID](d, col) = rv->values[d];

    //    idxPerComp[compID] = col+1; // increment
    //}

    // 4) For each connected component, we have a matrix of points => build a gmm_full with K=1
    //for (auto &entry : compData)
    //{
    //    unsigned long compID = entry.first;
    //    arma::mat &dataset   = entry.second; // each col is a sample

        // We'll create a gmm_full with exactly 1 Gaussian component
//        arma::gmm_full model;
  //      model.create(1, dim); // 1 component, 'dim' dimension
        // Means: set_size(dim, 1)
        // Covs : 3D: we can do model.covariances.resize(1) etc.

        // We compute mean & cov ourselves:
    //    arma::vec mu = arma::mean(dataset, 1); // dim x 1
        // Compute covariance
        //  (the columns of dataset are points, so we do .each_col() - mu, etc.)
      //  arma::mat centered = dataset;
      //  centered.each_col() -= mu;
      //  arma::mat cov = (centered * centered.t()) / (double)dataset.n_cols; 
        // that’s a full covariance

        // Put them into model: 1 component
        // We must do something like:
        //    model.means.slice(0) or so, except we have no direct slice. Instead:
        // Because .create(...) sets model.means, model.fcovs, model.hefts, etc.
        // We can do:
       // model.means.col(0)      = mu; // 1st col is the mean
       // model.fcovs(0)          = cov; // the 0th covariance
      //  model.hefts(0)         = 1.0; // single component => weight=1

        // So now we have a GMM with one component that has mean/cov
        // Store it in perComponentGMMs_
       // perComponentGMMs_[compID] = model;
    //}

    // That’s it. Now each connected component cID is associated
    // with an “arma::gmm_full” that has a single Gaussian with its own covariance.
    // If you want "diag" instead, you can do gmm_diag instead of gmm_full.
    // Done!
//}

std::vector<std::vector<double>> SDCL::collectRoadmapStates() const
{
    std::vector<std::vector<double>> data;
    //std::lock_guard<std::mutex> lock(graphMutex_);  // deadlock

    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        const base::State* st = stateProperty_[v];
        auto *rv = st->as<base::RealVectorStateSpace::StateType>();
        unsigned int dim = si_->getStateDimension();

        std::vector<double> sample(dim);
        for (unsigned int d = 0; d < dim; ++d)
            sample[d] = rv->values[d];

        data.push_back(std::move(sample));
    }
    return data;
}
// 2D Scatter plot
#ifdef USE_SCIPLOT
#include <sciplot/sciplot.hpp>
//#endif

//#include "SDCL.h"
void SDCL::plotRoadmapScatter(const std::string &filename) //const
{
//#ifdef USE_SCIPLOT

    // If we have sciplot, do the real plotting:
    std::lock_guard<std::mutex> lock(graphMutex_);

    // We'll gather data grouped by connected component
    std::unordered_map<unsigned long, std::vector<double>> X, Y;
    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        // representative of this vertex's component
        Vertex comp = disjointSets_.find_set(v);
        unsigned long compID = static_cast<unsigned long>(comp);

        // 2D example
        const base::State *st = stateProperty_[v];
        auto *rv = st->as<base::RealVectorStateSpace::StateType>();
        double theta1 = rv->values[0];
        double theta2 = rv->values[1];

        X[compID].push_back(theta1);
        Y[compID].push_back(theta2);
    }

    using namespace sciplot; 
    Plot2D plot;
    plot.legend().atTopRight().displayHorizontal().displayExpandWidthBy(2).fontSize(7);
    // gather *all* points from all components in one big set of arrays
    std::vector<double> xsAll, ysAll;
    xsAll.reserve( X.size() * 100 ); // optional rough reserve
    ysAll.reserve( X.size() * 100 ); // just a guess to reduce re-allocations

    for (const auto &entry : X)
    {
        unsigned long compID = entry.first;
        const auto &compXs  = entry.second;     // the X-values for this component
        const auto &compYs  = Y.at(compID);     // the Y-values for the same component

        xsAll.insert(xsAll.end(), compXs.begin(), compXs.end());
        ysAll.insert(ysAll.end(), compYs.begin(), compYs.end());
    }

    // Plot all of them at once (optional label like "All points")
    plot.drawPoints(xsAll, ysAll).pointSize(1.0).label("All components");
    
    // Each component separately, then loop again and do a different label / color for each comp.
    for (auto &kv : X)
    {
        unsigned long compID = kv.first;
        std::vector<double> &xs = kv.second;
        std::vector<double> &ys = Y[compID];

        std::string label = "component " + std::to_string(compID);
        plot.drawPoints(xs, ys).pointSize(1.0).label(label);
    }

    Figure fig = {{ plot }};
    Canvas canvas = {{ fig }};
    canvas.title("Roadmap scatter with components");
    canvas.save(filename);
    std::cout << "Saved scatter plot to " << filename << "\n";

#else
    // If sciplot is not available, just do a stub
    std::cout << "Plotting to file: " << filename 
              << " (stub function). No sciplot available.\n";
#endif
}


void SDCL::plotGMMClusters(const std::string &filename)
{
#ifdef USE_SCIPLOT
    using namespace arma;
    using namespace sciplot;

    std::lock_guard<std::mutex> lock(graphMutex_);

    // 1) Collect states into a std::vector<std::vector<double>>.
    auto dataVec = collectRoadmapStates();
    if (dataVec.empty())
    {
        std::cout << "No roadmap data to plot.\n";
        return;
    }

    // only handle 2D for plotting:
    size_t dim = dataVec[0].size();
    if (dim != 2)
    {
        std::cout << "plotGMMClusters() only supports 2D data.\n";
        return;
    }

    // 2) Convert to arma::mat
    size_t N = dataVec.size();
    arma::mat dataset(dim, N);
    for (size_t i = 0; i < N; ++i)
    {
        for (size_t d = 0; d < dim; d++)
            dataset(d, i) = dataVec[i][d];
    }

    // Ensure GMM is valid (trained).
    if (armadilloDiagGMM_.means.n_cols == 0)
    {
        std::cout << "GMM is empty (not trained?). Cannot plot clusters.\n";
        return;
    }

    // 3) Assign each sample to its cluster.
    //    Must specify the distance mode, e.g. eucl_dist.
    arma::urowvec assignments = armadilloDiagGMM_.assign(dataset, arma::eucl_dist);

    // 4) Group points by cluster ID.
    std::unordered_map<unsigned int, std::vector<double>> X, Y;
    for (size_t i = 0; i < N; i++)
    {
        unsigned int cid = assignments[i];
        double xVal = dataset(0, i);
        double yVal = dataset(1, i);

        X[cid].push_back(xVal);
        Y[cid].push_back(yVal);
    }

    // 5) Make a sciplot::Plot2D, set labels separately.
    Plot2D plot;
    plot.legend().atTopRight().displayHorizontal().displayExpandWidthBy(2).fontSize(8);

    plot.xlabel("X");
    plot.ylabel("Y");

    // Plot each cluster with a distinct label.
    for (auto &kv : X)
    {
        unsigned int cID = kv.first;
        const auto &xs = kv.second;
        const auto &ys = Y[cID];

        std::string label = "Cluster " + std::to_string(cID);
        plot.drawPoints(xs, ys).pointSize(1.5).label(label);
    }

    // 6) Wrap into a figure, save to disk.
    Figure fig = {{ plot }};
    Canvas canvas = {{ fig }};
    canvas.title("GMM Clusters (2D)");
    canvas.save(filename);

    std::cout << "Saved GMM cluster-assignments plot to " << filename << "\n";
#else
    // If sciplot not available:
    std::cout << "No sciplot available; plotGMMClusters() is a stub.\n";
#endif
}
