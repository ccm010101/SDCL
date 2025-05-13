#include "SDCL.h"
#include <iostream>
#include <boost/bind.hpp>

#ifdef USE_SCIPLOT
#include <sciplot/sciplot.hpp>
#endif

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

SDCL::ShortLongComponentsAndStates 
SDCL::reportShortAndLongComponents(std::size_t maxSize)
{
    // This struct will hold our final results.
    ShortLongComponentsAndStates result;

    // 1) Gather vertices by connected component
    std::unordered_map<unsigned long, std::vector<Vertex>> compVertices;
    compVertices.reserve(boost::num_vertices(g_)); // optional

    // Also collect *all* roadmap states in a single container for later visualization
    // (similar to collectRoadmapStates()).
    result.allRoadmapStates.reserve(boost::num_vertices(g_));

    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        // Identify the component representative
        unsigned long cID = static_cast<unsigned long>(disjointSets_.find_set(v));
        compVertices[cID].push_back(v);

        // Also record the RealVector coordinates for this vertex
        const base::State* st = stateProperty_[v];
        auto *rv = st->as<base::RealVectorStateSpace::StateType>();
        unsigned int dim = si_->getStateDimension();

        std::vector<double> coords(dim);
        for (unsigned int d = 0; d < dim; ++d)
            coords[d] = rv->values[d];
        result.allRoadmapStates.push_back(std::move(coords));
    }

    // 2) Now we know each component’s size. Separate them into short vs. long
    for (const auto &kv : compVertices)
    {
        unsigned long cID = kv.first;
        std::size_t compSize = kv.second.size();

        if (compSize <= maxSize)
        {
            // short component
            result.shortCompIDs.push_back(cID);
        }
        else
        {
            // long component
            result.longCompIDs.push_back(cID);
        }
    }

    // 3) Return the struct with short, long, and all states
    return result;
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
    
   

    auto shortLongInfo = reportShortAndLongComponents(30);


    

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
    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT; //order changed


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


// -----------SDCL part removed--------------
// ----------GMM Part------------------------

void SDCL::buildPerComponentGMMsDiag(int minK, int maxK)
{
    // Force minK = maxK = 1 for a single-cluster model
    minK = 1;
    maxK = 1;
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
        //if (N < 2)
        //{
        //    OMPL_INFORM("Component %lu has only %zu states => skipping GMM or storing trivial model.",
        //                compID, N);
        //    continue;
        //}

        // dimension = state dimension
        unsigned int dim = si_->getStateDimension();
        arma::mat dataset(dim, N);

        // Fill dataset columns from the roadmap states
        for (size_t i = 0; i < N; ++i)
        {   
            Vertex vv = vertexVec[i];
            const base::State* st = stateProperty_[vv];
            auto *rv = st->as<base::RealVectorStateSpace::StateType>();
            for (unsigned int d = 0; d < dim; d++)
                dataset(d, i) = rv->values[d];

        }

        // BIC loop
        //arma::gmm_diag bestModel;
        //double bestBIC = std::numeric_limits<double>::infinity();
        //int bestK      = 0;

        //for (int k = minK; k <= maxK; k++)
        //{
        //    arma::gmm_diag candidate;
            // Built-in K-means + EM for diagonal Gaussians:
        //    bool ok = candidate.learn(
        //        dataset,            // data
        //        k,                  // # components
        //        arma::eucl_dist,           // dist_mode
                //arma::gaussian_init,// init method
        //        arma::static_spread,  // or arma::random_subset
        //        10,                 // max EM iters
        //        10,                 // max k-means iters
        //      1e-10,              // EM tolerance
        //        false               // no verbose
        //    );

        //    if (!ok) 
        //    {
                // E-M can fail for degenerate or tiny data
        //        continue;
        //    }

            arma::gmm_diag singleModel;
            bool ok = singleModel.learn(
                dataset,            // data
                1,                  // single component
                arma::eucl_dist,    // dist_mode
                arma::static_spread,// init method
                10,                 // max EM iters
                10,                 // max k-means iters
                1e-10,              // EM tolerance
                false               // no verbose
            );
            if (!ok)
            {
                // EM can fail for degenerate data
                OMPL_INFORM("Component %lu => single-cluster GMM learn() failed with N=%zu", compID, N);
                // We can store an empty model or skip
                compGMMsDiag_[compID] = arma::gmm_diag(); 
                continue;
            }
            compGMMsDiag_[compID] = singleModel;
            OMPL_INFORM("Component %lu => single-cluster GMM built with %zu states", compID, N);
    }
        OMPL_INFORM("Done building per component diagonal GMMs => compGMMsDiag_");
}

            // compute log-likelihood
            //arma::rowvec lp = candidate.log_p(dataset);
            //double ll = arma::accu(lp);

            // for diagonal-cov GMM with k comps, dimension=dim:
            //   paramCount = (k - 1) + k*dim + k*dim = 2*k*dim + (k - 1)
            //double paramCount = 2.0 * k * dim + (k - 1);
            //double bic = -2.0 * ll + paramCount * std::log((double)N);

            //if (bic < bestBIC)
            //{
            //    bestBIC  = bic;
            //    bestModel= candidate;
            //    bestK    = k;
            //}
        //}

        //compGMMsDiag_[compID] = bestModel; // store final
        //unsigned int finalComps = bestModel.means.n_cols;
        //OMPL_INFORM("Component %lu => best BIC=%.2f with %d mixture comps",
        //            compID, bestBIC,  bestK, finalComps); //bestModel.n_gaus);
    //}

    //OMPL_INFORM("Done building per component diagonal GMMs => compGMMsDiag_");
//}



// ----------Product Part------------------------

//std::vector<unsigned long> SDCL::reportShortComponents(std::size_t maxSize)
//{
    // 1) Group vertices by component
//    std::unordered_map<unsigned long, std::vector<Vertex>> compVertices;

//    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
//    {
//        unsigned long cID = static_cast<unsigned long>(disjointSets_.find_set(v));
//        compVertices[cID].push_back(v);
//    }

    // This will hold all the "short" component IDs
//    std::vector<unsigned long> shortComps;

    // 2) Identify and print the "short" components
//    OMPL_INFORM("Components with <= %zu states:", maxSize);

//    for (const auto &kv : compVertices)
//    {
//        unsigned long cID = kv.first;
//        std::size_t compSize = kv.second.size();

//        if (compSize <= maxSize)
//        {
            // Print
//            OMPL_INFORM("  - Component %lu has %zu states.", cID, compSize);
            // Store
//            shortComps.push_back(cID);
//        }
//    }

//    return shortComps;
//}

//#include "SDCL.h" 



static void productOfGaussians(
    const std::vector<arma::vec> &means,
    const std::vector<arma::mat> &covs,
    arma::vec &muProd,
    arma::mat &covProd)
{
    using namespace arma;
    size_t N = means.size();
    if (N == 0)
    {
        std::cerr << "[productOfGaussians] No Gaussians provided!\n";
        return;
    }
    size_t d = means[0].n_elem;

    mat sumPrec(d, d, fill::zeros);
    vec sumPrecMu(d, fill::zeros);

    for (size_t i = 0; i < N; i++)
    {
        const mat &Cov_i = covs[i];
        const vec &m_i   = means[i];
        mat Prec_i = inv(Cov_i);
        sumPrec   += Prec_i;
        sumPrecMu += Prec_i * m_i;
    }

    covProd = inv(sumPrec);
    muProd  = covProd * sumPrecMu;
}

void SDCL::sampleFromProductOfShortComps(std::size_t maxSize)
{
    // 1) Get the short (and long) comps. We'll only focus on short comps here.
    ShortLongComponentsAndStates info = reportShortAndLongComponents(maxSize);
    std::vector<unsigned long> shortComps = info.shortCompIDs;

    // We’ll need the vertices grouped by component
    std::unordered_map<unsigned long, std::vector<Vertex>> compVerts;
    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        unsigned long cID = (unsigned long)disjointSets_.find_set(v);
        compVerts[cID].push_back(v);
    }

    auto numBefore = boost::num_vertices(g_);
    unsigned int samplesPerCluster = 50; // how many to sample per Gaussian cluster

    // 2) For each short component, sample from its own (fallback or GMM) distribution
    for (unsigned long compID : shortComps)
    {
        const auto &verts = compVerts[compID];
        std::size_t compSize = verts.size();
        if (compSize == 0)
            continue; 

        if (compSize < 5)
        {
            // Fallback: single state => small diagonal covariance
            Vertex onlyV = verts[0];
            const base::State* st = stateProperty_[onlyV];
            auto *rv = st->as<base::RealVectorStateSpace::StateType>();
            unsigned int dim = si_->getStateDimension();

            // Mean = that one state
            arma::vec mu(dim);
            for (unsigned int d = 0; d < dim; d++)
                mu[d] = rv->values[d];

            // Cov = identity * small eps
            double eps = 1.0;
            arma::mat covMat = eps * arma::eye<arma::mat>(dim, dim);

            // Sample from this single fallback Gaussian
            arma::mat L = arma::chol(covMat, "lower");
            for (unsigned int i = 0; i < samplesPerCluster; i++)
            {
                arma::vec z = arma::randn<arma::vec>(dim);
                arma::vec x = mu + L * z;

                // Convert x -> OMPL state
                base::State *sampleState = si_->allocState();
                auto *rvSamp = sampleState->as<base::RealVectorStateSpace::StateType>();
                for (unsigned int d = 0; d < dim; ++d)
                    rvSamp->values[d] = x[d];

                // Check validity
                if (si_->isValid(sampleState))
                    addMilestone(sampleState);
                else
                    si_->freeState(sampleState);
            }
        }
        else
        {
            // We have >=5 states => we assume we built a GMM for this comp
            auto it = compGMMsDiag_.find(compID);
            if (it == compGMMsDiag_.end())
            {
                OMPL_INFORM("No GMM found for short comp %lu (size=%zu). Skipping.", compID, compSize);
                continue;
            }

            const arma::gmm_diag &model = it->second;
            arma::uword k = model.n_gaus(); 
            if (k == 0)
            {
                OMPL_INFORM("Short comp %lu has an empty GMM? Skipping.", compID);
                continue;
            }

            // For each cluster in this GMM:
            // sample ~100 points from that cluster's diagonal covariance
            for (arma::uword c = 0; c < k; c++)
            {
                arma::vec mu   = model.means.col(c);  // cluster mean
                arma::vec dVar = model.dcovs.col(c);  // diag variances
                arma::mat cov  = arma::diagmat(dVar);

                arma::mat L = arma::chol(cov, "lower");
                unsigned int dim = mu.n_elem;

                for (unsigned int i = 0; i < samplesPerCluster; i++)
                {
                    arma::vec z = arma::randn<arma::vec>(dim);
                    arma::vec x = mu + L * z;

                    // Build OMPL state
                    base::State *sampleState = si_->allocState();
                    auto *rvSamp = sampleState->as<base::RealVectorStateSpace::StateType>();
                    for (unsigned int d = 0; d < dim; ++d)
                        rvSamp->values[d] = x[d];

                    // Collision / bounds check
                    if (si_->isValid(sampleState))
                        addMilestone(sampleState);
                    else
                        si_->freeState(sampleState);
                }
            }
        }
    }

    // 3) Print how many new vertices we added
    auto numAfter = boost::num_vertices(g_);
    OMPL_INFORM("Added %lu new vertices to the roadmap from sampling each short comp individually",
                (numAfter - numBefore));
}


void SDCL::sampleFromProductOfLongComps(std::size_t maxSize)
{
    // 1) Get all short & long comp IDs from the new helper
    ShortLongComponentsAndStates info = reportShortAndLongComponents(maxSize);
    // The "long" comps are directly in info.longCompIDs
    std::vector<unsigned long> longComps = info.longCompIDs;

    // 2) Collect the single Gaussians from each "long" comp
    std::vector<arma::vec> means;
    std::vector<arma::mat> covs;

    for (unsigned long cID : longComps)
    {
        // We assume compGMMsDiag_ has a single-cluster GMM for each
        const arma::gmm_diag &model = compGMMsDiag_.at(cID);
        // single-cluster => model.n_gaus() = 1
        arma::vec mu      = model.means.col(0);
        arma::vec diagVar = model.dcovs.col(0);
        arma::mat covMat  = arma::diagmat(diagVar);

        means.push_back(mu);
        covs.push_back(covMat);
    }

    if (means.empty())
    {
        OMPL_INFORM("No long comps to sample from. Exiting.");
        return;
    }

    // 3) Multiply them into one product Gaussian
    arma::vec muProd;
    arma::mat covProd;
    productOfGaussians(means, covs, muProd, covProd);

    // 4) Sample from that product
    auto numBefore = boost::num_vertices(g_);
    unsigned int nSamples = 1000;
    arma::mat L = arma::chol(covProd, "lower");

    for (unsigned int i = 0; i < nSamples; i++)
    {
        arma::vec z = arma::randn<arma::vec>(muProd.n_elem);
        arma::vec x = muProd + L * z;

        base::State *sampleState = si_->allocState();
        auto *rv = sampleState->as<base::RealVectorStateSpace::StateType>();
        for (unsigned int d = 0; d < muProd.n_elem; ++d)
            rv->values[d] = x[d];

        // Collision / bounds check
        if (si_->isValid(sampleState))
            addMilestone(sampleState);
        else
            si_->freeState(sampleState);
    }

    auto numAfter = boost::num_vertices(g_);
    OMPL_INFORM("Added %lu new vertices to the roadmap from product-of-long-comps",
                (numAfter - numBefore));
}



//void SDCL::sampleFromProductOfShortComps(std::size_t maxSize)
//{
    // 1) Identify the “short” connected components
//    std::vector<unsigned long> shortComps = reportShortComponents(maxSize);

    // gather all Gaussians (clusters) from these short components
//    std::vector<arma::vec> means;
//    std::vector<arma::mat> covs;

    // group the vertices by component
//    std::unordered_map<unsigned long, std::vector<Vertex>> compVerts;
//    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
//    {
//        unsigned long cID = (unsigned long)disjointSets_.find_set(v);
//        compVerts[cID].push_back(v);
//    }

//    for (unsigned long compID : shortComps)
//    {
//        auto &verts = compVerts[compID];
//        std::size_t compSize = verts.size();

        // Fallback if exactly one state
//        if (compSize == 1)
//        {
//            Vertex theOnlyV = verts[0];
//            const base::State* singleSt = stateProperty_[theOnlyV];
//            auto *rv = singleSt->as<ompl::base::RealVectorStateSpace::StateType>();
//            unsigned int d = si_->getStateDimension();

//            arma::vec mu(d);
//            for (unsigned int i = 0; i < d; i++)
//                mu[i] = rv->values[i];

            // small diagonal
//            double eps = 1.0;
//            arma::vec diagVar(d, arma::fill::ones);
//            diagVar *= eps;
//            arma::mat covMat = arma::diagmat(diagVar);

//            means.push_back(mu);
//            covs.push_back(covMat);
//            continue;
//        }

        // If >= 2 states, see if we built a GMM for it
//        auto it = compGMMsDiag_.find(compID);
//        if (it == compGMMsDiag_.end())
//        {
//            OMPL_INFORM("No GMM found for component %lu (size=%zu), skipping component.", compID, compSize);
//            continue;
//        }

//        const arma::gmm_diag &model = it->second;
//        arma::uword k = model.n_gaus();
//        if (k == 0)
//        {
//            OMPL_INFORM("GMM for comp %lu has zero clusters, skipping.", compID);
//            continue;
//        }
        // Each cluster => diagonal covariance
//        for (arma::uword c = 0; c < k; c++)
//        {
//            arma::vec m   = model.means.col(c);
//            arma::vec dCv = model.dcovs.col(c);
//            arma::mat covMat = arma::diagmat(dCv);

//            means.push_back(m);
//            covs.push_back(covMat);
//        }
//    }

//    if (means.empty())
//    {
//        OMPL_INFORM("No clusters or fallback single-states from short comps. Nothing to do.");
//        return;
//    }

    // 2) Multiply them into a single “product” Gaussian
//    arma::vec muProd;
//    arma::mat covProd;
//    productOfGaussians(means, covs, muProd, covProd);

 //   OMPL_INFORM("Product of Gaussians across short comps => muProd:");
 //   muProd.t().print(std::cout, "muProd:");
 //   OMPL_INFORM("covProd = ");
 //   covProd.print(std::cout);

    // 3) Sample from that product distribution
//    auto numBefore = boost::num_vertices(g_);
//    unsigned int nSamples = 2000;
//    arma::mat L = arma::chol(covProd, "lower"); // Cholesky factor

//    for (unsigned int i = 0; i < nSamples; i++)
//    {
//        arma::vec z = arma::randn<arma::vec>(muProd.n_elem);
//        arma::vec x = muProd + L*z;  // sample in d dimensions

        // fill an OMPL RealVectorState
//        base::State *sampleState = si_->allocState();
//        auto *rv = sampleState->as<ompl::base::RealVectorStateSpace::StateType>();
//        for (unsigned int d = 0; d < muProd.n_elem; ++d)
//            rv->values[d] = x[d];

        // collision / bounds check
//        if (si_->isValid(sampleState))
//        {
            // Debug printing for all d:
//            std::ostringstream oss;
//            oss << "[";
//            for (unsigned int dd=0; dd<muProd.n_elem; dd++)
//            {
//                oss << x[dd];
//                if (dd+1 < muProd.n_elem) oss << ", ";
 //           }
 //           oss << "]";

//            OMPL_INFORM("Sample %u: %s is valid, adding to roadmap", i, oss.str().c_str());
 //           addMilestone(sampleState);
 //       }
 //       else
 //       {
            // Freed if invalid
 //           si_->freeState(sampleState);
 //       }
 //   }

 //   auto numAfter = boost::num_vertices(g_);
 //   OMPL_INFORM("Added %lu new vertices to the roadmap from product-of-short-comps",
 //               (numAfter - numBefore));
//}




//void SDCL::sampleFromProductOfLongComps(std::size_t maxSize)
//{
    // 1) Gather ALL compIDs
//    std::unordered_set<unsigned long> shortSet;
//    for (auto sc : reportShortComponents(maxSize))
//        shortSet.insert(sc);

    // Now gather all "long" comps
//    std::vector<unsigned long> longComps;
//    for (auto &kv : compGMMsDiag_)
//    {
//        unsigned long cID = kv.first;
//        if (shortSet.find(cID) == shortSet.end())
//            longComps.push_back(cID);
//    }

    // 2) Collect the single Gaussians from each "long" comp
//    std::vector<arma::vec> means;
//    std::vector<arma::mat> covs;

 //   for (unsigned long cID : longComps)
 //   {
 //       const arma::gmm_diag &model = compGMMsDiag_.at(cID);
        // single-cluster => model.n_gaus() = 1
 //       arma::vec mu = model.means.col(0);
//        arma::vec diagVar = model.dcovs.col(0);
//        arma::mat covMat = arma::diagmat(diagVar);

//        means.push_back(mu);
//        covs.push_back(covMat);
//    }

 //   if (means.empty())
 //   {
 //       OMPL_INFORM("No long comps to sample from. Exiting.");
 //       return;
 //   }

    // 3) Multiply them into one product Gaussian
 //   arma::vec muProd;
 //   arma::mat covProd;
 //   productOfGaussians(means, covs, muProd, covProd);

    // 4) Sample
 //   auto numBefore = boost::num_vertices(g_);
 //   unsigned int nSamples = 1000;
 //   arma::mat L = arma::chol(covProd, "lower");

 //   for (unsigned int i = 0; i < nSamples; i++)
 //   {
 //       arma::vec z = arma::randn<arma::vec>(muProd.n_elem);
 //       arma::vec x = muProd + L * z;

 //       base::State *sampleState = si_->allocState();
 //       auto *rv = sampleState->as<ompl::base::RealVectorStateSpace::StateType>();
 //       for (unsigned int d = 0; d < muProd.n_elem; ++d)
 //           rv->values[d] = x[d];

 //       if (si_->isValid(sampleState))
 //           addMilestone(sampleState);
 //       else
 //           si_->freeState(sampleState);
 //   }

//    auto numAfter = boost::num_vertices(g_);
//    OMPL_INFORM("Added %lu new vertices to the roadmap from product-of-long-comps",
//                (numAfter - numBefore));
//}


//-----------------distance ------------------------------

double bhattacharyyaDistance(const arma::vec &m1, const arma::mat &S1,
                             const arma::vec &m2, const arma::mat &S2)
{
    // Average covariance
    arma::mat Sigma = 0.5 * (S1 + S2);

    arma::vec diff = m1 - m2;
    // (1/8) * diff^T * Sigma^-1 * diff
    arma::mat SigmaInv = arma::inv(Sigma);
    double term1 = 0.125 * arma::as_scalar(diff.t() * SigmaInv * diff);

    // 0.5 * ln( |Sigma| / sqrt(|S1| * |S2| ) )
    double detS  = arma::det(Sigma);
    double detS1 = arma::det(S1);
    double detS2 = arma::det(S2);
    double term2 = 0.5 * std::log(detS / std::sqrt(detS1 * detS2));

    return term1 + term2;
}

// Helper to convert an arma::gmm_diag clusters diagonal cov + mean => full mat
arma::mat diagCovToMat(const arma::vec &diagVar)
{
    // Just put the diagonal entries in a full NxN matrix
    return arma::diagmat(diagVar);
}


void SDCL::computePairwiseBD()
{
    // compGMMsDiag_ is map<componentID, gmm_diag>
    // iterate over pairs (compA, compB) with compA != compB
    std::vector<unsigned long> compIDs;
    compIDs.reserve(compGMMsDiag_.size());
    for (auto &kv : compGMMsDiag_)
        compIDs.push_back(kv.first);

    // Loop over pairs of distinct components
    for (size_t i = 0; i < compIDs.size(); ++i)
    {
        unsigned long compA = compIDs[i];
        const auto &modelA = compGMMsDiag_.at(compA);

        for (size_t j = i + 1; j < compIDs.size(); ++j) // j>i => skip same or duplicates
        {
            unsigned long compB = compIDs[j];
            const auto &modelB = compGMMsDiag_.at(compB);

            // Number of clusters in each GMM
            unsigned int kA = modelA.n_gaus();
            unsigned int kB = modelB.n_gaus();

            // For each cluster in compA and compB
            for (unsigned int cA = 0; cA < kA; ++cA)
            {
                // means.col(cA) => cluster cA's mean
                arma::vec muA = modelA.means.col(cA);
                // dcovars.col(cA) => cluster cA's diagonal variances
                arma::mat covA = diagCovToMat(modelA.dcovs.col(cA));

                for (unsigned int cB = 0; cB < kB; ++cB)
                {
                    arma::vec muB = modelB.means.col(cB);
                    arma::mat covB = diagCovToMat(modelB.dcovs.col(cB));

                    double bd = bhattacharyyaDistance(muA, covA, muB, covB);
// Skip if BD is extremely close to zero
// (use a small epsilon to account for floating-point rounding errors)
if (std::fabs(bd) < 1e-9)
    continue;  // Don't print or store

                    // Print or store the result
                    std::cout
                      << "[Comp " << compA << ", Clust " << cA << "] vs "
                      << "[Comp " << compB << ", Clust " << cB << "] => BD = "
                      << bd << std::endl;
                }
            }
        }
    }
}




// ---------------plotting ----------------------------


void SDCL::plotPerComponentGMMClusters(const std::string &filename)
{
#ifdef USE_SCIPLOT
    using namespace arma;
    using namespace sciplot;

    // Step A: group the graph by connected component
    std::unordered_map<unsigned long, std::vector<Vertex>> compVerts;
    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        unsigned long cID = (unsigned long)disjointSets_.find_set(v);
        compVerts[cID].push_back(v);
    }

    // Identify short comps for color, e.g. short if size <= 5
    // OLD:
    //   std::vector<unsigned long> shortVec = reportShortComponents(5);
    //   std::unordered_set<unsigned long> shortSet(shortVec.begin(), shortVec.end());

    // NEW (minimal change):
    auto shortLongInfo = reportShortAndLongComponents(5);
    std::unordered_set<unsigned long> shortSet(
        shortLongInfo.shortCompIDs.begin(),
        shortLongInfo.shortCompIDs.end()
    );

    Plot2D plot;
    plot.legend().atOutsideBottom().displayHorizontal().displayExpandWidthBy(2).fontSize(5);
    plot.xlabel("X");
    plot.ylabel("Y");
    plot.xrange(-4.0, 4.0);
    plot.yrange(-4.0, 4.0);

    // Step B: for each component, look up its GMM
    for (auto &kv : compVerts)
    {
        unsigned long cID = kv.first;
        auto it = compGMMsDiag_.find(cID);
        if (it == compGMMsDiag_.end())
            continue; // no GMM

        const arma::gmm_diag &model = it->second;
        size_t N = kv.second.size();
        if (N < 1)
            continue;

        // dimension check
        unsigned int dim = si_->getStateDimension();
        if (dim != 2)
        {
            OMPL_INFORM("plotPerComponentGMMClusters: skipping comp %lu, dim != 2", cID);
            continue;
        }

        // Build dataset
        arma::mat dataset(dim, N);
        for (size_t i = 0; i < N; i++)
        {
            Vertex v = kv.second[i];
            auto *st = stateProperty_[v]->as<base::RealVectorStateSpace::StateType>();
            dataset(0, i) = st->values[0];
            dataset(1, i) = st->values[1];
        }

        // GMM with single cluster => everything goes to cluster 0
        arma::urowvec assignments = model.assign(dataset, arma::eucl_dist);

        // Separate points by cluster ID (though there's only 1 if we used k=1)
        std::unordered_map<unsigned int, std::vector<double>> X, Y;
        for (size_t i = 0; i < N; i++)
        {
            unsigned int cid = assignments[i];
            X[cid].push_back(dataset(0, i));
            Y[cid].push_back(dataset(1, i));
        }

        // Decide color for short vs. long
        bool isShort = (shortSet.find(cID) != shortSet.end());
        std::string colorStr = isShort ? "red" : "blue";

        // Draw each cluster (though there's just 1) with that color
        for (auto &ckv : X)
        {
            unsigned int cCluster = ckv.first;
            const auto &xs = ckv.second;
            const auto &ys = Y[cCluster];

            std::string label = "Comp " + std::to_string(cID) + 
                                " (clust " + std::to_string(cCluster) + ")";
            plot.drawPoints(xs, ys)
                .pointSize(1.0)
                .label(label)
                .lineColor(colorStr); // color by short vs. long
        }
    }

    Figure fig = {{ plot }};
    Canvas canvas = {{ fig }};
    canvas.title("Per-Component Single-Gaussian (2D) with Short/Long Coloring");
    canvas.save(filename);
    std::cout << "Saved single-cluster GMM 2D plot to " << filename << "\n";
#else
    std::cout<<"(no sciplot) skipping plotPerComponentGMMClusters\n";
#endif
}






void SDCL::plotRoadmapScatter(const std::string &filename) //const
{
#ifdef USE_SCIPLOT

    std::lock_guard<std::mutex> lock(graphMutex_);

    // gather data grouped by connected component
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
    plot.legend().atOutsideBottom().displayHorizontal().displayExpandWidthBy(2).fontSize(7);
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
    plot.drawPoints(xsAll, ysAll).pointSize(0.5).label("All components");
    
    // Each component separately, then loop again and do a different label / color for each comp.
    for (auto &kv : X)
    {
        unsigned long compID = kv.first;
        std::vector<double> &xs = kv.second;
        std::vector<double> &ys = Y[compID];

        std::string label = "component " + std::to_string(compID);
        plot.drawPoints(xs, ys).pointSize(0.5).label(label);
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



