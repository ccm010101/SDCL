#include "SDCL.h"
#include <boost/bind.hpp>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#define foreach BOOST_FOREACH

// Optional: remove old #includes for collision if not needed...
#include <iostream>
#include <armadillo>

// If you do sciplot plotting:
#ifdef USE_SCIPLOT
#include <sciplot/sciplot.hpp>
#endif

// -------------------------------------------------- 
//  If you still use or reference NLOPT, SVM, Thundersvm, etc.:
#include "nlopt.hpp"  // if needed
// #include "someSvmHeader.h" // etc.

// Place your code for objfunc, objfunc2, findClosestPoint, etc., if still used...

namespace ompl
{
    namespace magic
    {
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;
        static const double ROADMAP_BUILD_TIME = 0.2;
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;

        // Possibly also define MAX_VALID_SAMPLE_ATTEMPTS if you want
        // static const unsigned int MAX_VALID_SAMPLE_ATTEMPTS = 100;
    }
} // namespace ompl

//--------------------------------------------------------------------
//  Constructor / destructor
//--------------------------------------------------------------------

SDCL::SDCL(const base::SpaceInformationPtr &si,
           bool starStrategy,
           bool use_training,
           bool use_Gaussian)
  : base::Planner(si, "SDCL")
  , starStrategy_(starStrategy)
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
  , stddev_(si->getMaximumExtent() * ompl::magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
  , use_training_(use_training)
  , use_Gaussian_(use_Gaussian)
  , lower_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low)
  , upper_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high)
{
    specs_.recognizedGoal        = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions  = true;
    specs_.optimizingPaths       = true;
    specs_.multithreaded         = true;

    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this,
                                            &SDCL::setMaxNearestNeighbors,
                                            &SDCL::getMaxNearestNeighbors,
                                            std::string("8:1000"));

    // Add your progress properties if you want to track them
    addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationCount(); });
    addPlannerProgressProperty("best cost REAL", [this] { return getBestCost(); });
    addPlannerProgressProperty("milestone count INTEGER", [this] { return getMilestoneCountString(); });
    addPlannerProgressProperty("edge count INTEGER", [this] { return getEdgeCountString(); });
}

SDCL::~SDCL()
{
    freeMemory();
}

//--------------------------------------------------------------------
//  setup / clearing
//--------------------------------------------------------------------

void SDCL::setup()
{
    Planner::setup();

    if (!nn_)
    {
        specs_.multithreaded = false; // temporarily off
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;

        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) {
            return distanceFunction(a, b);
        });
    }
    if (!connectionStrategy_)
        setDefaultConnectionStrategy();
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &) { return true; };

    // If no optimization objective was specified, default to path length
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
        OMPL_INFORM("%s: no problem definition set => deferring setup", getName().c_str());
        setup_ = false;
    }
}

void SDCL::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        specs_.multithreaded = false;
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) {
            return distanceFunction(a, b);
        });
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
    {
        connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>(
            [this] { return milestoneCount(); },
            nn_, si_->getStateDimension());
    }
    else
    {
        connectionStrategy_ =
            ompl::geometric::KStrategy<Vertex>(ompl::magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
    }
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
    OMPL_INFORM("%s: calling clear()...", getName().c_str());
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();

    if (nn_)
        nn_->clear();

    clearQuery();

    iterations_ = 0;
    bestCost_   = base::Cost(std::numeric_limits<double>::quiet_NaN());

    // free or reset your SVM data, etc. if needed
    if (!savedModelData_.vectors)
    {
        delete [] savedModelData_.vectors;
        savedModelData_.vectors = nullptr;
    }
    if (!savedModelData_.coef)
    {
        delete [] savedModelData_.coef;
        savedModelData_.coef = nullptr;
    }

    collisionPoints_.reset(new pvec());
    stats_ = proof_stats();
    addedToTraining_ = 0;
}

void SDCL::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();

    if (!savedModelData_.vectors)
        delete [] savedModelData_.vectors;
    if (!savedModelData_.coef)
        delete [] savedModelData_.coef;
}

//--------------------------------------------------------------------
//  solve
//--------------------------------------------------------------------
ompl::base::PlannerStatus SDCL::solve(const base::PlannerTerminationCondition &ptc)
{
    auto start_time = std::chrono::steady_clock::now();

    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (!goal)
    {
        OMPL_ERROR("%s: unknown goal type", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: no valid start states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // ensure there's a valid goal
    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // ensure at least one valid goal
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: start planning with %lu states in datastructure",
                getName().c_str(), nrStartStates);

    addedNewSolution_ = false;
    base::PathPtr sol;

    // Instead of separate threads:
    // We'll do a single call to "constructRoadmap"
    // or you might do expansions until time up:
    base::PlannerTerminationCondition ptcOrSolution([this, &ptc]{return ptc || addedNewSolution();});

    constructRoadmap(ptcOrSolution);

    // after building the roadmap, check for solutions
    checkForSolution(ptc, sol);

    OMPL_INFORM("%s: Created %u states total.",
        getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    OMPL_INFORM("Total training time is %f, total sampling time is %f.",
        stats_.training_time, stats_.sampling_time);

    if (sol)
    {
        // we found an exact solution
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);

        stats_.solved = 1;

        std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
        stats_.total_time += tot.count();

        return base::PlannerStatus::EXACT_SOLUTION;
    }
    else
    {
        // Attempt approximate solution
        ompl::base::Cost diff = constructApproximateSolution(startM_, goalM_, sol);
        if (!opt_->isFinite(diff))
        {
            OMPL_INFORM("Closest path is still start and goal");
            return base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("Using approximate solution, cost-to-go is %f", diff.value());
        pdef_->addSolutionPath(sol, true, diff.value(), getName());

        std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
        stats_.total_time += tot.count();

        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }
}

//--------------------------------------------------------------------
//  constructing the roadmap
//--------------------------------------------------------------------
void SDCL::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup()) setup();

    // ensure we have a valid-state sampler
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    // if we want a "regular" state sampler for expansions:
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    // We bounce between "grow" and "expand"
    // e.g. 2:1 ratio
    std::vector<base::State *> xstates(ompl::magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);

    bool doGrow = true;
    bestCost_ = opt_->infiniteCost();
    while (!ptc())
    {
        if (doGrow)
        {
            growRoadmap(base::plannerOrTerminationCondition(
                            ptc, base::timedPlannerTerminationCondition(2.0 * ompl::magic::ROADMAP_BUILD_TIME)),
                        xstates[0]);
        }
        else
        {
            expandRoadmap(base::plannerOrTerminationCondition(
                              ptc, base::timedPlannerTerminationCondition(ompl::magic::ROADMAP_BUILD_TIME)),
                          xstates);
        }
        doGrow = !doGrow;
    }

    si_->freeStates(xstates);
}

//--------------------------------------------------------------------
//  new GROW approach => single sample() call each iteration
//--------------------------------------------------------------------
void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc,
                       base::State *workState)
{
    // We'll sample exactly once per iteration
    // If it fails, we break
    while (!ptc)
    {
        iterations_++;

        bool gotValid = sampler_->sample(workState);
        if (gotValid)
        {
            // the returned state is valid => add as milestone
            addMilestone(si_->cloneState(workState));
        }
        else
        {
            // The ValidStateSampler gave up
            OMPL_WARN("ValidStateSampler failed to find a valid sample. Stopping growth.");
            break;
        }
    }
}

//--------------------------------------------------------------------
//  old approach => multi-sample or custom collision logging is removed
//--------------------------------------------------------------------

/*
void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc,
                       base::State *workState)
{
    // Original code with double while + sampleAndSaveCollisionPoints
    // is commented out. If you no longer need it, remove it entirely.
    // ...
}
*/

//--------------------------------------------------------------------
//  expandRoadmap => your existing BFS expansions or random bounce motion
//--------------------------------------------------------------------

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
    // etc. your code for bounce expansions or so
    // ...
}

//--------------------------------------------------------------------
//  checkForSolution, maybeConstructSolution, constructSolution, etc.
//--------------------------------------------------------------------
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

bool SDCL::maybeConstructSolution(const std::vector<Vertex> &starts,
                                  const std::vector<Vertex> &goals,
                                  base::PathPtr &solution)
{
    // your code
    // ...
    return false;
}

bool SDCL::addedNewSolution() const
{
    return addedNewSolution_;
}

ompl::base::Cost SDCL::constructApproximateSolution(const std::vector<Vertex> &starts,
                                                    const std::vector<Vertex> &goals,
                                                    base::PathPtr &solution)
{
    // your existing code
    // ...
    return opt_->infiniteCost();
}

ompl::base::PathPtr SDCL::constructSolution(const Vertex &start, const Vertex &goal)
{
    // ...
    return nullptr;
}

//--------------------------------------------------------------------
//  addMilestone, uniteComponents, sameComponent
//--------------------------------------------------------------------
SDCL::Vertex SDCL::addMilestone(base::State *state)
{
    std::lock_guard<std::mutex> _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    disjointSets_.make_set(m);

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

//--------------------------------------------------------------------
//  getPlannerData
//--------------------------------------------------------------------
void SDCL::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    for (unsigned long i : startM_)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[i],
            const_cast<SDCL *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : goalM_)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[i],
            const_cast<SDCL *>(this)->disjointSets_.find_set(i)));

    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]),
                     base::PlannerDataVertex(stateProperty_[v2]));
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]),
                     base::PlannerDataVertex(stateProperty_[v1]));

        data.tagState(stateProperty_[v1],
            const_cast<SDCL *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2],
            const_cast<SDCL *>(this)->disjointSets_.find_set(v2));
    }
}

ompl::base::Cost SDCL::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

//--------------------------------------------------------------------
//  GMM code
//--------------------------------------------------------------------
arma::gmm_diag SDCL::buildSingleCovDiagGMM(const arma::mat& dataset, int k)
{
    using namespace arma;

    size_t d = dataset.n_rows;
    size_t N = dataset.n_cols;

    // Step 1: K-means
    mat centroids(d, k);
    bool kmSuccess = kmeans(centroids, dataset, k, static_spread, 10, false);
    if (!kmSuccess)
    {
        OMPL_WARN("K-means failed for k=%d", k);
        return gmm_diag(); // empty
    }

    // Step 2: assign each point
    rowvec assignments(N, fill::zeros);
    vec clusterCounts(k, fill::zeros);

    for(size_t i=0; i<N; i++)
    {
        double bestDist = DBL_MAX;
        uword bestC = 0;
        for(int c=0; c<k; c++)
        {
            double dsq = accu(square(dataset.col(i) - centroids.col(c)));
            if (dsq<bestDist)
            {
                bestDist = dsq; 
                bestC = c;
            }
        }
        assignments[i] = bestC;
        clusterCounts[bestC]++;
    }

    // refine means
    centroids.zeros();
    for (size_t i=0; i<N; i++)
    {
        uword c = (uword) assignments[i];
        centroids.col(c) += dataset.col(i);
    }
    for (int c=0; c<k; c++)
    {
        if (clusterCounts[c]>0)
            centroids.col(c) /= clusterCounts[c];
    }

    // weights
    vec weights = clusterCounts / double(N);

    // single diag => compute overall variance from entire dataset
    vec globalMean = mean(dataset, 1);
    vec globalVar(d, fill::zeros);

    for (size_t i=0; i<N; i++)
    {
        vec diff = dataset.col(i)-globalMean;
        globalVar += diff%diff;
    }
    globalVar /= double(N);
    // clamp tiny variance
    globalVar.for_each([](double &val){ if(val<1e-12) val=1e-12; });

    // build the model
    arma::mat outMeans(d, k, fill::zeros);
    arma::mat outDcovs(d, k, fill::zeros);
    arma::rowvec outHefts(k, fill::zeros);

    for (int c=0; c<k; c++)
    {
        outMeans.col(c) = centroids.col(c);
        outHefts(c) = weights[c];
        outDcovs.col(c) = globalVar;
    }

    double sumW = accu(outHefts);
    if (sumW>0)
        outHefts = outHefts*(1.0/sumW);

    arma::gmm_diag model;
    model.set_params(outMeans, outDcovs, outHefts);
    return model;
}

void SDCL::trainGMMArmadilloBIC(const std::vector<std::vector<double>> &data,
                                int minK, int maxK)
{
    if (data.empty() || minK<=0 || maxK<minK)
    {
        OMPL_INFORM("No data or invalid range for single diag GMM BIC training.");
        return;
    }
    size_t dim = data[0].size();
    size_t N   = data.size();
    arma::mat dataset(dim, N);
    for (size_t i=0; i<N; i++)
    {
        for (size_t d=0; d<dim; d++)
            dataset(d,i)= data[i][d];
    }

    double bestBIC = std::numeric_limits<double>::infinity();
    arma::gmm_diag bestModel;

    for (int k=minK; k<=maxK; k++)
    {
        arma::gmm_diag model = buildSingleCovDiagGMM(dataset, k);
        if (model.means.n_cols==0)
            continue; // failed or empty

        // compute log-likelihood
        arma::rowvec logpVec = model.log_p(dataset);
        double ll = arma::accu(logpVec);
        // single diag => p = (k-1) + k*dim + dim
        double p = (k-1) + double(k*dim)+double(dim);
        double bic = -2.0*ll + p*std::log(N);

        OMPL_INFORM("k=%d => logLik=%.2f, BIC=%.2f", k, ll, bic);

        if (bic<bestBIC)
        {
            bestBIC  = bic;
            bestModel= model;
        }
    }

    OMPL_INFORM("Single diag => best BIC=%.2f", bestBIC);
    armadilloDiagGMM_ = bestModel;
}

std::vector<std::vector<double>> SDCL::collectRoadmapStates() const
{
    std::vector<std::vector<double>> data;

    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        const base::State* st = stateProperty_[v];
        auto *rv = st->as<base::RealVectorStateSpace::StateType>();
        unsigned int dim = si_->getStateDimension();

        std::vector<double> sample(dim);
        for (unsigned int d=0; d<dim; d++)
            sample[d]= rv->values[d];

        data.push_back(std::move(sample));
    }
    return data;
}

// 2D scatter plot => plotRoadmapScatter
void SDCL::plotRoadmapScatter(const std::string &filename)
{
#ifdef USE_SCIPLOT
    std::lock_guard<std::mutex> lock(graphMutex_);

    std::unordered_map<unsigned long, std::vector<double>> X, Y;
    for (auto v : boost::make_iterator_range(boost::vertices(g_)))
    {
        Vertex comp = disjointSets_.find_set(v);
        unsigned long compID = static_cast<unsigned long>(comp);

        const base::State *st = stateProperty_[v];
        auto *rv = st->as<base::RealVectorStateSpace::StateType>();
        double x = rv->values[0];
        double y = rv->values[1];

        X[compID].push_back(x);
        Y[compID].push_back(y);
    }

    using namespace sciplot;
    Plot2D plot;
    plot.legend().atTopRight().displayHorizontal().displayExpandWidthBy(2).fontSize(7);

    // gather all
    std::vector<double> xsAll, ysAll;
    xsAll.reserve( X.size()*100 );
    ysAll.reserve( X.size()*100 );

    for (auto &entry : X)
    {
        unsigned long cID = entry.first;
        auto &compXs = entry.second;
        auto &compYs = Y[cID];

        xsAll.insert(xsAll.end(), compXs.begin(), compXs.end());
        ysAll.insert(ysAll.end(), compYs.begin(), compYs.end());
    }

    // all at once
    plot.drawPoints(xsAll, ysAll).pointSize(1.0).label("All components");

    // then each component
    for (auto &kv : X)
    {
        unsigned long compID = kv.first;
        auto &xs = kv.second;
        auto &ys = Y[compID];

        std::string label = "component " + std::to_string(compID);
        plot.drawPoints(xs, ys).pointSize(1.0).label(label);
    }

    Figure fig = {{ plot }};
    Canvas canvas = {{ fig }};
    canvas.title("Roadmap scatter with components");
    canvas.save(filename);
    std::cout<<"Saved scatter plot to "<<filename<<"\n";
#else
    std::cout<<"(Stub) No sciplot => not saving scatter.\n";
#endif
}

// GMM clusters
void SDCL::plotGMMClusters(const std::string &filename)
{
#ifdef USE_SCIPLOT
    using namespace sciplot;
    std::lock_guard<std::mutex> lock(graphMutex_);

    auto dataVec = collectRoadmapStates();
    if (dataVec.empty())
    {
        std::cout<<"No roadmap data => cannot plot GMM.\n";
        return;
    }
    if (dataVec[0].size()<2)
    {
        std::cout<<"GMM clusters only 2D => skipping.\n";
        return;
    }

    // check if GMM is trained
    if (armadilloDiagGMM_.means.n_cols==0)
    {
        std::cout<<"GMM is empty => not trained => no cluster plot.\n";
        return;
    }

    size_t N = dataVec.size();
    size_t dim= dataVec[0].size();
    arma::mat dataset(dim, N);
    for (size_t i=0; i<N; i++)
    {
        for (size_t d=0; d<dim; d++)
            dataset(d,i)= dataVec[i][d];
    }

    // assign each sample
    arma::urowvec assignments= armadilloDiagGMM_.assign(dataset, arma::eucl_dist);

    std::unordered_map<unsigned int, std::vector<double>> X, Y;
    for (size_t i=0; i<N; i++)
    {
        unsigned int cid = assignments[i];
        double x = dataset(0,i);
        double y = dataset(1,i);

        X[cid].push_back(x);
        Y[cid].push_back(y);
    }

    Plot2D plot;
    plot.legend().atTopRight().displayHorizontal().displayExpandWidthBy(2).fontSize(8);
    plot.xlabel("X");
    plot.ylabel("Y");

    for (auto &kv : X)
    {
        unsigned int cid = kv.first;
        auto &xs = kv.second;
        auto &ys = Y[cid];

        std::string label = "Cluster " + std::to_string(cid);
        plot.drawPoints(xs, ys).pointSize(1.5).label(label);
    }

    Figure fig = {{ plot }};
    Canvas canvas = {{ fig }};
    canvas.title("GMM Clusters (2D)");
    canvas.save(filename);
    std::cout<<"Saved GMM cluster-assignments plot to "<<filename<<"\n";
#else
    std::cout<<"(Stub) no sciplot => not plotting GMM clusters.\n";
#endif
}

//--------------------------------------------------------------------------------
//  (If you want a function for sampleAndSaveCollisionPoints, you can comment it out.)
//--------------------------------------------------------------------------------
/*
bool SDCL::sampleAndSaveCollisionPoints(base::State* workState, bool use_Gaussian)
{
    // If you no longer want the double while approach, remove or comment out
    return false;
}
*/

