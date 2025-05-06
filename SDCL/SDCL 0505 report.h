#ifndef SDCL_H
#define SDCL_H

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/config.h>
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/copy.hpp>

// ------- ThunderSVM Commented out----------------------
#include <armadillo>
#include <flann/flann.hpp>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <set>
#include <math.h>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <atomic>
#include <fstream>
#include <thread>
#include <typeinfo>

using namespace ompl;

/** ------------- AStar Found Goal ------------- **/
struct AStarFoundGoal
{
};  // exception for termination

// visitor that terminates when we find the goal
// V is the vertex type
template <typename V>
class AStarGoalVisitor : public boost::default_astar_visitor
{
public:
    AStarGoalVisitor(const V &goal) : goal_(goal)
    {
    }

    // G is the graph type
    template <typename G>
    void examine_vertex(const V &u, const G & /*unused*/)
    {
        if (u == goal_)
            throw AStarFoundGoal();
    }

private:
    V goal_;
};

/** ------------- proof_stats ------------- **/
struct proof_stats
{
    int solved = 0;
    int n_facets = 0;
    int n_collision_points = 0;
    int n_valid_mpoints = 0;
    int n_itr = 0;
    double tc_time = 0;
    double mpoints_time = 0;
    double check_time = 0;
    double total_time = 0;
    double training_time = 0;
    double sampling_time = 0;
    bool useGaussian = false;
    bool useTraining = false;
};

/** 
 * ------------- ModelData removed-------------
 * Used originally for SVM
**/
// struct ModelData
// {
//     ModelData() {
//         b = 0;
//         num_vectors = 0;
//         gamma = 0;
//         coef = NULL;
//         vectors = NULL;
//     };
//     double b;
//     int num_vectors;
//     double gamma;
//     double* coef;
//     double* vectors;
// };

// // double myconstraint(unsigned n, const double *x, double *grad, void *data);
// // double objfunc(unsigned n, const double *x, double *grad, void *data);
// // int findClosestPoint(double *p, double *startingP, double *res, int n, ModelData svm_data);

//OPTIONAL: mlpack / GMM ////////////////////
//#ifdef ENABLE_MLPACK
//#include <mlpack/methods/gmm/gmm.hpp>
//#endif

/**
 * \brief The SDCL Planner with PRM-like approach
 */
class SDCL : public base::Planner
{
public:
    struct vertex_state_t
    {
        using kind = boost::vertex_property_tag;
    };

    struct vertex_total_connection_attempts_t
    {
        using kind = boost::vertex_property_tag;
    };

    struct vertex_successful_connection_attempts_t
    {
        using kind = boost::vertex_property_tag;
    };

    using Graph = boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS,
        boost::property<
            vertex_state_t, base::State *,
            boost::property<
                vertex_total_connection_attempts_t, unsigned long int,
                boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                                boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                boost::property<boost::vertex_rank_t, unsigned long int>>>>>,
        boost::property<boost::edge_weight_t, base::Cost>>;

    using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
    using Edge   = boost::graph_traits<Graph>::edge_descriptor;

    using RoadmapNeighbors = std::shared_ptr<ompl::NearestNeighbors<Vertex>>;
    using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;
    using ConnectionFilter   = std::function<bool(const Vertex &, const Vertex &)>;

    using pt   = std::vector<double>;
    using pvec = std::vector<std::vector<double>>;
    using label = std::vector<int>;

    // default constructor

    //SDCL() : base::Planner(nullptr) 
    //{
    //}
    /** \brief Constructor */
    SDCL(const base::SpaceInformationPtr &si, bool starStrategy = false,
         bool use_training = false, bool use_Gaussian = false);
    


    ~SDCL() override;

    void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

    /** \brief Set the connection strategy function (like K-nearest neighbors). */
    void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
    {
        connectionStrategy_ = connectionStrategy;
        userSetConnectionStrategy_ = true;
    }

    /** \brief Set the default connection strategy. */
    void setDefaultConnectionStrategy();

    /** \brief Set max neighbors (for KStrategy). */
    void setMaxNearestNeighbors(unsigned int k);
    unsigned int getMaxNearestNeighbors() const;

    void setConnectionFilter(const ConnectionFilter &connectionFilter)
    {
        connectionFilter_ = connectionFilter;
    }

    void getPlannerData(base::PlannerData &data) const override;

    void constructRoadmap(const base::PlannerTerminationCondition &ptc);
    void growRoadmap(double growTime);
    void growRoadmap(const base::PlannerTerminationCondition &ptc);
    void expandRoadmap(double expandTime);
    void expandRoadmap(const base::PlannerTerminationCondition &ptc);

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
    void clearQuery() override;
    void clear() override;

    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if (nn_ && nn_->size() == 0)
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        nn_ = std::make_shared<NN<Vertex>>();
        if (!userSetConnectionStrategy_)
            setDefaultConnectionStrategy();
        if (isSetup())
            setup();
    }

    void setup() override;

    const Graph &getRoadmap() const
    {
        return g_;
    }

    unsigned long int milestoneCount() const
    {
        return boost::num_vertices(g_);
    }

    unsigned long int edgeCount() const
    {
        return boost::num_edges(g_);
    }

    const RoadmapNeighbors &getNearestNeighbors()
    {
        return nn_;
    }
    
    // --------Add the public prototype for sampleAndSaveCollisionPoints-----------------
    bool sampleAndSaveCollisionPoints(base::State* workState, bool use_Gaussian);

protected:
    void freeMemory();
    Vertex addMilestone(base::State *state);
    void uniteComponents(Vertex m1, Vertex m2);
    bool sameComponent(Vertex m1, Vertex m2);
    void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);
    void expandRoadmap(const base::PlannerTerminationCondition &ptc,
                       std::vector<base::State *> &workStates);
    void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);
    bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                base::PathPtr &solution);
    ompl::base::Cost constructApproximateSolution(const std::vector<Vertex> &starts,
                                                  const std::vector<Vertex> &goals,
                                                  base::PathPtr &solution);
    bool addedNewSolution() const;
    base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);
    base::Cost costHeuristic(Vertex u, Vertex v) const;
    double distanceFunction(const Vertex a, const Vertex b) const
    {
        return si_->distance(stateProperty_[a], stateProperty_[b]);
    }

    // Planner progress
    std::string getIterationCount() const
    {
        return std::to_string(iterations_);
    }
    std::string getBestCost() const
    {
        return std::to_string(bestCost_.value());
    }
    std::string getMilestoneCountString() const
    {
        return std::to_string(milestoneCount());
    }
    std::string getEdgeCountString() const
    {
        return std::to_string(edgeCount());
    }

    bool starStrategy_{false};

    base::ValidStateSamplerPtr sampler_;
    base::StateSamplerPtr simpleSampler_;
    RoadmapNeighbors nn_;
    Graph g_;

    std::vector<Vertex> startM_;
    std::vector<Vertex> goalM_;

    boost::property_map<Graph, vertex_state_t>::type stateProperty_;
    boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;
    boost::property_map<Graph, vertex_successful_connection_attempts_t>::type successfulConnectionAttemptsProperty_;
    boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

    boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                         boost::property_map<Graph, boost::vertex_predecessor_t>::type>
        disjointSets_;

    ConnectionStrategy connectionStrategy_;
    ConnectionFilter connectionFilter_;
    bool userSetConnectionStrategy_{false};

    RNG rng_;
    bool addedNewSolution_{false};
    mutable std::mutex graphMutex_;
    base::OptimizationObjectivePtr opt_;

    // Planner progress
    unsigned long int iterations_{0};
    base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

    // No SVM or NLOpt references
    // ThunderSVM code commented out

    // Collision points / stats
    proof_stats stats_;
    std::shared_ptr<pvec> collisionPoints_;
    mutable std::mutex collisionPointsMutex_;

    double stddev_{0.0};
    bool use_training_{false};
    bool use_Gaussian_{false};
    std::vector<double> upper_bound_;
    std::vector<double> lower_bound_;
    unsigned int addedToTraining_{0};

    // ----------MLPACK GMM ---------------------------------

//#ifdef ENABLE_MLPACK
//    #include <mlpack/methods/gmm/gmm.hpp>

public:
    std::vector<std::vector<double>> collectRoadmapStates() const;
    std::vector<Vertex> findNarrowPassages(double clearanceThreshold, unsigned int minNeighborCount);
    void plotRoadmapScatter(const std::string &filename); //scatterplot void plotRoadmapScatter(const std::string &filename) const;
    //mlpack::gmm::GMM gmmModel_; // store best GMM if desired
    arma::gmm_diag buildSingleCovDiagGMM(const arma::mat &dataset, int k);
    void buildPerComponentGMMsDiag(int minK, int maxK);
    //void trainGMMArmadilloBIC(const std::vector<std::vector<double>> &data,
    //                        int minComponents, int maxComponents);
    //void plotGMMClusters(const std::string &filename);
    void plotPerComponentGMMClusters(const std::string &filename);
    void trainGMMArmadilloBIC(const std::vector<std::vector<double>>& data,
                              int param1,
                              int param2);
    void computePairwiseBD();
    void reportShortComponents(std::size_t maxSize);

private:
 // store the best model as an Armadillo gmm_diag:
    arma::gmm_diag armadilloDiagGMM_; 
    // one GMM per component ID
    //std::unordered_map<unsigned long, arma::gmm_diag> perComponentGMMs_;

    //std::unordered_map<unsigned long, arma::gmm_full> perComponentGMMs_;

    std::unordered_map<unsigned long, arma::gmm_diag> compGMMsDiag_;  // one diag GMM per component

//#endif
};

#endif  // SDCL_H
