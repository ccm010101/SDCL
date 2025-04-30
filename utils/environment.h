#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <string>
#include <iostream>
#include <algorithm> // for std::min if needed
#include <opencv2/opencv.hpp> // if you want to do image stuff
#include <cmath>      // e.g., sqrt
#include <amino.h>    // if needed for HighDof, etc.
#include <amino/rx/rxtype.h>
#include <amino/rx/scene_collision.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/rx/scene_fk.h>
#include <amino/rx/scene_win.h>
#include <amino/mat.h>

// ---------------------------------------------------------------------------
// Some type aliases
// ---------------------------------------------------------------------------
using pt   = std::vector<double>;
using pvec = std::vector<std::vector<double>>;

// ---------------------------------------------------------------------------
//  Base Abstract Environment
// ---------------------------------------------------------------------------
class Environment
{
public:
    Environment() = default;
    virtual ~Environment() = default;

    virtual bool   isStateValid(pt point)                 = 0;
    virtual double penetrationDist(pt point, pt &close_pt)= 0;

    pt get_lows()  const { return lows_; }
    pt get_highs() const { return highs_; }
    int get_dim()  const { return N_; }
    double get_delta() const { return delta_; }
    pt get_start() const { return start_; }
    pt get_goal()  const { return goal_; }


protected:
    pt    start_, goal_, highs_, lows_;
    int   N_{0};
    double delta_{0.0};
};

// HighDofEnvironment
class HighDofEnvironment : public Environment
{
public:
    HighDofEnvironment(pt start, pt goal,
                       std::vector<std::string> &joints,
                       const char* scene_name,
                       const char* path,
                       pt lows,
                       pt highs,
                       double delta);

    ~HighDofEnvironment() override;

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_point) override;

private:
    void setup_scene(std::vector<std::string> &joints,
                     const char* scene_name,
                     const char* path);

    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);
    int  isStateValid_(pt point);

    // The Amino pointers:
    aa_rx_sg*        sg_{nullptr};
    aa_rx_fk*        fk_{nullptr};
    aa_rx_cl*        cl_{nullptr};
    aa_rx_cl_set*    cl_set_{nullptr};
    aa_rx_cl_dist*   cl_dist_{nullptr};
    aa_rx_config_id* ids_{nullptr};
    aa_dvec*         config_{nullptr};
};


// ---------------------------------------------------------------------------
//  HyperBallEnvironment
// ---------------------------------------------------------------------------

class HyperBallEnvironment : public Environment
{
public:
    HyperBallEnvironment(pt lows, pt highs, pt start, pt goal, int N);
    ~HyperBallEnvironment() override;

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_point) override;

private:
    double inner_radius_{0.0};
    double outer_radius_{0.0};
};

// ---------------------------------------------------------------------------
// Example: A 2D environment (fully inline, for demonstration)
// ---------------------------------------------------------------------------
class Environment2D : public Environment
{
public:
    Environment2D(const std::string &path,
                  pt lows, pt highs, pt start, pt goal);
    ~Environment2D() override;

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_point) override;

private:
    bool isStateValidLocal(cv::Point p);
    bool isWhite(cv::Point p);
    int  mapXPoint(double x);
    int  mapYPoint(double y);

    cv::Mat image;
    int row{0}, col{0};
};

// environment.h

class MultiRobotEnv : public Environment
{
public:
    MultiRobotEnv(pt lows, pt highs, pt start, pt goal,
                  pvec squares, double delta);
    ~MultiRobotEnv() override {}

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_point) override;

private:
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);

    bool robotInSquare(int r, int s);
    bool lineIntersectSquare(int r1, int r2, int s);
    bool lineIntersectVerticalLine(pt p1, pt p2,
                                   double x, double y1, double y2);
    bool lineIntersectHorizontalLine(pt p1, pt p2,
                                     double y, double x1, double x2);

    pvec squares_;
    pt   robots_;
    int  n_robots_{0};
};

// environment.h (example snippet)

class PosScaleEnv : public Environment
{
public:
    PosScaleEnv(pt start, pt goal,
                std::vector<std::string> &joints,
                const char* scene_name,
                const char* path,
                pt lows, pt highs,
                double pos_scale,
                double delta = 0.0);

    ~PosScaleEnv() override;

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_point) override;

private:
    void setup_scene(std::vector<std::string> &joints,
                     const char* scene_name,
                     const char* path);
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);
    int  isStateValid_(pt point);
    pt   scale_point(pt point);

    // Amino pointers, if needed:
    aa_rx_sg*        sg_{nullptr};
    aa_rx_fk*        fk_{nullptr};
    aa_rx_cl*        cl_{nullptr};
    aa_rx_cl_set*    cl_set_{nullptr};
    aa_rx_cl_dist*   cl_dist_{nullptr};
    aa_rx_config_id* ids_{nullptr};
    aa_dvec*         config_{nullptr};

    double pos_scale_{1.0};
};

#endif // ENVIRONMENT_H

