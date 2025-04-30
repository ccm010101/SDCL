#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// -- Includes you had before:
#include <opencv2/opencv.hpp>
#include <map>
#include <tuple>
#include <mutex>
#include <vector>
#include <iostream>
#include <string>
#include <math.h>
#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scene_collision.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/rx/scene_fk.h>
#include <amino/rx/scene_win.h>
#include <amino/mat.h>

// -- Type aliases
typedef std::vector<std::vector<double>> pvec;
typedef std::vector<double>              pt;
typedef std::map<pt, std::vector<double>> dist_hash;
typedef std::map<pt, std::vector<double*>> point_hash;

// ---------------------------------------------------------------------------
//  Abstract Base Environment
// ---------------------------------------------------------------------------
class Environment {
public:
    Environment() {}
    virtual ~Environment() {}

    // Pure virtual => must be overridden
    virtual bool   isStateValid(pt point)                 = 0;
    virtual double penetrationDist(pt point, pt &close_pt)= 0;

    // Some getters
    pt     get_start() const { return start_; }
    pt     get_goal()  const { return goal_;  }
    pt     get_highs() const { return highs_; }
    pt     get_lows()  const { return lows_;  }
    int    get_dim()   const { return N_;     }
    double get_delta() const { return delta_; }

    void set_lows(pt &lows)   { lows_  = lows; }
    void set_highs(pt &highs) { highs_ = highs; }

protected:
    pt     goal_;
    pt     start_;
    pt     highs_;
    pt     lows_;
    int    N_{0};
    double delta_{0.0};
};

// ---------------------------------------------------------------------------
//  HyperBallEnvironment
//    (As in your code, it references 'inner_radius' and 'outter_radius'.)
// ---------------------------------------------------------------------------
class HyperBallEnvironment : public Environment {
public:
    HyperBallEnvironment(pt lows, pt highs, pt start, pt goal, int N);
    ~HyperBallEnvironment();

    bool   isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_pt) override;

private:
    double inner_radius;   // intentionally spelled as in your snippet
    double outter_radius;  // "outter" from your snippet
};

// ---------------------------------------------------------------------------
//  Concrete 2D Bitmap Environment
// ---------------------------------------------------------------------------
class Environment2D : public Environment
{
public:
    Environment2D(const std::string path, pt lows, pt highs, pt start, pt goal)
    {
        // Basic initialization
        lows_  = lows;
        highs_ = highs;
        start_ = start;
        goal_  = goal;
        N_     = 2;
        delta_ = 0.0;

        std::cout << "Environment2D loading image: " << path << std::endl;
        image = cv::imread(path, cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::cerr << "Warning: could not read image: " << path << std::endl;
        }
        row = image.rows;
        col = image.cols;
    }

    ~Environment2D() override
    {
        // If needed
        if (!image.empty())
            image.release();
        std::cout << "Environment2D destructor called.\n";
    }

    // Overrides
    bool isStateValid(pt point) override
    {
        // Example logic:
        int x = mapXPoint(point[0]);
        int y = mapYPoint(point[1]);
        return isStateValidLocal(cv::Point(x,y));
    }

    double penetrationDist(pt point, pt &close_point) override
    {
        // If valid => -1, else => 1
        if (isStateValid(point))
            return -1.0;
        return 1.0;
    }

    // Helpers
    bool isStateValidLocal(cv::Point p)
    {
        // e.g. check if pixel is "white"
        return isWhite(p);
    }

    int mapXPoint(double x)
    {
        // scale [lows_[0], highs_[0]] -> [0, col]
        if (highs_[0] - lows_[0] == 0) return 0; // avoid divide by zero
        return (int)((x - lows_[0]) / (highs_[0] - lows_[0]) * (col));
    }

    int mapYPoint(double y)
    {
        // scale [lows_[1], highs_[1]] -> [row, 0]
        if (highs_[1] - lows_[1] == 0) return 0; // avoid divide by zero
        return (int)((y - lows_[1]) / (highs_[1] - lows_[1]) * (-row) + row);
    }

private:
    bool isWhite(cv::Point p)
    {
        if (p.x < 0 || p.x >= col || p.y < 0 || p.y >= row)
        {
            // out of bounds => treat as collision or not valid
            return false;
        }
        cv::Vec3b color = image.at<cv::Vec3b>(p);
        int b = color[0], g = color[1], r = color[2];
        // e.g. threshold for white
        if (std::abs(b-255)<10 && std::abs(g-255)<10 && std::abs(r-255)<10)
            return true;
        return false;
    }

    cv::Mat image;
    int row{0}, col{0};
};

// ---------------------------------------------------------------------------
//  HighDofEnvironment
// ---------------------------------------------------------------------------
class HighDofEnvironment : public Environment {
public:
    HighDofEnvironment(pt start, pt goal, std::vector<std::string> &joints,
                       const char* scene_name, const char* path,
                       pt lows, pt highs,
                       double delta = 0.0);
    ~HighDofEnvironment();

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt& close_point) override;

    aa_rx_sg*      sg_{nullptr};
    aa_rx_fk*      fk_{nullptr};
    aa_rx_cl*      cl_{nullptr};
    aa_rx_cl_set*  cl_set_{nullptr};
    aa_rx_cl_dist* cl_dist_{nullptr};
    aa_rx_config_id* ids_{nullptr};
    aa_dvec*       config_{nullptr};
    std::mutex     scene_mutex;
    std::vector<std::tuple<std::string, std::string>> linkpair_list;

private:
    void setup_scene(std::vector<std::string>& joints, const char* scene_name, const char* path);
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);
    int  isStateValid_(pt point);
};

// ---------------------------------------------------------------------------
//  MultiRobotEnv
// ---------------------------------------------------------------------------
class MultiRobotEnv : public Environment
{
public:
    MultiRobotEnv(pt lows, pt highs, pt start, pt goal, pvec squares, double delta);
    ~MultiRobotEnv() override {}
    
    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt &close_point) override;

    // Additional helpers
    bool robotInSquare(int r, int s);
    bool lineIntersectSquare(int r1, int r2, int s);
    bool lineIntersectVerticalLine(pt p1, pt p2, double x, double y1, double y2);
    bool lineIntersectHorizontalLine(pt p1, pt p2, double y, double x1, double x2);

private:
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);

    std::vector<pt> squares_;
    pt              robots_;
    int             n_robots_{0};
};

// ---------------------------------------------------------------------------
//  PosScaleEnv
// ---------------------------------------------------------------------------
class PosScaleEnv : public Environment
{
public:
    PosScaleEnv(pt start, pt goal, std::vector<std::string> &joints,
                const char* scene_name, const char* path,
                pt lows, pt highs,
                double pos_scale,
                double delta = 0.0);

    ~PosScaleEnv();

    bool isStateValid(pt point) override;
    double penetrationDist(pt point, pt& close_point) override;

    aa_rx_sg*        sg_{nullptr};
    aa_rx_fk*        fk_{nullptr};
    aa_rx_cl*        cl_{nullptr};
    aa_rx_cl_set*    cl_set_{nullptr};
    aa_rx_cl_dist*   cl_dist_{nullptr};
    aa_rx_config_id* ids_{nullptr};
    aa_dvec*         config_{nullptr};

private:
    void  setup_scene(std::vector<std::string>& joints, const char* scene_name, const char* path);
    bool  outOfLimits(pt point);
    bool  outOfLimitsCollision(pt point);
    int   isStateValid_(pt point);
    pt    scale_point(pt point);

    double pos_scale_{1.0};
};

#endif // ENVIRONMENT_H

