#include "environment.h" // Must be quoted or correct path
#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scene_collision.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/rx/scene_fk.h>
#include <amino/rx/scene_win.h>
#include <amino/mat.h>

#include <iostream>
#include <vector>
#include <string>

HighDofEnvironment::HighDofEnvironment(pt start,
                                       pt goal,
                                       std::vector<std::string> &joints,
                                       const char* scene_name,
                                       const char* path,
                                       pt lows,
                                       pt highs,
                                       double delta)
{
    start_ = start;
    goal_  = goal;
    lows_  = lows;
    highs_ = highs;
    delta_ = delta;
    N_     = (int)start.size();

    setup_scene(joints, scene_name, path);
}

void HighDofEnvironment::setup_scene(std::vector<std::string> &joints,
                                     const char* scene_name,
                                     const char* path)
{
    // 1) Create scene graph
    sg_ = aa_rx_sg_create(); // from <amino/rx/scenegraph.h>
    aa_rx_dl_sg_at(path, scene_name, sg_, "");
    aa_rx_sg_init(sg_);

    // 2) Forward kinematics
    fk_ = aa_rx_fk_malloc(sg_);

    // 3) Collision
    aa_rx_sg_cl_init(sg_);
    cl_ = aa_rx_cl_create(sg_);

    // 4) Collision set
    cl_set_ = aa_rx_cl_set_create(sg_);

    // 5) Collision dist
    cl_dist_ = aa_rx_cl_dist_create(cl_);

    // 6) Setup the config vector
    config_ = new aa_dvec[1];
    config_->len  = aa_rx_sg_config_count(sg_);
    config_->data = new double[config_->len];
    config_->inc  = 1;

    // 7) Retrieve config IDs for each joint
    ids_ = new aa_rx_config_id[N_];
    for (int i = 0; i < N_; i++)
    {
        ids_[i] = aa_rx_sg_config_id(sg_, joints[i].c_str());
    }

    // 8) Initialize the config data with start_ values
    for (int i = 0; i < config_->len; i++)
    {
        // If i < N_, then set the joint; if not, maybe 0 or something
        if (i < N_)
            config_->data[ids_[i]] = start_[i];
        else
            config_->data[i] = 0.0;
    }

    // 9) For debugging, do an initial collision check
    aa_rx_fk_all(fk_, config_);
    aa_rx_cl_set_clear(cl_set_);
    int res = aa_rx_cl_check(cl_,
                             aa_rx_fk_cnt(fk_),
                             aa_rx_fk_data(fk_),
                             aa_rx_fk_ld(fk_),
                             cl_set_);

    aa_rx_cl_allow_set(cl_, cl_set_);

    std::cout << "Scene setup done! Collision check result = " << res << std::endl;
}

HighDofEnvironment::~HighDofEnvironment()
{
    // Destroy all your references
    if (cl_dist_)  aa_rx_cl_dist_destroy(cl_dist_);
    if (cl_set_)   aa_rx_cl_set_destroy(cl_set_);
    if (cl_)       aa_rx_cl_destroy(cl_);
    if (fk_)       aa_rx_fk_destroy(fk_);
    if (sg_)       aa_rx_sg_destroy(sg_);

    // Freed arrays
    if (ids_)      delete [] ids_;

    if (config_)
    {
        if (config_->data) delete [] config_->data;
        delete [] config_;
    }
}

bool HighDofEnvironment::outOfLimits(pt point)
{
    for (int i = 0; i < N_; i++)
    {
        if (point[i] < lows_[i] || point[i] > highs_[i])
            return true;
    }
    return false;
}

bool HighDofEnvironment::outOfLimitsCollision(pt point)
{
    for (int i = 0; i < N_; i++)
    {
        // If 'delta_' extends the valid region?
        if (point[i] < lows_[i] - delta_ || point[i] > highs_[i] + delta_)
            return true;
    }
    return false;
}

bool HighDofEnvironment::isStateValid(pt point)
{
    // If there's a delta for out-of-limits collision check
    if (delta_ > 0)
    {
        if (outOfLimitsCollision(point))
            return true;  // e.g. treat out-of-limits as "valid"? Or maybe false?

        if (outOfLimits(point))
            return false;
    }

    // Then do the real collision check
    int res = isStateValid_(point);
    if (res == 0) // 0 => collision-free //    return (res == 0);
        return true;
    else
        return false;
}

int HighDofEnvironment::isStateValid_(pt point)
{
    // Create a temporary config
    aa_dvec* cfg = new aa_dvec[1];
    cfg->len  = N_;
    cfg->data = new double[N_];
    cfg->inc  = 1;

    // Copy point into the config
    for (int i = 0; i < N_; i++)
    {
        cfg->data[ids_[i]] = point[i];
    }

    // Create a local fk, local cl
    aa_rx_fk* tmp_fk = aa_rx_fk_malloc(sg_);
    aa_rx_cl* tmp_cl = aa_rx_cl_create(sg_);

    // Use the existing allow set
    aa_rx_cl_allow_set(tmp_cl, cl_set_);

    // Compute forward kinematics
    aa_rx_fk_all(tmp_fk, cfg);

    // Check collision
    int res = aa_rx_cl_check(tmp_cl,
                             aa_rx_fk_cnt(tmp_fk),
                             aa_rx_fk_data(tmp_fk),
                             aa_rx_fk_ld(tmp_fk),
                             NULL);

    // Cleanup
    delete [] cfg->data;
    delete [] cfg;
    aa_rx_fk_destroy(tmp_fk);
    aa_rx_cl_destroy(tmp_cl);

    return res; // 0 => no collision
}

double HighDofEnvironment::penetrationDist(pt point, pt &close_point)
{
    // For now, returns 0. Could implement a real distance measure
    return 0.0;
}
