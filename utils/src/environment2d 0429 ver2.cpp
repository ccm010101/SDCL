
#include "environment.h"

#include <cmath> // for abs, etc.
#include <iostream>
#include <opencv2/opencv.hpp> 

using namespace cv; // optional if you like

Environment2D::Environment2D(const std::string &path,
                             pt lows,
                             pt highs,
                             pt start,
                             pt goal)
{
    // Basic bookkeeping
    lows_  = std::move(lows);
    highs_ = std::move(highs);
    start_ = std::move(start);
    goal_  = std::move(goal);
    N_     = 2;   // dimension is 2D
    delta_ = 0.0; // set as needed

    std::cout << "Loading 2D image from: " << path << std::endl;
    image = imread(path, IMREAD_COLOR); // or IMREAD_GRAYSCALE, as you prefer

    if (image.empty())
    {
        std::cout << "Could not read the image: " << path << std::endl;
        exit(1);
    }
    row = (double)image.rows;
    col = (double)image.cols;
}

Environment2D::~Environment2D()
{
    image.release();  // or any cleanup
}

bool Environment2D::isStateValid(pt point)
{
    // map to image coords
    int point_x = mapXPoint(point[0]);
    int point_y = mapYPoint(point[1]);

    return isStateValidLocal(Point(point_x, point_y));
}

bool Environment2D::isStateValidLocal(Point point)
{
    if (isWhite(point))
        return true;
    else
        return false;
}

int Environment2D::mapXPoint(double x)
{
    // e.g. scale [lows_[0], highs_[0]] -> [0, col]
    return (int)((x - lows_[0]) / (highs_[0] - lows_[0]) * col);
}

int Environment2D::mapYPoint(double y)
{
    // e.g. scale [lows_[1], highs_[1]] -> [row, 0] if inverted
    return (int)((y - lows_[1]) / (highs_[1] - lows_[1]) * (-row) + row);
}

double Environment2D::penetrationDist(pt point, pt& close_point)
{
    // If the point is valid => -1
    // If in collision => 1
    if (isStateValid(point))
        return -1.0;
    else
        return 1.0;
}

bool Environment2D::isWhite(Point pt)
{
    if (pt.x < 0 || pt.x >= image.cols || pt.y < 0 || pt.y >= image.rows)
    {
        // out of image bounds => treat as collision or something
        // or treat as not white
        return false;
    }

    Vec3b color = image.at<Vec3b>(pt); // BGR
    int b = color[0];
    int g = color[1];
    int r = color[2];

    // White with some tolerance
    if (std::abs(b - 255) < 10 && std::abs(g - 255) < 10 && std::abs(r - 255) < 10)
        return true;
    else
        return false;
}

