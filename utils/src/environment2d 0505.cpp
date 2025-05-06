#include "environment.h"
#include <iostream>      // if needed
#include <opencv2/opencv.hpp> // for cv::Point, cv::Mat, etc.

Environment2D::Environment2D(const std::string &path,
                             pt lows,
                             pt highs,
                             pt start,
                             pt goal)
{
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

Environment2D::~Environment2D()
{
    if (!image.empty())
        image.release();
    std::cout << "Environment2D destructor called.\n";
}

bool Environment2D::isStateValid(pt point)
{
    int x = mapXPoint(point[0]);
    int y = mapYPoint(point[1]);
    return isStateValidLocal(cv::Point(x, y));
}

double Environment2D::penetrationDist(pt point, pt &close_point)
{
    // Example: if valid => -1, else => 1
    if (isStateValid(point))
        return -1.0;
    return 1.0;
}

bool Environment2D::isStateValidLocal(cv::Point p)
{
    return isWhite(p);
}

int Environment2D::mapXPoint(double x)
{
    // scale [lows_[0], highs_[0]] -> [0, col]
    if (highs_[0] - lows_[0] == 0) return 0;
    return static_cast<int>((x - lows_[0]) / (highs_[0] - lows_[0]) * (col));
}

int Environment2D::mapYPoint(double y)
{
    // scale [lows_[1], highs_[1]] -> [row, 0]
    if (highs_[1] - lows_[1] == 0) return 0;
    return static_cast<int>((y - lows_[1]) / (highs_[1] - lows_[1]) * (-row) + row);
}

bool Environment2D::isWhite(cv::Point p)
{
    if (p.x < 0 || p.x >= col || p.y < 0 || p.y >= row)
    {
        // out of image bounds => treat as collision or not valid
        return false;
    }
    cv::Vec3b color = image.at<cv::Vec3b>(p);
    int b = color[0], g = color[1], r = color[2];
    // e.g. threshold for white
    if (std::abs(b-255)<10 && std::abs(g-255)<10 && std::abs(r-255)<10)
        return true;
    return false;
}

