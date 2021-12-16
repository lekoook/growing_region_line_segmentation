#include <ros/ros.h>
#include "LineSegmenter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_segmentation");

    LineSegmenter segmenter;

    ros::spin();
    
    return 0;
}