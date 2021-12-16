#include "LineSegmenter.hpp"

LineSegmenter::LineSegmenter()
{
    std::string scanTopic;
    ros::param::param<std::string>("scan_topic", scanTopic, "scan");
    ros::param::param<int>("seed_segment_points", _seedSegPoints, 5);
    ros::param::param<int>("segment_min_points", _segMinPoints, 7);
    ros::param::param<double>("point_to_line_threshold", _ptToLineThresh, 0.01);
    ros::param::param<double>("point_to_point_threshold", _ptToPtThresh, 0.1);
    
    _scanSub = _nh.subscribe(scanTopic, 1, &LineSegmenter::_scanCb, this);
};

void LineSegmenter::_scanCb(const sensor_msgs::LaserScanConstPtr& msg)
{
    sensor_msgs::LaserScan msgCopy = *msg;
    auto ss = _getFirstSeedSegment(msgCopy);
}

geometry_msgs::Point LineSegmenter::_polar2Cart(double r, double theta)
{
    geometry_msgs::Point pt;
    pt.x = r * cos(theta);
    pt.y = r * sin(theta);
    pt.z = 0;
    return pt;
}

SeedSegment LineSegmenter::_getFirstSeedSegment(sensor_msgs::LaserScan& scanMsg)
{
    for (int i = 0; i < scanMsg.ranges.size() - _segMinPoints; i++)
    {
        auto start = _polar2Cart(scanMsg.ranges[i], scanMsg.angle_min + (i * scanMsg.angle_increment));
        int endIdx = i + _seedSegPoints;
        auto end = _polar2Cart(scanMsg.ranges[endIdx], scanMsg.angle_min + (i * scanMsg.angle_increment));

        
    }
}
