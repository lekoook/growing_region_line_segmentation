#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

struct SeedSegment
{
    double startX = 0;
    double endX = 0;
    double startY = 0;
    double endY = 0;
};

struct Line
{
    double xCoeff = 0;
    double yCoeff = 0;
    double constant = 0;
};

class LineSegmenter
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber _scanSub;
    int _seedSegPoints;
    int _segMinPoints;
    double _ptToLineThresh;
    double _ptToPtThresh;

    void _scanCb(const sensor_msgs::LaserScanConstPtr& msg);
    geometry_msgs::Point _polar2Cart(double r, double theta);
    SeedSegment _getFirstSeedSegment(sensor_msgs::LaserScan& scanMsg);

public:
    LineSegmenter();
};