#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

typedef geometry_msgs::Point Point;

struct PolarPoint2D
{
    double distance = 0.0f;
    double theta = 0.0f;
    PolarPoint2D() {};
    PolarPoint2D(double distance, double theta) : distance(distance), theta(theta)
    {};
};

struct Line
{
    double xCoeff = 0;
    double yCoeff = 0;
    double constant = 0;
    Line() {};
    Line(double xCoeff, double yCoeff, double constant)
        : xCoeff(xCoeff), yCoeff(yCoeff), constant(constant)
    {};
};

struct LineSegment
{
    int startIdx;
    int endIdx;
    Point startPoint;
    Point endPoint;
    PolarPoint2D startPolarPoint;
    PolarPoint2D endPolarPoint;
    Line line;
    LineSegment() {};
    LineSegment(
        int startIdx,
        int endIdx,
        Point startPoint, 
        Point endPoint, 
        PolarPoint2D startPolarPoint, 
        PolarPoint2D endPolarPoint, 
        Line line) 
        : 
        startIdx(startIdx),
        endIdx(endIdx),
        startPoint(startPoint), 
        endPoint(endPoint), 
        startPolarPoint(startPolarPoint), 
        endPolarPoint(endPolarPoint), 
        line(line)
    {};
};

class LineSegmenter
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _lineMarkerPub;
    ros::Subscriber _scanSub;
    ros::Timer _computeTimer;
    int _seedSegPoints;
    int _segMinPoints;
    double _ptToLineThresh;
    double _ptToPtThresh;
    double _colThresh;
    double _updateFreq;
    double _minLen;
    bool _toCompute;

    void _scanCb(const sensor_msgs::LaserScanConstPtr& msg);
    void _timerCb(const ros::TimerEvent& event);
    PolarPoint2D _getPolar(const sensor_msgs::LaserScan& scanMsg, int index);
    Point _polar2Cart(PolarPoint2D polarPoint2D);
    double _pt2PtDist2D(Point point1, Point point2);
    double _pt2LineDist2D(Point point, Line line);
    Point _getPredictedPt(double pointBearing, Line line);
    Line _orthgLineFit(const sensor_msgs::LaserScan& scanMsg, int start, int end);
    std::vector<LineSegment> _generateSegments(const sensor_msgs::LaserScan& scanMsg);
    bool _generateSeed(const sensor_msgs::LaserScan& scanMsg, int start, int end, LineSegment& seed_);
    bool _growSeed(const sensor_msgs::LaserScan& scanMsg, LineSegment& seed);
    void _processOverlap(const sensor_msgs::LaserScan& scanMsg, std::vector<LineSegment>& segments);
    void _generateEndpoints(std::vector<LineSegment>& segments);

    // Visualization helpers
    int _lineId = 0;
    void _markLine(Line line, Point pt1, Point pt2, int id=-1, std::string ns="lines");
    void _markLine(Line line, double x, int id=-1, std::string ns="lines");
    void _clearLines(std::string ns="lines");

public:
    LineSegmenter();
};