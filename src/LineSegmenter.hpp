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

struct ScanPoint
{
    PolarPoint2D polarPoint;
    Point cartesianPoint;
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
    int firstIdx;
    int lastIdx;
    ScanPoint firstPoint;
    ScanPoint lastPoint;
    Line line;
    LineSegment() {};
    LineSegment(
        int startIdx,
        int lastIdx,
        ScanPoint startPoint, 
        ScanPoint endPoint, 
        Line line) 
        : 
        firstIdx(startIdx),
        lastIdx(lastIdx),
        firstPoint(startPoint), 
        lastPoint(endPoint), 
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
    std::string _laserFrame;
    std::vector<ScanPoint> _scanPoints;
    std::vector<LineSegment> _segments;

    void _scanCb(const sensor_msgs::LaserScanConstPtr& msg);
    void _timerCb(const ros::TimerEvent& event);
    void _extractPoints(const sensor_msgs::LaserScan& scanMsg);
    Point _polar2Cart(PolarPoint2D polarPoint2D);
    double _pt2PtDist2D(Point point1, Point point2);
    double _pt2LineDist2D(Point point, Line line);
    Point _getPredictedPt(double pointBearing, Line line);
    Line _orthgLineFit(int start, int end);
    void _generateSegments();
    bool _generateSeed(int start, int end, LineSegment& seed_);
    bool _growSeed(LineSegment& seed);
    void _processOverlap();
    void _generateEndpoints();

    std::vector<ScanPoint> _filterFront();

    // Visualization helpers
    int _lineId = 0;
    void _markLine(Point pt1, Point pt2, int id=-1, std::string ns="lines");
    void _markLine(Line line, double x, int id=-1, std::string ns="lines");
    void _markLine(std::vector<ScanPoint> _scanPoints);
    void _clearLines(std::string ns="lines");

public:
    LineSegmenter();
};