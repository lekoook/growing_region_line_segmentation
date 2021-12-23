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
    Point startPoint;
    Point endPoint;
    Line line;
    std::vector<bool> outlierMask;
    LineSegment() : outlierMask(std::vector<bool>(false)) {};
    LineSegment(
        int startIdx,
        int lastIdx,
        ScanPoint firstPoint, 
        ScanPoint lastPoint, 
        Line line) 
        : 
        firstIdx(startIdx),
        lastIdx(lastIdx),
        firstPoint(firstPoint), 
        lastPoint(lastPoint), 
        line(line),
        outlierMask(std::vector<bool>(false))
    {};
    void generateEndpoints()
    {
        double xC = line.xCoeff;
        double yC = line.yCoeff;
        double c = line.constant;
        double xC2 = xC * xC;
        double yC2 = yC * yC;
        double xCyC = xC * yC;
        double xCc = xC * c;
        double yCc = yC * c;
        double denom = (xC * xC) + (yC * yC);
        // Calculate start point of segment.
        double fX = ((yC2 * firstPoint.cartesianPoint.x) - (xCyC * firstPoint.cartesianPoint.y) - (xCc)) / denom;
        double fY = ((xC2 * firstPoint.cartesianPoint.y) - (xCyC * firstPoint.cartesianPoint.x) - (yCc)) / denom;
        startPoint.x = fX;
        startPoint.y = fY;
        // Calculate end point of segment.
        double lX = ((yC2 * lastPoint.cartesianPoint.x) - (xCyC * lastPoint.cartesianPoint.y) - (xCc)) / denom;
        double lY = ((xC2 * lastPoint.cartesianPoint.y) - (xCyC * lastPoint.cartesianPoint.x) - (yCc)) / denom;
        endPoint.x = lX;
        endPoint.y = lY;
    };
};

class LineSegmenter
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _linesPub;
    ros::Publisher _lineMarkerPub;
    ros::Subscriber _scanSub;
    ros::Timer _computeTimer;
    uint32_t _headerSeq;
    int _seedSegPoints;
    int _segMinPoints;
    int _outlierMaxCount;
    double _ptToLineThresh;
    double _outlierThresh;
    double _ptToPtThresh;
    double _colThresh;
    double _colDistThresh;
    double _updateFreq;
    double _minLen;
    double _maxLen;
    double _searchRadius;
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
    double _pt2LineSegmentDist2D(Point point, LineSegment lineSegment);
    double _pt2LineSegmentDist2D(Point point, LineSegment lineSegment, double& pointPos_);
    std::vector<bool> _orOutlierMask(std::vector<bool>& first, std::vector<bool>& second);
    Point _getPredictedPt(double pointBearing, Line line);
    Line _orthgLineFit(int start, int end, std::vector<bool>& outlierMask);
    bool _collinearOverlap(const LineSegment& first, const LineSegment& second);
    bool _lineSegmentsIntersect(LineSegment& first, LineSegment& second);
    void _generateSegments();
    bool _generateSeed(int start, int end, LineSegment& seed_);
    bool _growSeed(LineSegment& seed);
    void _processOverlap();
    void _generateEndpoints();
    double _segmentMean(LineSegment& segment);
    void _pubSegments(ros::Time rosTime);

    // Visualization helpers
    int _lineId = 0;
    void _markLine(Point pt1, Point pt2, double mean, int id=-1, std::string ns="lines");
    void _markLine(Line line, double x, int id=-1, std::string ns="lines");
    void _markLine(std::vector<ScanPoint> _scanPoints);
    void _clearLines();

public:
    LineSegmenter();
};