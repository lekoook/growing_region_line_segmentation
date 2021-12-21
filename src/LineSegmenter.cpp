#include "LineSegmenter.hpp"
#include <visualization_msgs/Marker.h>

LineSegmenter::LineSegmenter() : _toCompute(false)
{
    std::string scanTopic;
    ros::param::param<std::string>("~scan_topic", scanTopic, "scan");
    ros::param::param<int>("~seed_segment_points", _seedSegPoints, 10);
    ros::param::param<int>("~segment_min_points", _segMinPoints, 20);
    ros::param::param<double>("~point_to_line_threshold_m", _ptToLineThresh, 0.01f);
    ros::param::param<double>("~point_to_point_threshold_m", _ptToPtThresh, 0.1f);
    ros::param::param<double>("~collinear_threshold_rad", _colThresh, 0.1f);
    ros::param::param<double>("~update_frequency", _updateFreq, 10.0f);
    ros::param::param<double>("~min_line_length_m", _minLen, 1.0f);
    
    _lineMarkerPub = _nh.advertise<visualization_msgs::Marker>("line_marker", 0);
    _scanSub = _nh.subscribe(scanTopic, 1, &LineSegmenter::_scanCb, this);
    _computeTimer = _nh.createTimer(ros::Duration(1.0 / _updateFreq), &LineSegmenter::_timerCb, this);
};

void LineSegmenter::_scanCb(const sensor_msgs::LaserScanConstPtr& msg)
{
    if (_toCompute)
    {
        _toCompute = false;
        _clearLines("seed_segments");
        _clearLines("raw_segments");
        _clearLines("processed");
        sensor_msgs::LaserScan msgCopy = *msg;

        auto segments = _generateSegments(msgCopy);
    }
}

void LineSegmenter::_timerCb(const ros::TimerEvent& event)
{
    _toCompute = true;
}

PolarPoint2D LineSegmenter::_getPolar(const sensor_msgs::LaserScan& scanMsg, int index)
{
    return PolarPoint2D(scanMsg.ranges[index], scanMsg.angle_min + (index * scanMsg.angle_increment));
}

Point LineSegmenter::_polar2Cart(PolarPoint2D polarPoint2D)
{
    geometry_msgs::Point pt;
    pt.x = polarPoint2D.distance * cos(polarPoint2D.theta);
    pt.y = polarPoint2D.distance * sin(polarPoint2D.theta);
    pt.z = 0;
    return pt;
}

double LineSegmenter::_pt2PtDist2D(Point point1, Point point2)
{
    double xDiff = point1.x - point2.x;
    double yDiff = point1.y - point2.y;
    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double LineSegmenter::_pt2LineDist2D(Point point, Line line)
{
    double numerator = fabs((line.xCoeff * point.x) + (line.yCoeff * point.y) + (line.constant));
    double denominator = sqrt((line.xCoeff * line.xCoeff) + (line.yCoeff * line.yCoeff));
    return numerator / denominator;
}

Point LineSegmenter::_getPredictedPt(double pointBearing, Line line)
{
    double cosBear = cos(pointBearing);
    double sinBear = sin(pointBearing);
    double pX = -(line.constant * cosBear) / ((line.xCoeff * cosBear) + (line.yCoeff * sinBear));
    double pY = -(line.constant * sinBear) / ((line.xCoeff * cosBear) + (line.yCoeff * sinBear));
    Point pt;
    pt.x = pX;
    pt.y = pY;
    pt.z = 0.0f;
    return pt;
}

Line LineSegmenter::_orthgLineFit(const sensor_msgs::LaserScan& scanMsg, int start, int end)
{
    double sX = 0.0f, sY = 0.0f, mX = 0.0f, mY = 0.0f, sXX = 0.0, sXY = 0.0, sYY = 0.0;
    double xCoeff = 0.0f, yCoeff = 0.0f, constant = 0.0f;
    int n = 0;

    //Calculate sums of X and Y and their means.
    for (int i = start; i < end; i++)
    {
        Point pt = _polar2Cart(_getPolar(scanMsg, i));
        sX += pt.x;
        sY += pt.y;
        n++;
    }
    mX = sX / (double)n;
    mY = sY / (double)n;

    //Calculate sum of X squared, sum of Y squared and sum of X * Y
    //(components of the scatter matrix)
    for (int i = start; i < end; i++)
    {
        Point pt = _polar2Cart(_getPolar(scanMsg, i));
        sXX += (pt.x - mX) * (pt.x - mX);
        sXY += (pt.x - mX) * (pt.y - mY);
        sYY += (pt.y - mY) * (pt.y - mY);
    }

    // Find line equation coefficients and constant.
    bool isVertical = sXY == 0 && sXX < sYY;
    bool isHorizontal = sXY == 0 && sXX > sYY;
    bool isIndeterminate = sXY == 0 && sXX == sYY;
    double slope = std::numeric_limits<double>::quiet_NaN();
    double intercept = std::numeric_limits<double>::quiet_NaN();

    if (isVertical) // Conventional math
    {
        xCoeff = 1.0f;
        yCoeff = 0.0f;
        constant = mX;
    }
    else if (isHorizontal)
    {
        xCoeff = 0.0f;
        yCoeff = 1.0f;
        constant = mY;
    }
    else if (isIndeterminate)
    {
        xCoeff = std::numeric_limits<double>::quiet_NaN();
        yCoeff = std::numeric_limits<double>::quiet_NaN();
        constant = std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
        slope = (sYY - sXX + sqrt((sYY - sXX) * (sYY - sXX) + 4.0 * sXY * sXY)) / (2.0 * sXY);
        intercept = mY - slope * mX;
        double normFactor = (intercept >= 0.0 ? 1.0 : -1.0) * sqrt(slope * slope + 1.0);
        xCoeff = slope / normFactor;
        yCoeff = -1.0 / normFactor;
        constant = intercept / normFactor;
    }

    Line line;
    line.xCoeff = xCoeff;
    line.yCoeff = yCoeff;
    line.constant = constant;
    return line;
}

std::vector<LineSegment> LineSegmenter::_generateSegments(const sensor_msgs::LaserScan& scanMsg)
{
    int seedId = 0, fullId = 0, procId = 0;
    std::vector<LineSegment> segments;
    // Generate seed segments and grow each one into a full line segment.
    for (int start = 0; start < ((int)scanMsg.ranges.size() - _segMinPoints); start++)
    {
        int end = start + _seedSegPoints;
        // TODO: Skip inf points for now. Assuming there is not inf points after the first point from 'start' to 'end' points.
        bool skip = false;
        for (int i = start; i < end; i++)
        {
            if (std::isinf(scanMsg.ranges[i]))
            {
                skip = true;
                break;
            }
        }
        if (skip)
        {
            continue;
        }

        // Generate a possible seed segment and try to grow it.
        LineSegment seed;
        if (_generateSeed(scanMsg, start, end, seed))
        {
            _markLine(seed.line, seed.startPoint, seed.endPoint, seedId++, "seed_segments");
            // Grow this seed into a full line segment.
            if (_growSeed(scanMsg, seed))
            {
                start = seed.endIdx;
                segments.push_back(seed);
                _markLine(seed.line, seed.startPoint, seed.endPoint, fullId++, "raw_segments");
            }
        }
    }

    // Process the overlapping segments.
    _processOverlap(scanMsg, segments);
    for (int i = 0; i < (int)segments.size(); i++)
    {
        auto seg = segments[i];
        // Remove lines that are too short.
        if (_pt2PtDist2D(seg.startPoint, seg.endPoint) < _minLen)
        {
            segments.erase(segments.begin() + i);
        }
    }

    // Generate the endpoints for each line.
    _generateEndpoints(segments);
    for (auto seg : segments)
    {
        _markLine(seg.line, seg.startPoint, seg.endPoint, procId++, "processed");
    }
    
    return segments;
}

bool LineSegmenter::_generateSeed(const sensor_msgs::LaserScan& scanMsg, int start, int end, LineSegment& seed_)
{
    Line bestLine = _orthgLineFit(scanMsg, start, end);
    bool foundSeed = true;
    for (int k = start; k < end; k++)
    {
        Point pt = _polar2Cart(_getPolar(scanMsg, k));
        Point pPt = _getPredictedPt(scanMsg.angle_min + (k * scanMsg.angle_increment), bestLine);
        double pt2PtDist = _pt2PtDist2D(pt, pPt);
        double pt2LineDist = _pt2LineDist2D(pt, bestLine);

        if (pt2PtDist > _ptToPtThresh)
        {
            foundSeed = false;
            break;
        }

        if (pt2LineDist > _ptToLineThresh)
        {
            foundSeed = false;
            break;
        }
    }
    if (foundSeed)
    {
        PolarPoint2D startPolar = _getPolar(scanMsg, start);
        PolarPoint2D endPolar = _getPolar(scanMsg, end - 1);
        Point startPt = _polar2Cart(startPolar);
        Point endPt = _polar2Cart(endPolar);
        seed_ = LineSegment(start, end - 1, startPt, endPt, startPolar, endPolar, bestLine);
    }
    return foundSeed;
}

bool LineSegmenter::_growSeed(const sensor_msgs::LaserScan& scanMsg, LineSegment& seed)
{
    int pb = seed.startIdx;
    int pf = seed.endIdx + 1;

    // TODO: Refit and grow both ends of the segment together in a one iteration instead of two.

    // Refit and grow the end.
    while (pf < scanMsg.ranges.size())
    {
        if (std::isinf(scanMsg.ranges[pf]) || (_pt2LineDist2D(_polar2Cart(_getPolar(scanMsg, pf)), seed.line) >= _ptToLineThresh))
        {
            break;
        }
        else
        {
            // Refit the current segment to the new point.
            auto newLine = _orthgLineFit(scanMsg, pb, pf);
            seed.line = newLine;
            pf++;
        }
    }
    int o = pf;
    pf--; // Reset pf back to the largest possible index value.
    seed.endIdx = pf;
    seed.endPolarPoint = _getPolar(scanMsg, pf);
    seed.endPoint = _polar2Cart(seed.endPolarPoint);

    // Refit and grow the start.
    pb = pb - 1;
    while (pb >= 0)
    {
        if (std::isinf(scanMsg.ranges[pb]) || (_pt2LineDist2D(_polar2Cart(_getPolar(scanMsg, pb)), seed.line) >= _ptToLineThresh))
        {
            break;
        }
        else
        {
            auto newLine = _orthgLineFit(scanMsg, pb, pf);
            seed.line = newLine;
            pb--;
        }
    }
    pb++; // Reset pb back to the lowest possible index value.
    seed.startIdx = pb;
    seed.startPolarPoint = _getPolar(scanMsg, pb);
    seed.startPoint = _polar2Cart(seed.startPolarPoint);

    double lineLen = _pt2PtDist2D(seed.startPoint, seed.endPoint);
    int linePoints = pf - pb + 1;
    if (lineLen >= _minLen && linePoints >= _segMinPoints)
    {
        return true;
    }
    return false;
}

void LineSegmenter::_processOverlap(const sensor_msgs::LaserScan& scanMsg, std::vector<LineSegment>& segments)
{
    // Refit all the collinear segments
    for (int i = 0; i < ((int)segments.size() - 1); i++)
    {
        auto& first = segments[i];
        for (int j = i + 1; j < segments.size(); j++)
        {
            auto second = segments[j];
            if (first.endIdx >= second.startIdx)
            {
                double firstTheta = atan2(-first.line.xCoeff, first.line.yCoeff);
                double secondTheta = atan2(-second.line.xCoeff, second.line.yCoeff);
                double deltaTheta = fabs(firstTheta - secondTheta);

                if (deltaTheta < _colThresh)
                {
                    int newStart = std::min<double>(first.startIdx, second.startIdx);
                    auto newBestLine = _orthgLineFit(scanMsg, newStart, second.endIdx);
                    first.line = newBestLine;
                    first.startIdx = newStart;
                    first.endIdx = second.endIdx;
                    first.startPolarPoint = _getPolar(scanMsg, first.startIdx);
                    first.endPolarPoint = _getPolar(scanMsg, first.endIdx);
                    first.startPoint = _polar2Cart(first.startPolarPoint);
                    first.endPoint = _polar2Cart(first.endPolarPoint);
                    segments.erase(segments.begin() + j);
                    j--;
                }
            }
        }
    }

    // Properly separate non-collinear segments.
    for (int i = 0; i < ((int)segments.size() - 1); i++)
    {
        auto& firstSeg = segments[i];
        int firstEnd = firstSeg.endIdx;
        auto& secondSeg = segments[i + 1];
        int secondStart = secondSeg.startIdx;
        
        if (secondStart <= firstEnd)
        {
            int k = secondStart;
            for (; k <= firstEnd; k++)
            {
                Point ptK = _polar2Cart(_getPolar(scanMsg, k));
                double dik = _pt2LineDist2D(ptK, firstSeg.line);
                double djk = _pt2LineDist2D(ptK, secondSeg.line);
                if (dik > djk)
                {
                    break;
                }
            }
            firstEnd = k - 1;
            secondStart = k;
        }
        else
        {
            break;
        }

        // Refit both segments.
        firstSeg.endIdx = firstEnd;
        firstSeg.endPolarPoint = _getPolar(scanMsg, firstEnd);
        firstSeg.endPoint = _polar2Cart(firstSeg.endPolarPoint);
        firstSeg.line = _orthgLineFit(scanMsg, firstSeg.startIdx, firstSeg.endIdx + 1);
        secondSeg.startIdx = secondStart;
        secondSeg.startPolarPoint = _getPolar(scanMsg, secondStart);
        secondSeg.startPoint = _polar2Cart(secondSeg.startPolarPoint);
        secondSeg.line = _orthgLineFit(scanMsg, secondSeg.startIdx, secondSeg.endIdx + 1);
    }
}

void LineSegmenter::_generateEndpoints(std::vector<LineSegment>& segments)
{
    for (auto& seg : segments)
    {
        double xC = seg.line.xCoeff;
        double yC = seg.line.yCoeff;
        double c = seg.line.constant;
        double xC2 = xC * xC;
        double yC2 = yC * yC;
        double xCyC = xC * yC;
        double xCc = xC * c;
        double yCc = yC * c;
        double denom = (xC * xC) + (yC * yC);
        // Calculate new first point.
        double fX = ((yC2 * seg.startPoint.x) - (xCyC * seg.startPoint.y) - (xCc)) / denom;
        double fY = ((xC2 * seg.startPoint.y) - (xCyC * seg.startPoint.x) - (yCc)) / denom;
        seg.startPoint.x = fX;
        seg.startPoint.y = fY;
        // Calculate new last point.
        double lX = ((yC2 * seg.endPoint.x) - (xCyC * seg.endPoint.y) - (xCc)) / denom;
        double lY = ((xC2 * seg.endPoint.y) - (xCyC * seg.endPoint.x) - (yCc)) / denom;
        seg.endPoint.x = lX;
        seg.endPoint.y = lY;
    }
}

void LineSegmenter::_markLine(Line line, Point pt1, Point pt2, int id, std::string ns)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    if (id < 0)
    {
        marker.action = visualization_msgs::Marker::DELETEALL;
    }
    else
    {
        marker.action = visualization_msgs::Marker::ADD;
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.points.push_back(pt1);
    marker.points.push_back(pt2);
    _lineMarkerPub.publish(marker);
}

void LineSegmenter::_markLine(Line line, double x, int id, std::string ns)
{
    geometry_msgs::Point pt1, pt2;
    pt1.x = -x;
    pt1.y = -((line.xCoeff * pt1.x) + line.constant) / line.yCoeff;
    pt2.x = x;
    pt2.y = -((line.xCoeff * pt2.x) + line.constant) / line.yCoeff;
    _markLine(line, pt1, pt2, id, ns);
}

void LineSegmenter::_clearLines(std::string ns)
{
    Line line;
    _markLine(line, 0.0, -1, ns);
}
