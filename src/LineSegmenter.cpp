#include "LineSegmenter.hpp"
#include <visualization_msgs/Marker.h>
#include <growing_region_line_segmentation/LineSegmentsList.h>

typedef growing_region_line_segmentation::LineSegment LineMsg;
typedef growing_region_line_segmentation::LineSegmentsList LinesListMsg;

LineSegmenter::LineSegmenter() : _toCompute(false)
{
    std::string scanTopic;
    ros::param::param<std::string>("~scan_topic", scanTopic, "scan");
    ros::param::param<int>("~seed_segment_points", _seedSegPoints, 10);
    ros::param::param<int>("~segment_min_points", _segMinPoints, 20);
    ros::param::param<double>("~point_to_line_threshold_m", _ptToLineThresh, 0.01f);
    ros::param::param<double>("~growth_outlier_threshold_m", _outlierThresh, 0.1f);
    ros::param::param<int>("~growth_outlier_max_count", _outlierMaxCount, 3);
    ros::param::param<double>("~point_to_point_threshold_m", _ptToPtThresh, 0.1f);
    ros::param::param<double>("~collinear_threshold_rad", _colThresh, 0.1f);
    ros::param::param<double>("~collinear_point_segment_threshold_m", _colDistThresh, 0.1f);
    ros::param::param<double>("~update_frequency", _updateFreq, 10.0f);
    ros::param::param<double>("~min_line_length_m", _minLen, 0.1f);
    ros::param::param<double>("~max_line_length_m", _maxLen, 3.0f);
    ros::param::param<double>("~search_radius_m", _searchRadius, 3.0f);

    if (_maxLen <= 0)
    {
        _maxLen = std::numeric_limits<double>::infinity();
    }
    if (_searchRadius <= 0)
    {
        _searchRadius = std::numeric_limits<double>::infinity();
    }
    
    _linesPub = _nh.advertise<LinesListMsg>("segmented_lines", 1);
    _lineMarkerPub = _nh.advertise<visualization_msgs::Marker>("line_marker", 0);
    _scanSub = _nh.subscribe(scanTopic, 1, &LineSegmenter::_scanCb, this);
    _computeTimer = _nh.createTimer(ros::Duration(1.0 / _updateFreq), &LineSegmenter::_timerCb, this);
};

void LineSegmenter::_scanCb(const sensor_msgs::LaserScanConstPtr& msg)
{
    if (_toCompute)
    {
        _toCompute = false;
        _extractPoints(*msg);
        _generateSegments();
        _pubSegments(msg->header.stamp);
    }
}

void LineSegmenter::_markLine(std::vector<ScanPoint> _scanPoints)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = _laserFrame;
    marker.header.stamp = ros::Time();
    marker.ns = "points_sequence";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.001;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    
    for (int i = 0; i < (int)_scanPoints.size(); i++)
    {
        marker.points.push_back(_scanPoints[i].cartesianPoint);

    }
    _lineMarkerPub.publish(marker);
}

void LineSegmenter::_timerCb(const ros::TimerEvent& event)
{
    _toCompute = true;
}

void LineSegmenter::_extractPoints(const sensor_msgs::LaserScan& scanMsg)
{
    _laserFrame = scanMsg.header.frame_id;
    _scanPoints.clear();
    double searchRange = std::min<double>((double)scanMsg.range_max, _searchRadius);
    for (int i = 0; i < (int)scanMsg.ranges.size(); i++)
    {
        if (!std::isinf(scanMsg.ranges[i]) && (scanMsg.ranges[i] <= searchRange))
        {
            ScanPoint pt;
            pt.polarPoint.distance = scanMsg.ranges[i];
            pt.polarPoint.theta = scanMsg.angle_min + (i * scanMsg.angle_increment);
            pt.cartesianPoint = _polar2Cart(pt.polarPoint);
            _scanPoints.push_back(pt);
        }
    }
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

double LineSegmenter::_pt2LineSegmentDist2D(Point point, LineSegment lineSegment)
{
    double dummy;
    return _pt2LineSegmentDist2D(point, lineSegment, dummy);
}

double LineSegmenter::_pt2LineSegmentDist2D(Point point, LineSegment lineSegment, double& pointPos_)
{
    double x = point.x;
    double y = point.y;
    lineSegment.generateEndpoints(); 
    double x1 = lineSegment.startPoint.x;
    double y1 = lineSegment.startPoint.y;
    double x2 = lineSegment.endPoint.x;
    double y2 = lineSegment.endPoint.y;
    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    pointPos_ = -1.0f;
    if (len_sq != 0) //in case of 0 length line
    {
        pointPos_ = dot / len_sq;
    }

    double xx, yy;

    if (pointPos_ < 0)
    {
        xx = x1;
        yy = y1;
    }
    else if (pointPos_ > 1)
    {
        xx = x2;
        yy = y2;
    }
    else
    {
        xx = x1 + pointPos_ * C;
        yy = y1 + pointPos_ * D;
    }

    double dx = x - xx;
    double dy = y - yy;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<bool> LineSegmenter::_orOutlierMask(std::vector<bool>& first, std::vector<bool>& second)
{
    size_t cap = std::max(first.size(), second.size());
    size_t less = std::min(first.size(), second.size());
    std::vector<bool> comb(cap, false);
    for (int i = 0; i < (int)less; i++)
    {
        comb[i] = (first[i] || second[i]);
    }
    return comb;
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

Line LineSegmenter::_orthgLineFit(int start, int end, std::vector<bool>& outlierMask)
{
    double sX = 0.0f, sY = 0.0f, mX = 0.0f, mY = 0.0f, sXX = 0.0, sXY = 0.0, sYY = 0.0;
    double xCoeff = 0.0f, yCoeff = 0.0f, constant = 0.0f;
    int n = 0;

    //Calculate sums of X and Y and their means.
    for (int i = start; i < end; i++)
    {
        if (!outlierMask[i])
        {
            Point pt = _scanPoints[i].cartesianPoint;
            sX += pt.x;
            sY += pt.y;
            n++;
        }
    }
    mX = sX / (double)n;
    mY = sY / (double)n;

    //Calculate sum of X squared, sum of Y squared and sum of X * Y
    //(components of the scatter matrix)
    for (int i = start; i < end; i++)
    {
        if (!outlierMask[i])
        {
            Point pt = _scanPoints[i].cartesianPoint;
            sXX += (pt.x - mX) * (pt.x - mX);
            sXY += (pt.x - mX) * (pt.y - mY);
            sYY += (pt.y - mY) * (pt.y - mY);
        }
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

bool LineSegmenter::_collinearOverlap(const LineSegment& first, const LineSegment& second)
{
    // First check if the two segments are collinear. Return immediately if not.
    double firstTheta = atan2(-first.line.xCoeff, first.line.yCoeff);
    double secondTheta = atan2(-second.line.xCoeff, second.line.yCoeff);
    double deltaTheta = fabs(firstTheta - secondTheta);
    bool collinear = deltaTheta < _colThresh;
    if (!collinear)
    {
        return false;
    }

    /* 
    Check if any of the end points of one segment is within the two endpoints of the other segment and if that endpoint 
    is close to the other segment (closest distance).
    */
    double pointPos = 0.0f;
    double dist1 = _pt2LineSegmentDist2D(first.endPoint, second, pointPos);
    bool pt1 = (dist1 < _colDistThresh && (pointPos >=0) && (pointPos <=1));
    double dist2 = _pt2LineSegmentDist2D(first.startPoint, second, pointPos);
    bool pt2 = (dist2 < _colDistThresh && (pointPos >=0) && (pointPos <=1));
    return pt1 || pt2;
}

void LineSegmenter::_generateSegments()
{
    _segments.clear();
    if ((int)_scanPoints.size() < _segMinPoints)
    {
        return; // Don't bother if too little points.
    }
    
    int seedId = 0, fullId = 0, procId = 0;
    // Generate seed segments and grow each one into a full line segment.
    for (int start = 0; start < ((int)_scanPoints.size() - _segMinPoints); start++)
    {
        int end = start + _seedSegPoints;

        // Generate a possible seed segment and try to grow it.
        LineSegment seed;
        seed.outlierMask = std::vector<bool>(_scanPoints.size(), false);
        if (_generateSeed(start, end, seed))
        {
            _markLine(seed.firstPoint.cartesianPoint, seed.lastPoint.cartesianPoint, 0.0, seedId++, "seed_segments");
            // Grow this seed into a full line segment.
            if (_growSeed(seed))
            {
                start = seed.lastIdx;
                _segments.push_back(seed);
                _markLine(seed.firstPoint.cartesianPoint, seed.lastPoint.cartesianPoint, 0.0, fullId++, "raw_segments");
            }
        }
    }

    // Process the overlapping segments.
    _processOverlap();
    for (int i = 0; i < (int)_segments.size(); i++)
    {
        auto seg = _segments[i];
        // Remove lines that are too short or too long.
        auto segLen = _pt2PtDist2D(seg.firstPoint.cartesianPoint, seg.lastPoint.cartesianPoint);
        if (segLen < _minLen || segLen > _maxLen)
        {
            _segments.erase(_segments.begin() + i);
        }
    }

    // Generate the endpoints for each line.
    _generateEndpoints();
    for (auto seg : _segments)
    {
        _markLine(seg.startPoint, seg.endPoint, _segmentMean(seg), procId++, "processed");
    }
}

bool LineSegmenter::_generateSeed(int start, int end, LineSegment& seed_)
{
    Line bestLine = _orthgLineFit(start, end, seed_.outlierMask);
    bool foundSeed = true;
    Point prevPt = _scanPoints[start].cartesianPoint;
    for (int k = start; k < end; k++)
    {
        Point pt = _scanPoints[k].cartesianPoint;
        Point pPt = _getPredictedPt(_scanPoints[k].polarPoint.theta, bestLine);
        double pt2PtDist = _pt2PtDist2D(pt, pPt);
        double pt2LineDist = _pt2LineDist2D(pt, bestLine);
        double pt2PrevPtDist = _pt2PtDist2D(pt, prevPt);

        if (pt2PtDist > _ptToPtThresh || pt2LineDist > _ptToLineThresh || pt2PrevPtDist > _ptToPtThresh)
        {
            foundSeed = false;
            break;
        }

        prevPt = pt;

    }
    if (foundSeed)
    {
        seed_.firstIdx = start;
        seed_.lastIdx = end-1;
        seed_.firstPoint = _scanPoints[start];
        seed_.lastPoint = _scanPoints[end - 1];
        seed_.line = bestLine;
        seed_.generateEndpoints();
    }
    return foundSeed;
}

bool LineSegmenter::_growSeed(LineSegment& seed)
{
    int pb = seed.firstIdx;
    int pf = seed.lastIdx;
    int outlierCnt = 0;

    // TODO: Refit and grow both ends of the segment together in a one iteration instead of two.

    // Refit and grow the end.
    pf = pf + 1;
    while (pf < (int)_scanPoints.size() && outlierCnt < _outlierMaxCount)
    {
        if (_pt2LineSegmentDist2D(_scanPoints[pf].cartesianPoint, seed) > _outlierThresh)
        {
            seed.outlierMask[pf] = true;
            outlierCnt++;
        }
        else
        {
            // Refit the current segment to the new point.
            auto newLine = _orthgLineFit(pb, pf, seed.outlierMask);
            seed.line = newLine;
            seed.lastIdx = pf;
            seed.lastPoint = _scanPoints[pf];
            seed.generateEndpoints();
            outlierCnt = 0;
        }
        pf++;
    }

    outlierCnt = 0;

    // Refit and grow the start.
    pb = pb - 1;
    while (pb >= 0 && outlierCnt < _outlierMaxCount)
    {
        if (_pt2LineSegmentDist2D(_scanPoints[pb].cartesianPoint, seed) > _outlierThresh)
        {
            seed.outlierMask[pb] = true;
            outlierCnt++;
        }
        else
        {
            auto newLine = _orthgLineFit(pb, pf, seed.outlierMask);
            seed.line = newLine;
            seed.firstIdx = pb;
            seed.firstPoint = _scanPoints[pb];
            seed.generateEndpoints();
            outlierCnt = 0;
        }
        pb--;
    }

    double lineLen = _pt2PtDist2D(seed.firstPoint.cartesianPoint, seed.lastPoint.cartesianPoint);
    int linePoints = seed.lastIdx - seed.firstIdx + 1;
    if (lineLen >= _minLen && linePoints >= _segMinPoints)
    {
        return true;
    }
    return false;
}

void LineSegmenter::_processOverlap()
{
    // Refit all the collinear segments
    for (int i = 0; i < ((int)_segments.size() - 1); i++)
    {
        auto& first = _segments[i];
        for (int j = i + 1; j < (int)_segments.size(); j++)
        {
            auto second = _segments[j];
            if (_collinearOverlap(first, second))
            {
                int newStart = std::min<double>(first.firstIdx, second.firstIdx);
                auto outlier = _orOutlierMask(first.outlierMask, second.outlierMask);
                auto newBestLine = _orthgLineFit(newStart, second.lastIdx, outlier);
                first.outlierMask = outlier;
                first.line = newBestLine;
                first.firstIdx = newStart;
                first.lastIdx = second.lastIdx;
                first.firstPoint = _scanPoints[first.firstIdx];
                first.lastPoint = _scanPoints[first.lastIdx];
                _segments.erase(_segments.begin() + j);
                j--;
            }
        }
    }

    // Properly separate non-collinear segments.
    // TODO: Fix this not separating properly due to 'mixing' indices.
    for (int i = 0; i < ((int)_segments.size() - 1); i++)
    {
        auto& firstSeg = _segments[i];
        int firstEnd = firstSeg.lastIdx;
        auto& secondSeg = _segments[i + 1];
        int secondStart = secondSeg.firstIdx;
        
        if (secondStart <= firstEnd)
        {
            int k = secondStart;
            for (; k <= firstEnd; k++)
            {
                Point ptK = _scanPoints[k].cartesianPoint;
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
        firstSeg.lastIdx = firstEnd;
        firstSeg.lastPoint = _scanPoints[firstEnd];
        firstSeg.line = _orthgLineFit(firstSeg.firstIdx, firstSeg.lastIdx + 1, firstSeg.outlierMask);
        secondSeg.firstIdx = secondStart;
        secondSeg.firstPoint = _scanPoints[secondStart];
        secondSeg.line = _orthgLineFit(secondSeg.firstIdx, secondSeg.lastIdx + 1, secondSeg.outlierMask);
    }
}

void LineSegmenter::_generateEndpoints()
{
    for (auto& seg : _segments)
    {
        seg.generateEndpoints();
    }
}

double LineSegmenter::_segmentMean(LineSegment& segment)
{
    int samples = 0;
    double sum = 0.0f;
    for (int i = segment.firstIdx; i <= segment.lastIdx; i++)
    {
        if (!segment.outlierMask[i])
        {
            double dist = _pt2LineDist2D(_scanPoints[i].cartesianPoint, segment.line) * 1000.0f;
            sum += (dist * dist) / 1000.0f;
            samples++;
        }
    }
    return (sum / (double)samples);
}

void LineSegmenter::_pubSegments(ros::Time rosTime)
{
    LinesListMsg list;
    list.header.frame_id = _laserFrame;
    list.header.seq = _headerSeq++;
    list.header.stamp = rosTime;
    for (auto& seg : _segments)
    {
        LineMsg line;
        line.endpoint_1 = seg.startPoint;
        line.endpoint_2 = seg.endPoint;
        line.mean_sq_m = _segmentMean(seg);
        list.lines.push_back(line);
    }
    _linesPub.publish(list);
}

void LineSegmenter::_markLine(Point pt1, Point pt2, double mean, int id, std::string ns)
{
    bool showEndpoints = false;
    visualization_msgs::Marker marker;
    marker.header.frame_id = _laserFrame;
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
    if (ns == "seed_segments")
    {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.scale.x = 0.07;
    }
    else if (ns == "previous")
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    }
    else if (ns == "processed")
    {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.scale.x = 0.01;
        showEndpoints = true;
    }
    else
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.scale.x = 0.01;
    }
    marker.points.push_back(pt1);
    marker.points.push_back(pt2);
    _lineMarkerPub.publish(marker);

    if (showEndpoints)
    {
        visualization_msgs::Marker ep1;
        ep1.header.frame_id = _laserFrame;
        ep1.header.stamp = marker.header.stamp;
        ep1.ns = "end_points";
        ep1.id = id + 10000;
        ep1.type = visualization_msgs::Marker::SPHERE;
        ep1.action = visualization_msgs::Marker::ADD;
        ep1.pose.position = pt1;
        ep1.pose.orientation.x = 0.0;
        ep1.pose.orientation.y = 0.0;
        ep1.pose.orientation.z = 0.0;
        ep1.pose.orientation.w = 1.0;
        ep1.scale.x = 0.03;
        ep1.scale.y = 0.03;
        ep1.scale.z = 0.03;
        ep1.color.a = 1.0; // Don't forget to set the alpha!
        ep1.color.r = 0.0;
        ep1.color.g = 1.0;
        ep1.color.b = 0.0;
        _lineMarkerPub.publish(ep1);
        visualization_msgs::Marker ep2;
        ep2.header.frame_id = _laserFrame;
        ep2.header.stamp = marker.header.stamp;
        ep2.ns = "end_points";
        ep2.id = id + 20000;
        ep2.type = visualization_msgs::Marker::SPHERE;
        ep2.action = visualization_msgs::Marker::ADD;
        ep2.pose.position = pt2;
        ep2.pose.orientation.x = 0.0;
        ep2.pose.orientation.y = 0.0;
        ep2.pose.orientation.z = 0.0;
        ep2.pose.orientation.w = 1.0;
        ep2.scale.x = 0.03;
        ep2.scale.y = 0.03;
        ep2.scale.z = 0.03;
        ep2.color.a = 1.0; // Don't forget to set the alpha!
        ep2.color.r = 1.0;
        ep2.color.g = 0.0;
        ep2.color.b = 0.0;
        _lineMarkerPub.publish(ep2);

        visualization_msgs::Marker meanMarker;
        meanMarker.header.frame_id = _laserFrame;
        meanMarker.header.stamp = ros::Time();
        meanMarker.ns = "means";
        meanMarker.id = id;
        meanMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        meanMarker.scale.z = 0.05;
        meanMarker.color.a = 1.0; // Don't forget to set the alpha!
        meanMarker.color.r = 0.0;
        meanMarker.color.g = 1.0;
        meanMarker.color.b = 0.0;
        meanMarker.pose.position.x = (pt1.x + pt2.x) / 2.0f;
        meanMarker.pose.position.y = (pt1.y + pt2.y) / 2.0f;
        meanMarker.pose.position.z = 0.2;
        meanMarker.text = std::to_string(mean);
        _lineMarkerPub.publish(meanMarker);
    }
}

void LineSegmenter::_markLine(Line line, double x, int id, std::string ns)
{
    geometry_msgs::Point pt1, pt2;
    pt1.x = -x;
    pt1.y = -((line.xCoeff * pt1.x) + line.constant) / line.yCoeff;
    pt2.x = x;
    pt2.y = -((line.xCoeff * pt2.x) + line.constant) / line.yCoeff;
    _markLine(pt1, pt2, 0.0, id, ns);
}

void LineSegmenter::_clearLines()
{
    Line line;
    _markLine(line, 0.0, -1, "");
}
