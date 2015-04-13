#include "Partition.h"
#include "Target.h"

int Partition::s_id = 0;

Partition::Partition(void)
    : _initialized(false),
      _id(-1),
      _xMin(0.0f),
      _yMin(0.0f),
      _xMax(0.0f),
      _yMax(0.0f)
{

}

Partition::Partition(const float xMin, const float yMin, const float size)
    : _initialized(true),
      _id(++s_id),
      _xMin(xMin),
      _yMin(yMin),
      _xMax(xMin + size),
      _yMax(yMin + size)
{

}

Partition::Partition(const Eigen::Vector2f& center, const float size)
    : _initialized(true),
      _id(++s_id),
      _xMin(center.x() - size * 0.5f),
      _yMin(center.y() - size * 0.5f),
      _xMax(center.x() + size * 0.5f),
      _yMax(center.y() + size * 0.5f)
{

}

visualization_msgs::Marker Partition::getMarkerMsg(void) const
{
    visualization_msgs::Marker msg;

    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time();
    msg.ns              = "";
    msg.id              = _id;
    msg.type            = visualization_msgs::Marker::LINE_LIST;
    msg.action          = visualization_msgs::Marker::ADD;

    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    msg.scale.x = 0.1;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 1.0;
    msg.color.b = 1.0;


    /* Set the borderline. */
    msg.points.resize(8);

    msg.points[0].x = _xMin;
    msg.points[0].y = _yMin;
    msg.points[0].z = 0.0;
    msg.points[1].x = _xMax;
    msg.points[1].y = _yMin;
    msg.points[1].z = 0.0;

    msg.points[2].x = _xMax;
    msg.points[2].y = _yMin;
    msg.points[2].z = 0.0;
    msg.points[3].x = _xMax;
    msg.points[3].y = _yMax;
    msg.points[3].z = 0.0;

    msg.points[4].x = _xMax;
    msg.points[4].y = _yMax;
    msg.points[4].z = 0.0;
    msg.points[5].x = _xMin;
    msg.points[5].y = _yMax;
    msg.points[5].z = 0.0;

    msg.points[6].x = _xMin;
    msg.points[6].y = _yMax;
    msg.points[6].z = 0.0;
    msg.points[7].x = _xMin;
    msg.points[7].y = _yMin;
    msg.points[7].z = 0.0;

    return msg;
}
