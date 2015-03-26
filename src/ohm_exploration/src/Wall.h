/************************************************************************************************************
 * Class Wall: This class represents a wall on the occupancy grid. The wall class can be constructed by a
 * set of points. If you received walls as ROS message then use this class for a better handling. And of course
 * this class is still messy like.
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___WALL_H___
#define ___WALL_H___

#include "Line.h"

#include <vector>
#include <ostream>

#include <Eigen/Core>

#include <visualization_msgs/Marker.h>
#include <ohm_autonomy/Wall.h>

typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > PointVector;

class Wall
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Orientation {
        None  = 0,
        Up    = (1 << 0),
        Down  = (1 << 1),
        Left  = (1 << 2),
        Right = (1 << 3),
        All   = 0xff
    };

    Wall(void) : _valid(false) { }
    Wall(const PointVector& points);
    Wall(const ohm_autonomy::Wall& wall);

    inline const Line& model(void) const { return _model; }
    inline const PointVector& points(void) const { return _points; }
    inline unsigned int id(void) const { return _id; }
    inline const Eigen::Vector2f& center(void) const { return _center; }
    inline Eigen::Vector3f origin(void) const { return Eigen::Vector3f(_origin.x, _origin.y, _origin.z); }
    inline float resolution(void) const { return _resolution; }
    inline bool valid(void) const { return _valid; }
    inline float length(void) const { return _length; }
    inline Orientation orientation(void) const { return _orientation; }
    inline float distance(void) const { return _distance; }

    inline void setResolution(const float res) { _resolution = res; }
    inline void setOrigin(const geometry_msgs::Point& origin) { _origin = origin; }
    void setOrientation(const Orientation orientation);
    inline void setDistance(const float distance) { _distance = distance; }

    visualization_msgs::Marker getMarkerMessage(void) const;
    ohm_autonomy::Wall getWallMessage(void) const;

    /* compare operator for std::sort */
    bool operator()(const Eigen::Vector2i& left, const Eigen::Vector2i& right) const;
    inline bool operator<(const Wall& wall) const { return _distance < wall._distance; }

private:
    Line _model;
    PointVector _points;
    unsigned int _id;
    Eigen::Vector2f _center;
    float _resolution;
    geometry_msgs::Point _origin;
    Orientation _orientation;
    bool _valid;
    float _length;
    float _distance;

    static unsigned int s_id;
};


inline std::ostream& operator<<(std::ostream& os, const Wall& wall)
{
    os << "Wall:" << std::endl;
    os << "-----------------------------------" << std::endl;
    os << "ID         : " << wall.id() << std::endl;
    os << "Model      : " << wall.model() << std::endl;
    os << "Orientation: " << static_cast<int>(wall.orientation()) << std::endl;
    os << "Center     : " << wall.center().x() << ", " << wall.center().y() << std::endl;
    os << "Length     : " << wall.length() << std::endl;
    os << "Resolution : " << wall.resolution() << std::endl;
    os << "Valid      : " << wall.valid() << std::endl;

    return os;
}

#endif
