#ifndef ___WALL_H___
#define ___WALL_H___

#include "Line.h"

#include <vector>
#include <ostream>

#include <Eigen/Core>

#include <visualization_msgs/Marker.h>

typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > PointVector;

class Wall
{
public:

    enum Orientation {
        None  = 0,
        Up    = (1 << 0),
        Down  = (1 << 1),
        Left  = (1 << 2),
        Right = (1 << 3),
        All   = 0xff
    };

    Wall(void) { }
    Wall(const PointVector& points);
    Wall(const Wall& wall);

    inline const Line& model(void) const { return _model; }
    inline const PointVector& points(void) const { return _points; }
    inline unsigned int id(void) const { return _id; }
    inline const Eigen::Vector2f& center(void) const { return _center; }
    inline Eigen::Vector3f origin(void) const { return Eigen::Vector3f(_origin.x, _origin.y, _origin.z); }
    inline float resolution(void) const { return _resolution; }
    inline bool valid(void) const { return _valid; }
    inline void setResolution(const float res) { _resolution = res; }
    inline void setOrigin(const geometry_msgs::Point& origin) { _origin = origin; }
    inline void setOrientation(const Orientation orientation) { _orientation = orientation; }
    inline Orientation orientation(void) const { return _orientation; }

    visualization_msgs::Marker getMarkerMessage(void) const;

    /* compare operator for std::sort */
    bool operator()(const Eigen::Vector2i& left, const Eigen::Vector2i& right) const;

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

    static unsigned int s_id;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


inline std::ostream& operator<<(std::ostream& os, const Wall& wall)
{
    os << "Wall:" << std::endl;
    os << "-----------------------------------" << std::endl;
    os << "Model: " << wall.model() << std::endl;
    os << "Valid: " << wall.valid() << std::endl;

    return os;
}

#endif
