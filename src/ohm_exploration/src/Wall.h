#ifndef ___WALL_H___
#define ___WALL_H___

#include "Line.h"

#include <vector>
#include <ostream>

#include <Eigen/Core>

typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > PointVector;

class Wall
{
public:
    Wall(void) { }
    Wall(const PointVector& points);
    Wall(const Wall& wall);

    inline const Line& model(void) const { return _model; }
    inline const PointVector& points(void) const { return _points; }

private:
    Line _model;
    PointVector _points;
};


inline std::ostream& operator<<(std::ostream& os, const Wall& wall)
{
    os << "Wall:" << std::endl;
    os << "-----------------------------------" << std::endl;
    os << "Model: " << wall.model() << std::endl;
//    os << "Points: " << std::endl;
//
//    for (std::vector<Eigen::Vector2i>::const_iterator point(wall.points().begin());
//         point < wall.points().end();
//         ++point)
//    {
//        os << *point;
//    }
//
//    os << std::endl;

    return os;
}

#endif
