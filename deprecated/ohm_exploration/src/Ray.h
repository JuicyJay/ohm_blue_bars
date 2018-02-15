/************************************************************************************************************
 * Class Ray. This represents a ray in a 2D array. The users can walk easily through the array in a for-loop.
 * The class depends on the DDA algorithm. I red them in a "how programm a game" tutorial. The game was DOOM.
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___RAY_H___
#define ___RAY_H___

#include <Eigen/Core>

class Ray
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Ray(const Eigen::Vector2f origin, const Eigen::Vector2f v, const float length);

    bool next(void);
    const Eigen::Vector2i& position(void) const { return _position; }

private:
    Eigen::Vector2f _origin;
    Eigen::Vector2f _v;
    float           _length;

    Eigen::Vector2i _position;
    Eigen::Vector2f _delta;
    Eigen::Vector2i _step;
    Eigen::Vector2f _sideDistance;
};

#endif
