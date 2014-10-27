#ifndef ___RAY_H___
#define ___RAY_H___

#include <Eigen/Core>

class Ray
{
public:
    Ray(const Eigen::Vector2f origin, const Eigen::Vector2f v, const float length);

    bool next(void);
    const Eigen::Vector2i& position(void) const { return _position; }

private:
    Eigen::Vector2f _origin;
    Eigen::Vector2f _v;
    float _length;

    Eigen::Vector2i _position;
    Eigen::Vector2f _delta;
    Eigen::Vector2i _step;
    Eigen::Vector2f _sideDistance;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
