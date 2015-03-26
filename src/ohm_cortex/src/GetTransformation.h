#ifndef ___GET_TRANSFORMATION_H___
#define ___GET_TRANSFORMATION_H___

#include <Eigen/Geometry>
#include <tf/transform_listener.h>

class GetTransformation
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static GetTransformation* instance(void);

    const Eigen::Vector3f& position(void) const { return _position; }
    const Eigen::Quaternionf& orientation(void) const { return _orientation; }

    bool lookUpTransform(const std::string& from, const std::string& to);
    bool waitAndLookUpTransform(const std::string& from, const std::string& to, const float sec);

private:
    GetTransformation(void);

    Eigen::Vector3f       _position;
    Eigen::Quaternionf    _orientation;
    tf::TransformListener _listener;

    static GetTransformation* _instance;
};

#endif
