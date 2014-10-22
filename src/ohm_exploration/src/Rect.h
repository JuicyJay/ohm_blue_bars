#ifndef ___RECT_H___
#define ___RECT_H___

#include <Eigen/Core>

class Rect
{
public:

    Rect(const unsigned int xValue = 0, const unsigned int yValue = 0,
         const unsigned int wValue = 0, const unsigned int hValue = 0)
        : _x(xValue),
          _y(yValue),
          _width(wValue),
          _height(hValue)
    {

    }

    Rect(const Eigen::Vector2u& leftTop, const Eigen::Vector2u& rightBottom)
        : _x(leftTop.x()),
          _y(leftTop.y()),
          _width(rightBottom.x() - _x),
          _height(rightBottom.y() - _y)
    {

    }

    inline bool isNull(void) const { return !_width && !_height; }
    inline unsigned int x(void) const { return _x; }
    inline unsigned int y(void) const { return _y; }
    inline unsigned int width(void) const { return _width; }
    inline unsigned int height(void) const { return _height; }
    inline Eigen::Vector2u pointLeftTop(void) const { return Eigen::Vector2u(_x, _y); }
    inline Eigen::Vector2u pointRightBottom(void) const { return Eigen::Vector2u(_x + _width, _y + _height); }

private:
    unsigned int _x;
    unsigned int _y;
    unsigned int _width;
    unsigned int _height;
};

#endif
