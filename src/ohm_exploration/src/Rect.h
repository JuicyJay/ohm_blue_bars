/************************************************************************************************************
 * Class Rect: Is a simple representation of a rectangle that makes the life easier.
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
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

    Rect(const Eigen::Vector2i& leftTop, const Eigen::Vector2i& rightBottom)
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
    inline Eigen::Vector2i pointLeftTop(void) const { return Eigen::Vector2i(_x, _y); }
    inline Eigen::Vector2i pointRightBottom(void) const { return Eigen::Vector2i(_x + _width, _y + _height); }

private:
    unsigned int _x;
    unsigned int _y;
    unsigned int _width;
    unsigned int _height;
};

#endif
