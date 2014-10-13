#include "Wall.h"
#include "LeastSquare.h"

Wall::Wall(const PointVector& points)
    : _points(points)
{
    LeastSquare::estimateLine(_points, _model);
}

Wall::Wall(const Wall& wall)
    : _model(wall._model),
      _points(wall._points)
{

}
