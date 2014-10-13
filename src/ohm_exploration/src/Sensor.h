#ifndef ___SENSOR_H___
#define ___SENSOR_H___

#include <cmath>

class Sensor
{
public:

    Sensor(void)
    : _beamAngleH(60.0f * M_PI / 180.0f),
      _beamAngleV(40.0f * M_PI / 180.0f),
      _range(1.0f)
    {

    }

    inline float beamAngleH(void) const { return _beamAngleH; }
    inline float beamAngleV(void) const { return _beamAngleV; }
    inline float range(void) const { return _range; }

private:
    float _beamAngleH;
    float _beamAngleV;
    float _range;
};

#endif
