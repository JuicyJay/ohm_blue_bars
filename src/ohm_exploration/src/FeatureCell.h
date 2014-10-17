#ifndef ___FEATURE_CELL_H___
#define ___FEATURE_CELL_H___

struct FeatureCell
{
    FeatureCell(void)
    : outerFace(false),
      orientation(0)
    {

    }

    bool outerFace;
    uint8_t orientation;
};

#endif
