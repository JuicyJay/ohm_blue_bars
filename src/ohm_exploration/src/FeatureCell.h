#ifndef ___FEATURE_CELL_H___
#define ___FEATURE_CELL_H___

struct FeatureCell
{
    FeatureCell(void)
    : border(false),
      orientation(0)
    {

    }

    bool border;
    uint8_t orientation;
};

#endif
