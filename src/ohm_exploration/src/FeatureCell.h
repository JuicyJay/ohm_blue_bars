#ifndef ___FEATURE_CELL_H___
#define ___FEATURE_CELL_H___

struct FeatureCell
{
    enum eOrientation {
        None  = 0,
        Up    = (1 << 0),
        Down  = (1 << 1),
        Left  = (1 << 2),
        Right = (1 << 3)
    };

    FeatureCell(void)
    : outerFace(false),
      orientation(None)
    {

    }

    bool outerFace;
    uint8_t orientation;
};

#endif
