#ifndef ___FEATURE_CELL_H___
#define ___FEATURE_CELL_H___

#include <ostream>

struct FeatureCell
{
    FeatureCell(void)
    : border(false),
      saw(0),
      orientation(0)
    {

    }

    FeatureCell(const bool b, const bool s, const uint8_t o)
        : border(b),
          saw(s),
          orientation(o)
    {

    }

    inline bool operator==(const FeatureCell& cell) const
    {
        return border == cell.border  &&  saw & cell.saw  &&  orientation & cell.orientation;
    }

    bool border;
    uint8_t saw;
    uint8_t orientation;
};

inline std::ostream& operator<<(std::ostream& os, const FeatureCell& cell)
{
    os << "FeatureCell: boder = " << cell.border << " saw = " << cell.saw << " orientation = " << cell.orientation;
    return os;
}

#endif
