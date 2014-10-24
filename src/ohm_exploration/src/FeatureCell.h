#ifndef ___FEATURE_CELL_H___
#define ___FEATURE_CELL_H___

#include <ostream>

struct FeatureCell
{
    FeatureCell(void)
    : border(false),
      saw(false),
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
        return border == cell.border && saw == cell.saw && orientation & cell.orientation;
    }

    bool border;
    bool saw;
    uint8_t orientation;
};

inline std::ostream& operator<<(std::ostream& os, const FeatureCell& cell)
{
    os << "FeatureCell: boder = " << cell.border << " saw = " << cell.saw << " orientation = " << cell.orientation;
    return os;
}

#endif
