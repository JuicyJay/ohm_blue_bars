#ifndef ___CELL_H___
#define ___CELL_H___

struct Cell
{
    Cell(void) : inspected(false), wall(false) { }

    bool inspected;
    bool wall;
};

#endif
