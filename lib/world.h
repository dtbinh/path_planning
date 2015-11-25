/**
 * simulator.h
 * Copyright 2015
 *
 *
 * Uber Advanced Technology Center
 * Pittsburgh, PA
 *
 * Control Instructions (who can see this code):
 * Confidential.  Not for public release
 *
 *
 * This notice must appear in all copies of this file and its derivatives.
 */

#ifndef WORLD_H_
#define WORLD_H_

#include "geometry.h"

#include <cstdint>
#include <vector>

typedef uint64_t guid;

namespace world
{
    struct corner
    {
        guid id;
        geometry::box_2d bounding_box;
        geometry::point_2d center;

        std::vector<guid> crosswalks;
    };

    struct crosswalk
    {
        guid id;
        geometry::line_segment_2d center_line;
        double width_m;

        guid corner_1;
        guid corner_2;

        guid signal_id;
    };

    struct cw_signal
    {
        enum state_type { STOP, GO, BLINK };

        guid id;
        state_type state;
    };

} // namespace world


#endif // WORLD_H_
