/**
 * Copyright 2015
 *
 * Uber Advanced Technology Center
 * Pittsburgh, PA
 *
 * Control Instructions (who can see this code):
 * Confidential.  Not for public release
 *
 * This notice must appear in all copies of this file and its derivatives.
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

namespace geometry
{
    struct point_2d
    {
        double x;
        double y;
    };

    struct pose_2d
    {
        point_2d position;
        double heading_rad;
    };

    struct box_2d
    {
        point_2d upper_left;
        point_2d lower_right;
    };

    struct line_segment_2d
    {
        point_2d pt_1;
        point_2d pt_2;
    };    
} // namespace geometry


#endif // GEOMETRY_H_
