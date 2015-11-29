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

#include "new_actor.h"

#include <cmath>

new_actor::new_actor(sim::actor_state& initial_state, sim::world_model& world):
    sim::actor(initial_state, world)
{}

sim::actor_command new_actor::act_(sim::world_state& w_state)
{
    sim::actor_state a_state = get_state();
    sim::world_model w_model = get_world();

    guid target_corner  = a_state.target_corner;
    world::corner& tc   = w_model.corners[target_corner];

    sim::actor_command cmd = {0.0f, 0.0f};

    geometry::point_2d disp_vector  = {0.0f, 0.0f};
    geometry::point_2d target_vector= {0.0f, 0.0f};
    geometry::point_2d avoid_coll   = {0.0f, 0.0f};
    geometry::point_2d zero         = {0.0f, 0.0f};

    float theta_dev = 0.0f;
    float dist = 0.0f;
    float dist_threshold = 0.01f;
    float wt = 0.0f;
    float wc = 0.0f;
    float speed = 0.0f;

    for(auto corner: w_model.corners)
    {
        if(in_corner(a_state.pose.position, corner.second.bounding_box))
        {
            rc_id = corner.first;
            break;
        }
    }

    auto rc = w_model.corners[rc_id];

    if(in_corner(a_state.pose.position, rc.bounding_box))
    {
        auto crosswalk_1 = w_model.crosswalks[rc.crosswalks[0]];
        auto crosswalk_2 = w_model.crosswalks[rc.crosswalks[1]];
        if(crosswalk_1.corner_1 == target_corner || crosswalk_1.corner_2 == target_corner)
        {
            temp_corner = target_corner;
            if(w_state.signal_states[crosswalk_1.signal_id] == 1)
            {
                target_vector = {tc.center.x - a_state.pose.position.x, tc.center.y - a_state.pose.position.y};
                speed = 1.34f;
            }

            else
                speed = 0.0f;
        }

        else if(crosswalk_2.corner_1 == target_corner || crosswalk_2.corner_2 == target_corner)
        {
            temp_corner = target_corner;
            if(w_state.signal_states[crosswalk_2.signal_id] == 1)
            {
                target_vector = {tc.center.x - a_state.pose.position.x, tc.center.y - a_state.pose.position.y};
                speed = 1.34f;
            }

            else
                speed = 0.0f;
        }

        else
        {
            if(w_state.signal_states[crosswalk_1.signal_id] == 1)
            {
                if(rc_id == w_model.corners[crosswalk_1.corner_1].id)
                {
                    target_vector = {w_model.corners[crosswalk_1.corner_2].center.x - a_state.pose.position.x, w_model.corners[crosswalk_1.corner_2].center.y - a_state.pose.position.y};
                    temp_corner = crosswalk_1.corner_2;
                }

                else
                {
                    target_vector = {w_model.corners[crosswalk_1.corner_1].center.x - a_state.pose.position.x, w_model.corners[crosswalk_1.corner_1].center.y - a_state.pose.position.y};
                    temp_corner = crosswalk_1.corner_1;
                }

                speed = 1.34f;
            }

            else
            {
                if(rc_id == w_model.corners[crosswalk_2.corner_1].id)
                {
                    target_vector = {w_model.corners[crosswalk_2.corner_2].center.x - a_state.pose.position.x, w_model.corners[crosswalk_2.corner_2].center.y - a_state.pose.position.y};
                    temp_corner = crosswalk_2.corner_2;
                }

                else
                {
                    target_vector = {w_model.corners[crosswalk_2.corner_1].center.x - a_state.pose.position.x, w_model.corners[crosswalk_2.corner_1].center.y - a_state.pose.position.y};
                    temp_corner = crosswalk_2.corner_1;
                }

                speed = 1.34f;
            }
        }
    }

    else
    {
        target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x, w_model.corners[temp_corner].center.y - a_state.pose.position.y};
        speed = 1.34f;
    }

    for(auto actor : w_state.actor_states)
    {
        if(actor.first == a_state.id)
            continue;

        disp_vector.x = actor.second.pose.position.x - a_state.pose.position.x;
        disp_vector.y = actor.second.pose.position.y - a_state.pose.position.y;
        theta_dev = atan2(disp_vector.y, disp_vector.x);
        dist = eucledian_dist(disp_vector, zero);

        avoid_coll.x += exp(-5*dist)*sin(theta_dev);
        avoid_coll.y += exp(-5*dist)*cos(theta_dev);
    }

    dist = eucledian_dist(avoid_coll, zero);
    if(dist > dist_threshold)
    {
        avoid_coll.x = avoid_coll.x/dist;
        avoid_coll.y = avoid_coll.y/dist;
    }

    dist = eucledian_dist(target_vector, zero);

    if(dist > dist_threshold)
    {
        target_vector.x = target_vector.x/dist;
        target_vector.y = target_vector.y/dist;
    }

    wt = 1.2f;
    wc = -1.0f;

    disp_vector.x = wt*target_vector.x + wc*avoid_coll.x;
    disp_vector.y = wt*target_vector.y + wc*avoid_coll.y;

    cmd.heading_rad = atan2(disp_vector.y, disp_vector.x);
    cmd.velocity_mps = speed; //this is typical human walking speed in meters per second;

    return cmd;
}

float new_actor::eucledian_dist(geometry::point_2d pt_1, geometry::point_2d pt_2)
{
    return sqrt(pow(pt_2.y - pt_1.y, 2) + pow(pt_2.x - pt_1.x, 2));
}

bool new_actor::in_corner(const geometry::point_2d actor_pt, const geometry::box_2d corner_box)
{
    if(actor_pt.x < corner_box.upper_left.x)
        return false;

    if(actor_pt.y > corner_box.upper_left.y)
        return false;

    if(actor_pt.x > corner_box.lower_right.x)
        return false;

    if(actor_pt.y < corner_box.lower_right.y)
        return false;

    return true;
}
