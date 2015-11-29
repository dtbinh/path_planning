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

// Constructor
new_actor::new_actor(sim::actor_state& initial_state, sim::world_model& world):
    sim::actor(initial_state, world)
{
    temp_corner = 0;
    ref_corner  = 0;
    w_t         = 1.2f;
    w_c         = -1.0f;
    speed       = 1.34f;    // This is the typical human walking speed in mps
}

// Destructor
new_actor::~new_actor()
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

    float theta_dev         = 0.0f;     // Placeholder for angles
    float dist              = 0.0f;     // Placeholder for distances
    float dist_threshold    = 0.01f;    // Threshold to avoid dividing by zero
    speed = 1.34f;

    // Compute current corner of actor and store it in ref_corner
    for(auto corner: w_model.corners)
    {
        if(in_corner(a_state.pose.position, corner.second.bounding_box))
        {
            ref_corner = corner.first;
            break;
        }
    }

    auto rc = w_model.corners[ref_corner];

    // Case: When actor has reached target_corner
    if(ref_corner == target_corner)
    {
        std::cout << "In target corner\n";
        target_vector = {tc.center.x - a_state.pose.position.x,
                        tc.center.y - a_state.pose.position.y};
        temp_corner = 0;
        ref_corner = 0;
    }

    // Case: When actor is on corner connecting the crosswalks between start and target corner
    else if(ref_corner == temp_corner)
    {
        std::cout << "In intermediate corner, but not target corner\n";
        if(eucledian_dist(a_state.pose.position, w_model.corners[temp_corner].center) < 0.5f)
            temp_corner = target_corner;

        target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                        w_model.corners[temp_corner].center.y - a_state.pose.position.y};
    }

    // Case: When actor is in reference corner
    else if(in_corner(a_state.pose.position, rc.bounding_box))
    {
        std::cout << "In a corner, but not target\n";
        auto crosswalk_1 = w_model.crosswalks[rc.crosswalks[0]];
        auto crosswalk_2 = w_model.crosswalks[rc.crosswalks[1]];

        // Target Corner is on crosswalk 1
        if(crosswalk_1.corner_1 == target_corner || crosswalk_1.corner_2 == target_corner)
        {
            if(temp_corner == 0)
                temp_corner = target_corner;

            else if(eucledian_dist(a_state.pose.position, w_model.corners[temp_corner].center) < 0.5f)
                temp_corner = target_corner;

            if(w_state.signal_states[crosswalk_1.signal_id] == world::cw_signal::state_type::GO)    // Checking for signal status
            {
                target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                                w_model.corners[temp_corner].center.y - a_state.pose.position.y};
            }

            else
                speed = 0.0f;
        }

        // Target corner is on crosswalk 2
        else if(crosswalk_2.corner_1 == target_corner || crosswalk_2.corner_2 == target_corner)
        {
            if(temp_corner == 0)
                temp_corner = target_corner;

            else if(eucledian_dist(a_state.pose.position, w_model.corners[temp_corner].center) < 0.5f)
                temp_corner = target_corner;

            if(w_state.signal_states[crosswalk_2.signal_id] == world::cw_signal::state_type::GO)    // Checking for signal status
            {
                target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                                w_model.corners[temp_corner].center.y - a_state.pose.position.y};
            }

            else
                speed = 0.0f;
        }

        // Target corner is not one cross-walk away. Checking if cross-walk 1 signal status is GO
        else if(w_state.signal_states[crosswalk_1.signal_id] == world::cw_signal::state_type::GO)
        {
            if(ref_corner == w_model.corners[crosswalk_1.corner_1].id)
            {
                target_vector = {w_model.corners[crosswalk_1.corner_2].center.x - a_state.pose.position.x,
                                w_model.corners[crosswalk_1.corner_2].center.y - a_state.pose.position.y};
                temp_corner = crosswalk_1.corner_2;
            }

            else
            {
                target_vector = {w_model.corners[crosswalk_1.corner_1].center.x - a_state.pose.position.x,
                                w_model.corners[crosswalk_1.corner_1].center.y - a_state.pose.position.y};
                temp_corner = crosswalk_1.corner_1;
            }
        }

        // Target corner is not one cross-walk away. Checking if cross-walk 2 signal status is GO
        else if(w_state.signal_states[crosswalk_2.signal_id] == world::cw_signal::state_type::GO)
        {
            if(ref_corner == w_model.corners[crosswalk_2.corner_1].id)
            {
                target_vector = {w_model.corners[crosswalk_2.corner_2].center.x - a_state.pose.position.x,
                                w_model.corners[crosswalk_2.corner_2].center.y - a_state.pose.position.y};
                temp_corner = crosswalk_2.corner_2;
            }

            else
            {
                target_vector = {w_model.corners[crosswalk_2.corner_1].center.x - a_state.pose.position.x,
                                w_model.corners[crosswalk_2.corner_1].center.y - a_state.pose.position.y};
                temp_corner = crosswalk_2.corner_1;
            }
        }

        // Queueing up actors in corner
        else
            speed = 0.0f;
    }

    // Case: When actor is on cross-walk
    else
    {
        std::cout << "On crosswalk between reference and temp target\n";
        target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                        w_model.corners[temp_corner].center.y - a_state.pose.position.y};
    }

    // This loop computes collision avoidance vector from current actor to other actors
    for(auto actor : w_state.actor_states)
    {
        if(actor.first == a_state.id)
            continue;

        disp_vector = {actor.second.pose.position.x - a_state.pose.position.x, actor.second.pose.position.y - a_state.pose.position.y};
        theta_dev = atan2(disp_vector.y, disp_vector.x);
        dist = eucledian_dist(disp_vector, zero);
        avoid_coll = {avoid_coll.x + exp(-5*dist)*sin(theta_dev), avoid_coll.y + exp(-5*dist)*cos(theta_dev)};
    }

    dist = eucledian_dist(avoid_coll, zero);
    if(dist > dist_threshold)
        avoid_coll = {avoid_coll.x/dist, avoid_coll.y/dist};

    dist = eucledian_dist(target_vector, zero);
    if(dist > dist_threshold)
        target_vector = {target_vector.x/dist, target_vector.y/dist};

    disp_vector = {w_t*target_vector.x + w_c*avoid_coll.x, w_t*target_vector.y + w_c*avoid_coll.y};

    cmd.heading_rad = atan2(disp_vector.y, disp_vector.x);
    cmd.velocity_mps = speed;

    std::cout << "ref:    " << ref_corner << "\ntemp:   " << temp_corner << "\ntarget: " << target_corner << "\nspeed: " << speed << "\n";

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
