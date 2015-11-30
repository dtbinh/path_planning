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

    srand(time(NULL));

    // (1.34 +- 0.1) mps is the typical walking speed of humans.
    speed       = 124 + (float)(rand()%20);
    speed       = speed/100;

    coll_param  = -0.1f;
}

// Destructor
new_actor::~new_actor()
{}

sim::actor_command new_actor::act_(sim::world_state& w_state)
{
    // Obatin actor and world model
    sim::actor_state a_state = get_state();
    sim::world_model w_model = get_world();

    // Obtain target corner
    guid target_corner  = a_state.target_corner;
    world::corner& tc   = w_model.corners[target_corner];

    // Actor command
    sim::actor_command cmd = {0.0f, 0.0f};

    // Utility Vectors
    geometry::point_2d disp_vector  = {0.0f, 0.0f};     // Final displacement vector
    geometry::point_2d target_vector= {0.0f, 0.0f};     // Velocity vector to target/temp corner
    geometry::point_2d avoid_coll   = {0.0f, 0.0f};     // Collision Avoidance vector
    geometry::point_2d zero         = {0.0f, 0.0f};     // Reference vector

    // Utility variables
    float dist              = 0.0f;     // Placeholder for distances
    float dist_threshold    = 0.0f;    // Threshold to avoid dividing by zero
    float corner_dist       = 0.1f;     // Distance for successfully reaching a corner
    float sp                = speed;

    // Default speed, this is the walking speed of humans in mps
    w_t = 1.2f;
    w_c = -1.0f;

    // Compute current corner of actor and store it in ref_corner
    for(auto corner: w_model.corners)
        if(in_corner(a_state.pose.position, corner.second.bounding_box))
            ref_corner = corner.first;

    auto rc = w_model.corners[ref_corner];

    // Case: When actor has reached target_corner
    if(ref_corner == target_corner)
    {
        target_vector = {tc.center.x - a_state.pose.position.x,
                        tc.center.y - a_state.pose.position.y};

        temp_corner = 0;
        ref_corner = 0;
    }

    // Case: When actor is on corner connecting the crosswalks between start and target corner
    else if(ref_corner == temp_corner)
    {
        if(eucledian_dist(a_state.pose.position, w_model.corners[temp_corner].center) < corner_dist)
            temp_corner = target_corner;

        target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                        w_model.corners[temp_corner].center.y - a_state.pose.position.y};
    }

    // Case: When actor is in reference corner
    else if(in_corner(a_state.pose.position, rc.bounding_box))
    {
        auto crosswalk_1 = w_model.crosswalks[rc.crosswalks[0]];
        auto crosswalk_2 = w_model.crosswalks[rc.crosswalks[1]];

        // Target Corner is on crosswalk 1
        if(crosswalk_1.corner_1 == target_corner || crosswalk_1.corner_2 == target_corner)
        {
            if(temp_corner == 0)
                temp_corner = target_corner;

            else if(eucledian_dist(a_state.pose.position, w_model.corners[temp_corner].center) < corner_dist)
                temp_corner = target_corner;

            if(w_state.signal_states[crosswalk_1.signal_id] == world::cw_signal::state_type::GO)    // Checking for signal status
                target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                                w_model.corners[temp_corner].center.y - a_state.pose.position.y};

            // Queueing up actors in corner
            else
                sp = 0.0f;
        }

        // Target corner is on crosswalk 2
        else if(crosswalk_2.corner_1 == target_corner || crosswalk_2.corner_2 == target_corner)
        {
            if(temp_corner == 0)
                temp_corner = target_corner;

            else if(eucledian_dist(a_state.pose.position, w_model.corners[temp_corner].center) < corner_dist)
                temp_corner = target_corner;

            if(w_state.signal_states[crosswalk_2.signal_id] == world::cw_signal::state_type::GO)    // Checking for signal status
                target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                                w_model.corners[temp_corner].center.y - a_state.pose.position.y};

            // Queueing up actors in corner
            else
                sp = 0.0f;
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
            sp = 0.0f;
    }

    // Case: When actor is on cross-walk
    else
        target_vector = {w_model.corners[temp_corner].center.x - a_state.pose.position.x,
                        w_model.corners[temp_corner].center.y - a_state.pose.position.y};

    // Normalizing target velocity
    dist = eucledian_dist(target_vector, zero);
    if(dist > dist_threshold)
        target_vector = {target_vector.x/dist, target_vector.y/dist};

    // This loop computes collision avoidance vector from current actor to other actors
    for(auto actor : w_state.actor_states)
    {
        if(actor.first == a_state.id)
            continue;

        disp_vector = {actor.second.pose.position.x - a_state.pose.position.x,
                        actor.second.pose.position.y - a_state.pose.position.y};

        dist = eucledian_dist(disp_vector, zero);
        if(dist < 1.25f)        // 1.25m is the personal space of the actor
            avoid_coll = {avoid_coll.x + exp(coll_param*dist)*disp_vector.x,
                            avoid_coll.y + exp(coll_param*dist)*disp_vector.y};
    }

    // Normalizing collision avoidance
    dist = eucledian_dist(avoid_coll, zero);
    if(dist > dist_threshold)
        avoid_coll = {avoid_coll.x/dist, avoid_coll.y/dist};

    // Superimpose weighted collision avoidance vector onto weighted velocity vector
    disp_vector = {w_t*target_vector.x + w_c*avoid_coll.x,
                    w_t*target_vector.y + w_c*avoid_coll.y};

    // saturating the speed of the actor to its walking speed
    dist = eucledian_dist(disp_vector, zero);
    if(sp == 0)
        sp = dist > speed ? speed : dist;

    // Setting the command variable of actor
    cmd.heading_rad = atan2(disp_vector.y, disp_vector.x);
    cmd.velocity_mps = sp;

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
