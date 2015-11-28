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
{

}

sim::actor_command new_actor::act_(sim::world_state& w_state)
{
    sim::actor_state a_state = get_state();
    sim::world_model w_model = get_world();

    guid target_corner = a_state.target_corner;
    world::corner& tc = w_model.corners[target_corner];

    sim::actor_command cmd;

    geometry::point_2d disp_vector = {0.0f, 0.0f};
    geometry::point_2d avoid_coll = {0.0f, 0.0f};
    geometry::point_2d zero= {0.0f, 0.0f};

    std::map<guid, sim::actor_state> actors = w_state.actor_states;
    std::map<guid, world::corner> corners = w_model.corners;

    float theta_dev = 0.0f;
    float dist = 0.0f;
    float dist_threshold = 0.01f;
    float w_target = 1.0f;
    float w_collision = -1.0f;

    std::cout << "Actor: " << a_state.id << ",\t current position: [" << a_state.pose.position.x << ", " << a_state.pose.position.y << "]";
    for(auto corner: corners)
        if(in_corner(a_state.pose.position, corner.second.bounding_box))
            break;

    for(auto actor : actors)
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

    disp_vector.x = tc.center.x - a_state.pose.position.x;
    disp_vector.y = tc.center.y - a_state.pose.position.y;
    dist = eucledian_dist(disp_vector, zero);

    if(dist > dist_threshold)
    {
        disp_vector.x = disp_vector.x/dist;
        disp_vector.y = disp_vector.y/dist;
    }

    w_target = 1.2;
    w_collision = -1;

    disp_vector.x = w_target*disp_vector.x + w_collision*avoid_coll.x;
    disp_vector.y = w_target*disp_vector.y + w_collision*avoid_coll.y;

    cmd.heading_rad = atan2(disp_vector.y, disp_vector.x);
    // cmd.velocity_mps = 1.34; //this is typical human walking speed in meters per second;
    cmd.velocity_mps = 0.1;

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
