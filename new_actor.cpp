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

    geometry::point_2d disp_vector = {0, 0};
    geometry::point_2d collision_avoid = {0, 0};
    geometry::point_2d zero= {0, 0};
    std::map<guid, sim::actor_state> actors = w_state.actor_states;
    float theta_dev = 0;
    float dist = 0;
    float dist_threshold = 0.01;
    float w_target = 1;
    float w_collision = -1;

    // std::cout << "Agent ID:" << a_state.id << "\t: [" << a_state.pose.position.x << ", " << a_state.pose.position.y << "]\n";

    for(auto actor : actors)
    {
        if(actor.first == a_state.id)
            continue;


        disp_vector.x = actor.second.pose.position.x - a_state.pose.position.x;
        disp_vector.y = actor.second.pose.position.y - a_state.pose.position.y;
        // std::cout << "Agent ID:" << actor.first << "\tDisp Vector: [" << disp_vector.x << ", " << disp_vector.y << "]\n";
        theta_dev = atan2(disp_vector.y, disp_vector.x);
        dist = eucledian_dist(disp_vector, zero);
        // std::cout << "dist: " << dist << std::endl;
        // std::cout << "angle_to_actor: " << theta_dev*180/M_PI << std::endl;
        if(dist > dist_threshold)
        {
            collision_avoid.x += exp(-1*dist)*sin(theta_dev);
            collision_avoid.y += exp(-1*dist)*cos(theta_dev);
        }
    }

    dist = eucledian_dist(collision_avoid, zero);
    if(dist > dist_threshold)
    {
        collision_avoid.x = collision_avoid.x/dist;
        collision_avoid.y = collision_avoid.y/dist;
    }

    // std::cout << "collision_avoid: [" << collision_avoid.x << ", " << collision_avoid.y << "]\n\n";

    disp_vector.x = tc.center.x - a_state.pose.position.x;
    disp_vector.y = tc.center.y - a_state.pose.position.y;
    dist = eucledian_dist(disp_vector, zero);

    if(dist > dist_threshold)
    {
        disp_vector.x = disp_vector.x/dist;
        disp_vector.y = disp_vector.y/dist;
    }

    w_target = 1.5;
    w_collision = -1;

    disp_vector.x = w_target*disp_vector.x + w_collision*collision_avoid.x;
    disp_vector.y = w_target*disp_vector.y + w_collision*collision_avoid.y;

    cmd.heading_rad = atan2(disp_vector.y, disp_vector.x);
    cmd.velocity_mps = 1.34; //this is typical human walking speed in meters per second;
    return cmd;
}

float new_actor::eucledian_dist(geometry::point_2d pt_1, geometry::point_2d pt_2)
{
    return sqrt(pow(pt_2.y - pt_1.y, 2) + pow(pt_2.x - pt_1.x, 2));
}
