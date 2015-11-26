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

    // geometry::point_2d collision_avoid;
    std::map<guid, sim::actor_state> actors = std::move(w_state.actor_states);

    std::cout << "Id: " << a_state.id << ",\tNeighbors: ";

    float dist_to_actor = 0;
    for(auto actor : actors)
    {
        if(actor.first == a_state.id)
            continue;

        // std::cout << actor.first << ", ";
        // dist_to_actor += eucledian_dist(a_state.pose.position, actor.second.pose.position);
        dist_to_actor += 1;
    }

    std::cout << dist_to_actor << std::endl;
    cmd.heading_rad = atan2(tc.center.y - a_state.pose.position.y, tc.center.x - a_state.pose.position.x);
    cmd.velocity_mps = 1.34; //this is typical human walking speed in meters per second;

    return cmd;
}

float new_actor::eucledian_dist(geometry::point_2d pt_1, geometry::point_2d pt_2)
{
    return sqrt(pow(pt_2.y - pt_1.y, 2) + pow(pt_2.x - pt_1.x, 2));
}
