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

#include "default_actor.h"

#include <cmath>
 
default_actor::default_actor(sim::actor_state& initial_state, sim::world_model& world):
    sim::actor(initial_state, world) 
{ 

}

sim::actor_command default_actor::act_(sim::world_state& w_state)
{
    sim::actor_state a_state = get_state();
    sim::world_model w_model = get_world();

    guid target_corner = a_state.target_corner;
    world::corner& tc = w_model.corners[target_corner];

    sim::actor_command cmd;

    cmd.heading_rad = atan2(tc.center.y - a_state.pose.position.y, tc.center.x - a_state.pose.position.x);
    cmd.velocity_mps = 1.34; //this is typical human walking speed in meters per second;

    return cmd;
}
