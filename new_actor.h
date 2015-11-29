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

#ifndef NEW_ACTOR_H_
#define NEW_ACTOR_H_

#include "lib/simulator.h"

class new_actor: public sim::actor
{
public:
    new_actor(sim::actor_state& initial_state, sim::world_model& world);
    ~new_actor();

private:
    guid rc_id;
    guid temp_corner;
    sim::actor_command act_(sim::world_state& w_state) override;
    float eucledian_dist(geometry::point_2d pt_1, geometry::point_2d pt_2);
    bool in_corner(const geometry::point_2d actor_pt, const geometry::box_2d corner_box);
};

class new_actor_factory: public sim::actor_factory
{
public:
    ~new_actor_factory();
    std::unique_ptr<sim::actor> create_actor(sim::actor_state& initial_state, sim::world_model& world) override
    {
        return std::move(std::unique_ptr<sim::actor>(new new_actor(initial_state, world)));
    }
};

#endif // DEFAULT_ACTOR_H_
