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

#ifndef DEFAULT_ACTOR_H_
#define DEFAULT_ACTOR_H_

#include "lib/simulator.h"

class default_actor: public sim::actor
{
public:
    default_actor(sim::actor_state& initial_state, sim::world_model& world);
    
private:
    sim::actor_command act_(sim::world_state& w_state) override;
};

class default_actor_factory: public sim::actor_factory
{
public:
    std::unique_ptr<sim::actor> create_actor(sim::actor_state& initial_state, sim::world_model& world) override
    {
        return std::move(std::unique_ptr<sim::actor>(new default_actor(initial_state, world)));
    }

};

#endif // DEFAULT_ACTOR_H_
