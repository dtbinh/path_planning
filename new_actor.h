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

/**
 *  CLASS: new_actor
 *  DERIVED: sim:actor
 *
 *  Contains methods and variables to determine action taken by an actor
 *  using a finite-state model controller
 */

class new_actor: public sim::actor
{
public:

    // Constructor and Destructor
    new_actor(sim::actor_state& initial_state, sim::world_model& world);
    ~new_actor();

private:
    // ref_corner: current reference corner of actor (corner last visited)
    guid ref_corner;

    // temp_corner: current temporary target corner of actor
    guid temp_corner;

    // Weights for movement to target and collision avoidance
    float w_t;
    float w_c;

    // Speed of actor
    float speed;

    // Exponent parameter for collision avoidance
    float coll_param;

    // method to determine action taken by actor at each time step
    sim::actor_command act_(sim::world_state& w_state) override;

    // method to compute eucledian distance between two points
    float eucledian_dist(geometry::point_2d pt_1, geometry::point_2d pt_2);

    // method to determine presence of actor in a corner (or a bounding box in general)
    bool in_corner(const geometry::point_2d actor_pt, const geometry::box_2d corner_box);
};

/**
 *  CLASS: new_actor_factory
 *  DERIVED: sim:actor_factory
 *
 *  Purpose of this class is to create new actors
 */

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
