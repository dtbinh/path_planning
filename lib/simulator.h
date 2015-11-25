/**
 * simulator.h
 * Copyright 2015
 *
 *
 * Uber Advanced Technology Center
 * Pittsburgh, PA
 *
 * Control Instructions (who can see this code):
 * Confidential.  Not for public release
 *
 *
 * This notice must appear in all copies of this file and its derivatives.
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "geometry.h"
#include "world.h"

#include <vector>
#include <map>
#include <stdexcept>
#include <iostream>
#include <memory>

#include <chrono>

namespace sim
{
    struct actor_state
    {
        guid id;
        geometry::pose_2d pose;
        guid target_corner;
    };

    struct actor_command
    {
        double velocity_mps;
        double heading_rad;
    };

    class world_model;
    class world_state;
    class actor
    {
    public:
        actor(actor_state& initial_state, world_model& world):state_(initial_state),world_(world) { }
        void act(world_state& w_state, double dt) { simulate(act_(w_state), dt); }
        void set_target_corner(guid corner_id) { state_.target_corner = corner_id; }
        const actor_state& get_state() { return state_; }

    protected:
        virtual actor_command act_(world_state& w_state) = 0;
        const world_model& get_world() { return world_; }

    private:
        void simulate(const actor_command& cmd, double dt);

        actor_state state_;
        const world_model& world_;
    };

    class actor_factory
    {
    public:
        virtual std::unique_ptr<actor> create_actor(actor_state& initial_state, world_model& world) = 0;
    };


    struct world_model
    {
        std::map<guid, world::corner> corners;
        std::map<guid, world::crosswalk> crosswalks;
        std::map<guid, world::cw_signal> cw_signals;
    };

    struct world_state
    {
        std::map<guid, actor_state> actor_states;
        std::map<guid, world::cw_signal::state_type> signal_states;
        double time;
    };

    class simulator
    {
    public:

        simulator();
        ~simulator();

        void initialize(std::unique_ptr<actor_factory> factory, int number_of_actors, unsigned int rand_seed = 0);
        void step();
        void step(double dt);

        const world_model& get_world() { return w_model; }
        const world_state& get_state() { return w_state; }

    private:

        double calc_elapsed_time(std::chrono::steady_clock::time_point time_point);
        const world::corner& get_random_corner();

        world_model w_model;
        world_state w_state;
        std::vector<std::unique_ptr<actor>> actors;
        std::unique_ptr<actor_factory> a_factory;

        std::chrono::steady_clock::time_point start_time;
        double time;
    };

} // namespace sim

#endif // SIMULATOR_H_
