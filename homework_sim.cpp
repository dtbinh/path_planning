#include <iostream>
#include <cmath>
#include <unistd.h>

#include "lib/simulator.h"
#include "default_actor.h"
#include "graphics.h"

int main(void) {

    sim::simulator simulator;
    sim_graphics_wrapper graphics;

    std::unique_ptr<sim::actor_factory> factory = std::move(std::unique_ptr<sim::actor_factory>(new default_actor_factory));

    simulator.initialize(std::move(factory), 1);
    graphics.initialize(simulator.get_world());

    while(1){
        double dt = 0.01;
        simulator.step(dt);
        graphics.update_state(simulator.get_state());
        usleep(dt * 1000000);
    }

    return 0;
}
