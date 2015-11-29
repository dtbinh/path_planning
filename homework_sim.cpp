#include <iostream>
#include <cmath>
#include <unistd.h>

#include "lib/simulator.h"
#include "new_actor.h"
#include "graphics.h"

int main(void) {

    sim::simulator simulator;
    sim_graphics_wrapper graphics;

    std::unique_ptr<sim::actor_factory> actors = std::move(std::unique_ptr<sim::actor_factory>(new new_actor_factory));

    simulator.initialize(std::move(actors), 2);
    graphics.initialize(simulator.get_world());

    while(1){
        double dt = 0.1;
        simulator.step(dt);
        graphics.update_state(simulator.get_state());
        usleep(dt * 1000000);
        // std::cin.ignore();
        // int n = system("clear");
        // std::cout << n << std::endl;
    }

    return 0;
}
