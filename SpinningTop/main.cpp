#include "Simulation.h"
int main(int argc, char* argv)
{
    double mass = 0.5;
    double h = 0.5;
    double r = 0.4;
    glm::dmat3 inertiaT;

    inertiaT[0] = { 3.0 / 5.0*mass*h*h + 3.0 / 20.0*mass*r*r,0.0,0.0 };
    inertiaT[1] = { 0.0,3.0 / 5.0*mass*h*h + 3.0 / 20.0*mass*r*r,0.0 };
    inertiaT[2] = { 0.0,0.0,3.0 / 10.0*mass*r*r };
    RigidBody rb{ {0.0,0.0,0.0},{0.0,0.0,0.0},mass,inertiaT };

    Simulation sim{ rb,0,100,0.01 };
    sim.run();

    return 0;
}