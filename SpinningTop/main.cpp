#include "Simulation.h"
int main(int argc, char *args[])
{
    double mass = 30.0;
    double h = 5;
    double r = 2.5;
    glm::dmat3 inertiaT;
    glm::dmat2x4 t;

    t[0];
    inertiaT[0] = { 3.0 / 5.0*mass*h*h + 3.0 / 20.0*mass*r*r,0.0,0.0 };
    inertiaT[1] = { 0.0,3.0 / 5.0*mass*h*h + 3.0 / 20.0*mass*r*r,0.0 };
    inertiaT[2] = { 0.0,0.0,3.0 / 10.0*mass*r*r };
    RigidBody rb{ {0.0,0.0,0.0},{0.0,0.0,0.0},mass,h,inertiaT };

    rb.setAngularMomentum({ 0.1736, 0.0, 0.9848 });
    rb.setVelocity({ 0.005, 0, 0 });
    Simulation sim{ rb,0,100,0.01 };
    sim.run();

    return 0;
}