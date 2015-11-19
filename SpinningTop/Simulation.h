#pragma once
#include"RigidBody.h"
#include <array>

class Simulation
{
public:
    Simulation(const RigidBody& rigidBody,double tStart, double tEnd, double deltaTime);
    ~Simulation();


    void run();
    void update(double deltaTime);

private:
    RigidBody _rigidBody;
    double _tStart;
    double _tEnd;
    double _deltaTime;
    


    


};

