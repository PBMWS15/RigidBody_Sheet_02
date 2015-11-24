#pragma once
#include"RigidBody.h"
#include<SDL/SDL.h>
class Simulation
{
public:
    Simulation(const RigidBody& rigidBody,double tStart, double tEnd, double deltaTime);
    ~Simulation();


    void run();
    void update(double deltaTime);
    void applyNudge();
    RigidBody getRigidBody() const;
private:
    RigidBody _rigidBody;
    double _tStart;
    double _tEnd;
    double _deltaTime;
    bool _nudge = false;
    


    


};

