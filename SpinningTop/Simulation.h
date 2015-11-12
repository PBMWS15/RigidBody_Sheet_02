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
    

    std::array<double, 18> StateToArray();
    void ArrayToRigidBody(const std::array<double,18>& data);

    std::array<double, 18> DtStateToArray();

    std::array<double, 18> dxdt(double t, std::array<double, 18> &rgData);
    


};

