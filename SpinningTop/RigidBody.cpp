#include "RigidBody.h"
#include "ode.h"
#include<glm/gtx/orthonormalize.hpp>
#include <iostream>

RigidBody::RigidBody(const glm::dvec3& pos, const glm::dvec3& vel, double mass, double height, const glm::dmat3& inertiaTensor, glm::dmat3 orient) : State{ pos,vel }, _orientation { orient },_inertiaTensor{ inertiaTensor }, _mass{ mass },_height{height}
{
    _invertedIntertiaTensor = glm::inverse(inertiaTensor);
    
    
}

RigidBody::~RigidBody()
{
}

glm::dmat3 RigidBody::getOrientation() const
{
    return _orientation;
}

glm::dvec3 RigidBody::getAngularMomentum() const
{
    return _angularMomentum;
}

glm::dvec3 RigidBody::getLinearMomentum() const
{
    return _linearMomentum;
}

glm::dmat3 RigidBody::getInverseTensor() const
{
    return _invertedIntertiaTensor;
}

glm::dvec3 RigidBody::getForce() const
{
    return _force;
}

void RigidBody::update(double deltaTime)
{
   
  
    auto derivFct = [&](double t, std::array<double, 18> u0)
    {
        RigidBody rb = *this;
        rb.setStateAndCalc(u0);
        rb.calculateForceAndTorque();  return rb.getDtStateArray();
    };

    //calculate the next result
    auto result = ODE::rk4<double, 18>(getStateArray(), 0, deltaTime, derivFct);

    //copy result back to rigid body
    setStateAndCalc(result);


    //reset forces
    _torque = { 0,0,0 };
    _force = { 0,0,0 };
}

void RigidBody::setOrientation(const glm::dmat3 orientation)
{
    _orientation = orientation;
}

void RigidBody::setLinearMomentum(glm::dvec3 linMoment)
{
    _linearMomentum = linMoment;
}

void RigidBody::setAngularMomentum(glm::dvec3 angMoment)
{
    _angularMomentum = angMoment;
}

void RigidBody::setInverseTensor(const glm::dmat3& iTensor)
{
    _invertedIntertiaTensor = iTensor;
}

double RigidBody::getMass() const
{
    return _mass;
}

void RigidBody::setAngularVelocity(glm::dvec3 angVel)
{
    _angularVelocity = angVel;
}

glm::dvec3 RigidBody::getAngularVelocity() const
{
    return _angularVelocity;
}

glm::dvec3 RigidBody::getTorque() const
{
    return _torque;
}

void RigidBody::setTorque(const glm::dvec3& torque)
{
    _torque = torque;
}

void RigidBody::setForce(glm::dvec3 force)
{
    _force = force;
}

std::array<double, 18> RigidBody::getStateArray()
{
    auto pos = getPosition();
    auto orient = getOrientation();
    auto L = getAngularMomentum();
    auto P = getLinearMomentum();
    return std::array<double, 18>{
        pos.x, pos.y, pos.z,
            orient[0][0], orient[0][1], orient[0][2],
            orient[1][0], orient[1][1], orient[1][2],
            orient[2][0], orient[2][1], orient[2][2],
            P.x, P.y, P.z,
            L.x, L.y, L.z
    };
}

std::array<double, 18> RigidBody::getDtStateArray()
{
 
        auto v = getVelocity();
        auto omega = getAngularVelocity();
        auto R = getOrientation();
        auto f = getForce();
        auto torque = getTorque();
       
        glm::dmat3 starOmega;
        starOmega[0] = { 0,omega.z,-omega.y };
        starOmega[1] = { -omega.z,0,omega.x };
        starOmega[2] = { omega.y,-omega.x,0 };

        auto Rdot = starOmega*R;
        //glm::dmat3 Rdot;
        Rdot[0] = glm::cross(omega, R[0]);
        Rdot[1] = glm::cross(omega, R[1]);
        Rdot[2] = glm::cross(omega, R[2]);

        return std::array<double, 18>
        {
                v.x, v.y, v.z,
                Rdot[0][0], Rdot[0][1], Rdot[0][2],
                Rdot[1][0], Rdot[1][1], Rdot[1][2],
                Rdot[2][0], Rdot[2][1], Rdot[2][2],
                f.x, f.y, f.z,
                torque.x, torque.y, torque.z
        };
}

void RigidBody::calculateForceAndTorque(bool nudge)
{
    glm::dvec3 normal{ 0.0,0.0,1.0 };


    glm::dvec3 r = _orientation*glm::dvec3{ 0.0,0.0,0.75*_height }+getPosition();
   

    _force += normal*9.81;
    _torque += glm::cross(r - getPosition(), _force);
 
}

void RigidBody::setStateAndCalc(const std::array<double, 18>& data)
{
    setPosition({ data[0],data[1],data[2] });
    setOrientation({ 
        data[3],data[4],data[5],
        data[6],data[7],data[8],
        data[9],data[10],data[11]
    });

    setLinearMomentum({ data[12],data[13],data[14] });
    setAngularMomentum({ data[15],data[16],data[17] });

    setVelocity(getLinearMomentum() / getMass());

    auto R = getOrientation();
    auto invI = getInverseTensor();
    setInverseTensor(R*invI*glm::transpose(R));

    setAngularVelocity(getInverseTensor()*getAngularMomentum());

    setOrientation(glm::orthonormalize(getOrientation()));
}

