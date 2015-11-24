#pragma once
#include"State.h"
#include <array>

class RigidBody : public State
{
public:
    RigidBody(const glm::dvec3& pos, const glm::dvec3&vel,double mass,double height,const glm::dmat3& inertiaTensor,const glm::dmat3 orient=glm::dmat3(1.0));
    ~RigidBody();
    glm::dmat3 getOrientation() const;
    glm::dvec3 getAngularMomentum() const;
    glm::dvec3 getLinearMomentum() const;
    glm::dmat3 getInverseTensor() const;
    glm::dvec3 getForce() const;
    void update(double deltaTime);
    void setOrientation(const glm::dmat3 orientation);
    void setLinearMomentum(glm::dvec3 linMoment);
    void setAngularMomentum(glm::dvec3 angMoment);
    void setInverseTensor(const glm::dmat3& iTensor);
    void setTorque(const glm::dvec3& torque);
    double getMass() const;
    void setAngularVelocity(glm::dvec3 angVel);
    glm::dvec3 getAngularVelocity() const;
    glm::dvec3 getTorque() const;
    void setForce(glm::dvec3 force);
    std::array<double, 18> getStateArray();
    std::array<double, 18> getDtStateArray();
private:
    void calculateForceAndTorque(bool nudge = false);
    void setStateAndCalc(const std::array<double, 18>& data);

    static void convertToBody(const std::array<double, 18>& data);
    glm::dmat3 _orientation;
    glm::dmat3 _inertiaTensor;
    
   
    glm::dvec3 _linearMomentum= {0.0, 0.0, 0.0};;
    glm::dvec3 _angularMomentum = { 0.0,0.0,0.0 };;
    double _mass;
    //because...
    double _height;
    /*derived quantities*/
    glm::dmat3 _invertedIntertiaTensor;
    glm::dvec3 _angularVelocity;

    /*computed values*/
    glm::dvec3 _force={0.0,0.0,0.0};
    glm::dvec3 _torque = { 0.0,0.0,0.0 };;
};

