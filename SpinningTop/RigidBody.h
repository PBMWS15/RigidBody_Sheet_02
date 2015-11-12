#pragma once
#include"State.h"
class RigidBody : public State
{
public:
    RigidBody(const glm::dvec3& pos, const glm::dvec3&vel,double mass,const glm::dmat3& inertiaTensor,const glm::dmat3 orient=glm::dmat3(1.0));
    ~RigidBody();
    glm::dmat3 getOrientation();
    glm::dvec3 getAngularMomentum();
    glm::dvec3 getLinearMomentum();
    glm::dmat3 getInverseTensor();
    glm::dvec3 getForce();

    void setOrientation(const glm::dmat3 orientation);
    void setLinearMomentum(glm::dvec3 linMoment);
    void setAngularMomentum(glm::dvec3 angMoment);
    void setInverseTensor(const glm::dmat3& iTensor);
    double getMass();
    void setAngularVelocity(glm::dvec3 angVel);
    glm::dvec3 getAngularVelocity();
    glm::dvec3 getTorque();
    void setForce(glm::dvec3 force);
private:
    glm::dmat3 _orientation;
    glm::dmat3 _inertiaTensor;
    
   
    glm::dvec3 _linearMomentum= {0.0, 0.0, 0.0};;
    glm::dvec3 _angularMomentum = { 0.0,0.0,0.0 };;
    double _mass;

    /*derived quantities*/
    glm::dmat3 _invertedIntertiaTensor;
    glm::dvec3 _angularVelocity;

    /*computed values*/
    glm::dvec3 _force={0.0,0.0,0.0};
    glm::dvec3 _torque = { 0.0,0.0,0.0 };;
};

