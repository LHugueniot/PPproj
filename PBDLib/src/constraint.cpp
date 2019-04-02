#include "constraint.h"
#include "kernel.h"

#define GLM_ENABLE_EXPERIMENTAL

#include "glm/gtx/normal.hpp"


namespace LuHu{

//-----------------------------------------------------------------Base Constraint class-------------------------------------------------


constraint::constraint(){}

constraint::~constraint(){}

void constraint::timeStep(){}

std::shared_ptr<point> constraint::getPoint(uint index) const{}

void constraint::setPoint(std::shared_ptr<point> _p, uint index){}

//-----------------------------------------------------------------Distance Constraint---------------------------------------------------

distanceConstraint::distanceConstraint(std::shared_ptr<point> _p1,
                                       std::shared_ptr<point> _p2):
    m_p1(_p1),
    m_p2(_p2)
{
    m_restLength=glm::length(m_p1->getP() - m_p2->getP());
}

distanceConstraint::distanceConstraint(point & _p1, point & _p2):
    m_p1(std::make_shared<point>(_p1)),
    m_p2(std::make_shared<point>(_p2))
{
    m_restLength=glm::length(m_p1->getP() - m_p2->getP());
}

void distanceConstraint::timeStep()
{
    glm::vec3 dir = m_p1->getTmpPos() - m_p2->getTmpPos();

    float len = glm::length(dir);
    float inv_mass=m_p1->getIM() + m_p2->getIM();

    m_p1->setTmp(
                (m_p1->getTmpPos()-
                 (m_p1->getIM()/inv_mass)*
                 (len - m_restLength)*
                 (dir/len)
                 )
                );


    m_p2->setTmp(
                (m_p2->getTmpPos()+
                 (m_p2->getIM()/inv_mass)*
                 (len - m_restLength)*
                 (dir/len)
                 )
                );
}
std::shared_ptr<point> distanceConstraint::getPoint(uint index) const

{
    if(index==0)
    {
        return m_p1;
    }
    else if(index==1)
    {
        return m_p2;
    }
    else
    {
        return NULL;
    }

}

float distanceConstraint::getRestLength() const
{
    return m_restLength;
}

void distanceConstraint::setPoint(std::shared_ptr<point> _p, uint index)
{
    if(index==0)
    {
        m_p1=_p;
    }
    else if(index==1)
    {
        m_p2=_p;
    }
    else
    {
        std::cout<<"distanceConstraint error, index too big, must be smaller than 2";
    }
}

void distanceConstraint::setRestLength(float _newRestLength)
{
    m_restLength=_newRestLength;
}

//-----------------------------------------------------------------Collision Constraint--------------------------------------------------

collisionConstraint::collisionConstraint(std::shared_ptr<point> _p1) :
    m_p1(_p1)
{

}

void collisionConstraint::timeStep()
{

}
std::shared_ptr<point> collisionConstraint::getPoint(uint index) const
{

}

void collisionConstraint::setPoint(std::shared_ptr<point> _p, uint index)
{
    if(index==0)
    {
        m_p1=_p;
    }
    else
    {
        std::cout<<"bendingConstraint error, index too big, must be equal to 0";
    }
}

//-----------------------------------------------------------------Bending Constraint----------------------------------------------------



bendingConstraint::bendingConstraint(std::shared_ptr<point> _p1,
                                     std::shared_ptr<point> _p2,
                                     std::shared_ptr<point> _p3,
                                     std::shared_ptr<point> _p4):
    m_p1(_p1), m_p2(_p2), m_p3(_p3), m_p4(_p4){

    auto _p1pos = m_p1->getP();
    auto _p2pos = m_p2->getP();
    auto _p3pos = m_p3->getP();
    auto _p4pos = m_p4->getP();

    auto n1=glm::triangleNormal(_p1pos, _p2pos, _p3pos);
    auto n2=glm::triangleNormal(_p4pos, _p2pos, _p3pos);

    m_angle=glm::acos(glm::dot(n1,n2));
}

bendingConstraint::bendingConstraint(point & _p1, point & _p2, point & _p3, point &_p4):
    m_p1(std::make_shared<point>(_p1)),
    m_p2(std::make_shared<point>(_p2)),
    m_p3(std::make_shared<point>(_p3)),
    m_p4(std::make_shared<point>(_p4))
{

    auto _p1pos = m_p1->getP();
    auto _p2pos = m_p2->getP();
    auto _p3pos = m_p3->getP();
    auto _p4pos = m_p4->getP();

    auto n1=glm::triangleNormal(_p1pos, _p2pos, _p3pos);
    auto n2=glm::triangleNormal(_p4pos, _p2pos, _p3pos);

    m_angle=glm::acos(glm::dot(n1,n2));
}

void bendingConstraint::timeStep()
{
    //    ∆pi = −
    //    wi
    //    √
    //    1−d
    //    2(arccos(d)−φ0)
    //    ∑j wj
    //    |qj
    //    |
    //    2
    //    qi
    glm::vec3 _p1pos = m_p1->getP();
    glm::vec3 _p2pos = m_p2->getP();
    glm::vec3 _p3pos = m_p3->getP();
    glm::vec3 _p4pos = m_p4->getP();

    glm::vec3 n1=glm::triangleNormal(_p1pos, _p2pos, _p3pos);
    glm::vec3 n2=glm::triangleNormal(_p4pos, _p2pos, _p3pos);

    auto d =glm::dot(n1,n2);

    float tempAngle = glm::acos(d);

    auto C = tempAngle - m_angle;


    glm::vec3 q1=( ( (_p2pos * n2) + ( n1 * _p2pos)*d) ) / glm::length( _p2pos * _p3pos);

    glm::vec3 q3=( ( (_p2pos * n1) + ( n2 * _p2pos)*d) ) / glm::length( _p2pos * _p4pos);

    glm::vec3 q2=( ( (_p3pos * n2) + ( n1 * _p3pos)*d) ) / glm::length(_p2pos * _p3pos) -
                 ( ( (_p4pos * n1) + ( n2 * _p4pos)*d) ) / glm::length(_p2pos * _p4pos);

    glm::vec3 q4= -q2 -q1 -q3;

    float w1 = m_p1->getIM();
    float w2 = m_p2->getIM();
    float w3 = m_p3->getIM();
    float w4 = m_p4->getIM();

    float wj=w1 + w2 + w3 + w4;
    float len_qj = glm::length(q1+q2+q3+q4);

    float jSum = wj*len_qj * wj*len_qj;

    m_p1->setTmp( m_p1->getTmpPos()+
                  q1 *
                  -(w1*sqrtf(1-d*d) *
                    C /
                    jSum
                    )
                  );
    printVec3(m_p1->getTmpPos());
    m_p2->setTmp( m_p2->getTmpPos()+
                  q2 *
                  -(w2*sqrtf(1-d*d) *
                    C /
                    jSum
                    )
                  );
    printVec3(m_p2->getTmpPos());
    m_p3->setTmp( m_p3->getTmpPos()+
                  q3 *
                  -(w3*sqrtf(1-d*d) *
                    C /
                    jSum
                    )
                  );
printVec3(m_p3->getTmpPos());
    m_p4->setTmp( m_p4->getTmpPos()+
                  q4 *
                  -(w4*sqrtf(1-d*d) *
                    C /
                    jSum
                    ) *
                  q4
                  );

printVec3(m_p4->getTmpPos());
}

std::shared_ptr<point> bendingConstraint::getPoint(uint index) const
{
    if(index==0)
    {
        return m_p1;
    }
    if(index==1)
    {
        return m_p2;
    }
    if(index==2)
    {
        return m_p3;
    }
    if(index==3)
    {
        return m_p4;
    }
    else
    {
        return NULL;
        std::cout<<"bendingConstraint error, index too big, must be smaller than 3";
    }
}

void bendingConstraint::setPoint(std::shared_ptr<point> _p, uint index)
{
    if(index==0)
    {
        m_p1=_p;
    }
    else if(index==1)
    {
        m_p2=_p;
    }
    else if(index==2)
    {
        m_p3=_p;
    }
    else if(index==3)
    {
        m_p4=_p;
    }
    else
    {
        std::cout<<"distanceConstraint error, index too big, must be smaller than 4";
    }
}

float bendingConstraint::getAngle()
{
    return m_angle;
}

void bendingConstraint::setAngle(float _angle)
{
    m_angle=_angle;
}

}
