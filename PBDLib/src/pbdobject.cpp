#include "pbdobject.h"


namespace LuHu {



PBDobject::PBDobject()
{

}

bool PBDobject::Initialize(std::string _model,uint _meshIndex, glm::vec3 _originalPos)
{
    m_modelName=_model;
    m_originalPosition=_originalPos;

    const aiScene* scene=getModel(m_modelName);
    if(scene==NULL)
    {
        return false;
    }

    auto allpoints = storePoints(scene, _meshIndex);
    auto edges = getEdges(scene, _meshIndex, allpoints);
    auto faceIndices=removeDuplicates(allpoints);

    m_Constraints =createDistanceConstraints(edges,faceIndices, m_Points);
    m_facePoints=getFaces(scene, _meshIndex , m_Points, allpoints);

    for(auto p : m_Points)
    {
        auto pos = p.get()->getP();
        p.get()->setP(pos+m_originalPosition );
    }
    return true;
}

bool PBDobject::Initialize(glm::vec3 _originalPos, std::vector<std::shared_ptr<point>> _Points)
{
    if(_Points.size() != 0)
    {
        m_Points=_Points;
        for(auto p : m_Points)
        {
            p->setP( p->getP() + _originalPos);

        }
        return true;
    }
    else
    {
        return false;
    }
}
const std::vector<std::shared_ptr<constraint>> & PBDobject::getConstraints() const
{
    return m_Constraints;
}


const std::vector<std::shared_ptr<point>>& PBDobject::getPoints() const
{
    return m_Points;
}

void PBDobject::addPoint(std::shared_ptr<point>  _newP)
{
    m_Points.push_back(_newP);
}

void PBDobject::addPoint(point &_newP)
{
    auto _tempP = std::make_shared<point>(_newP);
    m_Points.push_back(_tempP );
}

void PBDobject::addConstraint(std::shared_ptr<constraint> _newCon)
{
    m_Constraints.push_back(_newCon);
}

void PBDobject::addConstraint(constraint &_newCon)
{
    auto _tempC = std::make_shared<constraint>(_newCon);
    m_Constraints.push_back(_tempC);
}

const std::vector<std::shared_ptr<point>> & PBDobject::getFacesPoints() const
{
    return m_facePoints;
}

} // end of namespace

