#ifndef object_detection_PrimitiveObjectVisualization_H
#define object_detection_PrimitiveObjectVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <object_detection/Objects.hpp>

namespace vizkit3d
{
class RigidBodyStateVisualization;

class PrimitiveObjectVisualization
        : public vizkit3d::Vizkit3DPlugin<std::vector<object_detection::PrimitiveObject> >
        , boost::noncopyable
{
    Q_OBJECT
public:
    PrimitiveObjectVisualization();
    ~PrimitiveObjectVisualization();

    Q_INVOKABLE void updateData(std::vector<object_detection::PrimitiveObject> const &sample)
    {vizkit3d::Vizkit3DPlugin<std::vector<object_detection::PrimitiveObject> >::updateData(sample);}

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(std::vector<object_detection::PrimitiveObject> const& plan);

private:
    struct Data;
    Data* p;
    std::vector<RigidBodyStateVisualization*> _frameVisualizers;
    std::vector<osg::Vec4d> colors;
};
}
#endif
