#include <iostream>
#include "PrimitiveObjectVisualization.hpp"
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <vizkit3d/RigidBodyStateVisualization.hpp>

using namespace vizkit3d;

struct PrimitiveObjectVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::vector<object_detection::PrimitiveObject> data;
};


PrimitiveObjectVisualization::PrimitiveObjectVisualization()
    : p(new Data)
{
    colors.push_back(osg::Vec4d(1.0,0,0,1));
    colors.push_back(osg::Vec4d(0,1,0,1));
    colors.push_back(osg::Vec4d(0,0,1,1));
    colors.push_back(osg::Vec4d(1,1,0,1));
    colors.push_back(osg::Vec4d(1,0,1,1));
    colors.push_back(osg::Vec4d(0,1,1,1));
    colors.push_back(osg::Vec4d(1,1,1,1));
    colors.push_back(osg::Vec4d(0,0,0,1));
}

PrimitiveObjectVisualization::~PrimitiveObjectVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> PrimitiveObjectVisualization::createMainNode()
{
    return new osg::Group();
}

void PrimitiveObjectVisualization::updateMainNode ( osg::Node* node )
{
    osg::Group* group = node->asGroup();
    group->removeChildren( 0, group->getNumChildren() );

    for(uint i=0; i<_frameVisualizers.size(); i++){
        delete _frameVisualizers[i];
    }
    _frameVisualizers.clear();


    // go through the list of primitives
    for( size_t i=0; i<p->data.size(); i++ )
    {
        object_detection::PrimitiveObject &po( p->data[i] );

        osg::PositionAttitudeTransform* tn =
                new osg::PositionAttitudeTransform();

        // get the color of the object
        //osg::Vec4 color = osg::Vec4( po.color.x(), po.color.y(), po.color.z(), po.likelihood );
        osg::Vec4 color = colors[i%colors.size()];
        color.w() = po.likelihood;

        // set the pose of the object
        osg::Vec3 pos = osg::Vec3( po.position.x(), po.position.y(), po.position.z() );
        tn->setPosition( pos );
        osg::Quat q( po.orientation.x(), po.orientation.y(), po.orientation.z(), po.orientation.w() );
        tn->setAttitude(q);

        osg::Geode* obj = new osg::Geode;
        osg::Shape* shape;
        if( po.type == object_detection::BOX )
        {
            shape = new osg::Box( osg::Vec3(), po.param[0], po.param[1], po.param[2] );
        }
        else if( po.type == object_detection::CYLINDER )
        {
            shape = new osg::Cylinder( osg::Vec3(), po.param[0], po.param[1] );
        }
        osg::ShapeDrawable *drawable = new osg::ShapeDrawable(shape);
        drawable->setColor( color );
        obj->addDrawable( drawable );

        RigidBodyStateVisualization* rbsv = new RigidBodyStateVisualization();
        rbsv->setPluginName("Hello");
        rbsv->setPluginEnabled(true);
        rbsv->resetModel(0.2);
        tn->addChild(rbsv->getRootNode());

        tn->addChild( obj );
        group->addChild( tn );
    }
}

void PrimitiveObjectVisualization::updateDataIntern(std::vector<object_detection::PrimitiveObject> const& value)
{
    p->data = value;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(PrimitiveObjectVisualization)

