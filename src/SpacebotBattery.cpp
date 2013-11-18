#include "SpacebotBattery.hpp"

using namespace object_detection;

SpacebotBattery::SpacebotBattery()
{

}
/** @brief tries to assemble the objects based on the component candidates
  * @return successful
  */
bool SpacebotBattery::assembleObject(){

    if (!modelSet_) return false;

    PCL_WARN("Battery: assemble object with %d components\n",components_.size());

    points_.clear();

    //remove all planes which do not fit any of the three distinct panels of the Space Bot battery
    std::vector<Plane>::iterator compIter = components_.begin();

    if (this->use_color_){
        for(; compIter != components_.end() ; ){
            object_detection::Color color=compIter->getColor();
            if (debug_) PCL_WARN("Color of component (r,g,b) %f,%f,%f\n",color.r,color.g,color.b);
            if (((compIter ->matchesShape(1.0,boxModel_.dimensions[0],boxModel_.dimensions[1])<0.8) &&  //extension 0 and 1
              (compIter ->matchesShape(1.0,boxModel_.dimensions[0],boxModel_.dimensions[2])<0.8) &&  //extension 0 and 2
              (compIter ->matchesShape(1.0,boxModel_.dimensions[1],boxModel_.dimensions[2])<0.8)) ||  //extension 1 and 2
              compIter  ->matchesColor(boxModel_.color) < 0.8) //color has to match if using color
              compIter = components_.erase( compIter ) ; // advances iter
          else
              ++compIter; // don't remove
        }
    }

    else {

    for(; compIter != components_.end() ; )
        if ((compIter ->matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.y())<0.8) &&  //extension 0 and 1
            (compIter ->matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.z())<0.8) &&  //extension 0 and 2
            (compIter ->matchesShape(1.0,boxModel_.dimensions.y(),boxModel_.dimensions.z())<0.8))    //extension 1 and 2
        compIter = components_.erase( compIter ) ; // advances iter
      else
        ++compIter; // don't remove
    }

    if (debug_){
        PCL_WARN("Battery: %d components left after shape match\n",components_.size());
        BOOST_FOREACH(Plane plane,components_)
        {
            PCL_WARN("Color of battery component (r,g,b) %f,%f,%f\n",plane.getColor().r,plane.getColor().g,plane.getColor().b);
            PCL_WARN("Color matched model with %f\n",plane.matchesColor(boxModel_.color));
        }
    }

    //check if more than 1 component is available
    if (components_.size()<1) return false;


    //check which panel is found. Estimate size and direction based on this single panel
    //add point of planes to component pc

    Plane groundPlane(0,0,0,0,base::Pose(),base::Vector3d(0,0,1),base::Vector3d(0,0,0));


    BOOST_FOREACH(Plane plane, components_){

        //matches top panel //x-y plane
        if (plane.matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.y())>0.8) //matches top panel
        {
            PCL_WARN("Found top panel: %f,%f\n",boxModel_.dimensions.x(),boxModel_.dimensions.y());
        }
        //matches side panel //x-z plane
        if ((plane.matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.z())>0.8) && //matches side panel
            (plane.isOrthogonal(groundPlane)>0.9) && //is upright and not flat on the ground
            (plane.isAboveGround()<0.20)) //is close to the ground, assume ground is 0 level
        {
            PCL_WARN("Found side panel: %f,%f\n",boxModel_.dimensions.x(),boxModel_.dimensions.z());
            //translate base along the normal away from the viewpoint
            points_+=*plane.getPoints();
            setBatteryPose(plane,SIDE);
            likelihood_=0.50;
            return true;
        }
    }


    return false;
}


void SpacebotBattery::setBatteryPose(Plane plane,BOX_PANEL_SIDE side){

    base::Vector3d base_vector;
    base::Vector3d expansion=plane.getExpansion();

    switch (side){

        case FRONT_REAR:
            break;
        case SIDE:
            pose_.position=plane.getPosition()-plane.getNormal()*boxModel_.dimensions.y()/2.0;            
            base_vector=base::Vector3d (0,1,0);

            break;
        case TOP:
            break;
    }

    //COOL CODE THAT IS HOW TO DO IT RIGHT
    base::Vector3d normal_flip=-1*plane.getNormal();
    base::Vector3d quat_axis=base_vector.cross(normal_flip);

    double quat_ang = acos(base_vector.dot(normal_flip));
    Eigen::AngleAxisd aa(quat_ang,quat_axis);
    Eigen::Quaterniond q(aa);

    Eigen::AngleAxisd rot90deg(M_PI/2.0,Eigen::Vector3d::UnitY());

    pose_.orientation=q;

    //if battery standing upright
    if(expansion.z()>sqrt(pow(expansion.x(),2)+pow(expansion.y(),2))){
        pose_.orientation=pose_.orientation*rot90deg;
    }



}



