/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#include "SpacebotBase.hpp"

using namespace object_detection;

SpacebotBase::SpacebotBase()
{

}
/** @brief tries to assemble the objects based on the component candidates
  * @return successful
  */
bool SpacebotBase::assembleObject(){


    if (!modelSet_) return false;

    PCL_WARN("Base: assemble object with %d components\n",components_.size());

    points_.clear();    

    //remove all planes which do not fit any of the three distinct panels of the Space Bot base
    std::vector<Plane>::iterator compIter = components_.begin();    

    if (this->use_color_){
        for(; compIter != components_.end() ; ){
          if (((compIter ->matchesShape(1.0,boxModel_.dimensions[0],boxModel_.dimensions[1])<0.8) &&  //extension 0 and 1
              (compIter ->matchesShape(1.0,boxModel_.dimensions[0],boxModel_.dimensions[2])<0.8) &&  //extension 0 and 2
              (compIter ->matchesShape(1.0,boxModel_.dimensions[1],boxModel_.dimensions[2])<0.8)) ||  //extension 1 and 2
              compIter  ->matchesColor(boxModel_.color) < 0.8) //color has to match if using color!
              compIter = components_.erase( compIter ) ; // advances iter
          else
              ++compIter; // don't remove
        }
    }

    else {

    for(; compIter != components_.end() ; )
      if ((compIter ->matchesShape(1.0,boxModel_.dimensions[0],boxModel_.dimensions[1])<0.8) &&  //extension 0 and 1
          (compIter ->matchesShape(1.0,boxModel_.dimensions[0],boxModel_.dimensions[2])<0.8) &&  //extension 0 and 2
          (compIter ->matchesShape(1.0,boxModel_.dimensions[1],boxModel_.dimensions[2])<0.8))    //extension 1 and 2                  
          compIter = components_.erase( compIter ) ; // advances iter
      else
        ++compIter; // don't remove
    }

    if (debug_){
        PCL_WARN("Base: %d components left after shape match\n",components_.size());
        BOOST_FOREACH(Plane plane,components_)
        {
            PCL_WARN("Color of base component (r,g,b) %f,%f,%f\n",plane.getColor().r,plane.getColor().g,plane.getColor().b);
            PCL_WARN("Color matched model with %f\n",plane.matchesColor(boxModel_.color));
        }
    }
    //check if more than 1 component is available
    if (components_.size()<1) return false;

    if (components_.size()==1){ //if we have one component, try to estimate box based on this single component

        //check which panel is found. Estimate size and direction based on this single panel            
        //add point of planes to component pc
        BOOST_FOREACH(Plane plane, components_){
            points_+=*plane.getPoints();

            //matches top panel //x-y plane
            if (plane.matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.y())>0.8) //matches top panel
            {
                PCL_WARN("Found top panel: %f,%f\n",boxModel_.dimensions.x(),boxModel_.dimensions.y());
            }
            //matches side panel //x-z plane
            if (plane.matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.z())>0.8) //matches side panel
            {

                PCL_WARN("Found side panel: %f,%f\n",boxModel_.dimensions.x(),boxModel_.dimensions.z());
                //translate base along the normal away from the viewpoint
                setBasePose(plane,SIDE);
                likelihood_=0.50;
                return true;
            }

            //matches front or rear panel //y-z plane
            if (plane.matchesShape(1.0,boxModel_.dimensions.y(),boxModel_.dimensions.z())>0.8) //matches front or rear panel
            {                
                PCL_WARN("Found front or rear pannel: %f,%f\n",boxModel_.dimensions.y(),boxModel_.dimensions.z());
                //translate base along the normal away from the viewpoint
                setBasePose(plane,FRONT_REAR);
                likelihood_=0.50;
                return true;

            }            
        }
        return false;
    }

    //if more components are found, two HAVE to be related. Otherwise we assume no object is found.
    std::vector<Plane>::iterator iterA = components_.begin();
    std::vector<Plane>::iterator iterB;    
    int idA=1;
    int idB;

    //max distance between COG of two planes (is always smaller than largest edge
    double max_distance=std::max(boxModel_.dimensions.x(),boxModel_.dimensions.y());
    max_distance=std::max(max_distance,boxModel_.dimensions.z());

    for(;iterA!=--components_.end();iterA++){//run all elements and compare to each except the last one. Prefent self-compare

        iterB=iterA+1;
        idB=idA+1;
        for(;iterB!=components_.end();iterB++){//run all elements and compare to each except the last one. Prefent self-compare            
            if ((iterA->isOrthogonal(*iterB)>0.9) && (iterA->getCogDistance(*iterB)<max_distance))
            {                                                               
                points_+=*(iterA->getPoints());
                points_+=*(iterB->getPoints());

                //matches front or rear panel //y-z plane
                if (iterA->matchesShape(1.0,boxModel_.dimensions.y(),boxModel_.dimensions.z())>0.8)
                {
                    PCL_WARN("Found front or rear pannel: %f,%f\n",boxModel_.dimensions.y(),boxModel_.dimensions.z());
                    //translate base along the normal away from the viewpoint
                    setBasePose(*iterA,FRONT_REAR);
                    likelihood_=0.99;
                    return true;
                }

                if (iterB->matchesShape(1.0,boxModel_.dimensions.y(),boxModel_.dimensions.z())>0.8)
                {
                    PCL_WARN("Found front or rear pannel: %f,%f\n",boxModel_.dimensions.y(),boxModel_.dimensions.z());
                    //translate base along the normal away from the viewpoint
                    setBasePose(*iterB,FRONT_REAR);
                    likelihood_=0.99;
                    return true;
                }

                //matches top panel //x-y plane
                if (iterA->matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.y())>0.8)
                {
                    PCL_WARN("Found top panel: %f,%f\n",boxModel_.dimensions.x(),boxModel_.dimensions.y());
                }

                if (iterB->matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.y())>0.8)
                {
                    PCL_WARN("Found top panel: %f,%f\n",boxModel_.dimensions.x(),boxModel_.dimensions.y());
                }

                //matches side panel //x-z plane
                if (iterA->matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.z())>0.8)
                {
                    PCL_WARN("Found side panel: %f,%f\n",boxModel_.dimensions[0],boxModel_.dimensions[2]);
                    //translate base along the normal away from the viewpoint
                    setBasePose(*iterA,SIDE);
                    likelihood_=0.99;
                    return true;
                }
                if (iterB->matchesShape(1.0,boxModel_.dimensions.x(),boxModel_.dimensions.z())>0.8)
                {

                    PCL_WARN("Found side panel: %f,%f\n",boxModel_.dimensions[0],boxModel_.dimensions[2]);
                    //translate base along the normal away from the viewpoint
                    setBasePose(*iterB,SIDE);
                    likelihood_=0.99;
                    return true;
                }
            }
            //check components for spatial relationships (parallel, orthogonal, distance, common edge)
            idB++;

        }
        idA++;

    }

    return false;
}
void SpacebotBase::setBasePose(Plane plane,BOX_PANEL_SIDE side){

    base::Vector3d base_vector;    

    switch (side){

        case FRONT_REAR:
            pose_.position=plane.getPosition()-plane.getNormal()*boxModel_.dimensions.x()/2.0;
            //if hole is in front/rear pannel it is supposed to be the back side, otherwise flip orientation
            if (plane.hasSlot(0.04,0.10))
                base_vector=base::Vector3d (1,0,0);
            else{
                base_vector=base::Vector3d (-1,0,0);
            }
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


    pose_.orientation=q;

}


