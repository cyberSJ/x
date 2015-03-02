#include <iostream>
#include "pose.h"
#include "particle.h"
#include "motionModel.h"

template <class type, class type2>
type max(type a, type2 b){
  return a > b ? a:b;
}


int main (int argc, char** argv){
  std::cout << "max is: " << max(5,3) << std::endl;

  Pose<int, double> myPose(0,0,1);
  std::cout <<  "my pose is: " << myPose.x << " " << myPose.y << " " << myPose.theta << std::endl;

  Particle particle;
  std::cout << "my particle is: " << particle.pose.x << " " << particle.pose.y << " " << particle.pose.theta << " " << particle.weight << " " << particle.cumWeight << std::endl;
 
  MotionModel motionModel;
  Particle newParticle;
//  newParticle = motionModel.propagateParticle(particle);
//  std::cout<< "new particle is: " << newParticle.pose.x << " " << newParticle.pose.y << " " << newParticle.pose.theta << " " << newParticle.weight << " " << newParticle.cumWeight << std::endl;  

  return 0;
}
