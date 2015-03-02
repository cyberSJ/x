#ifndef POSE_H
#define POSE_H

template <class T, class A>
struct Pose{
  T x, y;
  A theta;
  inline Pose():x(0),y(0),theta(0){};
  inline Pose(T _x, T _y, A _theta):x(_x), y(_y), theta(_theta) {};
};



#endif
