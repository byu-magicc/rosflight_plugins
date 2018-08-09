#ifndef GAZEBO_COMPAT_H //Include guard
#define GAZEBO_COMPAT_H

#if GAZEBO_MAJOR_VERSION >= 8

#define GazeboVector ignition::math::Vector3d
#define GET_X(VECTOR) VECTOR.X()
#define GET_Y(VECTOR) VECTOR.Y()
#define GET_Z(VECTOR) VECTOR.Z()
#define GET_W(VECTOR) VECTOR.W() //For quaternions
#define SET_X(VECTOR,VALUE) VECTOR.X(VALUE)
#define SET_Y(VECTOR,VALUE) VECTOR.Y(VALUE)
#define SET_Z(VECTOR,VALUE) VECTOR.Z(VALUE)
#define SET_W(VECTOR,VALUE) VECTOR.W(VALUE)

#else //GAZEBO_MAJOR_VERSION >= 8

#define GazeboVector gazebo::math::Vector3
#define GET_X(VECTOR) VECTOR.x
#define GET_Y(VECTOR) VECTOR.y
#define GET_Z(VECTOR) VECTOR.z
#define GET_W(VECTOR) VECTOR.w //For quaternions
#define SET_X(VECTOR,VALUE) VECTOR.x = VALUE
#define SET_Y(VECTOR,VALUE) VECTOR.y = VALUE
#define SET_Z(VECTOR,VALUE) VECTOR.z = VALUE
#define SET_W(VECTOR,VALUE) VECTOR.w = VALUE

#endif //GAZEBO_MAJOR_VERSION >= 8

#endif //Include guard
