#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "math.h"

using namespace std;

struct Point {
  Point(float x = 0, float y = 0);
  float x;
  float y;
  Point operator-(Point p2);
};


struct Angle {
  static Angle degrees(float d);

  static Angle radians(float rad);

  float radians() const;
  float degrees() const;
  void set_degrees(float d);
  void set_radians(float theta);

  void standardize();

  bool operator==(Angle &rhs);
  operator float() { return theta; }
  Angle &operator/=(float d);
  Angle operator/(float d) const;
  Angle operator*(float d) const;
  Angle operator+(const Angle &rhs) const;
  Angle &operator+=(const Angle &rhs);
  Angle operator-();
  Angle operator-(const Angle &rhs) const;

 private:
  float theta;  // radians
};

struct Pose2d {
  Pose2d(Angle heading, Point position)
      : heading(heading), position(position) {}
  Angle heading;
  Point position;
};


//
// Angular geometry
//

float degrees(float radians);
float radians(float degrees);

//#returns theta in range of [-pi,pi)
float standardized_radians(float theta);
float standardized_degrees(float theta);

// returns theta2-theta1 in range of [-180,180)
float degrees_diff(float theta1, float theta2);

// returns length of vector (x,y)
float length(float x, float y);

//
// General geometry
//

// returns solutions to ax^2+bx+c=0
void quadratic(float a, float b, float c, float * solution1, float * solution2);

// returns distane from (x1,y1) to (x2,y2)
float distance(float x1, float y1, float x2, float y2);
float distance(Point p1, Point p2);
float distance_from_segment_to_pointt(Point start, Point end, Point p);

//
// Kinematics
//

// acceleration to go from v1 to v2 in distance d
float acceleration_for_distance_and_velocities(float x, float v1, float v2);

// returns first t greater than zero that will reach position x
float time_at_position(float x, float a, float v0, float x0 = 0.);

// returns velocity at time t with initial velocity v0 and acceleration a
float velocity_at_time(float t, float a, float v0);

// returns velocity at position x with acceleration a, initial velocity v0 and
// initial position x0
float velocity_at_position(float x, float a, float v0, float x0 = 0);

// returns unit length vector in the direction of p
Point unit_vector(Point p);

// angle between two vectors
// based on http://stackoverflow.com/a/16544330/383967
Angle angle_between(float x1, float y1, float x2, float y2);
Angle angle_between(Point p1, Point p2);

// returns direction from p1 to p2
Angle angle_to(Point p1, Point p2);

// returns y for given x based on x1,y1,x2,y2
float interpolate(float x, float x1, float y1, float x2, float y2);

float clamp(float value, float min_value, float max_value);


void test_geometry();

#endif  // GEOMETRY_H
