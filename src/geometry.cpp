#include "geometry.h"


float interpolate(float x, float x1, float y1, float x2, float y2){
  float m = (y2 - y1)/( x2 - x1 );
  return y1 + m * (x-x1);
}

Point::Point(float x, float y) :
  x(x),y(y){}


Point Point::operator -(Point rhs)
{
  return Point(x-rhs.x,y-rhs.y);
}


Angle Angle::degrees(float d) {
  Angle a;
  a.set_degrees(d);
  return a;
}

Angle Angle::radians(float rad) {
  Angle a;
  a.theta = rad;
  return a;
}

float Angle::radians() const {
  return theta;
}

float Angle::degrees() const  {
  return theta * 180. / M_PI;
}

void Angle::set_degrees(float d) {
  theta = d * M_PI/180.;
}

void Angle::set_radians(float theta_)
{
  theta = theta_;
}

void Angle::standardize() {
  theta = fmod(theta + 99*M_PI , 2.*M_PI) - M_PI;
}


bool Angle::operator ==(Angle &rhs)   {

  return theta == rhs.theta;
}

Angle &Angle::operator /=(float d)   {
  theta /= d;
  return *this;
}

Angle Angle::operator /(float d) const  {
  Angle rv;
  rv.theta = this->theta / d;
  return rv;
}

Angle Angle::operator *(float d) const  {
  Angle rv;
  rv.theta = this->theta * d;
  return rv;
}

Angle Angle::operator +(const Angle &rhs) const  {
  Angle rv;
  rv.theta = this->theta + rhs.theta;
  return rv;
}

Angle &Angle::operator +=(const Angle &rhs) {
  theta += rhs.theta;
  return *this;
}

Angle Angle::operator -() {
  return Angle::radians(-theta);
}

Angle Angle::operator -(const Angle &rhs) const  {
  Angle rv;
  rv.theta = this->theta - rhs.theta;
  rv.standardize();
  return rv;
}

float degrees(float radians) {
  return radians * 180 / M_PI;
}

float radians(float degrees) {
  return degrees* M_PI / 180;
}

float standardized_radians(float theta) {
  return fmod(theta + M_PI , 2.*M_PI) - M_PI;
}

float standardized_degrees(float theta) {
  return  fmod(theta + 180., 360) - 180.;
}

float degrees_diff(float theta1, float theta2) {
  return standardized_degrees(theta2 - theta1);
}

float length(float x, float y) {
  return sqrt(x*x+y*y);
}

void quadratic(float a, float b, float c, float * solution1, float * solution2) {

  *solution1 = (-b +sqrt(b*b - 4.*a*c))/(2.*a);
  *solution2 = (-b -sqrt(b*b-4.*a*c))/(2.*a);
}

float distance(float x1, float y1, float x2, float y2) {
  return length(x2-x1,y2-y1);
}

float distance(Point p1, Point p2) {
  return distance(p1.x,p1.y,p2.x,p2.y);
}

// returns distance of point p from segment from start to end
float distance_from_segment_to_pointt(Point start, Point end, Point p ) {
  float dx = end.x - start.x;
  float dy = end.y - start.y;
  float drx = p.x - start.x;
  float dry = p.y - start.y;

  float progress = (drx * dx + dry * dy)/(dx * dx + dy * dy);
  if(progress < 0)
    return distance(start,p);
  if(progress > 1)
    return distance(end,p);
  float l = length(dx,dy);
  float cte = (dry * dx - drx * dy) / l;
  return fabs(cte);
}


float velocity_at_time(float t, float a, float v0){
  return v0 + a * t;
}

float velocity_at_position(float x, float a, float v0, float x0){
  x = x-x0;
  float t = time_at_position(x,a,v0);
  return velocity_at_time(t,a,v0);
}

Point unit_vector(Point p) {
  auto l=length(p.x,p.y);
  return Point(p.x/l,p.y/l);
}

Angle angle_between(float x1, float y1, float x2, float y2) {
  float dot = x1*x2 + y1*y2; // dot product
  float det = x1*y2 - y1*x2; // determinant
  return Angle::radians( atan2(det, dot) );
}

Angle angle_between(Point p1, Point p2) {
  return angle_between(p1.x,p1.y,p2.x,p2.y);
}

Angle angle_to(Point p1, Point p2) {
  return Angle::radians(atan2(p2.y-p1.y,p2.x-p1.x));
}


float clamp(float value, float min_value, float max_value) {
  if(value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

float acceleration_for_distance_and_velocities(float d, float v1, float v2) {
  bool same_signs = (v2 >= 0) == (v1 >= 0);

  float a;
  if(same_signs) {
    a = (v2*v2 - v1*v1) / (2*d);
  } else {
    a = (v2*v2 + v1*v1) / (2*d);
  }

  return a;
}

float time_at_position(float x, float a, float v0, float x0){
  if(isnan(x)) {
    return NAN;
  }
  x = x-x0;

  if(a==0)
    return x/v0;
  float t0=NAN;
  float t1=NAN;
  quadratic(0.5*a, v0, -x, &t0, &t1);
  if (t0 < 0){
    return t1 > 0 ? t1 : NAN;
  }
  if (t1 < 0){
    return t0;
  }
  return t0 < t1 ? t0 : t1;
}