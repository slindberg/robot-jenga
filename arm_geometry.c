#include "arm_geometry.h"

// Finds the intersection between two circles defined by center points and
// radii. Note that there are two intersections, and this method reliably
// chooses the "same" one each time, i.e. it always chooses the clockwise
// intersection
struct Point2D circle_intersection(struct Point2D *p0, struct Point2D *p1, float r0, float r1) {
  float dx = p1->x - p0->x;
  float dy = p1->y - p0->y;

  // distance between the two circle center points
  float d = sqrt(dx*dx + dy*dy);

  // distance from center of first circle to the centerline between two
  // circular intersection points
  float a = (r0*r0 - r1*r1 + d*d)/(2*d);

  // normal distance from line between the points and the intersections
  float h = sqrt(r0*r0 - a*a);

  // intersection of the line between the points and the line between the
  // two circle intersection points
  float p2x = p0->x + a*dx/d;
  float p2y = p0->y + a*dy/d;

  // calculate one of the intersections using p2 and h
  float p3x = p2x - h*dy/d;
  float p3y = p2y + h*dx/d;

  struct Point2D p3 = { p3x, p3y };

  return p3;
}

// Find the 4-quadrant angle defined by the two points
float input_angle(struct Point2D *p0, struct Point2D *p1) {
  float dx = p1->x - p0->x;
  float dy = p1->y - p0->y;

  return atan2(dy, dx);
}
