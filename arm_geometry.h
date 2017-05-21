
#include <math.h>

// positions of guide rods
#define X_LEFT 90
#define Y_LEFT 104.45625
#define X_RIGHT 30
#define Y_RIGHT 244.45625

// lengths of arm links
#define L_LEFT_INPUT 336.675
#define L_LEFT_FOLLOWER 231
#define L_RIGHT_INPUT 243.325
#define L_RIGHT_FOLLOWER 190

struct Point2D {
  float x;
  float y;
};

struct Point2D circle_intersection(struct Point2D *p_0, struct Point2D *p_1, float r_0, float r_1);

float input_angle(struct Point2D *p_0, struct Point2D *p_1);
