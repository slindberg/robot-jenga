#include "arm_paths.h"

int8_t enter_movement_left[] = {
-5,-5,-6,-7,-8,-9,-10,-12,-14,-16,-18,-20,-23,-24,-27,-29,-31,-34,-35,-38,-39,-42,-43,-44,-46,-46,-48,-48,-48,-49,-48,-48,-49,-48,-48,-48,-48,-48,-47,-48,-48,-47,-48,-47,-47,-48,-47,-47,-47,-47,-46,-47,-47,-46,-47,-46,-47,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-46,-45,-46,-45,-46,-45,-45,-45,-45,-45,-45,-44,-45,-44,-45,-44,-43,-42,-42,-40,-39,-37,-36,-33,-33,-30,-28,-26,-24,-23,-20,-18,-16,-15,-13,-11,-9,-9,-7,-6,-5,-5,-4
};

int8_t enter_movement_right[] = {
1,1,2,1,1,2,1,1,2,1,1,0,1,-1,0,-2,-2,-2,-4,-4,-5,-5,-7,-7,-7,-8,-9,-10,-9,-11,-10,-11,-11,-11,-12,-12,-12,-13,-12,-13,-13,-14,-13,-14,-14,-14,-15,-15,-15,-15,-15,-16,-15,-16,-17,-16,-17,-17,-17,-17,-18,-18,-18,-19,-21,-21,-22,-24,-24,-25,-26,-26,-28,-28,-29,-30,-30,-31,-32,-32,-33,-34,-34,-34,-36,-35,-36,-37,-37,-37,-38,-36,-37,-35,-35,-34,-32,-32,-29,-28,-27,-24,-23,-21,-19,-18,-15,-14,-12,-11,-9,-8,-7,-6,-5,-5,-4
};

int8_t exit_movement_left[] = {
5,4,6,6,7,8,10,11,13,14,16,19,20,22,24,26,29,30,32,34,36,37,39,40,41,43,43,44,44,45,44,45,45,44,45,45,45,46,45,46,45,46,45,46,46,46,46,46,46,46,47,46,46,46,46,45,46,46,47,46,46,47,46,47,46,47,47,47,47,47,47,47,47,48,47,48,47,48,48,48,48,48,48,48,48,48,49,48,48,49,47,47,45,45,43,41,40,37,36,33,31,29,27,25,22,20,18,16,14,12,11,9,8,6,6,5,5
};

int8_t exit_movement_right[] = {
4,5,5,6,7,8,9,11,12,14,16,17,19,21,23,25,26,28,30,31,32,34,35,36,36,37,37,37,37,37,36,36,35,35,34,33,33,33,31,31,31,29,29,29,27,27,26,25,24,23,22,22,20,19,19,18,17,18,17,17,16,17,16,16,16,15,16,15,15,14,15,14,14,14,14,13,13,13,13,12,12,12,12,11,11,11,11,10,10,9,9,8,8,7,6,6,4,5,3,3,2,1,1,0,0,-1,-1,-1,-1,-2,-1,-1,-2,-1,-1,-1,-2
};

ArmPath arm_paths[] = {
  {
    .left = {
      .intervals = enter_movement_left,
      .length = sizeof(enter_movement_left),
    },
    .right = {
      .intervals = enter_movement_right,
      .length = sizeof(enter_movement_right),
    }
  },
  {
    .left = {
      .intervals = exit_movement_left,
      .length = sizeof(exit_movement_left),
    },
    .right = {
      .intervals = exit_movement_right,
      .length = sizeof(exit_movement_right),
    }
  }
};