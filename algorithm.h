#ifndef ALGORITHM__H
#define ALGORITHM__H

#include "head.h"

#define PARK_LENGTH           (150)    //cm
#define PARK_WIDTH            (60)     //cm
#define CAR_VELOCITY          (10)    //cm/sec
#define MIN_OBSTACLE_SPACE    (30)    //cm
#define MAX_LENGTH_AFTER_PARK (40)  //cm
#define PARK_RATIO_UPPER      (4)
#define PARK_RATIO_LOWER      (2)
#define SENSORS_NUMBERS       (6)
#define MIN_SIDE_PARK_SPACE   (10)    //cm
#define MIN_REAR_PARK_SPACE   (10)    //cm
#define MIN_CORNER_LEN        (40)  //cm

typedef unsigned char       tByte;
typedef unsigned int        tWord;


typedef enum{
    REST,
    DETECT_PARK,
    STOP,
    PARK_CAR,
  FINISH
} st_general;

typedef enum{
    STARTUP_MODE,
    NO_DETECTION,
    PAUSE_DETECTION,
    POSSIBLE_PARK_SPACE,
    START_OF_PARK_SPACE,
    POSSIBLE_END_OF_PARK_SPACE,
    END_OF_PARK_SPACE,
    FINISHED_DETECTION
} st_detect_park;

typedef enum{
    MOVE_FWD,
    TURN_WHEEL_R,
    REVERSE_R_OUT,
    REVERSE_R_IN,
    REVERSE_LEFT,
    FWD_RIGHT,
    TERMINATE
} st_park_car;

typedef struct{
    tByte general;
    tByte detect_park;
    tByte park_car;
} Vehicle;


void Algorithm_init(void);
void Algorithm_update(void);

#endif // ALGORITHM__H

