#include "head.h"
#include "algorithm.h"
#include <HCSR04.h>

UltraSonicDistanceSensor Sensor_0(S0_TRIG,S0_ECHO);
UltraSonicDistanceSensor Sensor_1(S1_TRIG,S1_ECHO);
UltraSonicDistanceSensor Sensor_2(S2_TRIG,S2_ECHO);
UltraSonicDistanceSensor Sensor_3(S3_TRIG,S3_ECHO);
//UltraSonicDistanceSensor Sensor_4(S4_TRIG,S4_ECHO);
UltraSonicDistanceSensor Sensor_5(S5_TRIG,S5_ECHO);
tByte State = MID;
double Sensors_Current[6] = {0};
double Sensors_Prev[6] = {0};
unsigned long T1,T2,TIME_TICK = 80;
static Vehicle V1;

void SET_DC(u8 value){
  switch(value){
    case FRONT:
    digitalWrite(DC_DIR_PIN, FRONT_DIR);
    analogWrite(DC_PWM_PIN,DC_SPEED_PWM);
    break;

    case REAR:
    digitalWrite(DC_DIR_PIN,REAR_DIR);
    analogWrite(DC_PWM_PIN,DC_SPEED_PWM);
    break;

    case STOP:
    analogWrite(DC_PWM_PIN,0);
    break;
  }
}

void SET_SERVO(u8 value){
  switch(value){
    case LEFT:
    switch(State){
      case MID:
      digitalWrite(STEER_DIR_PIN,LEFT_DIR);
      analogWrite(STEER_PWM_PIN,STEER_SPEED_PWM);
      delay(Delay_time_ms);
      analogWrite(STEER_PWM_PIN,0);
      break;

      case RIGHT:
      digitalWrite(STEER_DIR_PIN,LEFT_DIR);
      analogWrite(STEER_PWM_PIN,STEER_SPEED_PWM);
      delay((Delay_time_ms *2));
      analogWrite(STEER_PWM_PIN,0);
      break;
    }
    State = LEFT;
    break;

    case MID:
    switch(State){
      case LEFT:
      digitalWrite(STEER_DIR_PIN,RIGHT_DIR);
      analogWrite(STEER_PWM_PIN,STEER_SPEED_PWM);
      delay(Delay_time_ms*0.8);
      analogWrite(STEER_PWM_PIN,0);
      break;

      case RIGHT:
      digitalWrite(STEER_DIR_PIN,LEFT_DIR);
      analogWrite(STEER_PWM_PIN,STEER_SPEED_PWM);
      delay(Delay_time_ms*0.8);
      analogWrite(STEER_PWM_PIN,0);
      break;
    }
    State = MID;
    break;

    case RIGHT:
    switch(State){
      case MID:
      digitalWrite(STEER_DIR_PIN,RIGHT_DIR);
      analogWrite(STEER_PWM_PIN,STEER_SPEED_PWM);
      delay(Delay_time_ms);
      analogWrite(STEER_PWM_PIN,0);
      break;

      case LEFT:
      digitalWrite(STEER_DIR_PIN,RIGHT_DIR);
      analogWrite(STEER_PWM_PIN,STEER_SPEED_PWM);
      delay((Delay_time_ms*2));
      analogWrite(STEER_PWM_PIN,0);
      break;
    }
    State = RIGHT;
    break;
    
  }
}

void Algorithm_init(void){
    V1.general = REST;
    V1.detect_park = STARTUP_MODE;
    V1.park_car = MOVE_FWD;
}

void Algorithm_update(void){
    T1 = millis();
    static tWord time_count = 0;
    static tWord distance_cm  = 0;
    tWord   increment=0;
    tWord   side_sensor_ratio = 0;
    
    Sensors_Current[0] = Sensor_0.measureDistanceCm();
    Sensors_Current[1] = Sensor_1.measureDistanceCm();
    Sensors_Current[2] = Sensor_2.measureDistanceCm();
    Sensors_Current[3] = Sensor_3.measureDistanceCm();
    //Sensors_Current[4] = Sensor_4.measureDistanceCm();
    Sensors_Current[5] = Sensor_5.measureDistanceCm();
    Serial.print("Sensor 0: \n");
    Serial.println(Sensors_Current[0]);
    Serial.print("Sensor 1: \n");
    Serial.println(Sensors_Current[1]);
    Serial.print("Sensor 2: \n");
    Serial.println(Sensors_Current[2]);
    Serial.print("Sensor 3: \n");
    Serial.println(Sensors_Current[3]);
    Serial.print("Sensor 4: \n");
    Serial.println(Sensors_Current[4]);
    Serial.print("Sensor 5: \n");
    Serial.println(Sensors_Current[5]);
    
    switch(V1.general){

    case REST:
        Serial.print("State: REST");
        SET_DC(STOP);
        SET_SERVO(MID);
        V1.general = DETECT_PARK;
        break;

    case DETECT_PARK:
        Serial.print("State: DETECT_PARK");
        switch(V1.detect_park){

        case STARTUP_MODE:
            Serial.print("State: DETECT_PARK -> STARTUP_MODE");
            V1.detect_park = NO_DETECTION;
            SET_DC(FRONT);
            SET_SERVO(MID);
            break;

        case NO_DETECTION:
            Serial.print("State: DETECT_PARK -> NO_DETECTION");
            if (Sensors_Current[5] <= MIN_OBSTACLE_SPACE)
                V1.detect_park = PAUSE_DETECTION;
            else if (Sensors_Current[3] >= PARK_WIDTH)
                V1.detect_park = POSSIBLE_PARK_SPACE;
            break;

        case PAUSE_DETECTION:
            Serial.print("State: DETECT_PARK -> PAUSE_DETECTION");
            SET_DC(STOP);
            if (Sensors_Current[5] >= MIN_OBSTACLE_SPACE){
                V1.detect_park = NO_DETECTION;
                SET_DC(FRONT);
                }
            break;

        case POSSIBLE_PARK_SPACE:
            Serial.print("State: DETECT_PARK -> POSSIBLE_PARK_SPACE");
            if ((Sensors_Current[3] >= PARK_WIDTH) && (Sensors_Prev[3] >= PARK_WIDTH))
                V1.detect_park = START_OF_PARK_SPACE;
            else{
                V1.detect_park = NO_DETECTION;
            }
            break;

        case START_OF_PARK_SPACE:
            Serial.print("State: DETECT_PARK -> START_OF_PARK_SPACE");
            if (Sensors_Current[3] >= PARK_WIDTH){
                time_count += TIME_TICK;
            }
            else if (Sensors_Current[2] >= PARK_WIDTH){
                time_count += TIME_TICK;
            }
            else{
                distance_cm = CAR_VELOCITY * time_count /10;
                V1.detect_park = POSSIBLE_END_OF_PARK_SPACE;
            }
            break;

        case POSSIBLE_END_OF_PARK_SPACE:
            Serial.print("State: DETECT_PARK -> POSSIBLE_END_OF_PARK_SPACE");
            if ((Sensors_Current[3] <= PARK_WIDTH) && (Sensors_Current[2] <= PARK_WIDTH)){
                V1.detect_park = END_OF_PARK_SPACE;
            }
            else{
                V1.detect_park = START_OF_PARK_SPACE;
            }
            break;

        case END_OF_PARK_SPACE:
            Serial.print("State: DETECT_PARK -> END_OF_PARK_SPACE");
            if (distance_cm >= PARK_LENGTH){
                    V1.general = STOP;
                }
            else{
                    V1.detect_park = NO_DETECTION;
                }
            distance_cm = 0;
            time_count = 0;
            break;
        }
        break;

    case STOP:
        Serial.print("State: STOP");
        SET_DC(STOP);
        SET_SERVO(MID);
        V1.general = PARK_CAR;
        break;

    case PARK_CAR:
        Serial.print("State: PARK_CAR");
        switch(V1.park_car){

        case MOVE_FWD:
            Serial.print("State: PARK_CAR -> MOVE_FWD");
            SET_SERVO(MID);
            SET_DC(FRONT);
            time_count += TIME_TICK;
            distance_cm = CAR_VELOCITY * time_count /10;
            if (distance_cm >= MAX_LENGTH_AFTER_PARK){
                    SET_DC(STOP);
                    SET_SERVO(MID);
                    V1.park_car = TURN_WHEEL_R;
                    time_count = 0;
                    distance_cm = 0;
            }
            break;

        case TURN_WHEEL_R:
            Serial.print("State: PARK_CAR -> TURN_WHEEL_R");
            SET_SERVO(LEFT);   //OPPOSITE PARKING
            SET_DC(STOP);
            V1.park_car = REVERSE_R_OUT;
            break;

        case REVERSE_R_OUT:
            Serial.print("State: PARK_CAR -> REVERSE_R_OUT");
            SET_DC(REAR);
            for(increment=0;increment< SENSORS_NUMBERS;increment++){
                if (Sensors_Current[increment] <= MIN_OBSTACLE_SPACE){
          //Sensors_Current[increment] = Sensors_GetReading(increment);
          if (Sensors_Current[increment] <= MIN_OBSTACLE_SPACE){
            SET_DC(STOP);
            SET_SERVO(MID);
            V1.park_car = MOVE_FWD;
            break;
          }
                }
            }
            side_sensor_ratio = Sensors_Current[2]/Sensors_Current[3];
            if ((side_sensor_ratio>= PARK_RATIO_UPPER) || (side_sensor_ratio <= PARK_RATIO_LOWER)){
                V1.park_car = MOVE_FWD;
            }
            if (Sensors_Current[1] < MIN_CORNER_LEN)
            V1.park_car = REVERSE_R_IN;
            break;

        case REVERSE_R_IN:
            Serial.print("State: PARK_CAR -> REVERSE_R_IN");
            if ((Sensors_Current[2] <= MIN_SIDE_PARK_SPACE) || (Sensors_Current[0] <= MIN_REAR_PARK_SPACE))
                V1.park_car = REVERSE_LEFT;
            break;

        case REVERSE_LEFT:
            Serial.print("State: PARK_CAR -> REVERSE_LEFT");
            SET_DC(STOP);
            SET_SERVO(RIGHT);
            SET_DC(REAR);
            if(Sensors_Current[0] <= MIN_OBSTACLE_SPACE)
                V1.park_car = FWD_RIGHT;
            break;

        case FWD_RIGHT:
            Serial.print("State: PARK_CAR -> FWD_RIGHT");
            SET_DC(STOP);
            SET_SERVO(LEFT);
            SET_DC(FRONT);
            if(Sensors_Current[2] >= Sensors_Current[3])
                V1.park_car = TERMINATE;
            break;

        case TERMINATE:
            Serial.print("State: PARK_CAR -> TERMINATE");
            SET_DC(STOP);
            SET_SERVO(MID);
      V1.general = FINISH;
            break;
        }
        break;
    
  case FINISH:
    Serial.print("State: FINISH");
    SET_DC(STOP);
    SET_SERVO(MID);
    break;
    }

    Sensors_Prev[0] = Sensors_Current[0];
    Sensors_Prev[1] = Sensors_Current[1];
    Sensors_Prev[2] = Sensors_Current[2];
    Sensors_Prev[3] = Sensors_Current[3];
    Sensors_Prev[4] = Sensors_Current[4];
    Sensors_Prev[5] = Sensors_Current[5];
    T2 = millis();
    TIME_TICK = T2 - T1;
}
