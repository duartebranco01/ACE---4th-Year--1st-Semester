#include <Arduino.h>
#include <stdio.h>

#define AINB_PIN 4
#define AINA_PIN 5

#define BINB_PIN 2
#define BINA_PIN 3

#define ML_ENCA_PIN 6
#define ML_ENCB_PIN 7

#define MR_ENCA_PIN 8
#define MR_ENCB_PIN 9

#define IR1_PIN 28
#define IR2_PIN 27
#define IR3_PIN 26
#define IR4_PIN 22
#define IR5_PIN 21

#define TRIG_PIN 10
#define ECHO_PIN 11

#define MAX_SPEED 250
#define BASE_SPEED 125
#define SLOW_SPEED 25

#define WHEEL_RAIDIUS 5

//-----------------------------------------------------------------------------------------------------------------------------------------------

#define INFINITO 9999
#define END_NODE 21
#define X_MAX 6
#define Y_MAX 5




//ATENÇÃO: so funciona para 30=30, matriz 5x6 onde 0 é canto superior esquerdo, 29 é canto inferior direito

//adjacencia de ponto 0 ao todos os outros, se = 0 ligação nao existe
//fazer for, = 0 se distancia entre o ponto e ele mesmo ou o caminho nao existe (nao sao adjacentesw), senao = 1 

int adj[30][30] = {{0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0}};

int cost[30][30], distance[30], pred[30], path[30], path_size = 0;

int curr_maze_pos = 0, path_iterator = 0, start_node = 0;

//-----------------------------------------------------------------------------------------------------------------------------------------------

typedef struct 
{
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes_millis, tes_micros, tis_millis, tis_micros;
} fsm_t;

fsm_t fsm_Line, fsm_Junction, fsm_Obstacle, fsm_GridMaze_Random, fsm_GridMaze_Random_Obstacle;
fsm_t fsm_GridMaze_Dijkstra_Orientation, fsm_GridMaze_Dijkstra, fsm_GridMaze_Dijkstra_FollowPath;

typedef struct {
  uint8_t AINA = 0, AINB = 0, BINA = 0, BINB = 0, TRIG = 0;

} outputs_t;

typedef struct {
  uint8_t IR1 = 0, IR2 = 0, IR3 = 0, IR4 = 0, IR5 = 0, ECHO = 0;
} inputs_t;

inputs_t in, prev_in;
outputs_t out;


uint32_t curr_milis = 0, curr_micros = 0, prev_time_ultra = 0, time_echo_RE = 0, time_echo_FE = 0;

uint16_t ML_ENCA = 0, ML_ENCB = 0, MR_ENCA = 0, MR_ENCB = 0, old_ML_ENCA = 0, old_ML_ENCB = 0, old_MR_ENCA = 0, old_MR_ENCB = 0;

float distance_ultra, prev_distance_ultra;
double distance_A = 0, distance_B = 0;

uint8_t speed1=0;
uint8_t speed2=0;

float Kp_speed = 0.02;
float Ki_speed = 0.000005;
float Kd_speed = 0.1;

int error_speed;
int lastError_speed;
int P_speed;
int I_speed;
int D_speed;
float speed;
float avg = 0;

int rand2 = 0, rand3 = 0;

int gridMaze_MODE = 0;


void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes_millis = millis(); //time entering state
    fsm.tes_micros = micros();
    fsm.tis_millis = 0; //time in state
    fsm.tis_micros = 0;
  }
}



void function_fsm_Line();
void function_fsm_Junction();
void function_fsm_Obstacle();
void function_fsm_GridMaze_Random();
void function_fsm_GridMaze_Random_Obstacle();

void function_fsm_GridMaze_Dijkstra_Orientation();
void function_fsm_GridMaze_Dijkstra_FollowPath();
void function_fsm_GridMaze_Dijkstra();

void motorAction(char action, int speedA, int speedB);
void PID_speed();
void ultraSensor();
void isr_ml_enca_count();
void isr_ml_encb_count();
void isr_mr_enca_count();
void isr_mr_encb_count();

void Dijkstra();
void findPath();
void fillAdj();
void removeAdj(int i);


void setup() {

  fillAdj();

  pinMode(AINA_PIN, OUTPUT);
  pinMode(AINB_PIN, OUTPUT);

  pinMode(BINA_PIN, OUTPUT);
  pinMode(BINB_PIN, OUTPUT);

  pinMode(ML_ENCA_PIN, INPUT_PULLUP);
  pinMode(ML_ENCB_PIN, INPUT_PULLUP);

  pinMode(MR_ENCA_PIN, INPUT_PULLUP);
  pinMode(MR_ENCB_PIN, INPUT_PULLUP);

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);
  pinMode(IR5_PIN, INPUT);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  set_state(fsm_Line, 0);
  set_state(fsm_Junction, 0);
  set_state(fsm_Obstacle, 0);

  set_state(fsm_GridMaze_Random, 0);
  set_state(fsm_GridMaze_Random_Obstacle, 0);

  set_state(fsm_GridMaze_Dijkstra_Orientation, 0);
  set_state(fsm_GridMaze_Dijkstra, 0);
  set_state(fsm_GridMaze_Dijkstra_FollowPath, 0);

  attachInterrupt(digitalPinToInterrupt(ML_ENCA_PIN), isr_ml_enca_count, RISING);
  attachInterrupt(digitalPinToInterrupt(ML_ENCB_PIN), isr_ml_encb_count, RISING);
  attachInterrupt(digitalPinToInterrupt(MR_ENCA_PIN), isr_mr_enca_count, RISING);
  attachInterrupt(digitalPinToInterrupt(MR_ENCB_PIN), isr_mr_encb_count, RISING);

  Serial.begin(115200);

}

void loop() {

  curr_micros = micros();
  curr_milis = millis();
  prev_in = in;
  
  in.IR1 = !digitalRead(IR1_PIN);
  in.IR2 = !digitalRead(IR2_PIN);
  in.IR3 = !digitalRead(IR3_PIN);
  in.IR4 = !digitalRead(IR4_PIN);
  in.IR5 = !digitalRead(IR5_PIN);
  in.ECHO = digitalRead(ECHO_PIN);


  fsm_Line.tis_millis = curr_milis - fsm_Line.tes_millis;
  fsm_Junction.tis_millis = curr_milis - fsm_Junction.tes_millis;
  fsm_Obstacle.tis_millis = curr_milis - fsm_Obstacle.tes_millis;
  fsm_GridMaze_Dijkstra_Orientation.tis_millis = curr_milis - fsm_GridMaze_Dijkstra_Orientation.tes_millis;
  fsm_GridMaze_Random.tis_millis = curr_milis - fsm_GridMaze_Random.tes_millis;
  fsm_GridMaze_Random_Obstacle.tis_millis = curr_milis - fsm_GridMaze_Random_Obstacle.tes_millis;
  fsm_GridMaze_Dijkstra_FollowPath.tis_millis = curr_milis - fsm_GridMaze_Dijkstra_FollowPath.tes_millis;
  fsm_GridMaze_Dijkstra.tis_millis = curr_milis - fsm_GridMaze_Dijkstra.tes_millis;



  ultraSensor();
  function_fsm_Line();
  function_fsm_Junction();
  function_fsm_Obstacle();
  function_fsm_GridMaze_Random();
  function_fsm_GridMaze_Random_Obstacle();

  function_fsm_GridMaze_Dijkstra_Orientation();
  function_fsm_GridMaze_Dijkstra();
  function_fsm_GridMaze_Dijkstra_FollowPath();


  //motorAction('J', BASE_SPEED, BASE_SPEED);
 /*out.AINA=0; //160
  out.AINB=0;
  out.BINA=0; //150
  out.BINB=0;*/


  Serial.print("fsm_Line: ");
  Serial.print(fsm_Line.state);
  Serial.print(" | ");
  Serial.print("fsm_Junction: ");
  Serial.print(fsm_Junction.state);
  Serial.print(" | ");
  Serial.print("fsm_Obstacle: ");
  Serial.print(fsm_Obstacle.state);
  Serial.print(" | ");
  Serial.print("IR: ");  
  Serial.print(in.IR1);
  Serial.print(in.IR2);
  Serial.print(in.IR3);
  Serial.print(in.IR4);
  Serial.print(in.IR5);
  Serial.print(" | ");
  /*Serial.print("avg: ");
  Serial.print(avg);
  Serial.print(" | ");
  Serial.print("Speeds: ");
  Serial.print(out.AINA);
  Serial.print(" ");
  Serial.print(out.AINB);
  Serial.print(" | ");
  Serial.print(out.BINA);
  Serial.print(" ");
  Serial.print(out.BINB);
  Serial.print(" | ");
  Serial.print("Speed12: ");
  Serial.print(speed1);
  Serial.print(" ");
  Serial.print(speed2);
  Serial.print(" | ");
  Serial.print("rand: ");
  Serial.print(rand3);
  Serial.print(" | ");
  /*Serial.print("ML_ENCA_AB: ");
  Serial.print(ML_ENCA);
  Serial.print(" ");
  Serial.print(ML_ENCB);
  Serial.print(" | ");
  Serial.print("DAB: ");
  Serial.print(distance_A);
  Serial.print(" ");
  Serial.print(distance_B);
  Serial.print(" | ");*/
  Serial.print("Dij: ");
  Serial.print(fsm_GridMaze_Dijkstra.state);
  Serial.print(" | ");
  Serial.print("Dij_ORIENTATION: ");
  Serial.print(fsm_GridMaze_Dijkstra_Orientation.state);
  Serial.print(" | ");
  Serial.print("Dij_FOLLOWPATH: ");
  Serial.print(fsm_GridMaze_Dijkstra_FollowPath.state);
  Serial.print(" | ");
  Serial.print("CURR_POS: ");
  Serial.print(curr_maze_pos);
  Serial.print(" | ");
  Serial.print("Path + Iterator: ");
  Serial.print(path[path_iterator]);
  Serial.print(" | ");
  Serial.print("D_ultra: ");
  Serial.println(distance_ultra);
  
  //Serial.print(" | ");
  //Serial.print("S_pid: ");
  //Serial.println(speed);
  //Serial.println(trigPinLevel);
  //Serial.print(ENCA_inpulse);


  analogWrite(AINA_PIN, out.AINA);
  analogWrite(AINB_PIN, out.AINB);
  analogWrite(BINA_PIN, out.BINA);
  analogWrite(BINB_PIN, out.BINB);


}

void function_fsm_Line(){
  if(fsm_Obstacle.state != 0 || gridMaze_MODE == 1 || (gridMaze_MODE == 2)) fsm_Line.new_state = 0;
  else if(fsm_Junction.state != 0) fsm_Line.new_state = 0;
  else if(fsm_Line.state == 0 && !in.IR1 && !in.IR2 && !in.IR3 && !in.IR4 && !in.IR5 ) fsm_Line.new_state = 1; 
  else if(fsm_Line.state == 1 && fsm_Line.tis_millis >= 1000 && gridMaze_MODE != 2) fsm_Line.new_state = 2;                      
  else if(fsm_Line.state == 1 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5) >= 1) fsm_Line.new_state = 0;        
  else if(fsm_Line.state == 2 && fsm_Line.tis_millis >= 3000) fsm_Line.new_state = 3;                       
  else if(fsm_Line.state == 2 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5) >= 1) fsm_Line.new_state = 0;
  else if(fsm_Line.state == 3 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5) >= 1) fsm_Line.new_state = 0;

  set_state(fsm_Line, fsm_Line.new_state);

  if(fsm_Line.state == 0 && (gridMaze_MODE==0 || (gridMaze_MODE!=0 && gridMaze_MODE == 2 && fsm_GridMaze_Dijkstra_FollowPath.state == 1))){
    avg = (in.IR1*1000 + in.IR2*2000 + in.IR3*3000 + in.IR4*4000 + in.IR5*5000)/(in.IR1+in.IR2+in.IR3+in.IR4+in.IR5);
    PID_speed();
    if(avg <= 1000) motorAction('L', BASE_SPEED, BASE_SPEED);
    else if(avg >= 5000) motorAction('R', BASE_SPEED, BASE_SPEED);
    else motorAction('F', speed1, speed2);
  }
  else if(fsm_Line.state == 1){
    PID_speed();
    if(avg <= 2000) motorAction('L', BASE_SPEED, BASE_SPEED);
    else if(avg >= 4000) motorAction('R', BASE_SPEED, BASE_SPEED);
    else motorAction('F', speed1, speed2);
  }
  else if(fsm_Line.state == 2 ){

    if(avg < 3000) motorAction('L', BASE_SPEED, BASE_SPEED);
    else motorAction('R', BASE_SPEED, BASE_SPEED);
  }
  else if(fsm_Line.state == 3){
    motorAction('F', BASE_SPEED+10, BASE_SPEED);
  }

}

void function_fsm_Junction(){
  if(fsm_Obstacle.state != 0 || gridMaze_MODE != 0) fsm_Junction.new_state = 0;
  else if(fsm_Junction.state == 0 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5>=4)) fsm_Junction.new_state = 1;
  else if(fsm_Junction.state == 0 && (!in.IR1 && !in.IR2 && in.IR3 && in.IR4 && in.IR5)) fsm_Junction.new_state = 40;
  else if(fsm_Junction.state == 0 && (in.IR1 && in.IR2 && in.IR3 && !in.IR4 && !in.IR5)) fsm_Junction.new_state = 5;
  else if(fsm_Junction.state == 1 && fsm_Junction.tis_millis >= 400 && in.IR3) fsm_Junction.new_state = 2;
  else if(fsm_Junction.state == 1 && fsm_Junction.tis_millis >= 400 && !in.IR3) fsm_Junction.new_state = 3;
  else if(fsm_Junction.state == 2 && (!prev_in.IR3 && in.IR3)) fsm_Junction.new_state = 0;
  else if(fsm_Junction.state == 3 && (!prev_in.IR3 && in.IR3)) fsm_Junction.new_state = 0;
  else if(fsm_Junction.state == 40 && fsm_Junction.tis_millis >= 400) fsm_Junction.new_state = 4;
  else if(fsm_Junction.state == 4 && fsm_Junction.tis_millis >= 800) fsm_Junction.new_state = 0;
  else if(fsm_Junction.state == 5 && fsm_Junction.tis_millis >= 500 && in.IR3) fsm_Junction.new_state = 7;
  else if(fsm_Junction.state == 5 && fsm_Junction.tis_millis >= 500 && !in.IR3) fsm_Junction.new_state = 6; 
  else if(fsm_Junction.state == 7 && fsm_Junction.tis_millis >= 800) fsm_Junction.new_state = 0;
  else if(fsm_Junction.state == 6 && fsm_Junction.tis_millis >= 800) fsm_Junction.new_state = 0;

  set_state(fsm_Junction, fsm_Junction.new_state);
  
  if(fsm_Junction.state == 1 || fsm_Junction.state == 5 || fsm_Junction.state == 40){
    motorAction('F', BASE_SPEED+10, BASE_SPEED);
  }
  else if(fsm_Junction.state == 6){
    motorAction('L', BASE_SPEED, BASE_SPEED);
  }
  else if(fsm_Junction.state == 2){
    if(gridMaze_MODE == 0) motorAction('R', BASE_SPEED, BASE_SPEED);
  }
  else if(fsm_Junction.state == 3){
    if(gridMaze_MODE == 0) motorAction('R', BASE_SPEED, BASE_SPEED);
  }
  else if(fsm_Junction.state == 7){
    if(gridMaze_MODE == 0) motorAction('F', BASE_SPEED+10, BASE_SPEED);
  }
  else if(fsm_Junction.state == 4){
    motorAction('R', BASE_SPEED, BASE_SPEED);
  }
  
}

void function_fsm_Obstacle(){
  switch (gridMaze_MODE){
    case 0: //contornar objeto
      if(fsm_Obstacle.state == 0 && distance_ultra <= 15 && distance_ultra >= 5 && (prev_distance_ultra - distance_ultra) <= 10 ) fsm_Obstacle.new_state = 1;
      else if(fsm_Obstacle.state == 1 && fsm_Obstacle.tis_millis > 1000) fsm_Obstacle.new_state = 2;
      else if(fsm_Obstacle.state == 2 && distance_ultra <= 15 && distance_ultra >= 5) fsm_Obstacle.new_state = 1;
      else if(fsm_Obstacle.state == 2 && (in.IR1 + in.IR2 + in.IR3 + in.IR4 + in.IR5 >= 1)) fsm_Obstacle.new_state = 20;
      else if(fsm_Obstacle.state == 20 && fsm_Obstacle.tis_millis > 1000) fsm_Obstacle.new_state = 0;

      set_state(fsm_Obstacle, fsm_Obstacle.new_state);

      if(fsm_Obstacle.state == 1 || fsm_Obstacle.state == 20){
        motorAction('R', BASE_SPEED, BASE_SPEED);
      }
      else if(fsm_Obstacle.state == 2){
        motorAction('F', 100, 200);
      }
      break;

    case 1: //evitar objeto
      /*if(fsm_Obstacle.state == 0 && distance_ultra <= 15 && distance_ultra >= 5 && (prev_distance_ultra - distance_ultra) <= 10 ) fsm_Obstacle.new_state = 1;
      else if(fsm_Obstacle.state == 1 && fsm_Obstacle.tis_millis > 1500) fsm_Obstacle.new_state = 0;

      set_state(fsm_Obstacle, fsm_Obstacle.new_state);

      if(fsm_Obstacle.state == 1) motorAction('R', BASE_SPEED, BASE_SPEED);*/
      break;
    default:
      break;
  }

}



void function_fsm_GridMaze_Dijkstra_Orientation(){
  if(fsm_GridMaze_Dijkstra_Orientation.state == 0 && fsm_GridMaze_Dijkstra_FollowPath.state == 30) fsm_GridMaze_Dijkstra_Orientation.new_state = 2;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 0 && fsm_GridMaze_Dijkstra_FollowPath.state == 20) fsm_GridMaze_Dijkstra_Orientation.new_state = 3;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 1 && fsm_GridMaze_Dijkstra_FollowPath.state == 20) fsm_GridMaze_Dijkstra_Orientation.new_state = 2;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 1 && fsm_GridMaze_Dijkstra_FollowPath.state == 30) fsm_GridMaze_Dijkstra_Orientation.new_state = 3;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 2 && fsm_GridMaze_Dijkstra_FollowPath.state == 20) fsm_GridMaze_Dijkstra_Orientation.new_state = 0;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 2 && fsm_GridMaze_Dijkstra_FollowPath.state == 30) fsm_GridMaze_Dijkstra_Orientation.new_state = 1;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 3 && fsm_GridMaze_Dijkstra_FollowPath.state == 30) fsm_GridMaze_Dijkstra_Orientation.new_state = 0;
  else if(fsm_GridMaze_Dijkstra_Orientation.state == 3 && fsm_GridMaze_Dijkstra_FollowPath.state == 20) fsm_GridMaze_Dijkstra_Orientation.new_state = 1;

  set_state(fsm_GridMaze_Dijkstra_Orientation, fsm_GridMaze_Dijkstra_Orientation.new_state);
}

void function_fsm_GridMaze_Dijkstra(){
  if(fsm_GridMaze_Dijkstra.state == 0 && gridMaze_MODE == 2) fsm_GridMaze_Dijkstra.new_state = 1;
  else if(fsm_GridMaze_Dijkstra.state == 1 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5>=1)) fsm_GridMaze_Dijkstra.new_state = 2;
  else if(fsm_GridMaze_Dijkstra.state == 2) fsm_GridMaze_Dijkstra.new_state = 3;
  //else if(fsm_GridMaze_Dijkstra.state == 3 && distance_ultra <= 18 && distance_ultra >= 5) fsm_GridMaze_Dijkstra.new_state = 4;
  else if(fsm_GridMaze_Dijkstra.state == 4) fsm_GridMaze_Dijkstra.new_state = 2;
  else if(fsm_GridMaze_Dijkstra.state == 3 && curr_maze_pos == END_NODE) fsm_GridMaze_Dijkstra.new_state = 5;

  set_state(fsm_GridMaze_Dijkstra, fsm_GridMaze_Dijkstra.new_state);

  if(fsm_GridMaze_Dijkstra.state == 1) motorAction('F', BASE_SPEED+10, BASE_SPEED);
  else if(fsm_GridMaze_Dijkstra.state == 2){
    Dijkstra();
    findPath();
  }
  else if(fsm_GridMaze_Dijkstra.state == 4){

    if(fsm_GridMaze_Dijkstra_Orientation.state == 2) removeAdj(curr_maze_pos - 6);
    else if(fsm_GridMaze_Dijkstra_Orientation.state == 0) removeAdj(curr_maze_pos + 1);
    else if(fsm_GridMaze_Dijkstra_Orientation.state == 1) removeAdj(curr_maze_pos - 1);
    else if(fsm_GridMaze_Dijkstra_Orientation.state == 3) removeAdj(curr_maze_pos + 6);
    start_node = curr_maze_pos;
  }
   else if(fsm_GridMaze_Dijkstra.state == 5) motorAction('F', 0, 0);
}

void function_fsm_GridMaze_Dijkstra_FollowPath(){
  if(fsm_GridMaze_Dijkstra_FollowPath.state == 0 && fsm_GridMaze_Dijkstra.state == 3) fsm_GridMaze_Dijkstra_FollowPath.new_state = 1;
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 4 && ( (fsm_GridMaze_Dijkstra_Orientation.state == 2 && path[path_iterator+1] == curr_maze_pos + 1) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 0 && path[path_iterator+1] == curr_maze_pos + 6) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 1 && path[path_iterator+1] == curr_maze_pos - 6 ) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 3 && path[path_iterator+1] == curr_maze_pos - 1))) 

    {
      fsm_GridMaze_Dijkstra_FollowPath.new_state = 2;
    }
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 4 && ( (fsm_GridMaze_Dijkstra_Orientation.state == 2 && path[path_iterator+1] == curr_maze_pos - 1) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 0 && path[path_iterator+1] == curr_maze_pos - 6) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 1 && path[path_iterator+1] == curr_maze_pos + 6) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 3 && path[path_iterator+1] == curr_maze_pos + 1))) 

    {
      fsm_GridMaze_Dijkstra_FollowPath.new_state = 3;

    }
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 4 && ( (fsm_GridMaze_Dijkstra_Orientation.state == 2 && path[path_iterator+1] == curr_maze_pos - 6) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 0 && path[path_iterator+1] == curr_maze_pos + 1) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 1 && path[path_iterator+1] == curr_maze_pos - 1) ||
                                                            (fsm_GridMaze_Dijkstra_Orientation.state == 3 && path[path_iterator+1] == curr_maze_pos + 6))) 

    {
      fsm_GridMaze_Dijkstra_FollowPath.new_state = 1;

    }
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 2 && fsm_GridMaze_Dijkstra_FollowPath.tis_millis >= 800) fsm_GridMaze_Dijkstra_FollowPath.new_state = 20;
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 20) fsm_GridMaze_Dijkstra_FollowPath.new_state = 1;
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 3 && fsm_GridMaze_Dijkstra_FollowPath.tis_millis >= 800) fsm_GridMaze_Dijkstra_FollowPath.new_state = 30;
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 30) fsm_GridMaze_Dijkstra_FollowPath.new_state = 1;
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 1 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5>=3) && curr_maze_pos != END_NODE){
      fsm_GridMaze_Dijkstra_FollowPath.new_state = 40;
      path_iterator++;
      curr_maze_pos = path[path_iterator];
      
  } 
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 40 && fsm_GridMaze_Dijkstra_FollowPath.tis_millis >= 400) fsm_GridMaze_Dijkstra_FollowPath.new_state = 4;
  //else if(fsm_GridMaze_Dijkstra_FollowPath.state == 4) fsm_GridMaze_Dijkstra_FollowPath.new_state = 1;
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 1 && fsm_GridMaze_Dijkstra.state != 3) fsm_GridMaze_Dijkstra_FollowPath.new_state = 0;

  set_state(fsm_GridMaze_Dijkstra_FollowPath, fsm_GridMaze_Dijkstra_FollowPath.new_state);

  if(/*fsm_GridMaze_Dijkstra_FollowPath.state == 1 ||*/ fsm_GridMaze_Dijkstra_FollowPath.state == 40) motorAction('F', BASE_SPEED+10, BASE_SPEED);
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 2) motorAction('R', BASE_SPEED+10, BASE_SPEED);
  else if(fsm_GridMaze_Dijkstra_FollowPath.state == 3) motorAction('L', BASE_SPEED+10, BASE_SPEED);
}

bool R=0, L=0;
void function_fsm_GridMaze_Random(){
  if(fsm_GridMaze_Random_Obstacle.state != 0 || gridMaze_MODE != 1) fsm_GridMaze_Random.new_state = 0;
  else if(fsm_GridMaze_Random.state == 0 && (in.IR1+in.IR2+in.IR3+in.IR4+in.IR5>=4) && gridMaze_MODE) fsm_GridMaze_Random.new_state = 1;
  else if(fsm_GridMaze_Random.state == 0 && (in.IR1 && in.IR2 && in.IR3 && !in.IR4 && !in.IR5) && gridMaze_MODE) fsm_GridMaze_Random.new_state = 2;
  else if(fsm_GridMaze_Random.state == 0 && (!in.IR1 && !in.IR2 && in.IR3 && in.IR4 && in.IR5) && gridMaze_MODE) fsm_GridMaze_Random.new_state = 3;
  else if(fsm_GridMaze_Random.state == 1 && fsm_GridMaze_Random.tis_millis >= 400 && in.IR3) fsm_GridMaze_Random.new_state = 4;
  else if(fsm_GridMaze_Random.state == 1 && fsm_GridMaze_Random.tis_millis >= 400 && !in.IR3) fsm_GridMaze_Random.new_state = 5;
  else if(fsm_GridMaze_Random.state == 2 && fsm_GridMaze_Random.tis_millis >= 400 && in.IR3) fsm_GridMaze_Random.new_state = 6;
  else if(fsm_GridMaze_Random.state == 2 && fsm_GridMaze_Random.tis_millis >= 400 && !in.IR3) fsm_GridMaze_Random.new_state = 7;
  else if(fsm_GridMaze_Random.state == 3 && fsm_GridMaze_Random.tis_millis >= 400 && in.IR3) fsm_GridMaze_Random.new_state = 8;
  else if(fsm_GridMaze_Random.state == 3 && fsm_GridMaze_Random.tis_millis >= 400 && !in.IR3) fsm_GridMaze_Random.new_state = 9;
  else if((fsm_GridMaze_Random.state == 4 || fsm_GridMaze_Random.state == 5 || fsm_GridMaze_Random.state == 6 || fsm_GridMaze_Random.state == 7 || fsm_GridMaze_Random.state == 8 || fsm_GridMaze_Random.state == 9) && fsm_GridMaze_Random.tis_millis >= 900) {
    L=0;
    R=0;
    rand2 = random(2);
    rand3 = random(3);
    fsm_GridMaze_Random.new_state = 0;
  }
  set_state(fsm_GridMaze_Random, fsm_GridMaze_Random.new_state);

  if(fsm_GridMaze_Random.state == 1 || fsm_GridMaze_Random.state == 2 || fsm_GridMaze_Random.state == 3){
    motorAction('F', BASE_SPEED+10, BASE_SPEED);
  }
  else if(fsm_GridMaze_Random.state == 7){
    motorAction('L', BASE_SPEED, BASE_SPEED);
  }
  else if(fsm_GridMaze_Random.state == 9){
    motorAction('R', BASE_SPEED, BASE_SPEED);
  }
  else if(fsm_GridMaze_Random.state == 4){
    if(rand3 == 0) motorAction('F', BASE_SPEED+10, BASE_SPEED);
    else if(rand3 == 1){
      motorAction('R', BASE_SPEED, BASE_SPEED); 
      R=1;
      L=0;
    } 
    else if(rand3 == 2){
      motorAction('L', BASE_SPEED, BASE_SPEED); 
      L=1;
      R=0;
    }
  }
  else if(fsm_GridMaze_Random.state == 5){
    if(rand2 == 0){
      motorAction('L', BASE_SPEED, BASE_SPEED);
      L=1;
      R=0;
    }
    else if(rand2 == 1){
      motorAction('R', BASE_SPEED, BASE_SPEED);
      R=1;
      L=0;
    }
  }
  else if(fsm_GridMaze_Random.state == 6){
    if(rand2 == 0) {
      motorAction('L', BASE_SPEED, BASE_SPEED);
      L=1;
      R=0;
    }
    else if(rand2 == 1) motorAction('F', BASE_SPEED+10, BASE_SPEED);
  }
  else if(fsm_GridMaze_Random.state == 8){
    if(rand2 == 0) {
      motorAction('R', BASE_SPEED, BASE_SPEED);
      R=1;
      L=0;
    }
    else if(rand2 == 1) motorAction('F', BASE_SPEED+10, BASE_SPEED);
  }
}

void function_fsm_GridMaze_Random_Obstacle(){
  if(fsm_GridMaze_Random_Obstacle.state == 0 && gridMaze_MODE == 1 && distance_ultra <= 10 && distance_ultra >= 5 && R && !L) fsm_GridMaze_Random_Obstacle.new_state = 1;
  else if(fsm_GridMaze_Random_Obstacle.state == 0 && gridMaze_MODE==1 && distance_ultra <= 10 && distance_ultra >= 5 && !R && L) fsm_GridMaze_Random_Obstacle.new_state = 2;
  else if(fsm_GridMaze_Random_Obstacle.state == 0 && gridMaze_MODE==1 && distance_ultra <= 10 && distance_ultra >= 5 && !R && !L) fsm_GridMaze_Random_Obstacle.new_state = 3;
  else if((fsm_GridMaze_Random_Obstacle.state == 1 || fsm_GridMaze_Random_Obstacle.state == 2) && fsm_GridMaze_Random_Obstacle.tis_millis >= 800) fsm_GridMaze_Random_Obstacle.new_state = 3;  
  else if(fsm_GridMaze_Random_Obstacle.state == 3 && fsm_GridMaze_Random_Obstacle.tis_millis >= 800) fsm_GridMaze_Random_Obstacle.new_state = 0;

  set_state(fsm_GridMaze_Random_Obstacle, fsm_GridMaze_Random_Obstacle.new_state);

  if(fsm_GridMaze_Random_Obstacle.state == 1) motorAction('L', BASE_SPEED, BASE_SPEED);
  else if(fsm_GridMaze_Random_Obstacle.state == 2) motorAction('R', BASE_SPEED, BASE_SPEED);
  else if(fsm_GridMaze_Random_Obstacle.state == 3) motorAction('B', BASE_SPEED+10, BASE_SPEED);
}


void motorAction(char action, int speedA, int speedB){

  switch (action){
    case 'F':
      out.AINA=speedA;
      out.AINB=0;
      out.BINA=speedB;
      out.BINB=0;
      break;
    case 'B':
      out.AINA=0;
      out.AINB=speedA;
      out.BINA=0;
      out.BINB=speedB;
      break;
    case 'R':
      out.AINA=speedA;
      out.AINB=0;
      out.BINA=0;
      out.BINB=speedB;
      break;
    case 'L':
      out.AINA=0;
      out.AINB=speedB;
      out.BINA=speedA;
      out.BINB=0;
      break;
    default:
      break;
  }  
}

void PID_speed(){

  error_speed = 3000 - avg;
  P_speed = error_speed;
  I_speed += error_speed;
  D_speed = error_speed - lastError_speed;

  if(error_speed == 0) I_speed = 0;

  speed = Kp_speed*P_speed + Ki_speed*I_speed + Kd_speed*D_speed;

  speed1 = BASE_SPEED - speed;
  speed2 = BASE_SPEED + speed;
  
  if (speed1 > MAX_SPEED) {
      speed1 = MAX_SPEED;
  }
  else if (speed1 < 0) {
      speed1 = 0;
  }

  if (speed2 > MAX_SPEED) {
      speed2 = MAX_SPEED;
  }
  else if (speed2 < 0) {
      speed2 = 0;
  } 

  lastError_speed = error_speed;
}
    
void ultraSensor(){

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  prev_distance_ultra = distance_ultra;
  distance_ultra = duration * 0.034 / 2;
  
}

void isr_ml_enca_count(){
  ML_ENCA++;

  distance_A+=(double)WHEEL_RAIDIUS/(double)8;

}

void isr_ml_encb_count(){
  ML_ENCB++;

}

void isr_mr_enca_count(){
  MR_ENCA++;
  
  distance_B+=(double)WHEEL_RAIDIUS/(double)8;

}

void isr_mr_encb_count(){
  MR_ENCB++;


}



void Dijkstra() {
    
    int visited[30], count, min_distance, next_node, i, j;

    // inicializar
    // Creating cost matrix
    // se adj = 0, nao existe caminho, custo infinito
    for (i = 0; i < 30; i++){
        for (j = 0; j < 30; j++){
            if (adj[i][j] == 0) cost[i][j] = INFINITO;
            else cost[i][j] = adj[i][j];
        }
    }

    //distancia de todos os pontos a partir do start = 0
    //pred = predecessor = start
    //todos os pontos do visited[]=0, nao visitei nenhum 
    for (i = 0; i < 30; i++) {
        distance[i] = cost[start_node][i];
        pred[i] = start_node;
        visited[i] = 0;
    }

    //distancia do start ao start = 0
    //visitei o start
    //count = num de visitados
    distance[start_node] = 0;
    visited[start_node] = 1;
    count = 1;


    //enquanto nao percorri todos
    while (count < 30 - 1) {
        min_distance = INFINITO;

        //percorre todos os nos e guarda o menor distancia, next node é nó com menor distancia
        for (i = 0; i < 30; i++) {
        //distance de ponto 0 (start) a todos os outros (i) é menor que min distance atual e nao visistado
        //update min distance com a atual 
        //mindistance é distancia menor para o start dos nao visitados
        //next node é o nó com distancia menor
            if (distance[i] < min_distance && !visited[i]) {
                min_distance = distance[i];
                next_node = i;
            }
        }
        
        //apenas visito o com menor distancia
        visited[next_node] = 1;

        //distance(v)=(min(distance(v), cost+distance(w))
        //i é v, next node é w
        for (i = 0; i < 30; i++) {
            if (!visited[i]) {
                if (min_distance + cost[next_node][i] < distance[i]) {
                    distance[i] = min_distance + cost[next_node][i];
                    pred[i] = next_node;
                }
            }
        }
        count++;
    }
}

void findPath(){
    
    int curr_node = END_NODE;

    //path começa no destino
    path[0] = curr_node;
    path_size = 1;
    
    while (curr_node != start_node)
    {
        path[path_size] = pred[curr_node];
        curr_node = path[path_size];
        path_size++;
        //printf("%d\n", path_size);
    }
    
    //inverter path, terminar no destino
    for(int i=0; i<path_size/2; i++){
        int aux = path[i];
        path[i] = path[path_size-1-i];
        path[path_size-1-i] = aux;
    }

}

/*void printPath(){
    printf("\nPath: ");
    for(int i=0; i<path_size;i++) printf("%d ", path[i]);
    printf("\nPath_cost: %d\n", distance[END_NODE]);
}*/

void fillAdj(){ // so funciona para 30=30, matriz 5x6 onde 0 é canto superior esquerdo, 29 é canto inferior direito
    for(int i=0;i<30; i++){

        if(i%6!=0) adj[i][i-1]=1;
        if(!(i>=0 && i<=5)) adj[i][i-6]=1;
        if((i+1)%6!=0) adj[i][i+1]=1;
        if(!(i>=24 && i<=29)) adj[i][i+6]=1;
    }
}

void removeAdj(int i){

    //so tenho que ver que nao escrevo fora de memoria, nao importa se alterar adj de nodes que nao tocam porque fica a 0 na mesma
    if(i-1>=0) adj[i][i-1]=0;
    if(i+1<=29) adj[i][i+1]=0;
    if(i-6>=0) adj[i][i-6]=0;
    if(i+6<=29) adj[i][i+6]=0;
    

}



