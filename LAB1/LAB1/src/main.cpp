#include <Arduino.h>

#define S1_PIN 2
#define S2_PIN 3

#define LED1_PIN 6
#define LED2_PIN 7
#define LED3_PIN 8
#define LED4_PIN 9
#define LED5_PIN 10
#define LED6_PIN 11
#define LED7_PIN 12

#define MAX_BRIGHTNESS 255

unsigned long INTERVAL = 2000;

typedef struct 
{
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Input variables
uint8_t S1, prevS1;
uint8_t S2, prevS2;

// Output variables
int LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7;

// Our finite state machines
fsm_t fsm_LED1, fsm_LED2, fsm_LED3, fsm_LED4, fsm_LED5, fsm_LED6, fsm_LED7;
fsm_t fsm_BUTTONS, fsm_WhichConfig, fsm_Config0, fsm_Config1, fsm_Config2, fsm_LED7blinktime;
//fsm_t fsm_LEDOFF;
fsm_t fsm_LED1isON, fsm_LED2isON, fsm_LED3isON, fsm_LED4isON, fsm_LED5isON, fsm_LED6isON;

unsigned long prev_time=0, timer=0, last_timer=0;

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis(); //time entering state
    fsm.tis = 0; //time in state
  }
}


void setup() 
{
  // put your setup code here, to run once:
  pinMode(S1_PIN, INPUT);
  pinMode(S2_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);
  pinMode(LED6_PIN, OUTPUT);
  pinMode(LED7_PIN, OUTPUT);

  Serial.begin(115200);

  set_state(fsm_LED1, 0);
  set_state(fsm_LED2, 0);
  set_state(fsm_LED3, 0);
  set_state(fsm_LED4, 0);
  set_state(fsm_LED5, 0);
  set_state(fsm_LED6, 0);
  set_state(fsm_LED7, 0);
  set_state(fsm_BUTTONS, 0);
  set_state(fsm_WhichConfig, 0);
  set_state(fsm_Config0, 0);
  set_state(fsm_Config1, 0);
  set_state(fsm_Config2, 0);
  //set_state(fsm_LEDOFF, 0);
  set_state(fsm_LED1isON, 0);
  set_state(fsm_LED2isON, 0);
  set_state(fsm_LED3isON, 0);
  set_state(fsm_LED4isON, 0);
  set_state(fsm_LED5isON, 0);
  set_state(fsm_LED6isON, 0);
  set_state(fsm_LED7blinktime, 0);
}

void loop() 
{
  // put your main code here, to run repeatedly:

  //unsigned long time_now = millis();
  //prev_time = time_now;

  // Read the inputs
  prevS1 = S1;
  prevS2 = S2;
  S1 = !digitalRead(S1_PIN);
  S2 = !digitalRead(S2_PIN);

  // Update tis for all state machines
  unsigned long cur_time = millis();   // Just one call to millis()
  fsm_LED1.tis = cur_time - fsm_LED1.tes;
  fsm_LED2.tis = cur_time - fsm_LED2.tes;
  fsm_LED3.tis = cur_time - fsm_LED3.tes;
  fsm_LED4.tis = cur_time - fsm_LED4.tes;
  fsm_LED5.tis = cur_time - fsm_LED5.tes;
  fsm_LED6.tis = cur_time - fsm_LED6.tes;
  fsm_LED7.tis = cur_time - fsm_LED7.tes;
  fsm_BUTTONS.tis = cur_time - fsm_BUTTONS.tes;
  fsm_WhichConfig.tis = cur_time - fsm_WhichConfig.tes;
  fsm_Config0.tis = cur_time - fsm_Config0.tes;
  fsm_Config1.tis = cur_time - fsm_Config1.tes;
  fsm_Config2.tis = cur_time - fsm_Config2.tes;
  //fsm_LEDOFF.tis = cur_time - fsm_LEDOFF.tes;
  fsm_LED1isON.tis = cur_time - fsm_LED1isON.tes;
  fsm_LED2isON.tis = cur_time - fsm_LED2isON.tes;
  fsm_LED3isON.tis = cur_time - fsm_LED3isON.tes;
  fsm_LED4isON.tis = cur_time - fsm_LED4isON.tes;
  fsm_LED5isON.tis = cur_time - fsm_LED5isON.tes;
  fsm_LED6isON.tis = cur_time - fsm_LED6isON.tes;
  fsm_LED7blinktime.tis = cur_time - fsm_LED7blinktime.tes;


  //timer = cur_time - prev_time + last_timer;

  //RE
  uint8_t RE_S1 = (S1) && !(prevS1);
  uint8_t RE_S2 = (S2) && !(prevS2);

  //FE
  uint8_t FE_S1 = !(S1) && (prevS1);
  uint8_t FE_S2 = !(S2) && (prevS2);

  // Calculate next state for the first state machine

  //fsm_BUTTONS
  if(fsm_BUTTONS.state == 0 && RE_S1){
    fsm_BUTTONS.new_state = 1;
  } else if(fsm_BUTTONS.state == 0 && RE_S2){
    fsm_BUTTONS.new_state = 6;
  } else if(fsm_BUTTONS.state == 1 && fsm_BUTTONS.tis > 3000){
    fsm_BUTTONS.new_state = 2;
  } else if(fsm_BUTTONS.state == 1 && fsm_BUTTONS.tis < 3000 && FE_S1){
    fsm_BUTTONS.new_state = 5;
  } else if(fsm_BUTTONS.state == 1 && RE_S2){
    fsm_BUTTONS.new_state = 6;
  } else if(fsm_BUTTONS.state == 2 && RE_S1){
    fsm_BUTTONS.new_state = 3;
  } else if(fsm_BUTTONS.state == 3 && fsm_BUTTONS.tis > 3000){
    fsm_BUTTONS.new_state = 4;
  } else if(fsm_BUTTONS.state == 3 && FE_S1){
    fsm_BUTTONS.new_state = 2;
  } else if(fsm_BUTTONS.state == 4){
    fsm_BUTTONS.new_state = 0;
  } else if(fsm_BUTTONS.state == 5){
    fsm_BUTTONS.new_state = 0;
  } else if(fsm_BUTTONS.state == 6 && RE_S1){
    fsm_BUTTONS.new_state = 1;
  } else if(fsm_BUTTONS.state == 6 && RE_S2){
    fsm_BUTTONS.new_state = 7;
  } else if(fsm_BUTTONS.state == 7 && RE_S1){
    fsm_BUTTONS.new_state = 1;
  } else if(fsm_BUTTONS.state == 7){
    fsm_BUTTONS.new_state = 0;
  } else if(fsm_BUTTONS.state == 6 && fsm_BUTTONS.tis > 500){
    fsm_BUTTONS.new_state = 8;
  } /*else if(fsm_BUTTONS.state == 8 && RE_S1){
    fsm_BUTTONS.new_state = 1;
  } */else if(fsm_BUTTONS.state == 8 && RE_S2){
    fsm_BUTTONS.new_state = 0;
  }

  //fsm_WhichConfig
  if(fsm_WhichConfig.state == 0 && RE_S1 && ((fsm_BUTTONS.state == 2) || (fsm_BUTTONS.state == 3))){
    fsm_WhichConfig.new_state = 1;
  } else if(fsm_WhichConfig.state == 1 && RE_S1 && ((fsm_BUTTONS.state == 2) || (fsm_BUTTONS.state == 3))){
    fsm_WhichConfig.new_state = 2;
  } else if(fsm_WhichConfig.state == 1 && !((fsm_BUTTONS.state == 2) || (fsm_BUTTONS.state == 3))){
    fsm_WhichConfig.new_state = 0;
  } else if(fsm_WhichConfig.state == 2 && RE_S1 && ((fsm_BUTTONS.state == 2) || (fsm_BUTTONS.state == 3))){
    fsm_WhichConfig.new_state = 0;
  } else if(fsm_WhichConfig.state == 2 && !((fsm_BUTTONS.state == 2) || (fsm_BUTTONS.state == 3))){
    fsm_WhichConfig.new_state = 0;
  }

  //fsm_Config0
  if(fsm_Config0.state == 0 && RE_S2 && (fsm_WhichConfig.state == 0) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config0.new_state = 1;
  } else if(fsm_Config0.state == 1 && RE_S2 && (fsm_WhichConfig.state == 0) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config0.new_state = 2;
  } else if(fsm_Config0.state == 2 && RE_S2 && (fsm_WhichConfig.state == 0) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config0.new_state = 3;
  } else if(fsm_Config0.state == 3 && RE_S2 && (fsm_WhichConfig.state == 0) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config0.new_state = 0;
  } 

  //fsm_Config1
  if(fsm_Config1.state == 0 && RE_S2 && (fsm_WhichConfig.state == 1) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config1.new_state = 1;
  } else if(fsm_Config1.state == 1 && RE_S2 && (fsm_WhichConfig.state == 1) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config1.new_state = 2;
  } else if(fsm_Config1.state == 2 && RE_S2 && (fsm_WhichConfig.state == 1) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config1.new_state = 0;
  }
    
  //fsm_Config2
  if(fsm_Config2.state == 0 && RE_S2 && (fsm_WhichConfig.state == 2) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config2.new_state = 1;
  } else if(fsm_Config2.state == 1 && RE_S2 && (fsm_WhichConfig.state == 2) && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_Config2.new_state = 0;
  }

  //fsm_LEDOFF
  /*if(fsm_LEDOFF.state == 0 && (FE_S1 || FE_S2)){
    fsm_LEDOFF.new_state = 10;
  } else if(fsm_LEDOFF.state == 10 && timer > 1*INTERVAL){
    fsm_LEDOFF.new_state = 1;
  } else if(fsm_LEDOFF.state == 1 && timer > 2*INTERVAL){
    fsm_LEDOFF.new_state = 2;
  } else if(fsm_LEDOFF.state == 2 && timer > 3*INTERVAL){
    fsm_LEDOFF.new_state = 3;
  } else if(fsm_LEDOFF.state == 3 && timer > 4*INTERVAL){
    fsm_LEDOFF.new_state = 4;
  } else if(fsm_LEDOFF.state == 4 && timer > 5*INTERVAL){
    fsm_LEDOFF.new_state = 5;
  } else if(fsm_LEDOFF.state == 5 && timer > 6*INTERVAL){
    fsm_LEDOFF.new_state = 6;
  } else if(fsm_LEDOFF.state == 6 && timer > 7*INTERVAL){
    fsm_LEDOFF.new_state = 0;
  }*/

  
  //fsm_LED1isON
  if(fsm_LED1isON.state == 0 && (FE_S1 || FE_S2)){
    fsm_LED1isON.new_state = 1;
  } else if(fsm_LED1isON.state == 1 && timer > INTERVAL){
    fsm_LED1isON.new_state = 0;
  }

  //fsm_LED2isON
  if(fsm_LED2isON.state == 0 && (FE_S1 || FE_S2)){
    fsm_LED2isON.new_state = 1;
  } else if(fsm_LED2isON.state == 1 && timer > 2*INTERVAL){
    fsm_LED2isON.new_state = 0;
  }

  //fsm_LED3isON
  if(fsm_LED3isON.state == 0 && (FE_S1 || FE_S2)){
    fsm_LED3isON.new_state = 1;
  } else if(fsm_LED3isON.state == 1 && timer > 3*INTERVAL){
    fsm_LED3isON.new_state = 0;
  }

  //fsm_LED4isON
  if(fsm_LED4isON.state == 0 && (FE_S1 || FE_S2)){
    fsm_LED4isON.new_state = 1;
  } else if(fsm_LED4isON.state == 1 && timer > 4*INTERVAL){
    fsm_LED4isON.new_state = 0;
  }

  //fsm_LED5isON
  if(fsm_LED5isON.state == 0 && (FE_S1 || FE_S2)){
    fsm_LED5isON.new_state = 1;
  } else if(fsm_LED5isON.state == 1 && timer > 5*INTERVAL){
    fsm_LED5isON.new_state = 0;
  }
  
  //fsm_LED6isON
  if(fsm_LED6isON.state == 0 && (FE_S1 || FE_S2)){
    fsm_LED6isON.new_state = 1;
  } else if(fsm_LED6isON.state == 1 && timer > 6*INTERVAL){
    fsm_LED6isON.new_state = 0;
  }

  //fsm_LED1
  if(fsm_LED1.state == 0 && fsm_LED1isON.state == 1 && fsm_BUTTONS.state != 8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED1.new_state = 1;
  } else if(fsm_LED1.state == 0 && ((fsm_WhichConfig.state == 0 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)) || (fsm_Config2.state == 1 && timer > 6*INTERVAL))){
    fsm_LED1.new_state = 2;
  } else if(fsm_LED1.state == 1 && (fsm_LED1isON.state == 0 /*&& fsm_Config1.state == 0*/)){
    fsm_LED1.new_state = 0;
  } else if(fsm_LED1.state == 1 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) || (fsm_Config1.state == 1 && timer > 0.5*INTERVAL) || fsm_BUTTONS.state == 8)){
    fsm_LED1.new_state = 2;
  } else if(fsm_LED1.state == 1 && (fsm_LED1isON.state == 1 && fsm_Config1.state == 2 && timer > 0)){
    fsm_LED1.new_state = 4;
  } else if(fsm_LED1.state == 2 && (fsm_BUTTONS.state!=8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED1isON.state == 1 && (fsm_Config1.state != 1 || (fsm_Config1.state == 1 && timer <= 0.5*INTERVAL)))){
    fsm_LED1.new_state = 1;
  } else if(fsm_LED1.state == 2 && (fsm_LED1.tis > 100 && (( (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 0) || !(fsm_BUTTONS.state==2 || fsm_BUTTONS.state==3)))){
    fsm_LED1.new_state = 3;
  } else if(fsm_LED1.state == 2 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3 || fsm_BUTTONS.state==8) && fsm_LED1isON.state == 0) && !(timer > 6*INTERVAL && fsm_Config2.state == 1)){
    fsm_LED1.new_state = 0;
  } else if(fsm_LED1.state == 3 && (fsm_LED1.tis > 100 || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state != 0))){
    fsm_LED1.new_state = 2;
  } else if(fsm_LED1.state == 4 && (fsm_BUTTONS.state == 8 || (/*fsm_WhichConfig.state == 0 &&*/ (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)) || (fsm_Config1.state == 1 && timer > 0.5*INTERVAL))){
    fsm_LED1.new_state = 2;
  } else if(fsm_LED1.state == 4 && (fsm_LED1isON.state == 0 /*&& fsm_Config1.state == 2*/)){
    fsm_LED1.new_state = 0;
  } 

  //fsm_LED2
  if(fsm_LED2.state == 0 && fsm_LED2isON.state == 1 && fsm_BUTTONS.state != 8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED2.new_state = 1;
  } else if(fsm_LED2.state == 0 && ((fsm_WhichConfig.state == 1 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)) || (fsm_Config2.state == 1 && timer > 6*INTERVAL))){
    fsm_LED2.new_state = 2;
  } else if(fsm_LED2.state == 1 && (fsm_LED2isON.state == 0 /*&& fsm_Config1.state == 0*/)){
    fsm_LED2.new_state = 0;
  } else if(fsm_LED2.state == 1 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) || (fsm_Config1.state == 1 && timer > 1.5*INTERVAL) || fsm_BUTTONS.state == 8)){
    fsm_LED2.new_state = 2;
  } else if(fsm_LED2.state == 1 && (fsm_LED2isON.state == 1 && fsm_Config1.state == 2 && timer > INTERVAL)){
    fsm_LED2.new_state = 4;
  } else if(fsm_LED2.state == 2 && (fsm_BUTTONS.state!=8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED2isON.state == 1 && (fsm_Config1.state != 1 || (fsm_Config1.state == 1 && timer <= 1.5*INTERVAL)))){
    fsm_LED2.new_state = 1;
  } else if(fsm_LED2.state == 2 && ((fsm_LED2.tis > 100 && (((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1) || !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3))))){
    fsm_LED2.new_state = 3;
  } else if(fsm_LED2.state == 2 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3 || fsm_BUTTONS.state==8) && fsm_LED2isON.state == 0) && !(timer > 6*INTERVAL && fsm_Config2.state == 1)){
    fsm_LED2.new_state = 0;
  } else if(fsm_LED2.state == 3 && (fsm_LED2.tis > 100 || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state != 1))){
    fsm_LED2.new_state = 2;
  } else if(fsm_LED2.state == 4 && (fsm_BUTTONS.state == 8 || (/*fsm_WhichConfig.state == 1 &&*/ (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)) || (fsm_Config1.state == 1 && timer > 1.5*INTERVAL))){
    fsm_LED2.new_state = 2;
  } else if(fsm_LED2.state == 4 && (fsm_LED2isON.state == 0 /*&& fsm_Config1.state == 2*/)){
    fsm_LED2.new_state = 0;
  }

  //fsm_LED3
  if(fsm_LED3.state == 0 && fsm_LED3isON.state == 1 && fsm_BUTTONS.state != 8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED3.new_state = 1;
  } else if(fsm_LED3.state == 0 && ((fsm_WhichConfig.state == 2 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)) || (fsm_Config2.state == 1 && timer > 6*INTERVAL))){
    fsm_LED3.new_state = 2;
  } else if(fsm_LED3.state == 1 && (fsm_LED3isON.state == 0 /*&& fsm_Config1.state == 0*/)){
    fsm_LED3.new_state = 0;
  } else if(fsm_LED3.state == 1 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) || (fsm_Config1.state == 1 && timer > 2.5*INTERVAL) || fsm_BUTTONS.state == 8)){
    fsm_LED3.new_state = 2;
  } else if(fsm_LED3.state == 1 && (fsm_LED3isON.state == 1 && fsm_Config1.state == 2 && timer > 2*INTERVAL)){
    fsm_LED3.new_state = 4;
  } else if(fsm_LED3.state == 2 && (fsm_BUTTONS.state!=8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED3isON.state == 1 && (fsm_Config1.state != 1 || (fsm_Config1.state == 1 && timer <= 2.5*INTERVAL)))){
    fsm_LED3.new_state = 1;
  } else if(fsm_LED3.state == 2 && ((fsm_LED3.tis > 100 && (((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 2) || !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3))))){
    fsm_LED3.new_state = 3;
  } else if(fsm_LED3.state == 2 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3 || fsm_BUTTONS.state==8) && fsm_LED3isON.state == 0) && !(timer > 6*INTERVAL && fsm_Config2.state == 1)){
    fsm_LED3.new_state = 0;
  } else if(fsm_LED3.state == 3 && (fsm_LED3.tis > 100 || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state != 2))){
    fsm_LED3.new_state = 2;
  } else if(fsm_LED3.state == 4 && (fsm_BUTTONS.state == 8 || (/*fsm_WhichConfig.state == 2 &&*/ (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)) || (fsm_Config1.state == 1 && timer > 2.5*INTERVAL))){
    fsm_LED3.new_state = 2;
  } else if(fsm_LED3.state == 4 && (fsm_LED3isON.state == 0 /*&& fsm_Config1.state == 2*/)){
    fsm_LED3.new_state = 0;
  }

  //fsm_LED4
  if(fsm_LED4.state == 0 && fsm_LED4isON.state == 1 && fsm_BUTTONS.state != 8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED4.new_state = 1;
  } else if(fsm_LED4.state == 0 && (fsm_Config2.state == 1 && timer > 6*INTERVAL)){
    fsm_LED4.new_state = 2;
  } else if(fsm_LED4.state == 1 && (fsm_LED4isON.state == 0 /*&& fsm_Config1.state == 0*/)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 1 && ((fsm_Config1.state == 1 && timer > 3.5*INTERVAL) || fsm_BUTTONS.state == 8)){
    fsm_LED4.new_state = 2;
  } else if(fsm_LED4.state == 1 && fsm_LED4isON.state == 1 && fsm_Config1.state == 2 && timer > 3*INTERVAL){
    fsm_LED4.new_state = 4;
  } else if(fsm_LED4.state == 1 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED4.new_state = 5;
  } else if(fsm_LED4.state == 2 && fsm_BUTTONS.state != 8 && fsm_LED4isON.state == 1 && (fsm_Config1.state != 1 || (fsm_Config1.state == 1 && timer <= 3.5*INTERVAL))){
    fsm_LED4.new_state = 1;
  } else if(fsm_LED4.state == 2 && fsm_LED4.tis > 100 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){ // && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3 para parar de piscar quando entra no config se ja estava a piscar
    fsm_LED4.new_state = 3;
  } else if(fsm_LED4.state == 2 && (fsm_LED4isON.state == 0 && !(timer > 6*INTERVAL && fsm_Config2.state == 1) && fsm_BUTTONS.state != 8)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 3 && (fsm_LED4.tis > 100)){
    fsm_LED4.new_state = 2;
  } else if(fsm_LED4.state == 4 && (fsm_BUTTONS.state == 8 || (fsm_Config1.state == 1 && timer > 3.5*INTERVAL))){
    fsm_LED4.new_state = 2;
  } else if(fsm_LED4.state == 4 && (fsm_LED4isON.state == 0 /*&& fsm_Config1.state == 2*/)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 4 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED4.new_state = 6;
  } else if(fsm_LED4.state == 5 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 1)){
    fsm_LED4.new_state = 1;
  } else if(fsm_LED4.state == 5 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 0)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 6 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 1)){
    fsm_LED4.new_state = 4;
  } else if(fsm_LED4.state == 6 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 0)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 4 && (fsm_Config1.state == 0 || (fsm_Config1.state == 1 && timer<3.5*INTERVAL))){
    fsm_LED4.new_state = 1;
  }

  //fsm_LED5
  if(fsm_LED5.state == 0 && fsm_LED5isON.state == 1 && fsm_BUTTONS.state != 8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED5.new_state = 1;
  } else if(fsm_LED5.state == 0 && (fsm_Config2.state == 1 && timer > 6*INTERVAL)){
    fsm_LED5.new_state = 2;
  } else if(fsm_LED5.state == 1 && (fsm_LED5isON.state == 0 /*&& fsm_Config1.state == 0*/)){
    fsm_LED5.new_state = 0;
  } else if(fsm_LED5.state == 1 && ((fsm_Config1.state == 1 && timer > 4.5*INTERVAL) || fsm_BUTTONS.state == 8)){
    fsm_LED5.new_state = 2;
  } else if(fsm_LED5.state == 1 && fsm_LED5isON.state == 1 && fsm_Config1.state == 2 && timer > 4*INTERVAL){
    fsm_LED5.new_state = 4;
  } else if(fsm_LED5.state == 1 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED5.new_state = 5;
  } else if(fsm_LED5.state == 2 && fsm_BUTTONS.state!=8 && fsm_LED5isON.state == 1 && (fsm_Config1.state != 1 || (fsm_Config1.state == 1 && timer <= 4.5*INTERVAL))){
    fsm_LED5.new_state = 1;
  } else if(fsm_LED5.state == 2 && fsm_LED5.tis > 100 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED5.new_state = 3;
  } else if(fsm_LED5.state == 2 && fsm_BUTTONS.state != 8 && fsm_LED5isON.state == 0 && !(timer > 6*INTERVAL && fsm_Config2.state == 1)){
    fsm_LED5.new_state = 0;
  } else if(fsm_LED5.state == 3 && (fsm_LED5.tis > 100)){
    fsm_LED5.new_state = 2;
  } else if(fsm_LED5.state == 4 && (fsm_BUTTONS.state == 8 || (fsm_Config1.state == 1 && timer > 4.5*INTERVAL))){
    fsm_LED5.new_state = 2;
  } else if(fsm_LED5.state == 4 && (fsm_LED5isON.state == 0 /*&& fsm_Config1.state == 2*/)){
    fsm_LED5.new_state = 0;
  } else if(fsm_LED5.state == 4 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED5.new_state = 6;
  } else if(fsm_LED5.state == 5 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 1)){
    fsm_LED5.new_state = 1;
  } else if(fsm_LED5.state == 6 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 1)){
    fsm_LED5.new_state = 4;
  } else if(fsm_LED5.state == 4 && (fsm_Config1.state == 0 || (fsm_Config1.state == 1 && timer<4.5*INTERVAL))){
    fsm_LED5.new_state = 1;
  } else if(fsm_LED4.state == 5 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 0)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 6 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 0)){
    fsm_LED4.new_state = 0;
  }

  //fsm_LED6
  if(fsm_LED6.state == 0 && fsm_LED6isON.state == 1 && fsm_BUTTONS.state != 8 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED6.new_state = 1;
  } else if(fsm_LED6.state == 0 && (fsm_Config2.state == 1 && timer > 6*INTERVAL)){
    fsm_LED6.new_state = 2;
  } else if(fsm_LED6.state == 1 && (fsm_LED6isON.state == 0 /*&& fsm_Config1.state == 0*/)){
    fsm_LED6.new_state = 0;
  } else if(fsm_LED6.state == 1 && ((fsm_Config1.state == 1 && timer > 5.5*INTERVAL) || fsm_BUTTONS.state == 8)){
    fsm_LED6.new_state = 2;
  } else if(fsm_LED6.state == 1 && fsm_LED6isON.state == 1 && fsm_Config1.state == 2 && timer > INTERVAL){
    fsm_LED6.new_state = 4;
  } else if(fsm_LED6.state == 1 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED6.new_state = 5;
  } else if(fsm_LED6.state == 2 && fsm_BUTTONS.state !=8 &&fsm_LED6isON.state == 1 && (fsm_Config1.state != 1 || (fsm_Config1.state == 1 && timer <= 5.5*INTERVAL))){
    fsm_LED6.new_state = 1;
  } else if(fsm_LED6.state == 2 && fsm_LED6.tis > 100 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED6.new_state = 3;
  } else if(fsm_LED6.state == 2 && fsm_BUTTONS.state !=8 &&fsm_LED6isON.state == 0 && !(timer > 6*INTERVAL && fsm_Config2.state == 1)){
    fsm_LED6.new_state = 0;
  } else if(fsm_LED6.state == 3 && fsm_LED6.tis > 100){
    fsm_LED6.new_state = 2;
  } else if(fsm_LED6.state == 4 && (fsm_BUTTONS.state == 8 || (fsm_Config1.state == 1 && timer > 5.5*INTERVAL))){
    fsm_LED6.new_state = 2;
  } else if(fsm_LED6.state == 4 && (fsm_LED6isON.state == 0 /*&& fsm_Config1.state == 2*/)){
    fsm_LED6.new_state = 0;
  } else if(fsm_LED6.state == 4 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED6.new_state = 6;
  } else if(fsm_LED6.state == 5 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 1)){
    fsm_LED6.new_state = 1;
  } else if(fsm_LED6.state == 6 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 1)){
    fsm_LED6.new_state = 4;
  } else if(fsm_LED6.state == 4 && (fsm_Config1.state == 0 || (fsm_Config1.state == 1 && timer<5.5*INTERVAL))){
    fsm_LED6.new_state = 1;
  } else if(fsm_LED4.state == 5 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 0)){
    fsm_LED4.new_state = 0;
  } else if(fsm_LED4.state == 6 && (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_LED4isON.state == 0)){
    fsm_LED4.new_state = 0;
  }

  //fsm_LED7
  if(fsm_LED7.state == 0 && ((!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_Config2.state == 0 && timer > 6*INTERVAL)) || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_WhichConfig.state == 1 && (fsm_Config1.state == 0 /*|| fsm_Config1.state == 1*/)) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 0))))){
    fsm_LED7.new_state = 1;
  } else if(fsm_LED7.state == 0 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_WhichConfig.state == 0 /*|| (fsm_WhichConfig.state == 1 && fsm_Config1.state == 1)*/ || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 1)))){
    fsm_LED7.new_state = 2;
  } else if(fsm_LED7.state == 0 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 2)){
    fsm_LED7.new_state = 4;
  } else if(fsm_LED7.state == 0 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 1)){
    fsm_LED7.new_state = 5;
  } else if(fsm_LED7.state == 1 && timer < 6*INTERVAL && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3)){
    fsm_LED7.new_state = 0;
  } else if(fsm_LED7.state == 1 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_WhichConfig.state == 0 /*|| (fsm_WhichConfig.state == 1 && fsm_Config1.state == 1 && fsm_LED7.tis > INTERVAL/2)*/ || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 1)))){
    fsm_LED7.new_state = 2;
  } else if(fsm_LED7.state == 1 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 2)){
    fsm_LED7.new_state = 4;
  } else if(fsm_LED7.state == 1 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 1)){
    fsm_LED7.new_state = 5;
  } else if(fsm_LED7.state == 2 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_LED7.tis > INTERVAL && fsm_WhichConfig.state == 0) || (fsm_LED7.tis > 100 && ((fsm_WhichConfig.state == 1 && fsm_Config1.state == 1) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 1)))))){
    fsm_LED7.new_state = 3;
  } else if(fsm_LED7.state == 2 && ((!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_Config2.state == 0 && timer > 6*INTERVAL)) || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_WhichConfig.state == 1 && (fsm_Config1.state == 0 /*|| fsm_Config1.state == 1*/)) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 0))))){
    fsm_LED7.new_state = 1;
  } else if(fsm_LED7.state == 2 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (timer < 6*INTERVAL || (fsm_Config2.state == 1 && timer > 6*INTERVAL))){
    fsm_LED7.new_state = 0;
  } else if(fsm_LED7.state == 2 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 2){
    fsm_LED7.new_state = 4;
  } else if(fsm_LED7.state == 2 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && (fsm_LED7blinktime.state == 1 && fsm_LED7blinktime.tis>INTERVAL))){
    fsm_LED7.new_state = 5;
  } else if(fsm_LED7.state == 3 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (timer < 6*INTERVAL || (fsm_Config2.state == 1 && timer > 6*INTERVAL))){
    fsm_LED7.new_state = 0;
  } else if(fsm_LED7.state == 3 && ((!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_Config2.state == 0 && timer > 6*INTERVAL)) || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_WhichConfig.state == 1 && (fsm_Config1.state == 0 /*|| fsm_Config1.state == 1*/)) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 0))))){
    fsm_LED7.new_state = 1;
  } else if(fsm_LED7.state == 3 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_LED7.tis > INTERVAL && fsm_WhichConfig.state == 0) || (fsm_LED7.tis > 100 && ((fsm_WhichConfig.state == 1 && fsm_Config1.state == 1) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 1)))))){
    fsm_LED7.new_state = 2;
  } else if(fsm_LED7.state == 3 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 2)){
    fsm_LED7.new_state = 4;
  } else if(fsm_LED7.state == 4 && (((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 2 && LED_7 == 1) || (!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (timer < 6*INTERVAL || (fsm_Config2.state == 1 && timer > 6*INTERVAL))))){
    fsm_LED7.new_state = 0;
  } else if(fsm_LED7.state == 4 && ((!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_Config2.state == 0 && timer > 6*INTERVAL)) || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_WhichConfig.state == 1 && (fsm_Config1.state == 0 || fsm_Config1.state == 1)) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 0))))){
    fsm_LED7.new_state = 1;
  } else if(fsm_LED7.state == 4 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_WhichConfig.state == 0 /*|| (fsm_WhichConfig.state == 1 && fsm_Config1.state == 1)*/ || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 1)))){
    fsm_LED7.new_state = 2;
  } else if(fsm_LED7.state == 5 && ((!(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (fsm_Config2.state == 0 && timer > 6*INTERVAL)) || ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && ((fsm_WhichConfig.state == 1 && (fsm_Config1.state == 0 /*|| fsm_Config1.state == 1*/)) || (fsm_WhichConfig.state == 2 && fsm_Config2.state == 0))))){
    fsm_LED7.new_state = 1;
  } else if(fsm_LED7.state == 5 && !(fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && (timer < 6*INTERVAL || (fsm_Config2.state == 1 && timer > 6*INTERVAL))){
    fsm_LED7.new_state = 0;
  } else if(fsm_LED7.state == 5 && ((fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 2)){
    fsm_LED7.new_state = 4;
  } else if(fsm_LED7.state == 5 && (fsm_BUTTONS.state == 2 || fsm_BUTTONS.state == 3) && fsm_WhichConfig.state == 1 && fsm_Config1.state == 1 && fsm_LED7.tis> INTERVAL/2){
    fsm_LED7.new_state = 2;
  }

  //fsm_LED7blinktime
  if(fsm_LED7blinktime.state == 0 && (fsm_LED7.state==2 || fsm_LED7.state==3)){
    fsm_LED7blinktime.new_state = 1;
  } else if(fsm_LED7blinktime.state == 1 && !((fsm_LED7.state==2 || fsm_LED7.state==3))){
    fsm_LED7blinktime.new_state = 0;
  }

  // Update the states
  set_state(fsm_LED1, fsm_LED1.new_state);
  set_state(fsm_LED2, fsm_LED2.new_state);
  set_state(fsm_LED3, fsm_LED3.new_state);
  set_state(fsm_LED4, fsm_LED4.new_state);
  set_state(fsm_LED5, fsm_LED5.new_state);
  set_state(fsm_LED6, fsm_LED6.new_state);
  set_state(fsm_LED7, fsm_LED7.new_state);
  set_state(fsm_BUTTONS, fsm_BUTTONS.new_state);
  set_state(fsm_WhichConfig, fsm_WhichConfig.new_state);
  set_state(fsm_Config0, fsm_Config0.new_state);
  set_state(fsm_Config1, fsm_Config1.new_state);
  set_state(fsm_Config2, fsm_Config2.new_state);
  //set_state(fsm_LEDOFF, fsm_LEDOFF.new_state);
  set_state(fsm_LED1isON, fsm_LED1isON.new_state);
  set_state(fsm_LED2isON, fsm_LED2isON.new_state);
  set_state(fsm_LED3isON, fsm_LED3isON.new_state);
  set_state(fsm_LED4isON, fsm_LED4isON.new_state);
  set_state(fsm_LED5isON, fsm_LED5isON.new_state);
  set_state(fsm_LED6isON, fsm_LED6isON.new_state);
  set_state(fsm_LED7blinktime, fsm_LED7blinktime.new_state);

  // Actions set by the current state of the first state machine

  if (fsm_BUTTONS.state == 2){
    prev_time = cur_time;
  } else if (fsm_BUTTONS.state == 3){
    prev_time = cur_time;
  } else if (fsm_BUTTONS.state == 4){
    //algo
  } else if (fsm_BUTTONS.state == 5){
    //prev_time = timer;
    last_timer = 0;
  } else if (fsm_BUTTONS.state == 7){
    last_timer += INTERVAL;
  } else if (fsm_BUTTONS.state == 8 || fsm_BUTTONS.state == 6){ //VER AQUI
    prev_time = cur_time;
  }

  if(fsm_Config0.state == 0){
    INTERVAL = 2000;
  } else if (fsm_Config0.state == 1){
    INTERVAL = 4000;
  } else if (fsm_Config0.state == 2){
    INTERVAL = 8000;
  } else if (fsm_Config0.state == 3){
    INTERVAL = 1000;
  }

 

  if(fsm_LED1.state == 0 || fsm_LED1.state == 2){
    LED_1 = 0;
  }else if (fsm_LED1.state == 1 || fsm_LED1.state == 3){
    LED_1 = MAX_BRIGHTNESS;
  }
  else if(fsm_LED1.state == 4){
    LED_1=MAX_BRIGHTNESS*(INTERVAL-timer)/INTERVAL;
  }

  if(fsm_LED2.state == 0 || fsm_LED2.state == 2){
    LED_2 = 0;
  } else if (fsm_LED2.state == 1 || fsm_LED2.state == 3){
    LED_2 = MAX_BRIGHTNESS;
  } else if(fsm_LED2.state == 4){
    LED_2=MAX_BRIGHTNESS*(2*INTERVAL-timer)/(INTERVAL);
  }

  if(fsm_LED3.state == 0 || fsm_LED3.state == 2){
    LED_3 = 0;
  }else if (fsm_LED3.state == 1 || fsm_LED3.state == 3){
    LED_3 = MAX_BRIGHTNESS;
  } else if(fsm_LED3.state == 4){
    LED_3=MAX_BRIGHTNESS*(3*INTERVAL-timer)/(INTERVAL);
  }

  if(fsm_LED4.state == 0 || fsm_LED4.state == 2 || fsm_LED4.state == 5 || fsm_LED4.state == 6){
    LED_4 = 0;
  } else if (fsm_LED4.state == 1 || fsm_LED4.state == 3){
    LED_4 = MAX_BRIGHTNESS;
  } else if(fsm_LED4.state == 4){
    LED_4=MAX_BRIGHTNESS*(4*INTERVAL-timer)/(INTERVAL);
  }

  if(fsm_LED5.state == 0 || fsm_LED5.state == 2 || fsm_LED5.state == 5 || fsm_LED5.state == 6){
    LED_5 = 0;
  } else if (fsm_LED5.state == 1 || fsm_LED5.state == 3){
    LED_5 = MAX_BRIGHTNESS;
  } else if(fsm_LED5.state == 4){
    LED_5=MAX_BRIGHTNESS*(5*INTERVAL-timer)/(INTERVAL);
  }

  if(fsm_LED6.state == 0 || fsm_LED6.state == 2 || fsm_LED6.state == 5 || fsm_LED6.state == 6){
    LED_6 = 0;
  }else if (fsm_LED6.state == 1 || fsm_LED6.state == 3){
    LED_6 = MAX_BRIGHTNESS;
  } else if(fsm_LED6.state == 4){
    LED_6=MAX_BRIGHTNESS*(6*INTERVAL-timer)/(INTERVAL);
  }

  if(fsm_LED7.state == 1){
    LED_7 = MAX_BRIGHTNESS;
  }
  else if(fsm_LED7.state == 2){
    LED_7 = MAX_BRIGHTNESS;
  }
  else if(fsm_LED7.state == 0){
    LED_7 = 0;
  }
  else if(fsm_LED7.state == 3){
    LED_7 = 0;
  } else if(fsm_LED7.state == 4){
    LED_7 = MAX_BRIGHTNESS*(INTERVAL-fsm_LED7.tis)/INTERVAL;
  }

  

  // Set the outputs
  
  analogWrite(LED1_PIN, LED_1);
  analogWrite(LED2_PIN, LED_2);
  analogWrite(LED3_PIN, LED_3);
  analogWrite(LED4_PIN, LED_4);
  analogWrite(LED5_PIN, LED_5);
  analogWrite(LED6_PIN, LED_6);
  analogWrite(LED7_PIN, LED_7);

  // Debug using the serial port

  //if(fsm_LED7.tis == 1999 || fsm_LED7.tis == 2000 || fsm_LED7.tis == 0 || fsm_LED7.tis == 1){

  Serial.print("FSM_BUTTONS:");
  Serial.print(fsm_BUTTONS.state);
  Serial.print(" FSM_WHICHCONFIG:");
  Serial.print(fsm_WhichConfig.state);
  Serial.print(" FSM_CONFIG0:");
  Serial.print(fsm_Config0.state);
  Serial.print(" FSM_CONFIG1:");
  Serial.print(fsm_Config1.state);
  Serial.print(" FSM_CONFIG2:");
  Serial.print(fsm_Config2.state);
  //Serial.print(" FSM_LED1isON:");
  //Serial.print(fsm_LED1isON.state);
  Serial.print(" FSM_LED1:");
  Serial.print(fsm_LED1.state);
  //Serial.print(" FSM_LED2isON:");
  //Serial.print(fsm_LED2isON.state);
  Serial.print(" FSM_LED2:");
  Serial.print(fsm_LED2.state);
  //Serial.print(" FSM_LED3isON:");
  //Serial.print(fsm_LED3isON.state);
  Serial.print(" FSM_LED3:");
  Serial.print(fsm_LED3.state);
  Serial.print(" FSM_LED4:");
  Serial.print(fsm_LED4.state);
  Serial.print(" FSM_LED5:");
  Serial.print(fsm_LED5.state);
  Serial.print(" FSM_LED6:");
  Serial.print(fsm_LED6.state);
  Serial.print(" FSM_LED7:");
  Serial.print(fsm_LED7.state);
  Serial.print(" FSM_LED7.tis:");
  Serial.print(fsm_LED7.tis);
  Serial.print(" LED7.tis:");
  Serial.print(fsm_LED7.tis);
  Serial.print(" LED_7:");
  Serial.print(LED_7);
  
  // Serial.print("\n");}


  timer = cur_time - prev_time + last_timer;

  
  Serial.print(" timer: ");
  Serial.print(timer);
  Serial.print(" INTERVAL: ");
  Serial.print(INTERVAL);
  Serial.print("\n");

  prev_time = cur_time;
  last_timer = timer;
}