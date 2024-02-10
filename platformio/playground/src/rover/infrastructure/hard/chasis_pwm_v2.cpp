#include "config.h"

#if CONfIG_MAROVER_CHASIS_PWM_VERSION == 2
#include "Arduino.h"
#include "config.h"

#ifndef max
    #define max(a,b) ((a) > (b) ? (a) : (b))
#endif

#define WHEELS_LAYOUT 0

#if WHEELS_LAYOUT

// GPIO Setting
static int gpLb = 12; // Left Wheel Back
static int gpLf = 13; // Left Wheel Forward
static int gpRb = 15; // Right Wheel Back
static int gpRf = 14; // Right Wheel Forward

#else

static int gpLb = 13; // Left Wheel Back
static int gpLf = 12; // Left Wheel Forward
static int gpRb = 15; // Right Wheel Back
static int gpRf = 14; // Right Wheel Forward

#endif


static int pwm_freq = 500;
static int pwm_res = 8;

const int MAX_DUTY_CYCLE = (int)(pow(2, pwm_res) - 1); 

const int START_DUTY_CYCLE = (int)(MAX_DUTY_CYCLE / 5); // starting at 20% pwm

// const int START_DUTY_CYCLE = 0;



static int lbPc = 1;
static int lfPc = 2;
static int rbPc = 3;
static int rfPc = 4;


static int time_ms(){
   return esp_timer_get_time() /1000; 
}

static int64_t last_update = time_ms();


static bool lightState = false;
static bool moving = false;
static int move_start_time = 0;



void initChasisPWMChannels(){


  Serial.printf("Max Duty: %d, Start duty: %d\n",  MAX_DUTY_CYCLE, START_DUTY_CYCLE);
  
  ledcSetup(lbPc,pwm_freq,pwm_res);
  ledcSetup(lfPc,pwm_freq,pwm_res);
  ledcSetup(rbPc,pwm_freq,pwm_res);
  ledcSetup(rfPc,pwm_freq,pwm_res);

  ledcAttachPin(gpLb, lbPc);
  ledcAttachPin(gpLf, lfPc);
  ledcAttachPin(gpRb, rbPc);
  ledcAttachPin(gpRf, rfPc);

}


void setPWMChannel(float percent, int pwmChannel)
{

    uint32_t duty = percent > 0 ? map(percent, 0, 100, START_DUTY_CYCLE, MAX_DUTY_CYCLE) : 0;

    ledcWrite(pwmChannel, duty);
    Serial.printf("%d->%d ", pwmChannel, duty);
}


void wheelsActPwm(int nLf, int nLb, int nRf, int nRb)
{
    Serial.print("Wheels PWM: ");
    setPWMChannel(nLf, lfPc);
    setPWMChannel(nLb, lbPc);
    setPWMChannel(nRf, rfPc);
    setPWMChannel(nRb, rbPc);

    Serial.println();
}

static void stop(){
    Serial.print("Stop ");
    wheelsActPwm(0, 0, 0, 0);
    moving = false;
}


void chasis_pwm_axis(int *asix, int min,  int max){
    if (sizeof(asix) >=2){
        last_update = time_ms();


             
        int x = map(asix[0], min, max, -100, 100); 
        int y = map(asix[1],  min, max, -100, 100);
        int xMod = abs(x);
        int yMod = abs(y);
        if (xMod > 10 || yMod > 10){
            
            if (!moving) {
                move_start_time = time_ms();
                moving = true;    

            }

            //lets do some dummy vectors math
            
            int rightCorrection = 0;
            int leftCorrection = 0 ;

            int turn = xMod; //take part of movement speed for turns
            if(x > 0) {
                rightCorrection = turn;
            } else {
                leftCorrection = turn;
            }
            

            int leftTrottle = max(0, yMod - leftCorrection);
            int rightTrottle = max(0, yMod - rightCorrection);


            
            Serial.printf("Inputs [%d, %d]->[%d, %d] ", asix[0], asix[1], x, y);
            if (y < 0){
                Serial.printf("Forward L:%d%c R: %d%c => ", leftTrottle, 37, rightTrottle, 37);
                wheelsActPwm(leftTrottle, 0, rightTrottle, 0);
            } else {
                Serial.printf("Back L:%d%c R: %d%c => ", leftTrottle, 37, rightTrottle, 37);
                wheelsActPwm(0, leftTrottle, 0, rightTrottle);
            }
            
        } else {
            if(moving) {
                Serial.printf("Inputs [%d, %d] ", x, y);
                stop();
            }
        }
    }
}

#endif



