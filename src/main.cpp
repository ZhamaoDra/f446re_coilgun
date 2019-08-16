#include "mbed.h"
#include<string>
#include"PID.h"
#include"keyBoards.h"
#include"ssd1306.h"
#include"bold_font.h"
#include"standard_font.h"
#include"HCSR04.h"


#define SCREE_MID 160

#define ROWS 4
#define COLS 4 
#define DELAYTIME 5
#define SERVO_SHOOT_PWM  1355


//Serial PC(USBTX,USBRX);
Serial device(PC_10,PC_11);

PwmOut servo_pitch(PB_4);
PwmOut servo_yaw(PB_10);

SSD1306 oled(PB_13,PB_15,PB_14,PB_3,PB_5); //CS,RS,DC,CLK,D
HCSR04 Sonar(PC_3,PC_2);

DigitalOut myLED(LED1);
DigitalOut charge(PA_12);

DigitalOut keyBoardRows[ROWS] = {PC_5,PC_6,PC_8,PC_9};
DigitalIn keyBoardCols[COLS] = {PA_6,PA_7,PB_6,PC_7};


char  map[ROWS][COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'},
};

KEYBOARDS keyBoard(&(keyBoardRows[0]),&(keyBoardCols[0]) ,&(map[0][0]),ROWS,COLS,DELAYTIME);

PID yawPid(0.3,0.0,0.0,100);

int yawPwmNow = 1500,pwmDelta = 0;

int servo_min = 1000;
int servo_max = 2000;
//int servo_mid = (servo_max+servo_min)/2;
int servo_yaw_mid = 1455;
int servo_pitch_mid = 1175;

int buff_r[2] = {0};

int Time_Distance[20] = {1750,1780,1800,1900,2000,2200,2300,2400,2500,2600,2700,2800,2900,3200,3300,3700,3900,4000,4100,4300};
//////////////////////   200  205  210  215  220  225  235  240  245  250  255  260  265  270  275  280  285  290  295  300  


int getChargeTime(int distance){
    int i = (distance-200)/5;
    int t = (distance - (5*i+200))*(Time_Distance[i+1]-Time_Distance[i])/5 + Time_Distance[i];
    return t;
}

int getPitch(int distance){
    return 1500;
}

int getYaw(int angle){
    return servo_yaw_mid + 10.6 * angle;
}

void onlyShoot(int distance){
        int n = 50;

    myLED = 0;
    wait(0.2);

    int pitchPwmNow,chargeTime;
    chargeTime = getChargeTime(distance);

    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("charge time:%d\r\n",chargeTime);
    oled.update();

    for(int i = 1;i<=n;i++){
        pitchPwmNow = i*(SERVO_SHOOT_PWM - servo_pitch_mid)/n + servo_pitch_mid;
        servo_pitch.pulsewidth_us(pitchPwmNow);
        wait_ms(10);
    }

    charge = 1;
    wait_ms(chargeTime);
    charge = 0;

    wait(1);

    myLED = 1;
    wait(0.5);
    myLED = 0;    
}

void shoot(int distance,int angle){
    int n = 50;

    myLED = 0;
    wait(0.2);

    int yawPwm,yawPwmNow,pitchPwmNow,chargeTime;
    yawPwm = getYaw(angle);
    chargeTime = getChargeTime(distance);

    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("charge time:%d\r\n",chargeTime);
    oled.update();

    for(int i = 1;i<=n;i++){
        yawPwmNow = i*(yawPwm - servo_yaw_mid)/n + servo_yaw_mid;
        pitchPwmNow = i*(SERVO_SHOOT_PWM - servo_pitch_mid)/n + servo_pitch_mid;
        servo_yaw.pulsewidth_us(yawPwmNow);
        servo_pitch.pulsewidth_us(pitchPwmNow);
        wait_ms(10);
    }

    

    // for(int i = servo_pitch_mid;i <= SERVO_SHOOT_PWM ;i++){
    //     servo_pitch.pulsewidth_us(i);
    //     wait_ms(3);
    // }

    charge = 1;
    wait_ms(chargeTime);
    charge = 0;

    wait(1);

    myLED = 1;
    wait(0.5);
    myLED = 0;

}


bool catchTarget(){
    int cnt = 0;
    bool flag = false;
    char c = keyBoard.getPressedKey();
    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("Target did not find.\r\n");
    oled.update();
    while(cnt<100&&c != '*'){
        if(device.readable()){
            if(cnt){
                oled.clear();
                oled.printf("-----------\r\n");
                oled.printf("Catching target!!\r\n");
                oled.printf("cnt:%d\r\n",cnt);
                oled.update();
            }
            device.scanf("a%db%dc",&buff_r[0],&buff_r[1]);
            
            if(buff_r[0]==-1&&buff_r[1]==-1&&!flag) {pwmDelta = 0; break;}
            
            flag = true;
            pwmDelta = yawPid.calcPid(buff_r[0]-SCREE_MID);

            fabs(pwmDelta)>4?yawPwmNow -= pwmDelta:yawPwmNow +=0,cnt++; 

            if(yawPwmNow>servo_max) yawPwmNow = servo_max;
            if(yawPwmNow<servo_min) yawPwmNow = servo_min;
            
            servo_yaw.pulsewidth_us(yawPwmNow);
        }
        c = keyBoard.getPressedKey();
    }
    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("Catch target!\r\n");
    oled.update();
    return flag;
}

void scanForTarget(){
    int yawPwmNow = getYaw(-30);
    bool flag = false;
    servo_yaw.pulsewidth_us(yawPwmNow);
    wait(0.5);
    for(int i=-30;i<=30;i++){
        yawPwmNow = getYaw(i);
        servo_yaw.pulsewidth_us(yawPwmNow);
        wait_ms(5);
        flag = catchTarget();
        if(flag)    break;
    }
    //catchTarget();
    if(!flag){
        yawPwmNow = getYaw(30);
        servo_yaw.pulsewidth_us(yawPwmNow);
        wait_ms(300);
    }
    
    if(!flag){
        for(int i=30;i>=-30;i--){
            yawPwmNow = getYaw(i);
            servo_yaw.pulsewidth_us(yawPwmNow);
            wait_ms(5);
            flag = catchTarget();
            if(flag)    break;
        }   
    }
    onlyShoot(250);
}

void movingShoot(int distance){
    int yawPwm = 0;
    yawPwm = getYaw(-30);
    servo_yaw.pulsewidth_us(yawPwm);
    servo_pitch.pulsewidth_us(SERVO_SHOOT_PWM);
    wait_ms(200);

    int chargeTime = getChargeTime(distance);
    charge = 1;
    wait_ms(chargeTime);
    charge = 0;

    wait(1);

    for(int i = -29;i<=30;i++){
        yawPwm = getYaw(i);
        servo_yaw.pulsewidth_us(yawPwm);
//        if(device.readable()){
            device.scanf("a%db%dc",&buff_r[0],&buff_r[1]);
            if(buff_r[0]!=-1){
                oled.printf("%d\r\n",abs(buff_r[0]-SCREE_MID));
                oled.update();
            }
            if(buff_r[0] != -1 && abs(buff_r[0]-SCREE_MID)<10){
                myLED = 1;

            }
//        }
        wait_ms(30);
    }
    myLED = 0;
    wait_ms(100);
    for(int i = 29;i>=-30;i--){
        yawPwm = getYaw(i);
        servo_yaw.pulsewidth_us(yawPwm);
//        if(device.readable()){
            if(buff_r[0]!=-1){
                oled.printf("%d\r\n",abs(buff_r[0]-SCREE_MID));
                oled.update();
            }
            device.scanf("a%db%dc",&buff_r[0],&buff_r[1]);
            if(buff_r[0] != -1 && abs(buff_r[0]-SCREE_MID)<10){
                myLED = 1;
            }
//        }
        wait_ms(30);
    }
    myLED = 0;
}


void modeA(){
    char c;
    int yawPwmNow = servo_yaw_mid,pitchPwmNow = servo_pitch_mid;
    bool shoot_flag = false;
    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    int distance = 200,angle = 0;

    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("Mode: A\r\n");
    oled.printf("Help: You can press 2,4,5,6 to control servo, and press # to shoot\r\n");
    oled.printf("Quit:press * to quit\r\n");
    oled.update();

    c = keyBoard.getPressedKey();

    while(c == NULL){
        c = keyBoard.getPressedKey();
    }

    while(c!='*'){

        switch (c)
        {
        case '2':
            pitchPwmNow -= 5;
            break;
        case '5':
            pitchPwmNow += 5;
            break;
        case '6':
            yawPwmNow -= 5;
            break;
        case '4':
            yawPwmNow += 5;
            break;
        case '0':
            yawPwmNow = servo_yaw_mid;
            pitchPwmNow = servo_pitch_mid;
            break;
        case 'C':
            charge = 1;
            break;
        case 'D':
            charge = 0;
            break;
        case '*':
            break;
        case '#':
            myLED = 1;
            wait(0.5);
            myLED = 0;
            //shoot_flag = true;
            break;
        default:
            break;
        }
        if(shoot_flag){
            oled.clear();
            oled.printf("-----------\r\n");
            oled.printf("The cannonball has already shooted.We will quit in 2s\r\n");
            oled.update();
            wait(2);
            break;
        }else{
            oled.clear();
            servo_pitch.pulsewidth_us(pitchPwmNow);
            servo_yaw.pulsewidth_us(yawPwmNow);
            oled.printf("PWM:\r\npitch: %d\r\nyaw:%d\r\n",pitchPwmNow,yawPwmNow);
            oled.update();
            wait_ms(2);            
        }
        c = keyBoard.getPressingKey();
    }
    yawPwmNow = servo_yaw_mid;
    pitchPwmNow = servo_pitch_mid;
    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    oled.clear();

}

void modeB(){
    char c;
    int yawPwmNow = servo_yaw_mid,pitchPwmNow = servo_pitch_mid;
    bool shoot_flag = false;
    int distance = 0,angle = 0;

    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    
    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("Mode: B\r\n");
    oled.printf("Help: You can press D/A to enter the distance/yaw angle,then press # to shoot,C to clear,press * to quit\r\n");
    //oled.printf("Quit:press * to quit\r\n");
    oled.update();

    c = keyBoard.getPressedKey();

    while(c == NULL){
        c = keyBoard.getPressedKey();
    }

    while(c!='*'){
        switch (c)
        {
        case 'A':
            oled.clear();
            oled.printf("-----------\r\n");
            oled.printf("Please enter the yaw angle:\r\n");
            oled.printf("press # to enter.\r\n");
            oled.update();
            angle = keyBoard.getValue(true);//for negative angle
            break;
        case 'D':
            oled.clear();
            oled.printf("-----------\r\n");
            oled.printf("Please enter the distance:\r\n");
            oled.printf("press # to enter.\r\n");
            oled.update();
            distance = keyBoard.getValue();
            break;
        case 'C':
            distance = 0;
            angle = 0;
            break;
        case '*':
            break;
        case '#':
            shoot_flag = true;
            for(int i = 2;i>0;i--){
                oled.clear();
                oled.printf("-----------\r\n");
                oled.printf("distance:%d\r\nyaw angle:%d\r\n",distance,angle);
                oled.printf("Shoot after %ds\r\n",i);
                oled.update();
                wait_ms(999);
            }
            shoot(distance,angle);
            break;
        default:
            break;
        }
        
        if(shoot_flag){
            oled.clear();
            oled.printf("-----------\r\n");
            oled.printf("The cannonball has already shooted.We will quit in 2s\r\n");
            oled.update();
            wait(2);
            break;
        }else{
            oled.clear();
            oled.printf("-----------\r\n");
            oled.printf("distance:%d\r\nyaw angle:%d\r\n",distance,angle);
            oled.update();
        }
        c = keyBoard.getPressedKey();
    }
    yawPwmNow = servo_yaw_mid;
    pitchPwmNow = servo_pitch_mid;
    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    oled.clear();
}

void modeC(){
    char c;
    int yawPwmNow = servo_yaw_mid,pitchPwmNow = servo_pitch_mid;
    bool shoot_flag = false;
    int distance = 0,angle = 0;

    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    
    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("Mode: C\r\n");
    oled.printf("Help: Press # to let the robot find target automally\r\n");
    oled.printf("Quit:press * to quit\r\n");
    oled.update();

    c = keyBoard.getPressedKey();

    while(c == NULL){
        c = keyBoard.getPressedKey();
    }
    while(c != '*'){
        if(c == '#') {
            scanForTarget();
            break;
        }
        c = keyBoard.getPressedKey();
    }
    yawPwmNow = servo_yaw_mid;
    pitchPwmNow = servo_pitch_mid;
    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    oled.clear();
}

void modeD(){
    char c;
    int yawPwmNow = servo_yaw_mid,pitchPwmNow = servo_pitch_mid;
    bool shoot_flag = false;
    int distance = 0,angle = 0;

    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    
    oled.clear();
    oled.printf("-----------\r\n");
    oled.printf("Mode: D\r\n");
    oled.printf("Help: Prese # to start auto-moving-shoot.\r\n");
    //oled.printf("Quit:press * to quit\r\n");
    oled.update();

    c = keyBoard.getPressedKey();

    while(c == NULL){
        c = keyBoard.getPressedKey();
    }

    while(c != '*'){
        if(c == '#') {
            movingShoot(keyBoard.getValue());
            break;
        }
        c = keyBoard.getPressedKey();
    }
    yawPwmNow = servo_yaw_mid;
    pitchPwmNow = servo_pitch_mid;
    servo_pitch.pulsewidth_us(pitchPwmNow);
    servo_yaw.pulsewidth_us(yawPwmNow);
    oled.clear();

}

void selectMode(char mode){
    static bool flag = 1;
    switch (mode)
    {
    case 'A':
        modeA();
        flag = 1;
        break;
    case 'B':
        modeB();
        flag = 1;
        break;
    case 'C':
        modeC();
        flag = 1;
        break;
    case 'D':
        modeD();
        flag = 1;
        break;
    default:
        if(flag){
            oled.printf("-----------\r\n");
            oled.printf("Please select Mode:\r\n");
            oled.printf("A: Manual Mode\r\n");
            oled.printf("B: Point Mode\r\n");
            oled.printf("C: Auto Mode\r\n");
            oled.printf("D: Scan Mode\r\n");
            oled.update();
            flag = 0;
        }
        break;
    }

}


void init(){
    device.baud(9600);

    oled.initialise();
    oled.clear();
    oled.set_contrast(255);
    oled.set_font(standard_font,6);
    //oled.set_font(bold_font,8);
    oled.printf("-----------\r\n");
    oled.printf("STM32F446RE START!\r\n");
    oled.update();

    charge = 0;

    keyBoardCols[0].mode(PullDown);
    keyBoardCols[1].mode(PullDown);
    keyBoardCols[2].mode(PullDown);
    keyBoardCols[3].mode(PullDown);

    servo_pitch.period_ms(20);
    servo_yaw.period_ms(20);
    servo_pitch.pulsewidth_us(servo_pitch_mid);
    servo_yaw.pulsewidth_us(servo_yaw_mid);

}



int main() {
    init();
    while(true){
        selectMode(keyBoard.getPressedKey());
    }

    // uint8_t buff_r[2];
    // device.baud(9600);
    // servo_pitch.period_ms(20);
    // servo_yaw.period_ms(20);
    // servo_pitch.pulsewidth_us((servo_max+servo_min)/2);
    // servo_yaw.pulsewidth_us((servo_max+servo_min)/2);
    // wait(1);

    // while(1){
    //     myLED = 0;
    //     if(device.readable()){
    //         device.scanf("a%db%dc",&buff_r[0],&buff_r[1]);
    //         PC.printf("x:%d y:%d \n",buff_r[0],buff_r[1]);
            
    //         pwmDelta = yawPid.calcPid(buff_r[0]-SCREE_MID);

    //         fabs(pwmDelta)>3?yawPwmNow -= pwmDelta:yawPwmNow +=0; 
            
    //         PC.printf("before PID: %d delta:%d \n",yawPwmNow,pwmDelta);

    //         if(yawPwmNow>servo_max) yawPwmNow = servo_max;
    //         if(yawPwmNow<servo_min) yawPwmNow = servo_min;
            
    //         PC.printf("after PID: %d delta:%d \n",yawPwmNow,pwmDelta);
    //         servo_yaw.pulsewidth_us(yawPwmNow);
    //     }
    // }





    // servo_pitch.pulsewidth_us((servo_max+servo_min)/2);

    // servo_pitch.pulsewidth_us(servo_min);
    // wait(0.5);
    
    // while(true)
    // {
    //     for(int i = servo_min;i<servo_max;i+=2){
    //         servo_pitch.pulsewidth_us(i);
    //         servo_yaw.pulsewidth_us(i);
    //         wait_ms(10);
    //     }
    //     wait(1);
    //     for(int i = servo_max;i>servo_min;i-= 2){
    //         servo_pitch.pulsewidth_us(i);
    //         servo_yaw.pulsewidth_us(i);
    //         wait_ms(10);
    //     }
    // }

}

