#ifndef _KEYBOARDS_H
#define _KEYBOARDS_H

#include"mbed.h"
Serial PC(USBTX,USBRX);

class KEYBOARDS{
    public:
    
    KEYBOARDS( DigitalOut *rowsPin_, DigitalIn *colsPin_,char* map_,int rows_,int cols_,int time_ = 30);
    void setDelay(int time_);
    char getPressedKey();
    char getPressingKey();
    int getValue(bool en = false);

    private:
    
    int rows,cols,delaytime;
    bool pressed(DigitalIn pin);
    bool pressing(DigitalIn pin);
    DigitalOut* rowsPin;
    DigitalIn* colsPin;

    char* map;

};


KEYBOARDS::KEYBOARDS( DigitalOut* rowsPin_, DigitalIn* colsPin_,char* map_,int rows_,int cols_,int time_){
    rowsPin = rowsPin_;
    colsPin = colsPin_;
    
    map = map_;
    rows = rows_;
    cols = cols_;

    delaytime = time_;
}

void KEYBOARDS::setDelay(int time_){
    delaytime = time_ ;
}

bool KEYBOARDS::pressed(DigitalIn pin){
    bool flag = false;
    if(pin.read()){
        wait_ms(delaytime);
        while(pin.read()){
            flag = true;
        }
    }
    return flag;
}

bool KEYBOARDS::pressing(DigitalIn pin){
    bool flag = false;
    if(pin.read()){
        wait_ms(delaytime);
        if(pin.read()){
            flag = true;
        }
    }
    return flag;
}

char KEYBOARDS::getPressingKey(){
    for(int i = 0;i<rows;i++){
        (*(rowsPin+i)) = 1;
        for(int j = 0;j<cols; j++){
            if(pressing((*(colsPin+j)))){
                (*(rowsPin+i)) = 0;
                return (*(map + i*rows + j));
            }
        }
        (*(rowsPin+i)) = 0;
    }
    return NULL;
}

char KEYBOARDS::getPressedKey(){
    for(int i = 0;i<rows;i++){
        (*(rowsPin+i)) = 1;
        for(int j = 0;j<cols; j++){
            if(pressed((*(colsPin+j)))){
                (*(rowsPin+i)) = 0;
                return (*(map + i*rows + j));
            }
        }
        (*(rowsPin+i)) = 0;
    }
    return NULL;
}

int KEYBOARDS::getValue(bool en){
    char c;
    bool flag = false;
    int t = 0;
    int num[4]={0},cnt = 0;
    do
    {
        c = getPressedKey();
        if(c != NULL && c >= '0' && c <= '9') {
            num[cnt] = int(c - '0');
            PC.printf("num:%d \n",num[cnt]);
            cnt ++;
            if(cnt>3) break;
        }
        if(c == '*')    flag = true;
    } while (c!='#');
    
    for(int i = 0;i<cnt;i++){
        t*=10;
        t += num[i];
    }
    if(flag&&en) t = -t;
    return t;
}

#endif