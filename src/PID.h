class PID
{
private:
    float errNow,errLast,errI,errD;
    float kp,ki,kd;
    float max;
public:
    PID(float Kp,float Ki,float Kd,float Max);
    float calcPid(float err);
};

PID::PID(float Kp,float Ki,float Kd,float Max)
{
    kp = Kp;
    ki = Ki;
    kd = Kd;
    max = Max;
    errNow = errLast = errI = errD = 0;
}

float PID::calcPid(float err){
    double output = 0;
    errNow = err;
    errD = errNow - errLast;
    errI += errNow;
    output = kp*errNow + ki*errI + kd*errD;
    
    output>max?output = max:output = output;
    output<-max?output = -max:output = output;

    return output;
}
