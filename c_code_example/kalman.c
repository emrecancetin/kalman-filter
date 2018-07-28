
/*
 * C Code for kalman filter
 * */

float Q_angle  =  0.001; //0.001
float Q_gyro   =  0.003;  //0.003
float R_angle  =  0.03;  //0.03

float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float dt, y, S;
float K_0, K_1;


float kalmanCalculate(float newAngle, float newRate,int looptime) {
    dt = float(looptime)/1000;
    x_angle += dt * (newRate - x_bias);
    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
    P_01 +=  - dt * P_11;
    P_10 +=  - dt * P_11;
    P_11 +=  + Q_gyro * dt;

    y = newAngle - x_angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;

    x_angle +=  K_0 * y;
    x_bias  +=  K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;

    return x_angle;
}

/*
 * Example of usage for accelerometer and gyro combination (modified)
 * To find slope angle
 * */

//angle_calculation
float acc_angle,gyro2angle,filtered_angle=0;
float Kp=50.00,Kd=0.0001,Ki=0.00,pid,pre_error=0.00,integral=0.00,degree_factor=0.98,error;
int pwm1,pwm2,pwm_pulse=5000;
short int  pwm_calculation_flag=0;
int counter;
double dt;
//angle_calculation

// Converting 16 bits long gyro value to radians function
float gyro_angle(long int gyro)
{
    return (gyro*4.3634)/32767;
}

// Actual angle calculation function
float angle_clc(long int x_acc, long int z_acc, long int y_gyro)
{
    acc_angle = atan2 (z_acc,x_acc);
    
    gyro2angle=gyro_angle(y_gyro);

    x_angle +=  dt*(gyro2angle - x_bias);
    P_00 +=  -  dt*(P_10 + P_01) + Q_angle;
    P_01 +=  -  dt*P_11;
    P_10 +=  -  dt*P_11;
    P_11 +=  + Q_gyro*dt;

    y = acc_angle - x_angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;

    x_angle +=  K_0 * y;
    x_bias  +=  K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    

    return x_angle*180/3.1415927;
}

// PID pwm calculation
int pwm_calculation(float angle)
{	
    /*counter is time tick variable*/
    if(counter > 1900000 || pwm_calculation_flag==1)
    {
        pwm_calculation_flag=1;
        error=92.0-angle;
        integral=integral+error*dt;
        pid = Kp*error+Kd*(error-pre_error)/dt+Ki*integral;
        pre_error = error;
            
        return pid;
    }
}