#include "UM6LT.h"
#include "mbed.h"
#include "VNH5019.h"

/*
     M1:                        |    M2:
        INA:        p21         |       INA:        p26
        INB:        p22         |       INB:        p25
        M_PWM:      p23         |       M_PWM:      p24
        M_CS:       p20         |       M_CS:       p19
        M_ENDIAG:   p12         |       M_ENDIAG:   p11
*/

Serial pc(USBTX, USBRX);
UM6LT imu_sense(p13, p14);
DualVNH5019MotorShield shield(p21, p22, p12, p20, p23,     //M1 PINS
                              p26, p25, p11, p19, p24);    //M2 PINS
// TEST COMMENT
const double PITCH_SET = 0.0;
int sampleTime = 1000; // equivalent to 1 second...
double lasterror = 0;

int imuSetup();
void testSensorOutput();
double PID_v1(double* PID_gains, Timer t, int &lastTime);
double* twidleGains(double* param);
void setSampleTime(int newTime, double* params);
bool isTime(Timer t, int lastTime);

/*
    MOTOR CONTROLS:
    INa     INb     OUTa    OUTb    OP_MODE
     1       1       H       H      BRAKE TO Vcc
     1       0       H       L      CW
     0       1       L       H      CCW
     0       0       L       L      BRAKE TO GND
*/

int main()
{
    //MOTOR VALUES
    //float brakeVal = 1;
    float periodPWM = 0.025; // 400Hz
    float speedVal1 = 0;
    float speedVal2 = 0;
    int count = 0;

    shield.operator()(1).set_pwm_period(periodPWM);
    shield.operator()(2).set_pwm_period(periodPWM);

    shield.operator()(1).enable();
    shield.operator()(2).enable();
    pc.printf("---- MOTORS ENABLED ----\n\r");

    //IMU / PID VALUES
    Timer t;
    int lastTime = 0;
    double PID_out;
    double param[3] = {0,0,0};

    if(imuSetup() == 1)
        pc.printf("SETUP COMPLETE.");
    else
        pc.printf("FAILED TO SETUP.");

    pc.printf("---------------------------------------------------------------\n\r");



    t.start();
    twidleGains(param);

    while(1)
    {
        //MOTOR FAULT DETECTION
        pc.printf((shield.operator()(1).is_fault())?"-------------------------- M1 FAULT OCCURRED--------------------------\n\r":"");
        if(shield.operator()(1).is_fault())
            shield.operator()(1).clear_fault();
        pc.printf((shield.operator()(2).is_fault())?"-------------------------- M2 FAULT OCCURRED--------------------------\n\r":"");
        if(shield.operator()(2).is_fault())
            shield.operator()(2).clear_fault();

        /*if(isTime(t,lastTime) == true)
        {
            twidleGains(param);
        }*/

        PID_out = PID_v1(param, t, lastTime);
        pc.printf("PID OUTPUT MAIN: %f\n\r\n\n", PID_out);
        shield.operator()(1).speed(speedVal1 + PID_out);
        shield.operator()(2).speed(speedVal2 + PID_out);
        wait(0.25); //mesured in seconds...
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------
/**
 *
 * return true   if timer reached value greater than 10 seconds or less than a second
          false  otherwise
 */
bool isTime(Timer t, int lastTime)
{
    /*
        Decides if it is time to reconfigure the parameters after 10 seconds or if the PID just started...
    */
    if(t.read_us()-lastTime >= 1000000 || t.read_us()-lastTime < 1000 )
    {
        lastTime = t.read_us();
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------------------------------------------------------------------

/**
 * return int   1 if the setup was successful
 *              0 otherwise
 */
int imuSetup()
{
    int gyroBiasX = 0;
    int gyroBiasY = 0;
    int gyroBiasZ = 0;

    //Anything other than all 0 data causes checksum error...
    int data[9] = {0,0,0,0,0,0,0,0,0};

    pc.baud(115200);

    /*setCommParams(...) specifies:
        - broadcastRate: 20 - 300 HZ
        - baudrate: 9600, 14400, 19200, 38400, 57600 or 115200 (default)
        - dataToTransmit: array of ints, enables various data transmissions (Read more about UM6_COMMUNICATION 0x00 register)
        - broadcastEnabled: 1 or 0, enabled by default
    */
    imu_sense.setCommParams(20, 115200, data, 1);

    /*setConfigParams(...) specifies:
        - enabling PPS timing (not essential)
        - enabling quaternion estimation
        - enabling gyro calibration
        - enabling EKF accelerometer updates (pitch, roll correction)
        - enabling EKF magnetometer updates (yaw correction)
    */
    imu_sense.setConfigParams(1,1,1,1,1);

    //resets orientation and error covariance estimates
    imu_sense.resetEKF();

    //zeroGyros(...) updates gyro biases for more accurate readings
    imu_sense.zeroGyros(gyroBiasX, gyroBiasY, gyroBiasZ);

    //sets accelerometer reference vector for EKF estimation
    imu_sense.autoSetAccelRef();

    //sets magnetometer reference vector for EKF estimation
    imu_sense.autoSetMagRef();

    //initializes sensors and communication; returns 0 along with an error message if failure occurs, returns 1 if no error found
    return imu_sense.getStatus();
}

//------------------------------------------------------------------------------------------------------------------------------------------

void testSensorOutput()
{
        int angle, i, j, k;

        while(1)
        {

            //Accelerometer Output
            imu_sense.getAccel( i, j, k);
            pc.printf("accel_x: %f, accel_y: %f, accel_z: %f\n\r", (double)i/1000, (double)j/1000, (double)k/1000);
            /*
            //Gyro Rate Output
            imu_sense.getAngleRates(i, j, k);
            pc.printf("gyro_x: %f, gyro_y: %f, gyro_z: %f\n\r", (double)i/1000, (double)j/1000, (double)k/1000);

            //Magnetometer Output
            imu_sense.getMag(i, j, k);
            pc.printf("mag_x: %f, mag_y: %f, mag_z: %f\n\r", (double)i/1000, (double)j/1000, (double)k/1000);

            //Euler Angle Output
            imu_sense.getAngles(i, j, k);
            pc.printf("roll: %d, pitch: %d, yaw: %d\n\r", i, j, k);

            //Quaternion Output
            imu_sense.getQuaternion(angle, i, j, k);
            pc.printf("THETA: %f, i: %f, j: %f, k: %f\n\r", (double)angle, (double)i/1000, (double)j/1000, (double)k/1000);
            */
            wait(0.25); //mesured in seconds...

        }
}

//------------------------------------------------------------------------------------------------------------------------------------------

double PID_v1(double* PID_gains, Timer t, int &lastTime)
{
    double error = 0;
    double sumerror = 0;
    double differror = 0;
    double PID_out;
    int timeRead = t.read_us();

    pc.printf("TIME: %d\n\n\r", timeRead);

    int timeChange = (timeRead - lastTime);

    if(timeChange >= sampleTime)
    {
        error = (double)(PITCH_SET - imu_sense.getPitchAngle());
        pc.printf("PITCH ANGLE: %d\n\n\r",imu_sense.getPitchAngle());
        sumerror += error;
        differror = (error - lasterror)/sampleTime;
        lasterror = error;
        lastTime = timeRead;
        pc.printf("ERROR: %f\n\rDIFFERROR: %f\n\rSUMERROR: %f\n\r", error, differror, sumerror);

        for(int ii = 0; ii<3; ++ii)
        {
            pc.printf("PID %c GAIN: %f\n\r", (ii<=1)?((ii==0)? 'P' : 'I'):'D', PID_gains[ii]);
        }
        //ADD MAX CORRECTION AND MIN CORRECTION

        pc.printf("\n\rPID_OUT: %f\n\r", PID_out = error*PID_gains[0] + sumerror*PID_gains[1] + differror*PID_gains[2]);
        pc.printf("---------------------------------------------------------------\n\r");
        return PID_out;
    }
    return 0;
}

//------------------------------------------------------------------------------------------------------------------------------------------

double PID_twidle(double* PID_gains)
{
    double error = 0;
    double sumerror = 0;
    double differror = 0;
    double PID_out;

    error = (double)(PITCH_SET - imu_sense.getPitchAngle());
    sumerror += error;
    differror = (error - lasterror)/sampleTime;
    lasterror = error;
    //pc.printf("ERROR: %f\n\rDIFFERROR: %f\n\rSUMERROR: %f\n\r", error, differror, sumerror);

    PID_out = error*PID_gains[0] + sumerror*PID_gains[1] + differror*PID_gains[2];
    shield.operator()(1).speed(PID_out);
    shield.operator()(2).speed(PID_out);
    pc.printf("\n\rPID_OUT: %f\n\r", PID_out);
    //pc.printf("---------------------------------------------------------------\n\r");


    return PID_out;
}

//------------------------------------------------------------------------------------------------------------------------------------------

double* twidleGains(double* param)
{
    double diffparam[3]  = {1.0, 1.0, 1.0};
    double sum = 0;
    double threshold = 0.001;
    double PIDout;
    int count = 0;

    lasterror = 0;

    double bestPID = PID_twidle(param);

    for(int ii = 0; ii< 3; ++ii)
    {
        sum += diffparam[ii];
        pc.printf("DIFFPARAM VAL %d: %f", ii, diffparam[ii]);
    }

    //TAKE OUT COUNT CONDITION...
     while(sum > threshold)
     {
         pc.printf("\n\rCOUNT: %d, SUM: %f\n\r", count++, sum);
         for(int ii = 0; ii<3; ++ii)
         {
             param[ii] += diffparam[ii];
             pc.printf("PARAM %d: %f\n\r", ii, param[ii]);
             pc.printf("DIFFPARAM %d: %f\n\r", ii, diffparam[ii]);
             PIDout = PID_twidle(param);

             if(PIDout < bestPID)
             {
                 bestPID = PIDout;
                 diffparam[ii] *= 1.1;
             }
             else
             {
                 param[ii] -= 2*diffparam[ii];
                 PIDout = PID_twidle(param);

                 if(PIDout < bestPID)
                 {
                     bestPID = PIDout;
                     diffparam[ii] *= 1.1;
                 }

                 else
                {
                    param[ii] += diffparam[ii];
                    diffparam[ii] *= 0.9;
                }
             }

         }
         // re-evaluate sum...
         sum = 0;
         for(int ii = 0; ii< 3; ++ii)
        {
            sum += diffparam[ii];
        }
     }

     lasterror = 0;

     return param;
}

//------------------------------------------------------------------------------------------------------------------------------------------
/*
    function take from Improving the Beginner’s PID – Sample Time
    - made to be able to control sample time better
*/

void setSampleTime(int newTime, double* params)
{
   if (newTime > 0)
   {
      double ratio  = (double)newTime
                      / (double)sampleTime;

      params[1] *= ratio;
      params[2] /= ratio;

      sampleTime = (unsigned long)newTime;
   }
}
