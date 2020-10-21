/******************* TRABALHO DE GRADUAÇÃO**************************
 *  
                 SISTEMA DE NAVEGAÇÃO GPS/INS
              Aluna: Rosangela Miyeko Shigenari
            Orientador: Prof. Dr. Sergio Ronaldo

 Descrição: Sistema de navegação desenvolvido a partir da fusão sensorial dos dados obtidos pela 
 IMU LSM9DS1 e pelo GPS-NEO 6M através do Filtro de Kalman.
 
/*******************************GPS NEO-6M****************************************
  11 to GPS Module TX
  10 to GPS Module RX
  5v to GPS Module VCC
  GND to GPS Module GND
 **********************************************************************************/
 /*******************************IMU LSM9DS1***************************************
  SDA to IMU Module SDA
  SDA to IMU Module SDA
  3.3V to IMU Module VCC
  GND to IMU Module GND
 ***************************************************************************************/
 
#include <SoftwareSerial.h> //library to use software interface to use GPS module
#include <TinyGPS.h>   //functions to capture MNEA data
#include <SPI.h> // SPI library included for SparkFunLSM9DS1
#include <Wire.h> // I2C library included for SparkFunLSM9DS1
#include <SparkFunLSM9DS1.h> // SparkFun LSM9DS1 library
#include <MatrixMath.h> //matrix library


SoftwareSerial SerialGPS(10, 11);  //define ports to receive GPS data
TinyGPS GPS; //define the variable to receive GPS data
LSM9DS1 imu; //define the variable to receive IMU data

#define radius                           6371000
#define GYROSCOPE_SENSITIVITY            56.16
#define dt                               0.01
#define DECLINATION                     -8.58 // Declination (degrees) in Boulder, CO.
/************ Deviation of IMU *******************/
#define noise_ax_IMU (0.140)
#define noise_ay_IMU (0.140)
#define noise_wz_IMU (0.07)

/***** Deviation from GPS data position *********/
#define noise_px_GPS  (0.1*5)
#define noise_py_GPS  (0.1*5)
#define noise_psi  0.01

#define     dtK 0.1 //time step form Kalman filter propagation

#define     rad2deg(x)        (x*57.2957795131)     // * 180/pi = 57.2957795131
#define     deg2rad(x)        (x*0.01745329252)     // * pi/180 =  0.0174529252
#define     beta 0.0001      // parameter to avoid filter divergence
#define     saturate(value, lower, upper) value = min(max(value, lower), upper) //this function saturates a given value between lower and upper limits

/****** Auxiliaries variables to control the system ***********/
int counter = 0;
float initialX = 0;
float initialY = 0;
float initialZ = 0;
float coordinateX, coordinateY, coordinateZ;
float pitch;
float roll;
float yaw;
float aux_gps = 0;
/********* Kalman Filter Parameters *************/
float bias[3]; //system bias
float K[8][3];  // kalman gain matrix
float P_initial[8][8]; // error covariance matrix (initial state)
float P[8][8];  // covariance error matrix
float R[3][3];  // GPS covariance matrix
float Q[3][3];  // noise covariance matrix
float X_INS[5] = {0, 0, 0, 0, 0}; //Vector of true states in the inertial system. X_INS(t) = [vx(t),vy(t), px(t), py(t), ψ(t)]
float U[3];     // input vector for kalman filter
float S_inv[3][3]; // measurement prediction covariance, used at the validation gate (auxiliar measurement is ok or not)

float Ident[8][8]; // identity matrix 8x8
float H[3][8];
float A[8][8];
float B[8][3];
float X_E[8];
float Y_E[3];

boolean EKF_convergence_OK=false; //flag that indicates the convergence of the filter
byte prop_step=0, updt_step=1;

float GPS_pos[3]  = {0, 0, 0}; // Position calculated by GPS
float cumulative_error[3]   = {0, 0, 0};  // The cumulative error of the position
float estimate_pos[3]  = {0, 0, 0};  // The estimate position of the vehicle
float estimate_cumulative_error[3] = {0, 0, 0};  // used to estimate value of the cumulative error of the position

// for IMU
float actual_roll=0, actual_pitch=0, actual_yaw =0; // angles at IMU in degrees
float accFromImu[3] = {0,0,0}; //acceleration from accelerometer, X, Y and Z axis
float gyrZ = 0;

float temp1[8][8], temp2[8][8], temp3[8][8], temp4[8][8], temp5[8][8], temp6[8][8]; //Temporary matrices to store values within a bigger calculation


// ----------------------------------------------------------

float error_psi;
boolean did_update=false;
int x_est_signal=1;

uint16_t initLSM9DS1();
void kalmanFilterIniatialization();
void kalmanFilterPropagation();
void kalmanFilterUpdate();
void getGPSData(TinyGPS &gps);
void complementaryFilter (float ax, float ay, float az, float gx, float gy, float mx, float my, float mz);
void buildImuData (LSM9DS1 imu);
void printEstimatedOutput();
void printXINS();
void printPositionFromGPS();
void printAccData();
void printForCentral();

void setup() {
  
  SerialGPS.begin(9600);  
  uint16_t status = initLSM9DS1();

  pitch= 0;
  roll = 0;
  yaw = 0;

  kalmanFilterIniatialization();
 
  Serial.println("Inicializando Sistema...");
  Serial.begin(115200);   
}

void loop() {


  while (SerialGPS.available()) {
      /******************  IMU DATA  *********************************/
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() && imu.accelAvailable() && imu.magAvailable()){
    imu.readGyro(); // gx, gy, and gz variables with the most current data.
    imu.readAccel(); // ax, ay, and az variables with the most current data.
    imu.readMag(); // mx, my, and mz variables with the most current data.
  }
    if (GPS.encode(SerialGPS.read())) {
       Serial.write(SerialGPS.read());
       getGPSData(GPS);
       complementaryFilter (imu.ax, imu.ay,  imu.az, imu.gx, imu.gy, imu.mx, imu.my, imu.mz);
       buildImuData (imu);


       // ------ Running the EKF ------
       //Update step
      if(updt_step==1){ //every time there was a new message from GPS, runs the Update
        did_update=true;

        //Next time this task runs after the 1Hz task, do the second step of the kalman filter update
        kalmanFilterUpdate(); // Kalman Filter Update  step 1 ~10ms
        updt_step=2;
      
        kalmanFilterUpdate(); // Kalman Filter Update  step 2 ~10ms
        updt_step=0;
        x_est_signal=1;
    }
    else{
      did_update=false;
      updt_step = 1;
    }
    //*** END Update ***
    // -----------------------------------

      
    // ------ Running the EKF ------
    //*** Propagation Step (X[k+1] and P[k+1])***
    prop_step=1;
    kalmanFilterPropagation();  // Kalman Filter Propagation step 1 ~10ms
    delay(10);
    prop_step=2;
    kalmanFilterPropagation();  // Kalman Filter Propagation  step 2 ~10ms

    
    prop_step=0;
    x_est_signal=-1;
    
    //*** END Propagation ***
    // -----------------------------------

    
   printForCentral();

  }
  
}
}
