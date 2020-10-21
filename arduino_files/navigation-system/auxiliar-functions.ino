//Integration, find X_INS[k+1] from X_INS[k] and U[k]
void euler_integration2()
{
    //this routine takes 100us 
  saturate(U[0], 100, -100);
  saturate(U[1], 100, -100); // +-2 m/s^2 max acceleration
  saturate(U[2], deg2rad(90), deg2rad(-90)); // saturation at +-90º/s angle speed 
   
  X_INS[4] = X_INS[4] + U[2]*dt; //psi in X_INS[4] is in [radians]
  X_INS[0] = X_INS[0] + cos(X_INS[4])*U[0]*dt - sin(X_INS[4])*U[1]*dt; //velocity in [m/s]
  X_INS[1] = X_INS[1] + sin(X_INS[4])*U[0]*dt + cos(X_INS[4])*U[1]*dt; //velocity in [m/s]
  X_INS[2] = X_INS[2] + X_INS[0] * dt; //position in [m]
  X_INS[3] = X_INS[3] + X_INS[1] * dt; //position in [m]


  saturate(X_INS[0], 5, -5); //X inertial frame velocity
  saturate(X_INS[1], 5, -5); //Y inertial frame velocity
  saturate(X_INS[2], 3, 0); //test area size
  saturate(X_INS[3], 3, 0); //test area size

}

void rotate_body2inertial()
{

  float  roll = deg2rad(0);
  float pitch = deg2rad(0);
        

  float ROT[3][3];
  ROT[0][0]= 1   ;ROT[0][1]= 0           ;ROT[0][2]= 0;
  ROT[1][0]= 0   ;ROT[1][1]= cos(roll)   ;ROT[1][2]= -sin(roll);
  ROT[2][0]= 0   ;ROT[2][1]= sin(roll)   ;ROT[2][2]=  cos(roll);

  Matrix.Multiply((float*)ROT, (float*)accFromImu, 3, 3, 1, (float*)temp1);

  ROT[0][0]=  cos(pitch)   ;ROT[0][1]= 0    ;ROT[0][2]= sin(pitch); 
  ROT[1][0]=      0        ;ROT[1][1]= 1    ;ROT[1][2]=     0;
  ROT[2][0]= -sin(pitch)   ;ROT[2][1]= 0    ;ROT[2][2]= cos(pitch);

  Matrix.Multiply((float*)ROT, (float*)1, 3, 3, 1, (float*)U);
}

void clear_temporary_matrices()
{
  byte i,j;
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  temp1[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  temp2[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  temp3[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  temp4[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  temp5[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  temp6[i][j]=0; } }
}

void printEstimatedOutput(){
    Serial.print(Y_E[0]);
    Serial.print(",");
    Serial.print(Y_E[1]);
    Serial.print(",");
    Serial.println(Y_E[2]);
}

void printXINS(){
    Serial.print(X_INS[0]);
    Serial.print(",");
    Serial.print(X_INS[1]);
    Serial.print(",");
    Serial.print(X_INS[2]);
    Serial.print(",");
    Serial.print(X_INS[3]);
    Serial.print(",");
    Serial.println(X_INS[4]);
}

void printPositionFromGPS(){
    Serial.print(GPS_pos[0]);
    Serial.print(",");
    Serial.println(GPS_pos[1]);
}

void printAccData(){
    Serial.print(accFromImu[0]);
    Serial.print(",");
    Serial.print(accFromImu[1]);
    Serial.print(",");
    Serial.println(accFromImu[2]);
}

void printForCentral(){
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(GPS_pos[0]);
    Serial.print(",");
    Serial.print(GPS_pos[1]);
    Serial.print(",");
    Serial.print(absoluteValue(Y_E[0]));
    Serial.print(",");
    Serial.print(absoluteValue(Y_E[1]));
    Serial.print(",");
    Serial.print(bias[0]);
    Serial.print(",");
    Serial.print(bias[1]);
    Serial.print(",");
    Serial.print(bias[2]);
    Serial.print(",");
    Serial.print(GPS_pos[2]);
    Serial.print(",");
    Serial.print(absoluteValue(Y_E[2]));
}

 float absoluteValue(float value){
  if(value < 0){
    return (-value);  
  }
    return value;
 }
