/**************************************************
          Kalman Filter Implementation
***************************************************/

//First step is init the system
void kalmanFilterIniatialization(){
  
  bias[0] = 0; //accelerometer bias of x-axis
  bias[1] = 0; //accelerometer bias of y-axis
  bias[2] = 0; //accelerometer bias of z-axis
  
  X_INS[0] = 0; // true velocity x
  X_INS[1] = 0; // true velocity y
  X_INS[2] = 0; // true position x
  X_INS[3] = 0; // true position y
  X_INS[4] = 0; // true pitch angle

  R[0][0]= noise_px_GPS*noise_px_GPS  ;R[0][1]= 0                         ;R[0][2]= 0; 
  R[1][0]= 0                          ;R[1][1]= noise_py_GPS*noise_py_GPS ;R[1][2]= 0;
  R[2][0]= 0                          ;R[2][1]= 0                         ;R[2][2]= noise_psi*noise_psi;
  
  Q[0][0]= noise_ax_IMU*noise_ax_IMU  ;Q[0][1]= 0                           ;Q[0][2]= 0; 
  Q[1][0]= 0                          ;Q[1][1]= noise_ay_IMU*noise_ay_IMU   ;Q[1][2]= 0;
  Q[2][0]= 0                          ;Q[2][1]= 0                           ;Q[2][2]= noise_wz_IMU*noise_wz_IMU;
  
  H[0][0]= 0 ;H[0][1]=  0 ;H[0][2]= 1 ;H[0][3]= 0 ;H[0][4]= 0 ;H[0][5]= 0 ;H[0][6]= 0 ;H[0][7]= 0 ;
  H[1][0]= 0 ;H[1][1]=  0 ;H[1][2]= 0 ;H[1][3]= 1 ;H[1][4]= 0 ;H[1][5]= 0 ;H[1][6]= 0 ;H[1][7]= 0 ;
  H[2][0]= 0 ;H[2][1]=  0 ;H[2][2]= 0 ;H[2][3]= 0 ;H[2][4]= 1 ;H[2][5]= 0 ;H[2][6]= 0 ;H[2][7]= 0 ;

  /***** create identity matrix 8 x 8 **************/
  byte i,j;
  for(i=0;i<8;i++){ 
    for(j=0;j<8;j++){  
      Ident[i][j]=0; 
    } 
  }
  
  for(i=0;i<8;i++){ 
    Ident[i][i]=1; 
  }

  Matrix.Copy((float*)Ident, 8, 8, (float*)P_initial);

  P_initial[0][0] = 50; 
  P_initial[1][1] = 50; 
  P_initial[2][2] = 50; 
  P_initial[3][3] = 50; 
  P_initial[4][4] = deg2rad(10); 
  P_initial[5][5] = 10; 
  P_initial[6][6] = 10; 
  P_initial[8][8] = 10;
  
  Matrix.Copy((float*)P_initial, 8, 8, (float*)P);
  Matrix.Scale((float*) P_initial, 8, 8, beta);
}

void kalmanFilterPropagation(){
    //Because the propagation takes 26ms, I'm going to split it in 2 steps
  if (prop_step == 1)
  {
//    digitalWrite(debugPin1, HIGH); //Debug on osciliscope
    clear_temporary_matrices(); //440us
    
    rotate_body2inertial(); //370us

    U[2] = deg2rad(gyrZ);
    U[0] = U[0] - bias[0];
    U[1] = U[1] - bias[1];
    U[2] = U[2] - bias[2];
    
    euler_integration2(); 

    float cos_psi = cos(X_INS[4]); 
    float sin_psi = sin(X_INS[4]);
  
    A[0][0]=    1    ;A[0][1]=    0    ;A[0][2]=   0   ;A[0][3]=   0   ;A[0][4]=    0   ;A[0][5]=     dtK*cos_psi        ;A[0][6]=    -dtK*sin_psi          ;A[0][7]=    0;
    A[1][0]=    0    ;A[1][1]=    1    ;A[1][2]=   0   ;A[1][3]=   0   ;A[1][4]=    0   ;A[1][5]=     dtK*sin_psi        ;A[1][6]=     dtK*cos_psi          ;A[1][7]=    0;
    A[2][0]=   dtK   ;A[2][1]=    0    ;A[2][2]=   1   ;A[2][3]=   0   ;A[2][4]=    0   ;A[2][5]=  dtK*dtK*cos_psi*0.5   ;A[2][6]= dtK*dtK*sin_psi*(-0.5)   ;A[2][7]=    0;
    A[3][0]=    0    ;A[3][1]=   dtK   ;A[3][2]=   0   ;A[3][3]=   1   ;A[3][4]=    0   ;A[3][5]=  dtK*dtK*sin_psi*0.5   ;A[3][6]=  dtK*dtK*cos_psi*0.5     ;A[3][7]=    0;
    A[4][0]=    0    ;A[4][1]=    0    ;A[4][2]=   0   ;A[4][3]=   0   ;A[4][4]=    1   ;A[4][5]=           0            ;A[4][6]=            0             ;A[4][7]=   dtK;
    A[5][0]=    0    ;A[5][1]=    0    ;A[5][2]=   0   ;A[5][3]=   0   ;A[5][4]=    0   ;A[5][5]=           1            ;A[5][6]=            0             ;A[5][7]=    0;
    A[6][0]=    0    ;A[6][1]=    0    ;A[6][2]=   0   ;A[6][3]=   0   ;A[6][4]=    0   ;A[6][5]=           0            ;A[6][6]=            1             ;A[6][7]=    0;
    A[7][0]=    0    ;A[7][1]=    0    ;A[7][2]=   0   ;A[7][3]=   0   ;A[7][4]=    0   ;A[7][5]=           0            ;A[7][6]=            0             ;A[7][7]=    1;
  
    B[0][0]=    dtK*cos_psi         ;B[0][1]=     -dtK*sin_psi         ;B[0][2]=   0;
    B[1][0]=    dtK*sin_psi         ;B[1][1]=      dtK*cos_psi         ;B[1][2]=   0;
    B[2][0]=  dtK*dtK*cos_psi*0.5   ;B[2][1]=  dtK*dtK*sin_psi*(-0.5)  ;B[2][2]=   0;
    B[3][0]=  dtK*dtK*sin_psi*0.5   ;B[3][1]=   dtK*dtK*cos_psi*0.5    ;B[3][2]=   0;
    B[4][0]=        0               ;B[4][1]=          0               ;B[4][2]=  dtK;
    B[5][0]=        0               ;B[5][1]=          0               ;B[5][2]=   0;
    B[6][0]=        0               ;B[6][1]=          0               ;B[6][2]=   0;
    B[7][0]=        0               ;B[7][1]=          0               ;B[7][2]=   0;
    
    Matrix.Multiply((float*)A, (float*)P, 8, 8, 8, (float*)temp1);
    Matrix.Transpose((float*)A, 8, 8, (float*)temp2);
  }
  if(prop_step == 2)
  {
    Matrix.Multiply((float*)temp1, (float*)temp2, 8, 8, 8, (float*)temp3);
    Matrix.Multiply((float*)B, (float*)Q, 8, 3, 3, (float*)temp4);
    Matrix.Transpose((float*)B, 8, 3, (float*)temp5);
    Matrix.Multiply((float*)temp4, (float*)temp5, 8, 3, 8, (float*)temp6);
    Matrix.Add((float*)temp3, (float*)temp6, 8, 8, (float*)temp1);
    Matrix.Add((float*)temp1, (float*)P_initial, 8, 8, (float*)P);
  }
}

void kalmanFilterUpdate(){
  //I'm also dividing the update in two steps
  if(updt_step == 1)
  {

    clear_temporary_matrices();

    Y_E[0] = X_INS[2] - GPS_pos[0]; 
    Y_E[1] = X_INS[3] - GPS_pos[1]; 
    Y_E[2] = X_INS[4] - deg2rad(actual_yaw);

     if(Y_E[2]>deg2rad(300)) // more than 300 degrees of error, it means the point is going from -180 to +180 or the vice versa
      Y_E[2] -= deg2rad(360);
    
    if(Y_E[2]<deg2rad(-300)) // more than 300 degrees of error, it means the point is going from -180 to +180 or the vice versa
      Y_E[2] += deg2rad(360);


    Matrix.Transpose((float*)H, 3, 8, (float*)temp1);
    Matrix.Multiply((float*)P, (float*)temp1, 8, 8, 3, (float*)temp2);
    Matrix.Multiply((float*)H, (float*)temp2, 3, 8, 3, (float*)temp3);
    Matrix.Add((float*)temp3, (float*)R, 3, 3, (float*)temp4);
    Matrix.Invert((float*)temp4, 3); 
    Matrix.Multiply((float*)temp2, (float*)temp4, 8, 3, 3, (float*)K); // Only ends the calculation of K if the validation gate passes 
  }

  if(updt_step == 2)
  {
    Matrix.Multiply((float*)K, (float*)H, 8, 3, 8, (float*)temp1);
    Matrix.Subtract((float*)Ident, (float*)temp1, 8, 8, (float*)temp2);
    Matrix.Multiply((float*)temp2, (float*)P, 8, 8, 8, (float*)temp3); //this step uses P as input, so I cannot put P as output also
    Matrix.Copy((float*)temp3, 8, 8, (float*)P);
    Matrix.Multiply((float*)K, (float*)Y_E, 8, 3, 1, (float*)temp1);
    Matrix.Scale((float*)temp2, 8, 8, 0);
    
    temp2[5][0] = bias[0]; 
    temp2[6][0] = bias[1]; 
    temp2[7][0] = bias[2];
    
    Matrix.Add((float*)temp2, (float*)temp1, 8, 1, (float*)X_E);
    
    //***  X_INS = X_INS - X_E(1:5); ***
    X_INS[0] = X_INS[0] - X_E[0];
    X_INS[1] = X_INS[1] - X_E[1];
    X_INS[2] = X_INS[2] - X_E[2];
    X_INS[3] = X_INS[3] - X_E[3];
    X_INS[4] = X_INS[4] - X_E[4];
  
    //***  Bias = Xhat_E(6:8); ***
    bias[0] = X_E[5];
    bias[1] = X_E[6];
    bias[2] = X_E[7];
  
    X_INS[4] -= deg2rad(360) * floor((X_INS[4] + deg2rad(180)) / deg2rad(360)); //Keeping the X_INS[4] withing the +-180 range
  }

}
