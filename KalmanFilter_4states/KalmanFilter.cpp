/*
 * kalmanfilter.cpp
 *
 *  Created on: 15 เม.ย. 2565
 *      Author: weera
 */

#include <KalmanFilter.h>

kalman_filter::kalman_filter() {
	matrix buf1(4, 1);
	predictX_old = buf1;
	matrix buf2(4, 4);
	P_old = buf2;
	float buf_val[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	matrix buf3(4, 4, buf_val);
	I33=buf3;
}

void kalman_filter::setAtoD(matrix a_in,matrix b_in,matrix c_in,matrix d_in){
	A=a_in;B=b_in,C=c_in;D=d_in;
}
void kalman_filter::setQGR(matrix q_in,matrix g_in,matrix r_in){
	Q=q_in;G=g_in;R=r_in;
}

void kalman_filter::run(matrix u_in, matrix y_in) {
	U = u_in;
	Y = y_in;
	doKalman_gain();
	doPredict_y();
	doCorrect_p();
	doCorrect();
	// Correct
	doPredict_x();
	doPredict_p();
	doResult();
	// Predict
	predictX_old = predictX_new;
	P_old = P_new;
	//update
}

void kalman_filter::doKalman_gain(){
matrix C_tra = C.transpose();
matrix buf2 = ((C*P_old)*C_tra);
matrix buf2_5=  buf2+R;
gainK = (P_old*C_tra).gain(1/(buf2_5.data[0][0]));
}

void kalman_filter::doPredict_y(){
matrix buf = C*predictX_old;
matrix buf2 = D*U;
errorY = (Y-buf)+buf2;
}

void kalman_filter::doCorrect_p(){
matrix buf = gainK*C;
P = (I33-buf)*P_old;
//P_old = (I33-(gainK*C))*P_old;
}

void kalman_filter::doCorrect(){
	matrix buf1 = (gainK*errorY);
predictX = buf1+predictX_old;
}

void kalman_filter::doPredict_x(){
matrix buf = B*U;
predictX_new = (A*predictX)+buf;
//predictX_new = (A*predictX)+(B*U);
}

void kalman_filter::doPredict_p(){
matrix A_tran = A.transpose();
matrix G_tran = G.transpose();
matrix buf = (G*Q);
buf = buf*G_tran;
matrix buf1 = (A*P)*A_tran;
P_new = buf1+buf;
//P_new = ((A*P)*A.transpose())+((G*Q)*G.transpose());
}

void kalman_filter::doResult(){
matrix buf = D*U;
resultY = (C*predictX)+buf;
//resultY = (C*predictX)+(D*U);
resultX = predictX;
}

void kalman_filter::EstimateSpeed(int32_t measurePulse, float Vin){
  
}
