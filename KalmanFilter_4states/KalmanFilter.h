#include <stdint.h>
/*
 * kalmanfilter.h
 *
 *  Created on: 15 เม.ย. 2565
 *      Author: weera
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "Matrix.h"

class kalman_filter {
public:
	matrix A, B, C, D, R, G, Q;
	matrix gainK, errorY, U, Y, I33;
	matrix P, P_old, P_new;
	matrix predictX, predictX_old, predictX_new;
	matrix resultX, resultY;

	kalman_filter();
	void doKalman_gain();
	void doCorrect_p();
	void doPredict_y();
	void doCorrect();
	void doPredict_x();
	void doPredict_p();
	void doResult();
	void run(matrix u, matrix y);
	void setAtoD(matrix a_in, matrix b_in, matrix c_in, matrix d_in);
	void setQGR(matrix q_in, matrix g_in, matrix r_in);
  void EstimateSpeed(int32_t measurePulse, float Vin);
};

#endif /* INC_KALMANFILTER_H_ */
