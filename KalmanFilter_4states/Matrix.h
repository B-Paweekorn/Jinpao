/*
 * Matrix.h
 *
 *  Created on: 14 เม.ย. 2565
 *      Author: weera
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

class matrix {
public:
	int d_x;
	int d_y;
	float data[4][4];
	matrix();
	matrix(int x, int y);
	matrix(int x, int y,float *data_in);
//	virtual ~matrix();
	void read(float *data_in);
	void setx(int x);
	void sety(int y);
	matrix transpose();
	matrix gain(float in);
	float det();
	matrix inv();

	matrix operator+(matrix &in);
	matrix operator-(matrix &in);
	matrix operator*(matrix &in);
//	matrix operator/(matrix &in);
};

#endif /* INC_MATRIX_H_ */
