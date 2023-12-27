/*
 * Matrix.cpp
 *
 *  Created on: 14 เม.ย. 2565
 *      Author: weera
 */

#include <Matrix.h>

matrix::matrix() {
	d_x = 1;
	d_y = 1;
	for (int i = 0; i < 1; i++) {
		for (int j = 0; j < 1; ++j) {
			data[i][j] = 0;
		}
	};
}

matrix::matrix(int x, int y) {
	d_x = x;
	d_y = y;
	for (int i = 0; i < x; i++) {
		for (int j = 0; j < y; ++j) {
			data[i][j] = 0.0;
		}
	};
}

matrix::matrix(int x, int y, float *data_in) {
	d_x = x;
	d_y = y;
	for (int i = 0; i < x; i++) {
		for (int j = 0; j < y; ++j) {
			data[i][j] = data_in[i * y + j];
		}
	};
}

matrix matrix::operator+(matrix &in) {
	matrix result;
	result.setx(d_x);
	result.sety(d_y);

	for (int y = 0; y < d_y; y++) {
		for (int x = 0; x < d_x; x++) {
			float total = 0;
			total = data[x][y] + in.data[x][y];

			result.data[x][y] = total;
			// printf(" = %d\n",total);
		}
	}
	return result;
}

matrix matrix::operator-(matrix &in) {
	matrix result;
	result.setx(d_x);
	result.sety(d_y);

	for (int y = 0; y < d_y; y++) {
		for (int x = 0; x < d_x; x++) {
			float total = 0;
			total = data[x][y] - in.data[x][y];

			result.data[x][y] = total;
			// printf(" = %d\n",total);
		}
	}
	return result;
}

matrix matrix::operator*(matrix &in) {
	matrix result;
	result.setx(d_x);
	result.sety(in.d_y);
	for (int m = 0; m < d_x; m++) {
		for (int n = 0; n < in.d_y; n++) {
			float total = 0;
			for (int k = 0; k < d_y; k++) {
				total = total + data[m][k] * in.data[k][n];
			}
			result.data[m][n] = total;
		}
	}
	return result;
}

/*
matrix matrix::operator/(matrix &in) {
	matrix result;
	result.setx(d_x);
	result.sety(d_y);
	//result = this->matrix*in.inv();

	return result;
}
*/

void matrix::setx(int x) {
	d_x = x;
}
void matrix::sety(int y) {
	d_y = y;
}
void matrix::read(float *data_in) {
	for (int i = 0; i < d_x; i++) {
		for (int j = 0; j < d_y; ++j) {
			data[i][j] = data_in[i * d_y + j];
		}
	};
}
matrix matrix::transpose() {
	matrix result;
	result.setx(d_y);
	result.sety(d_x);
	for (int m = 0; m < d_x; m++) {
		for (int n = 0; n < d_y; ++n) {
			result.data[n][m] = data[m][n];
		}
	}
	return result;
}
float matrix::det() {
	float det_val = 0;
	if (d_x == 3 && d_y == 3) {
		for (int i = 0; i < 3; i++) {
			det_val = det_val
					+ (data[0][i]
							* (data[1][(i + 1) % 3] * data[2][(i + 2) % 3]
									- data[1][(i + 2) % 3]
											* data[2][(i + 1) % 3]));
		}
	} else if (d_x == 2 && d_y == 2) {
		det_val = (data[0][0] * data[1][1]) - (data[0][1] * data[1][0]);
	} else
		det_val = 1;

	return det_val;

}
matrix matrix::gain(float in){
	matrix result;
	result.setx(d_x);
	result.sety(d_y);
	for (int m = 0; m < d_x; m++) {
		for (int n = 0; n < d_y; n++) {
			result.data[m][n] = data[m][n]*in;
		}
	}
	return result;
}
matrix matrix::inv() {
	float det = matrix::det();
	matrix result(3, 3);
	if (d_x == 3 && d_y == 3) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				result.data[j][i] = ((data[(i + 1) % 3][(j + 1) % 3]
						* data[(i + 2) % 3][(j + 2) % 3])
						- (data[(i + 1) % 3][(j + 2) % 3]
								* data[(i + 2) % 3][(j + 1) % 3])) / det;
			}
		}
	}
	return result;
}
