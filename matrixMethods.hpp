//
//  matrixMethods.hpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 02/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef matrixMethods_hpp
#define matrixMethods_hpp

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

typedef std::vector<float> Row; // One row of the matrix
typedef std::vector<Row> Matrix; // Matrix: a vector of rows

Matrix matrixMultiply(const Matrix& a, const Matrix& b);
Matrix matrix_sum(const Matrix& a, const Matrix& b);
Matrix matrixTranspose(Matrix& m);

#endif /* matrixMethods_hpp */
