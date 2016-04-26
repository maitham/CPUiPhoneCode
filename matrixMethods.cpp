//
//  matrixMethods.cpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 02/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "matrixMethods.hpp"
#include <string>
#include <stdlib.h>
#include <math.h>
#include <iostream>


using namespace std;



Matrix matrixMultiply(const Matrix& a, const Matrix& b)
{
    int n = a.size();
    int m = a[0].size();
    int p = b[0].size();
    Matrix c(n, vector<float>(p, 0));
    
    for (int j = 0; j < p; ++j)
    {
        for (int k = 0; k < m; ++k)
        {
            for (int i = 0; i < n; ++i)
            {
                c[i][j] += a[i][k]*b[k][j];
            }
        }
    }
    return c;
}

Matrix matrix_sum(const Matrix& a, const Matrix& b)
{
    int nrows = a.size();
    int ncols = a[0].size();
    Matrix c(nrows, vector<float>(ncols));
    for (int i = 0; i < nrows; ++i)
    {
        for (int j = 0; j < ncols; ++j)
        {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
    return c;
}

Matrix matrixTranspose(Matrix& m)
{
    int n = m.size();
    for (int i = 0; i < n - 1; ++i)
    {
        for (int j = i + 1; j < n; ++j) {
            swap(m[i][j], m[j][i]);
        }
    }
    return m;
}
Matrix dotProd(Matrix a, Matrix b){
    int n = a.size();//rows
    int m= a[0].size();//columns
    int m2= b.size(); // columns
    int p= b[0].size(); // columns
    if(m!=m2)
    {
        cout<<"incompatible matrices"<<endl;
        exit(0);
    }else
    {
        Matrix dotProduct(n,Row(p));
        for (int j = 0; j < p; ++j)
        {
            for (int k = 0; k < m; ++k)
            {
                int temp =0;
                for (int i = 0; i < n; ++i)
                {
                    temp+=a[j][i]*b[i][k];
                    dotProduct[j][k] = temp;
                }
            }
        }
        return dotProduct;
    }
}
