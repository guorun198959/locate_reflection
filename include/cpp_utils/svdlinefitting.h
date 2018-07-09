//
// Created by waxz on 18-7-7.
//

#ifndef LOCATE_REFLECTION_SVDLINEFITTING_H
#define LOCATE_REFLECTION_SVDLINEFITTING_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>


inline double svdfit(vector<cv::Point2d> points) {
    using namespace Eigen;
    using namespace std;

#if 0
    // input vector of points
    //
    vector<cv::Point2d> points;
    for(int i=1;i<20;i++){
        points.push_back(cv::Point2d(i,0.567*i+1.28+((i%2==0)? 0.0:-0.0)));
    }

#endif
    // construct U
    MatrixXd P(int(points.size()), 2);
    for (int i = 0; i < points.size(); i++) {
        P(i, 0) = points[i].x;
        P(i, 1) = points[i].y;

    }

//    MatrixXf Pm(1,2);
    Eigen::VectorXd Pm(2);

//    Eigen::VectorXf Pm(2);
    Pm = P.colwise().mean();
    MatrixXd U = P.rowwise() - Pm.transpose();
#if 0
    cout<<"p=\n"<<P<<endl;
    cout<<"pm = \n"<<Pm<<endl;
    cout<<"U=\n"<<U<<endl;
    cout<<"U^T*U = \n"<<U.transpose()*U<<endl;
#endif


    // Eigenvalue
    // typedef Matrix<int, 3, 3> Matrix2d
    MatrixXd A(2, 2);
    A = U.transpose() * U;
    EigenSolver<Matrix2d> es(A.transpose() * A);
    // 对角矩阵，每一个对角线元素就是一个特征值，里面的特征值是由大到小排列的
    Matrix2d D = es.pseudoEigenvalueMatrix();
    // 特征向量（每一列）组成的矩阵
    Matrix2d V = es.pseudoEigenvectors();
#if 0

    cout << "Here is a 3x3 matrix, A:" << endl << A << endl << endl;

    cout << "The eigenvalue matrix D is:" << endl << D << endl << endl;
    cout << "The eigenvector matrix V is:" << endl << V << endl << endl;
    // 特征值分解
    cout << "Finally,A^t*A = \n "<<A.transpose() * A<<"\n V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;

    // cout << "Finally, V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;
    // 特征值&特征向量
    cout << "min-eigenvector & min-eigenvalue" << endl;
    cout << " <1> The min-eigenvalue for A^T*A:" << endl << D(D.rows()-1, D.rows()-1) << endl;
    cout << " <2> The min-eigenvector for A^T*A:" << endl << V.col(V.cols()-1) << endl;
    cout << " <3> (A^T*A)*min-eigenvector =" << endl << (A.transpose()*A) * V.col(V.cols()-1) << endl;
    cout << " <4> min-eigenvalue*min-eigenvector =" << endl << D(D.rows()-1, D.rows()-1)*V.col(V.cols()-1) << endl << endl;
#endif
    // SVD
    // Eigen::ComputeThinV | Eigen::ComputeThinU
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(A.transpose() * A, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Matrix2d S = svd.singularValues().asDiagonal();
    //得到最小奇异值的位置
    Matrix2d::Index minColIdx;
    svd.singularValues().rowwise().sum().minCoeff(&minColIdx);
#if 0
    cout << "The left singular vectors U is:" << endl << svd.matrixU() << endl << endl;
    cout << "The singular-value matrix S is:" << endl << S << endl << endl;
    cout << "The right singular vectors V is:" << endl << svd.matrixV() << endl << endl;
    cout << "The SVD: USV^T =" << endl << svd.matrixU()*S*svd.matrixV().transpose() << endl << endl;

    // 奇异值与特征值的关系
    cout << "The S^2 is:" << endl << S*S << endl << endl;
    cout << "The min-eigenvector for A^T*A:" << endl << svd.matrixV().col(minColIdx) << endl;
#endif
    double a, b, d;
    a = svd.matrixV().col(minColIdx)(0, 0);
    b = svd.matrixV().col(minColIdx)(1, 0);
    d = a * Pm(0, 0) + b * Pm(1, 0);
    printf("%f*x + %f*y = %f\n", a, b, d);
    printf("y = %f*x + %f \n", -a / b, d / b);


    double angle = atan2(-a, b);
    angle = (fabs(angle) > 0.5 * M_PI) ? angle - fabs(angle) * M_PI / angle : angle;
    return angle;


}


#endif //LOCATE_REFLECTION_SVDLINEFITTING_H
