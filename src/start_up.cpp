//
// Created by waxz on 18-6-8.
//

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <tuple>
#include <ctime>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <cpp_utils/random.h>


using std::vector;
using std::tuple;

template<class T>
double gen_normal_3(T &generator) {
    return generator();
}

struct AssignmentMatrix {
    Eigen::MatrixXf assignmentMatrix;
    vector<tuple<int, int> > assignments;

    AssignmentMatrix(int row, int col) : assignmentMatrix(Eigen::MatrixXf(row, col)) {
        assignmentMatrix.setZero();
        assignments.clear();
    };

    void clear() {
        assignmentMatrix.setZero();
        assignments.clear();
    }

    void add(int i, int m) {
        assignmentMatrix(i, m) = 1;
        assignments.push_back(std::make_tuple(i, m));
    }
};

void f() {
    using namespace std;
    using namespace Eigen;

    // parameters
    double radius = 0.3;

    // observe points and map points
    MatrixXf PointObs(3, 2);
    MatrixXf PointMap(3, 2);

    PointObs << -1, 2, 1, 2, 1.5, 1.5;
    PointMap << -1.1, 1.9, 0.9, 1.9, 1.4, 1.4;


    cout << "get obs points = \n" << PointObs << endl;
    cout << "get map points = \n" << PointMap << endl;
    cout << "distance = " << (PointObs - PointMap).norm() << endl;
    cout << "get obs shape = " << PointObs.rows() << "," << PointObs.cols() << endl;
    PointObs.colwise();

    // distance matrix
    MatrixXf distanceMatrixObs(3, 3);

    // assign matrix
    vector<AssignmentMatrix> rootvVec;




    // find start pair as root
    // return
    // select one pair [i,j] in obs
    for (int i = 0; i < PointObs.rows(); i++) {

        for (int j = 0; j < i; j++) {

            double distobs = (PointObs.row(i) - PointObs.row(j)).norm();
            printf("obs distance %f, between %d,%d \n", distobs, i, j);

            // loop in map
            for (int m = 0; m < PointMap.rows(); m++) {

                for (int n = 0; n < m; n++) {

                    double distMap = (PointMap.row(m) - PointMap.row(n)).norm();
                    printf("map distance %f, between %d,%d \n", distMap, m, n);

                    if (fabs(distobs - distMap) < radius) {
                        printf("i=%d, j=%d,m=%d,n=%d\n", i, j, m, n);
                        // assign matrix
                        AssignmentMatrix Am(PointObs.rows(), PointMap.rows());
                        Am.add(i, m);
                        Am.add(j, n);


                        rootvVec.push_back(Am);
                    }


                }
            }


        }
    }

    // chech assign matrix

    for (int idx = 0; idx < rootvVec.size(); idx++) {
        cout << "check assign = \n" << rootvVec[idx].assignmentMatrix << endl;


        // for each point in obs except root pair
        // find assignment in map

        // for each row in assign matrix
        // check it's norm , assigned or not
        //

        // for assign matrix
        int obsR = std::get<0>(rootvVec[idx].assignments[0]);
        int obsL = std::get<0>(rootvVec[idx].assignments[1]);
        int mapR = std::get<1>(rootvVec[idx].assignments[0]);
        int mapL = std::get<1>(rootvVec[idx].assignments[1]);


        // obsL  < obsR
        cout << "L" << obsL << "R" << obsR << endl;

        // if obsnum >2
        // it must be one case in below
        // else skip
        // after match , we get what ;
        //

        // check point < L
        int tmpL = mapL - 1;
        for (int obsid = obsL - 1; obsid >= 0; obsid--) {
            cout << "point < L obsid = " << obsid << endl;
            for (int mapid = tmpL; mapid >= 0; mapid--) {
                cout << "point < L mapid = " << mapid << endl;

                // how to compute match score
                // for one point
                // caculate distance to pointL, pointR,angle
                double distDiff = (PointObs.row(obsid) - PointObs.row(obsL)).norm() -
                                  (PointMap.row(mapid) - PointMap.row(mapL)).norm();
                double angleDiff = atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 1) - PointObs(obsL, 0))
                                   - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                                   - (atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                            PointMap(mapid, 1) - PointMap(mapL, 0))
                                      - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                              PointMap(mapR, 0) - PointMap(mapL, 0)));

                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << endl;
                if (1) {
                    tmpL = mapL - 1;

                }

            }

        }

        // check   L<point<R
        for (int obsid = obsL + 1; obsid < obsR; obsid++) {
            cout << "L<point<R obsid = " << obsid << endl;

            for (int mapid = mapL + 1; mapid < mapR; mapid++) {
                cout << "L<point<R mapid = " << mapid << endl;
                double distDiff = (PointObs.row(obsid) - PointObs.row(obsL)).norm() -
                                  (PointMap.row(mapid) - PointMap.row(mapL)).norm();
                double angleDiff = atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 1) - PointObs(obsL, 0))
                                   - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                                   - (atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                            PointMap(mapid, 1) - PointMap(mapL, 0))
                                      - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                              PointMap(mapR, 0) - PointMap(mapL, 0)));

                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << endl;

            }
        }

        // check point >R

        for (int obsid = obsR + 1; obsid < rootvVec[idx].assignmentMatrix.rows(); obsid++) {
            cout << "point >R obsid = " << obsid << endl;

            for (int mapid = obsR + 1; mapid < rootvVec[idx].assignmentMatrix.cols(); mapid++) {
                cout << "point >R mapid = " << mapid << endl;
                double distDiff = (PointObs.row(obsid) - PointObs.row(obsL)).norm() -
                                  (PointMap.row(mapid) - PointMap.row(mapL)).norm();
                double angleDiff = atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 1) - PointObs(obsL, 0))
                                   - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                                   - (atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                            PointMap(mapid, 1) - PointMap(mapL, 0))
                                      - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                              PointMap(mapR, 0) - PointMap(mapL, 0)));

                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << endl;

            }
        }


        // if obsnum = 2
        // add origin point to it and compute matrix distance
        // get matrix


        for (int asi = 0; asi < rootvVec[idx].assignments.size(); asi++) {
            // root1,root2

            // start from root1


            // for a point < root
            // compute distace matrix
            // distance to point1 and point2
            // angle matrix
            // angle between point1-point point1-point2

            // convert diff to score






        }




        // result in a full assign matrix
    }

    // check assign matrix
    // choose best




    // given point set
    // sort points set according to close wise order
    // get point wise distance matrix
    // select a element in matrix, get its location


    // inthe other matrix find neareast neighbor
    // element wise substraction and mask
    // get index matrix
    // get index

    // if observed more than 2 point use angle matrix
    // else no use

    // observe_num == 2
    // add origin point to compute angle matrix
    // only contain 1 element in the angle matrix


    // find matrix with least difference between angle matrix and distance matrix

    // if observe_num >= 3
    // to find next point , remember scan order
    //


    // how to end a loop
    // get what result

    //




}


Eigen::MatrixXf createMatrixFromVector(std::vector<float> vec, int row, int col) {
    using namespace Eigen;
//    if (row*col == vec.size()){
//
//    }
    float *data = &(vec[0]);

    // Eigen use colmajor as default order
    //If the storage order is not specified, then Eigen defaults to storing the entry in column-major. This is also the case if one of the convenience typedefs (Matrix3f, ArrayXXd, etc.) is used.

    Map<MatrixXf> m(data, row, col);
    return m;
}

// predefine matri
int main() {

    f();
    return 0;


    using namespace Eigen;
    using namespace std;
    int r = 3, c = 3;
    MatrixXf m(r, c);
    m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    MatrixXf n = m;

    cout << "m=" << endl << m << endl;
    m = (m + MatrixXf::Constant(3, 3, 1.2)) * 50;

    VectorXf v(3);


    v << 1, 0, 1;
    cout << "m*v=" << endl << m * v << endl;

    vector<float> d;
    for (int i = 0; i < 9; i++)
        d.push_back(i);
    MatrixXf mmc = createMatrixFromVector(d, 3, 3);
    cout << "mmc=" << endl << mmc;

//    m.setOnes();
    cout << "mask =\n" << (m.array() > m(0, 0));


    ArrayXXf a(3, 2), b(2, 2);
    a << -1, 2, 1, 2, 1.5, 1.5;
    b << 5, 6, 7, 8;

    cout << "a*b=\n" << a << endl;


#if 0
    cout<<"random generator\n"<<endl;
    random_util::NormalGenerator rng(2,0.1);
    vector<double> mean;
    mean.push_back(1);
    mean.push_back(1);
    mean.push_back(1);
    mean.push_back(1);

    random_util::NormalVectorGenerator v_rng(mean,mean);
    mean = v_rng.sample();
    for(size_t i=0; i<10; ++i)
        std::cout<<rng.sample()
                 <<std::endl;

#endif
}