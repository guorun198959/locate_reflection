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

#include <cpp_utils/parse.h>



using std::vector;
using std::tuple;

template<class T>
double gen_normal_3(T &generator) {
    return generator();
}

// store position
struct Position {
    double x;
    double y;
    double yaw;
};


struct AssignmentMatrix {
    Eigen::MatrixXf assignmentMatrix;
    vector<tuple<int, int> > assignments;
    double assignScores;
    vector<double> scoreVec;

    AssignmentMatrix(int row, int col) : assignmentMatrix(Eigen::MatrixXf(row, col)) {
        assignmentMatrix.setZero();
        assignments.clear();
        scoreVec.clear();
        assignScores = 0.0;
    };

    void clear() {
        assignmentMatrix.setZero();
        assignments.clear();
        scoreVec.clear();
    }

    void add(int i, int m, double score) {
        if ((assignmentMatrix.row(i).array() == 1).any()) {
            // if add this get better score;add
            // else return with unchange
            if (score < scoreVec[scoreVec.size() - 1])
                return;

            assignmentMatrix.row(i).setZero();
            assignments.pop_back();
            scoreVec.pop_back();
        }
        assignmentMatrix(i, m) = 1;
        assignments.push_back(std::make_tuple(i, m));
        scoreVec.push_back(score);
    }

    double getScore() {
        if (assignments.empty())
            return 0;
        double sum = 0;
        for (int i = 0; i < scoreVec.size(); i++) sum += scoreVec[i];
        sum += assignScores;

        return sum / double(scoreVec.size() + 1);
    }
};


double scoreFunc(double x, double base, double ratio) {
    return pow(base, ratio * x);
}


class PatternMatcher {
private:
    // parameter
    double radius_;
    double distScoreBase_;
    double angleScoreBase_;
    double distRatio_;
    double angleRatio_;
    double matchScore_;

    // method
    void initParams() {

        string filename = "board.yaml";
        Yaml::Node param_;
        try {
            param_ = Yaml::readFile(filename);

        } catch (...) {

            printf("read failed!!\n");
            exit(0);
        }
        radius_ = param_["radius"].As<double>(0.1);
        distScoreBase_ = param_["distScoreBase"].As<double>(0.1);
        angleScoreBase_ = param_["angleScoreBase"].As<double>(0.1);
        distRatio_ = param_["distRatio"].As<double>(2);
        angleRatio_ = param_["angleRatio"].As<double>(6);
        matchScore_ = param_["matchScore"].As<double>(1.1);


    }

public:

};
void f() {
    using namespace std;
    using namespace Eigen;

    // parameters
    double radius = 0.3;

    double distScoreBase_ = 0.1, angleScoreBase_ = 0.05;

    double matchScore_ = 1.0;

    // observe points and map points
    MatrixXf PointObs(3, 2);
    MatrixXf PointMap(4, 2);

    PointObs << -1, 2, 1, 2, 1.5, 1.5;
    PointMap << -3.1, 1.9, -1.1, 1.9, 0.9, 1.9, 1.4, 1.4;


    cout << "get obs points = \n" << PointObs << endl;
    cout << "get map points = \n" << PointMap << endl;
//    cout << "distance = " << (PointObs - PointMap).norm() << endl;
    cout << "get obs shape = " << PointObs.rows() << "," << PointObs.cols() << endl;



    // assign matrix
    vector<AssignmentMatrix> rootvVec;




    // find start pair as root
    // return
    // select one pair [i,j] in obs
    cout << "==== start find root " << endl;
    for (int i = 0; i < PointObs.rows(); i++) {

        for (int j = 0; j < i; j++) {

            double distobs = (PointObs.row(i) - PointObs.row(j)).norm();
            printf("@ obs distance %f, between %d,%d \n", distobs, i, j);

            // loop in map
            for (int m = 0; m < PointMap.rows(); m++) {

                for (int n = 0; n < m; n++) {

                    double distMap = (PointMap.row(m) - PointMap.row(n)).norm();
                    printf("map distance %f, between %d,%d \n", distMap, m, n);

                    if (fabs(distobs - distMap) < radius) {
                        printf("i=%d, j=%d,m=%d,n=%d\n", i, j, m, n);
                        // assign matrix
                        AssignmentMatrix Am(PointObs.rows(), PointMap.rows());
                        Am.add(i, m, 0);
                        Am.add(j, n, 0);


                        rootvVec.push_back(Am);
                    }


                }
            }


        }
    }
    cout << "==== done find root " << endl;

    cout << "==== start check assignment , Num " << rootvVec.size() << endl;

    // check assign matrix

    // calculate match sore for each matrix

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
        cout << "obsL " << obsL << " obsR " << obsR << endl;
        cout << "mapL " << mapL << " mapR " << mapR << endl;

        // if obsnum >2
        // it must be one case in below
        // else skip
        // after match , we get what ;
        //

        // check point < L
        int tmpL = mapL - 1, tmpR = mapR - 1;
        for (int obsid = obsL - 1; obsid >= 0; obsid--) {
            cout << "point < L obsid = " << obsid << endl;
            for (int mapid = tmpL; mapid >= 0; mapid--) {
                cout << "point < L mapid = " << mapid << endl;

                // how to compute match score
                // for one point
                // caculate distance to pointL, pointR,angle
                double distDiff = fabs((PointObs.row(obsid) - PointObs.row(obsL)).norm() +
                                       (PointObs.row(obsid) - PointObs.row(obsR)).norm() -
                                       ((PointMap.row(mapid) - PointMap.row(mapL)).norm() +
                                        (PointMap.row(mapid) - PointMap.row(mapR)).norm())
                );
                double angleDiff = fabs(
                        atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 0) - PointObs(obsL, 0))
                        - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                        - (atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                 PointMap(mapid, 0) - PointMap(mapL, 0))
                           - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                   PointMap(mapR, 0) - PointMap(mapL, 0))));

                // if distance < thresh
                // add to assignments
                // update matrix
                double score = scoreFunc(distDiff, distScoreBase_, 2) + scoreFunc(angleDiff, angleScoreBase_, 6);
                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << "score = " << score << endl;

                if (score > matchScore_) {
                    // update bound
                    tmpL = mapid - 1;
                    // update score
                    rootvVec[idx].add(obsid, mapid, score);
                    cout << "update assign matrix = \n " << rootvVec[idx].assignmentMatrix << endl;



                }

            }

        }

        // check   L<point<R
        tmpL = mapL + 1;
        for (int obsid = obsL + 1; obsid < obsR; obsid++) {
            cout << "L<point<R obsid = " << obsid << endl;

            for (int mapid = tmpL; mapid < mapR; mapid++) {
                cout << "L<point<R mapid = " << mapid << endl;
                double distDiff = fabs((PointObs.row(obsid) - PointObs.row(obsL)).norm() +
                                       (PointObs.row(obsid) - PointObs.row(obsR)).norm() -
                                       ((PointMap.row(mapid) - PointMap.row(mapL)).norm() +
                                        (PointMap.row(mapid) - PointMap.row(mapR)).norm())
                );
                double angleDiff = fabs(
                        atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 0) - PointObs(obsL, 0))
                        - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                        - (atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                 PointMap(mapid, 0) - PointMap(mapL, 0))
                           - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                   PointMap(mapR, 0) - PointMap(mapL, 0))));

                double score = scoreFunc(distDiff, distScoreBase_, 2) + scoreFunc(angleDiff, angleScoreBase_, 6);
                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << "score = " << score << endl;

                if (score > matchScore_) {
                    // update bound
                    tmpL = mapid + 1;
                    // update score
                    rootvVec[idx].add(obsid, mapid, score);
                    cout << "update assign matrix = \n " << rootvVec[idx].assignmentMatrix << endl;


                }


            }
        }

        // check point >R

        tmpR = mapR + 1;
        for (int obsid = obsR + 1; obsid < rootvVec[idx].assignmentMatrix.rows(); obsid++) {
            cout << "point >R obsid = " << obsid << endl;

            for (int mapid = tmpR; mapid < rootvVec[idx].assignmentMatrix.cols(); mapid++) {
                cout << "point >R mapid = " << mapid << endl;
                double distDiff = fabs((PointObs.row(obsid) - PointObs.row(obsL)).norm() +
                                       (PointObs.row(obsid) - PointObs.row(obsR)).norm() -
                                       ((PointMap.row(mapid) - PointMap.row(mapL)).norm() +
                                        (PointMap.row(mapid) - PointMap.row(mapR)).norm())
                );
                double angleDiff = fabs(
                        atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 0) - PointObs(obsL, 0))
                        - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                        - (atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                 PointMap(mapid, 0) - PointMap(mapL, 0))
                           - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                   PointMap(mapR, 0) - PointMap(mapL, 0))));

                double test = -(atan2(PointMap(mapid, 1) - PointMap(mapL, 1),
                                      PointMap(mapid, 0) - PointMap(mapL, 0))
                                - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                        PointMap(mapR, 0) - PointMap(mapL, 0)));

                double score = scoreFunc(distDiff, distScoreBase_, 2) + scoreFunc(angleDiff, angleScoreBase_, 6);
                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << "score = " << score << endl;

                if (score > matchScore_) {
                    // update bound
                    tmpR = mapid + 1;
                    // update score
                    rootvVec[idx].add(obsid, mapid, score);
                    cout << "update assign matrix = \n" << rootvVec[idx].assignmentMatrix << endl;


                }


            }
        }


        // if obsnum = 2
        // add origin point to it and compute matrix distance
        // get matrix
        MatrixXf originPoint(1, 2);
        originPoint << 0.0, 0.0;
        double distDiff = fabs(PointObs.row(obsL).norm() + PointObs.row(obsR).norm() -
                               (PointMap.row(mapL).norm() + PointMap.row(mapR).norm())
        );
        double angleDiff = fabs(atan2(-PointObs(obsL, 1), -PointObs(obsL, 0))
                                - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                                - (atan2(-PointMap(mapL, 1),
                                         -PointMap(mapL, 0))
                                   - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                           PointMap(mapR, 0) - PointMap(mapL, 0))));


        double score = scoreFunc(distDiff, distScoreBase_, 2) + scoreFunc(angleDiff, angleScoreBase_, 6);
        cout << "origin distDiff = " << distDiff << "angleDiff = " << angleDiff << "score = " << score << endl;

        rootvVec[idx].assignScores += score;






        // for each assignment matrix
        // caculate it's count
        // how to use diff


        // combine all matrix to one matrix
        // given accumulate score to each element
        // get each pattern a score








        // result in a full assign matrix
    }


    // set all assignment matrix value to one matrix
    // given each assignment a score
    MatrixXf scoreMatrix(PointObs.rows(), PointMap.rows());
    scoreMatrix.setZero();
    for (int i = 0; i < rootvVec.size(); i++) {

        cout << "each scorematrix = \n" << rootvVec[i].getScore() * rootvVec[i].assignmentMatrix << endl;
        scoreMatrix += rootvVec[i].getScore() * rootvVec[i].assignmentMatrix;
    }

    cout << "scoreMatrix = \n" << scoreMatrix << endl;

    int bestId = 0;
    double bestScore = 0;
    for (int i = 0; i < rootvVec.size(); i++) {

        double score = (scoreMatrix.array() * rootvVec[i].assignmentMatrix.array()).sum();
        cout << "each matrix score = \n" << score << endl;

        if (score > bestScore) {
            bestScore = score;
            bestId = i;
        }

    }
    cout << "get best id " << bestId << "best score " << bestScore << endl;

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


    string filename = "board.yaml";
    Yaml::Node param_;
    try {
        param_ = Yaml::readFile(filename);

    } catch (...) {

        printf("read failed!!\n");
    }
    auto visibel_angle = param_["visibel_angle1"].As<double>(666);
    auto visibel_range_ratio = param_["visibel_range_ratio"].As<double>();

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