//
// Created by waxz on 18-7-4.
//

#ifndef LOCATE_REFLECTION_PATTERNMATCHER_H
#define LOCATE_REFLECTION_PATTERNMATCHER_H

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

struct Position {
    double x;
    double y;
    double yaw;
    double length;
    double angle;

    Position(double x1, double y1, double yaw1, double length1, double angle1) {
        x = x1;
        y = y1;
        yaw = yaw1;
        length = length;
        angle = angle1;
    }

    Position() {
        x = 0;
        y = 0;
        yaw = 0;
        length = 0;
        angle = 0;
    }
};


inline Eigen::MatrixXf createMatrixFromVector(std::vector<float> vec, int row, int col) {
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

inline double scoreFunc(double x, double base, double ratio) {
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
    double finalScore_;

    // method
    void initParams();

public:
    PatternMatcher();

    vector<tuple<int, int> > match(vector<Position> &obsPos, vector<Position> &mapPos);


};


#endif //LOCATE_REFLECTION_PATTERNMATCHER_H
