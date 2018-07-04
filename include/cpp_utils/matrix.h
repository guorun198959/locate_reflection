//
// Created by waxz on 18-6-29.
//

#ifndef CATKIN_STARTUP_MATRIX_H
#define CATKIN_STARTUP_MATRIX_H
#inclde <vector>

#include <Eigen/Dense>

namespace linag {
    using std::vector;
    using namespace Eigen;

    vector<vector<int> > where(MatrixXf m) {
        ArrayXf a = m.array();
//        a.minCoeff()
    }
}
#endif //CATKIN_STARTUP_MATRIX_H
