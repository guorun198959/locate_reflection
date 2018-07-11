//
// Created by waxz on 18-7-5.
//

#include <locate_reflection/patternMatcher.h>

using Eigen::MatrixXf;
using std::endl;
using std::cout;


void PatternMatcher::initParams() {
    string filename = "board.yaml";
    Yaml::Node param_;
    try {
        param_ = Yaml::readFile(filename);

    } catch (...) {
        printf("read %s failed!!\n", filename.c_str());
        exit(0);
    }
    radius_ = param_["radius"].As<double>(0.1);
    distScoreBase_ = param_["distScoreBase"].As<double>(0.1);
    angleScoreBase_ = param_["angleScoreBase"].As<double>(0.1);
    distRatio_ = param_["distRatio"].As<double>(2);
    angleRatio_ = param_["angleRatio"].As<double>(6);
    matchScore_ = param_["matchScore"].As<double>(1.1);
    finalScore_ = param_["finalScore"].As<double>(1.1);
}

PatternMatcher::PatternMatcher() {
    initParams();
}

vector<tuple<int, int> > PatternMatcher::match(vector<Position> &obsPos, vector<Position> &mapPos) {
    // convert vector to matrix
    // for creating a circle loop
    int obsSize = int(obsPos.size());
    int mapSize = int(mapPos.size());
    MatrixXf PointObs(obsSize, 2);

    MatrixXf PointMap(mapSize, 2);
    for (int i = 0; i < obsSize; i++) {
        PointObs.row(i) << obsPos[i].x, obsPos[i].y;
    }

    for (int i = 0; i < mapSize; i++) {
        PointMap.row(i) << mapPos[i].x, mapPos[i].y;
    }

    cout << "get obs points = \n" << PointObs << endl;
    cout << "get map points = \n" << PointMap << endl;
//    cout << "distance = " << (PointObs - PointMap).norm() << endl;
//    cout << "get obs shape = " << PointObs.rows() << "," << PointObs.cols() << endl;

    // assign matrix
    vector<AssignmentMatrix> rootvVec;

    // find start pair as root
    // return
    // select one pair [i,j] in obs
    cout << "==== start find root " << endl;
    for (int i = 0; i < obsSize; i++) {

        for (int j = 0; j < i; j++) {

            double distobs = (PointObs.row(i) - PointObs.row(j)).norm();
            printf("@ obs distance %f, between %d,%d \n", distobs, i, j);

            // loop in map
            for (int m = 0; m < mapSize; m++) {

                for (int n = 0; n < m; n++) {

                    double distMap = (PointMap.row(m) - PointMap.row(n)).norm();

                    printf("map distance %f, between %d,%d \n", distMap, m, n);

                    double distDiff = fabs(distobs - distMap);
                    if (distDiff < radius_) {
                        double score = scoreFunc(distDiff, distScoreBase_, 2);
                        printf("i=%d, j=%d,m=%d,n=%d\n", i, j, m, n);
                        // assign matrix
                        AssignmentMatrix Am(obsSize, mapSize);
                        Am.add(i, m, score);
                        Am.add(j, n, score);
                        rootvVec.push_back(Am);
                    }
                }
            }
        }
    }
    cout << "==== done find root " << endl;
    // check if find root successful
    if (rootvVec.empty()) {
        cout << "find root failure" << endl;
        return vector<tuple<int, int> >();
    }

    cout << "==== start check assignment , Num " << rootvVec.size() << endl;
    // check assign matrix
    // calculate match sore for each matrix
    for (int idx = 0; idx < rootvVec.size(); idx++) {
        cout << "check assign = \n" << rootvVec[idx].assignmentMatrix << endl;
        // for each point in obs except root pair
        // find assignment in map
        // for each row in assign matrix
        // check it's norm , assigned or not
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
            for (int mapid = tmpL; mapid > mapR - mapSize; mapid--) {
                int tmpid;
                tmpid = (mapid < 0) ? mapid + mapSize : mapid;
                cout << "point < L mapid = " << tmpid << endl;

                // how to compute match score
                // for one point
                // caculate distance to pointL, pointR,angle
                double distDiff = fabs((PointObs.row(obsid) - PointObs.row(obsL)).norm() +
                                       (PointObs.row(obsid) - PointObs.row(obsR)).norm() -
                                       ((PointMap.row(tmpid) - PointMap.row(mapL)).norm() +
                                        (PointMap.row(tmpid) - PointMap.row(mapR)).norm())
                );
                double angleDiff = fabs(
                        atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 0) - PointObs(obsL, 0))
                        - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                        - (atan2(PointMap(tmpid, 1) - PointMap(mapL, 1),
                                 PointMap(tmpid, 0) - PointMap(mapL, 0))
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
                    rootvVec[idx].add(obsid, tmpid, score);
                    cout << "update assign matrix = \n" << rootvVec[idx].assignmentMatrix << endl;
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
        for (int obsid = obsR + 1; obsid < obsSize; obsid++) {
            cout << "point >R obsid = " << obsid << endl;

            for (int mapid = tmpR; mapid < mapSize + mapL; mapid++) {
                int tmpid = mapid;
                tmpid = (mapid > mapSize - 1) ? mapid - mapSize : mapid;
                cout << "point >R mapid = " << tmpid << endl;
                double distDiff = fabs((PointObs.row(obsid) - PointObs.row(obsL)).norm() +
                                       (PointObs.row(obsid) - PointObs.row(obsR)).norm() -
                                       ((PointMap.row(tmpid) - PointMap.row(mapL)).norm() +
                                        (PointMap.row(tmpid) - PointMap.row(mapR)).norm())
                );
                double angleDiff = fabs(
                        atan2(PointObs(obsid, 1) - PointObs(obsL, 1), PointObs(obsid, 0) - PointObs(obsL, 0))
                        - atan2(PointObs(obsR, 1) - PointObs(obsL, 1), PointObs(obsR, 0) - PointObs(obsL, 0))
                        - (atan2(PointMap(tmpid, 1) - PointMap(mapL, 1),
                                 PointMap(tmpid, 0) - PointMap(mapL, 0))
                           - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                   PointMap(mapR, 0) - PointMap(mapL, 0))));

                double test = -(atan2(PointMap(tmpid, 1) - PointMap(mapL, 1),
                                      PointMap(tmpid, 0) - PointMap(mapL, 0))
                                - atan2(PointMap(mapR, 1) - PointMap(mapL, 1),
                                        PointMap(mapR, 0) - PointMap(mapL, 0)));

                double score = scoreFunc(distDiff, distScoreBase_, 2) + scoreFunc(angleDiff, angleScoreBase_, 6);
                cout << "distDiff = " << distDiff << "angleDiff = " << angleDiff << "score = " << score << endl;

                if (score > matchScore_) {
                    // update bound
                    tmpR = mapid + 1;
                    // update score
                    rootvVec[idx].add(obsid, tmpid, score);
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
    }


    // set all assignment matrix value to one matrix
    // given each assignment a score
    MatrixXf scoreMatrix(PointObs.rows(), PointMap.rows());
    scoreMatrix.setZero();
    for (int i = 0; i < rootvVec.size(); i++) {

        cout << i << " each scorematrix = \n" << rootvVec[i].getScore() * rootvVec[i].assignmentMatrix << endl;
        scoreMatrix += rootvVec[i].getScore() * rootvVec[i].assignmentMatrix;
    }

    cout << "scoreMatrix = \n" << scoreMatrix << endl;

    int bestId = 0;
    double bestScore = 0;
    for (int i = 0; i < rootvVec.size(); i++) {

        double score = (scoreMatrix.array() * rootvVec[i].assignmentMatrix.array()).sum();
        cout << i << " each matrix score = \n" << score << endl;

        if (score > bestScore) {
            bestScore = score;
            bestId = i;
        }

    }

    // check score ;
    //
    cout << "get best id " << bestId << "best score " << bestScore << endl;
    if (bestScore > finalScore_) {
        return rootvVec[bestId].assignments;

    } else {
        return vector<tuple<int, int> >();

    }

}