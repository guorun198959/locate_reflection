//
// Created by waxz on 18-6-14.
//

#ifndef CATKIN_STARTUP_SEARCH_H
#define CATKIN_STARTUP_SEARCH_H

#include <kdtree/kdtree.h>
#include <iostream>
#include <array>
#include <vector>

namespace kdtree {
    using namespace std;

    // user-defined point type
// inherits std::array in order to use operator[]
    class Point2d : public array<double, 2> {
    public:

        // dimension of space (or "k" of k-d tree)
        // KDTree class accesses this member
        static const int DIM = 2;

        // the constructors
        Point2d() {}

        Point2d(double x, double y) {
            (*this)[0] = x;
            (*this)[1] = y;
        }


    };

    enum SearchMode {
        nn = 0, knn, radius
    };

    template<class T>
    class KdTree {
    private:
        kdt::KDTree<T> kdtree_;

    public:

        KdTree(vector<T> points) : kdtree_(kdt::KDTree<T>(points)) {};

        vector<int> queryIndex(T query, SearchMode mode, double arg) {
            vector<int> result;
            switch (mode) {
                case SearchMode::nn: {
                    int idx = kdtree_.nnSearch(query);
                    result.push_back(idx);
                    break;
                }

                case SearchMode::knn: {
                    result = kdtree_.knnSearch(query, int(min(arg, 1.0)));
                    break;
                }


                case SearchMode::radius : {
                    result = kdtree_.radiusSearch(query, double(max(arg, 0.1)));
                    break;
                }


            }
            return result;


        }

    };
}


#endif //CATKIN_STARTUP_SEARCH_H
