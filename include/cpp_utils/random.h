//
// Created by waxz on 18-6-29.
//

#ifndef CATKIN_STARTUP_RANDOM_H
#define CATKIN_STARTUP_RANDOM_H

#include <ctime>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <vector>

using std::vector;
namespace random_util {
    class NormalGenerator {

    private:
        boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator;

    public:
        NormalGenerator(double mean, double stddev) :
                generator(
                        boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(boost::mt19937(time(0)),
                                                                                                boost::normal_distribution<>(
                                                                                                        mean,
                                                                                                        stddev))) {
        }

        double sample() {
            return generator();
        }

    };

    class NormalVectorGenerator {
    private:
        vector<boost::variate_generator<boost::mt19937, boost::normal_distribution<> > > generators_;
    public:
        NormalVectorGenerator(vector<double> mean, vector<double> stddev) {
            if (mean.size() == stddev.size() && !mean.empty()) {
                for (int i = 0; i < mean.size(); i++) {
                    generators_.push_back(boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(
                            boost::mt19937(time(0) + i), boost::normal_distribution<>(mean[i], stddev[i])));
                }


            }

        }

        vector<double> sample() {
            vector<double> vec;
            for (int i = 0; i < generators_.size(); i++) {
                vec.push_back(generators_[i]());

            }
            return vec;
        }

    };


}


#endif //CATKIN_STARTUP_RANDOM_H
