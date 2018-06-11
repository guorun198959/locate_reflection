//
// Created by waxz on 18-6-11.
//

#ifndef CATKIN_STARTUP_THREADING_H
#define CATKIN_STARTUP_THREADING_H

#include <boost/thread.hpp>
#include <iostream>

using std::cout;
namespace util {

    class Threading {
    public:
        template<class T>
        void createThread(T task) {
            boost::thread thread(task);
        }

    };


    template<class T>
    struct Task {
        std::shared_ptr<T> data_;

        Task(std::shared_ptr<T> data) {
            data_ = data;
        }

        Task() {}

        void operator()() {
            cout << "do some thing!!" << endl;
        };

    };

}

/*
    Threading t;
    Task<int> task(data);
    t.createThread<Task<int>>(task);
 *
 * */
#endif //CATKIN_STARTUP_THREADING_H
