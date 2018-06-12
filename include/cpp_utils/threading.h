//
// Created by waxz on 18-6-11.
//

#ifndef CATKIN_STARTUP_THREADING_H
#define CATKIN_STARTUP_THREADING_H

#include <boost/thread.hpp>
#include <thread>
#include <iostream>

using std::cout;
namespace util {

    // create thread
    class Threading {
    public:
        template<class T>
        void createThread(T task) {
            boost::thread thread(task);
//        std::thread thread_2(task);
            // use std::thread must join
//        thread_2.join();
        }

    };


    // build a task
    template<class T>
    class Task {
    private:
        // share data with main thread
        std::shared_ptr<T> shared_data_;
        // while loop rate
        int durationMsec_;


        struct signal {
            string chat_;
            bool run_;
            bool exit_;

            explicit signal(bool run, string chat = "test") {
                run_ = run;
                chat_ = chat;
                exit_ = false;
            }
        };

        std::shared_ptr<signal> shared_signal_;

    public:
        Task(std::shared_ptr<T> data, int durationMsec = 100) {
            shared_data_ = data;
            durationMsec_ = durationMsec;
            shared_signal_ = std::make_shared<signal>(false);
        }


        void operator()() {
            while (1) {
                if (shared_signal_.get()->run_) {
                    // do some task
                    std::cout << shared_signal_.get()->chat_ << "=====================" << shared_data_.get()->cmd.data
                              << std::endl;

                    if (durationMsec_ < 0) {
                        shared_signal_.get()->exit_ = true;
                        return;
                    }
                }
                int sleep_time = 1;

                if (durationMsec_ > 0)
                    sleep_time = durationMsec_;
                else
                    sleep_time = 100;

                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));


                if (shared_signal_.get()->exit_)
                    return;
            }
        };

        void start() {
            if (!shared_signal_.get()->run_)
                shared_signal_.get()->run_ = true;
            if (shared_signal_.get()->exit_)
                std::cout << "task is exited, cannot start!";
        }

        void exit() {
            shared_signal_.get()->exit_ = true;
        }

        void chat(string msg) {
            shared_signal_.get()->chat_ = msg;

        }


    };

}

/*
 *  auto res = l.createSubcriber<node::mytopic>("chat", 2);
    std::shared_ptr<node::mytopic> data = std::get<0>(res);
    // data is shared_ptr
    Threading t;
    Task<node::mytopic> task(data,100);
    t.createThread<Task<node::mytopic>>(task);

    task.start();
    task.chat("start work);
    task.exit();

 *
 * */
#endif //CATKIN_STARTUP_THREADING_H
