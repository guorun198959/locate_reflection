//
// Created by waxz on 18-6-11.
//

#ifndef CATKIN_STARTUP_THREADING_H
#define CATKIN_STARTUP_THREADING_H

#include <boost/thread.hpp>
#include <thread>
#include <iostream>

using std::cout;
using std::endl;
namespace threading_util {

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


    // build a Base Task
    template<class T>
    class Task {
    private:
        // share data with main thread
        // while loop rate


        // singnal contation shared data and excution command
        struct signal {
            bool run_;
            bool pause_;
            bool exit_;
            std::shared_ptr<T> data_;


            explicit signal(bool run) {
                run_ = run;
                exit_ = false;
                pause_ = false;
            }

            void setData(std::shared_ptr<T> data) {
                data_ = data;
            }

            T getData() {
                return *data_;
            }
        };

    public:

        //wait run signal
        void wait() {
            if (shared_signal_.get()->run_)
                return;
            int sleep_time = 100;
            if (durationMsec_ > 0)
                sleep_time = durationMsec_;
            else
                sleep_time = 100;
            // sleep in a thread
            while (!shared_signal_.get()->run_)
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

        }

        // sleep in while loop
        void sleep() {
            if (durationMsec_ > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(durationMsec_));
            else
                shared_signal_.get()->exit_ = true;

        }

        virtual void doSomething() {
            while (1) {
                if (shared_signal_.get()->exit_)
                    return;
                wait();
                std::cout << "BaseClass========" << std::endl;
                sleep();


            }
        };


        int durationMsec_;

        std::shared_ptr<signal> shared_signal_;


        // constructor
        explicit Task(int durationMsec = 100) {
            durationMsec_ = durationMsec;
            shared_signal_ = std::make_shared<signal>(false);
        }


        void operator()() {

            // do something
            doSomething();



        };

        void start() {
            if (!shared_signal_.get()->run_) {
                shared_signal_.get()->run_ = true;

            }
            if (shared_signal_.get()->run_) {

                shared_signal_.get()->pause_ = false;
            }
            if (shared_signal_.get()->exit_)
                std::cout << "task is exited, cannot start!";
        }

        void pause() {
            if (shared_signal_.get()->run_)
                shared_signal_.get()->pause_ = true;
            if (shared_signal_.get()->exit_)
                std::cout << "task is exited, cannot pause!";
        }

        void exit() {
            shared_signal_.get()->exit_ = true;
        }

        bool isRun() {
            return shared_signal_.get()->run_;
        }

        bool isExit() {
            return shared_signal_.get()->exit_;

        }

        bool isPause() {
            return shared_signal_.get()->pause_;

        }

        T getData() {
            return shared_signal_.get()->getData();
        }




    };

    // child class

    template<class T>
    class MyTask : public Task<T> {
    public:
        /*The parent class has an explicit constructor, so compiler will not add an implicit 'empty' constructor to it.
         * Additionally your constructor has a parameter, so compiler can not generate an implicit call to it.
         * That's why you must do it explicitly.
         * */
        MyTask(int durationMsec, std::shared_ptr<T> data) : Task<T>(durationMsec) {
            this->shared_signal_->setData(data);

        }

        void doSomething() {
            /*https://stackoverflow.com/questions/22163607/not-declared-in-this-scope-in-the-case-of-inheritance
            *  Standard says that unqualified names in a template are generally
            non-dependent and must be looked up when the template is defined.
            Since the definition of a dependent base class is not known at that
            time (there may be specialisations of the base class template that
            have not yet been seen), unqualified names are never resolved to
            members of the dependent base class. Where names in the template are
            supposed to refer to base class members or to indirect base classes,
            they can either be made dependent by qualifying them or brought into
            the template's scope with a using-declaration
             * use this->data to get BaseClass member
             *
             * */
            while (1) {
                //wait-->do-->sleep-->exit


                // how to call BaseClass function
                //1) call other function
                this->wait();
                //2) call from virture function
#if 0
                Task<T>::doSomething();
#endif
                // exit
                if (this->shared_signal_->exit_)
                    return;


                // do something
                std::cout << "ChildClass=========";

                // sleep
                this->sleep();


            }
        }

    };

    template<class T>
    class ThreadPublisher : public Task<T> {
    public:
        ros::NodeHandle nh_;
        ros::Publisher pub_;

        explicit ThreadPublisher(int durationMsec, std::shared_ptr<T> data, ros::NodeHandle nh) : Task<T>(
                durationMsec), nh_(nh) {

            this->shared_signal_->setData(data);

            pub_ = nh.advertise<T>("chat", 2);

        }

        void doSomething() {
            this->wait();
            T msg;
            msg.header = this->shared_signal_.get()->getData().header;
            for (int i = 0; i < 10000; i++) {
                char tmp[200];
                sprintf(tmp, "Threding **** %d", i);
                msg.cmd.data = string(tmp);

                pub_.publish(msg);
                this->sleep();

            }
        }
    };

    template<class T>
    class ThreadTfPub : public Task<T> {
    public:

        tf::TransformBroadcaster *tfb_;

        explicit ThreadTfPub(int durationMsec, std::shared_ptr<T> data, tf::TransformBroadcaster *tfb) : Task<T>(
                durationMsec), tfb_(tfb) {
            if (durationMsec < 0) {
//                sprintf("durationMsec = %d ",durationMsec);
                exit(0);
            }

            this->shared_signal_->setData(data);


        }

        void doSomething() {
            this->wait();
            ros::Rate r(1000.0 / this->durationMsec_);
            while (ros::ok() && !this->isExit()) {
                if (this->isPause()) {
                    r.sleep();

                    continue;
                }
                T msg = this->getData();

                tfb_->sendTransform(msg);

                r.sleep();

            }

        }
    };


    class ThreadClass {
    public: // methods
        /** Constructor
         *
         * starts the internal thread running.
         */
        ThreadClass();

        /* add target function
         * */
        template<class T>
        void setTarget(T target, tf::TransformBroadcaster *tfb);

        /** Destructor
         *
         * Blocks until the thread has finished executing, if it hasn't
         * finished already.
         */
        ~ThreadClass();

    private: // methods
        /** This is the function that the thread executes.
         *
         * The thread will finish when this function returns.
         */
        template<class T>
        void threadMain(T target, tf::TransformBroadcaster *tfb);

    public:
        void start() {
            run_ = true;
        }

        void pause() {
            run_ = false;
        }
    private: // data
        boost::thread internalThread_;
        bool run_;

    }; // class

//*****************************************************************************


    //-----------------------------------------------------------------------------
    inline    ThreadClass::ThreadClass() {

        run_ = false;

    } // Constructor

    inline    ThreadClass::~ThreadClass() {
        internalThread_.interrupt();
        // internalThread_.join(); // make damn sure that the internal thread is gone
        // before we destroy the class data.
    } // Destructor


    //---------------------------------------------------------------------
    template<class T>
    void ThreadClass::setTarget(T target, tf::TransformBroadcaster *tfb) {
        // this should always be the last line in the constructor
        internalThread_ = boost::thread(boost::bind(&ThreadClass::threadMain<T>, this, target, tfb));
    }

//-----------------------------------------------------------------------------
    template<class T>
    inline void ThreadClass::threadMain(T target, tf::TransformBroadcaster *tfb) {
        try {
            /* add whatever code you want the thread to execute here. */
            while (1) {
                if (!run_) {
                    continue;
                }
                cout << "run once!!!" << endl;
                tfb->sendTransform(*target);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

            }

        }
        catch (boost::thread_interrupted &interruption) {
            // thread was interrupted, this is expected.
            cout << "******************thread was interrupted, this is expected." << endl;

        }
        catch (std::exception &e) {
            // an unhandled exception reached this point, this constitutes an error
            cout << "****************** an unhandled exception reached this point, this constitutes an error" << endl;

        }

    } // threadMain

//-----------------------------------------------------------------------------


}






/*
 *  auto res = l.createSubcriber<node::mytopic>("chat", 2);
    std::shared_ptr<node::mytopic> data = std::get<0>(res);
    // data is shared_ptr
    util::Threading t;
    util::Task<node::mytopic> task(100);
    util::MyTask<node::mytopic> mytask(100,data);
    util::ThreadPublisher<node::mytopic> pub_task(10,data,nh);
    t.createThread(task);
    t.createThread(mytask);
    t.createThread(pub_task);
    pub_task.start();

    task.start();
    task.chat("start work);
    task.exit();

 *
 * */
#endif //CATKIN_STARTUP_THREADING_H
