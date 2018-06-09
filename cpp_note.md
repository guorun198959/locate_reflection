1.boost::shared_ptr

    // use make_shared whenever you can (i.e. when you don't need a custom deleter
    boost::shared_ptr<T> data_ptr(boost::make_shared<T>());
    
2.class member function template

    class c{
        template <class T> 
        void f();
    }
    
    template <class T> 
    void c::f(){
        T t;
    }
3.template namespace

    template <class T>
    void Listener::callback(const typename T::ConstPtr &msg) {
    //    data = *msg;
    
        ROS_INFO("receive msg.simple");
    
        updated_ = true;
    
    }