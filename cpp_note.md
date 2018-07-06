1.boost::shared_ptr
https://stackoverflow.com/questions/20895648/difference-in-make-shared-and-normal-shared-ptr-in-c
https://stackoverflow.com/questions/6876751/differences-between-unique-ptr-and-shared-ptr
    
    // use make_shared whenever you can (i.e. when you don't need a custom deleter
    boost::shared_ptr<T> data_ptr(boost::make_shared<T>());
    
    class Animal{
    private:
        string name;
    public:
        Animal(string s){
            name = s;
        }
        string getname(){
            return name;
        }
    };
    void change_ptr(boost::shared_ptr<Animal> data_ptr){
        data_ptr.get()->name = "new google";
    }
    
    int main(){
        boost::shared_ptr<Animal> data_ptr(boost::make_shared<Animal>("google"));
    
        std::cout<<data_ptr.get()->getname();
        change_ptr(data_ptr);
        std::cout<<data_ptr.get()->getname();
        Animal a("fb");
        boost::shared_ptr<Animal> data_ptr2(boost::make_shared<Animal>(a));
        std::cout<<data_ptr2.get()->getname();
        boost::shared_ptr<Animal> data_ptr3 = data_ptr2;
        data_ptr3.get()->name = "3333";
        std::cout<<data_ptr2.get()->getname();
        }
    
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
    

4. map check key exists


    if ( m.find("f") == m.end() ) {
      // not found
    } else {
      // found
    }
    

5.Function passed as template argument

    template <typename F>
    void doOperation(F f)
    {
      int temp=0;
      f(temp);
      std::cout << "Result is " << temp << std::endl;
    }
    
    
6
You have two options to retrieve a reference to the object pointed to by a shared_ptr. 
Suppose you have a shared_ptr variable named ptr. You can get the reference either by using *ptr or *ptr.get(). 
These two should be equivalent, but the first would be preferred.

7.share_ptr swap memory 

        // swap memory
        std::shared_ptr<T> data = std::make_shared<T>();
        T* msg;
        T m = *msg;
        std::swap(*data,m);
        
8. how to pass share_ptr
Shortly, there is no reason to pass by value, unless the goal is to share ownership of an object (eg. between multiple threads).
9. copy parameter file in cmake;


    
    configure_file (
            "${PROJECT_SOURCE_DIR}/launch/board.yaml"
            "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/board.yaml"
    )
    
10. try and catch



    try {
    
    }catch (...){
    
        printf("catch any exception!!\n");
    }