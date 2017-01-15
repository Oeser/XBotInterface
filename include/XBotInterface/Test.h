#ifndef __DFDFDFDFDFFFFSWGSDGSDFG__
#define __DFDFDFDFDFFFFSWGSDGSDFG__

#include <ostream>
#include <iostream>
#include <sstream>

class InfoLoggerTest;

class LoggerEndl {


public: LoggerEndl(InfoLoggerTest * info):_info(info){}
public: friend void operator<<(std::ostream& ss, LoggerEndl& info);
private: void print(std::ostream& ss);

private: InfoLoggerTest * _info;

};

class InfoLoggerTest  {

friend LoggerEndl;

public:

    InfoLoggerTest():_endl(this){}

    std::stringstream& operator<<(std::stringstream& ss){

        std::cout << "InfoLoggerTest& operator<<(std::ostream& os)" << std::endl;

        _ss << ss.str();

        return _ss;

    }

//     InfoLoggerTest& operator<<(InfoLoggerTest& info){
//         std::cout << "InfoLoggerTest& operator<<(InfoLoggerTest& info" << std::endl;
//
//
//         return info;
//     }

    std::stringstream& operator<<(const char * text){
        std::cout << "InfoLoggerTest& operator<<(const char * text){" << std::endl;

        _ss << std::string(text);
        return _ss;
    }

//     template <typename T>
//     friend InfoLoggerTest& operator<<(InfoLoggerTest& info, T& obj);

//     InfoLoggerTest& operator<<(int a){
//         std::cout << "InfoLoggerTest& operator<<(int a){" << std::endl;
//
//         _ss << a;
//         return *this;
//     }
//
//     InfoLoggerTest& operator<<(double a){
//         std::cout << "InfoLoggerTest& operator<<(double a){" << std::endl;
//
//         _ss << a;
//         return *this;
//     }

    void header(const std::string& header){
        std::cout << "InfoLoggerTest& header(const std::string& header){" << std::endl;

        _header = header;

    }

    std::stringstream& log(){
        std::cout << "InfoLoggerTest& log()" << std::endl;
        _ss << _header;
        return _ss;

    }

    LoggerEndl& endl(){
        std::cout << "InfoLoggerTest& endl()" << std::endl;
        return _endl;
    }



    friend void operator<<(std::ostream& ss, InfoLoggerTest& info);
    friend void operator<<(std::ostream& ss, LoggerEndl& info);

    InfoLoggerTest& operator<<(LoggerEndl e){
        std::cout << "InfoLoggerTest& endl(){" << std::endl;

        std::cout << _ss.str() << std::endl;
        return *this;
    }

    private:

    void print(std::ostream& ss){
        std::cout << "void print(std::ostream& ss)" << std::endl;
        std::cout << ss.rdbuf() << std::endl;
    }


private:

    std::stringstream _ss;
    std::string _header;

    LoggerEndl _endl;

};

void operator<<(std::ostream& ss, LoggerEndl& info){
    std::cout << "void operator<<(std::ostream& ss, InfoLoggerTest& info)" << std::endl;
    info.print(ss);

}

// template <typename T>
// InfoLoggerTest& operator<<(InfoLoggerTest& info, T& obj)
// {
//     std::cout << "InfoLoggerTest& operator<<(InfoLoggerTest& info, T& obj)" << std::endl;
//
//     info << obj;
//     return info;
// }


class LoggerTest {

public:

    InfoLoggerTest& info(){
        _info.header("[INFO] ");
        return _info;
    }

private:

    InfoLoggerTest _info;


};


void LoggerEndl::print(std::ostream& ss)
{
    _info->print(ss);
}


#endif