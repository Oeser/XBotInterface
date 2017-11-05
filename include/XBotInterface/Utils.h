/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __XBOTINTERFACE_UTILS_H__
#define __XBOTINTERFACE_UTILS_H__

#include <XBotInterface/ModelInterface.h>
#include <SharedLibraryClassFactory.h>
#include <SharedLibraryClass.h>

namespace XBot {
    namespace Utils {

/**
    * @brief Computes a matrix S such that S(a)b = a x b
    *
    * @param v A 3D vector
    * @return The skew symmetric matrix which represents a vector product by v
    */
inline Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& v){

    Eigen::Matrix3d S;

    S <<      0, -v.z(),  v.y(),
          v.z(),      0, -v.x(),
         -v.y(),  v.x(),      0;

    return S;
}


/**
 * @brief Computes an orientation error between two frames such that
 * an angular velocity K*e (K > 0) brings "actual" towards "ref"
 *
 * @param ref Reference orientation
 * @param actual Current orientation
 * @param error The orientation error between ref and actual
 * @return void
 */
inline void computeOrientationError(const Eigen::Matrix3d& ref,
                                    const Eigen::Matrix3d& actual,
                                    Eigen::Vector3d& error)
{

    Eigen::Quaterniond q(actual), q_d(ref);

    if(q.dot(q_d) < 0){
        q.x() *= -1;
        q.y() *= -1;
        q.z() *= -1;
        q.w() *= -1;
    }

    error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());

}

template <typename SignalType>
class SecondOrderFilter {

public:

    typedef std::shared_ptr<SecondOrderFilter<SignalType>> Ptr;

    SecondOrderFilter():
        _omega(1.0),
        _eps(0.8),
        _ts(0.01),
        _reset_has_been_called(false)
    {
        computeCoeff();
    }

    SecondOrderFilter(double omega, double eps, double ts, const SignalType& initial_state):
        _omega(omega),
        _eps(eps),
        _ts(ts),
        _reset_has_been_called(false)
    {
        computeCoeff();
        reset(initial_state);
    }

    void reset(const SignalType& initial_state){
        _reset_has_been_called = true;
        _u = initial_state;
        _y = initial_state;
        _yd = initial_state;
        _ydd = initial_state;
        _udd = initial_state;
        _ud = initial_state;
    }

    const SignalType& process(const SignalType& input){

        if(!_reset_has_been_called) reset(input*0);


        _ydd = _yd;
        _yd = _y;
        _udd = _ud;
        _ud = _u;


        _u = input;
        _y = 1.0/_a0 * ( _u + _b1*_ud + _b2*_udd - _a1*_yd - _a2*_ydd );

        return _y;
    }

    const SignalType& getOutput() const {
        return _y;
    }

    void setOmega(double omega){
        _omega = omega;
        computeCoeff();
    }

    void setDamping(double eps){
        _eps = eps;
        computeCoeff();
    }

    void setTimeStep(double ts){
        _ts = ts;
        computeCoeff();
    }


private:

    void computeCoeff(){
        _b1 = 2.0;
        _b2 = 1.0;

        _a0 = 1.0 + 4.0*_eps/(_omega*_ts) + 4.0/std::pow(_omega*_ts, 2.0);
        _a1 = 2 - 8.0/std::pow(_omega*_ts, 2.0);
        _a2 = 1.0 + 4.0/std::pow(_omega*_ts, 2.0) - 4.0*_eps/(_omega*_ts);

        std::cout << "Coeffs: " << 1.0 << " -- " << _b1 << " -- " << _b2 << "\n" <<
        _a0 << " -- " << _a1 << " -- " << _a2 << std::endl;
    }

    double _omega;
    double _eps;
    double _ts;

    double _b1, _b2;
    double _a0, _a1, _a2;

    bool _reset_has_been_called;

    SignalType _y, _yd, _ydd, _u, _ud, _udd;

};


#define REGISTER_GENERIC_PLUGIN(plugin_name, scoped_class_name, base_class_name) SHLIBPP_DEFINE_SHARED_SUBCLASS(plugin_name ## _factory, scoped_class_name, base_class_name);

template <typename PluginType>
class PluginLoader {

public:

    bool load(std::string plugin_name)
    {

        _ioplugin_factory = std::make_shared<shlibpp::SharedLibraryClassFactory<PluginType>>();
        _ioplugin_class = std::make_shared<shlibpp::SharedLibraryClass<PluginType>>();

        std::string path_to_so = "lib" + plugin_name + ".so";
        computeAbsolutePath(path_to_so, LIB_MIDDLE_PATH, path_to_so);

        std::string factory_name = plugin_name + "_factory";
        _ioplugin_factory->open(path_to_so.c_str(), factory_name.c_str());

        if (!_ioplugin_factory->isValid()) {
            // NOTE print to celebrate the wizard
            printf("error (%s) : %s\n", shlibpp::Vocab::decode(_ioplugin_factory->getStatus()).c_str(),
                _ioplugin_factory->getLastNativeError().c_str());
            _load_success = false;
            return false;
        }

        _load_success = true;

        // open io plugin
        _ioplugin_class->open(*_ioplugin_factory);

        return true;

    }


    PluginType* getPtr()
    {
        if(!_load_success) return nullptr;
        else return &(*_ioplugin_class).getContent();
    }

private:

    static bool computeAbsolutePath (const std::string& input_path,
                                     const std::string& middle_path,
                                     std::string& absolute_path)
    {
        // if not an absolute path
        if(!(input_path.at(0) == '/')) {
            // if you are working with the Robotology Superbuild
            const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
            // check the env, otherwise error
            if(env_p) {
                std::string current_path(env_p);
                // default relative path when working with the superbuild
                current_path += middle_path;
                current_path += input_path;
                absolute_path = current_path;
                return true;
            }
            else {
                std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
                return false;
            }
        }
        // already an absolute path
        absolute_path = input_path;
        return true;
    }

    std::shared_ptr<shlibpp::SharedLibraryClassFactory<PluginType>> _ioplugin_factory;
    std::shared_ptr<shlibpp::SharedLibraryClass<PluginType>> _ioplugin_class;

    bool _load_success;

};


/**
 * @brief Computes the absolute path corresponging to a given path relative to the $ROBOTOLOGY_ROOT 
 * environment variable.
 */
inline std::string computeAbsolutePath(const std::string& input_path){
    
    // if not an absolute path
    if(input_path == "" || !(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += "/";
            current_path += input_path;
            return current_path;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return "";
        }
    }
    
    // already an absolute path
    return input_path;
}

/**
 * @brief getter for the default XBot config file set by the env variable $XBOT_CONFIG
 */
inline std::string getXBotConfig()
{
    const char* env_p = std::getenv("XBOT_CONFIG");
    // check the env, otherwise error
    if(env_p) {
        std::string xbot_config(env_p);
        YAML::Node core_cfg = YAML::LoadFile(xbot_config);

        YAML::Node xbot_path_node;
        // XBotInterface info
        if(core_cfg["XBOT_CONFIG"]) {
            xbot_path_node = core_cfg["XBOT_CONFIG"];
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : YAML file  " << xbot_config << " does not contain XBOT_CONFIG mandatory node!!!" << std::endl;
            return "";
        }
        
        std::string xbot_path = xbot_path_node.as<std::string>();
        
        std::cout << __func__ << " -> " << xbot_path << std::endl;
        return xbot_path;
    }
    else {
        std::cerr << "WARNING in " << __func__ << " : XBOT_CONFIG env variable not set." << std::endl;
        return "";
    }
}

inline Eigen::Matrix6d GetAdjointFromRotation(const Eigen::Matrix3d& R){
    
    Eigen::Matrix6d I;
    I.setZero();
    
    I.block<3,3>(0,0) = R;
    I.block<3,3>(3,3) = R;
    
    return I;
    
}

    }

}

#endif