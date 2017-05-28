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
        _yd = initial_state*0;
        _ydd = initial_state*0;
        _udd = initial_state*0;
        _ud = initial_state*0;
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

        std::cout << "Coeffs: " << 1.0/_a0 << " -- " << _b1/_a0 << " -- " << _b2/_a0 << "\n" <<
        1.0 << " -- " << _a1/_a0 << " -- " << _a2/_a0 << std::endl;
    }

    double _omega;
    double _eps;
    double _ts;

    double _b1, _b2;
    double _a0, _a1, _a2;
    
    bool _reset_has_been_called;

    SignalType _y, _yd, _ydd, _u, _ud, _udd;

};


    }

}

#endif