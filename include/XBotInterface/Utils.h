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

namespace XBot { namespace Utils {

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


}


#endif