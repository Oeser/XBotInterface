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

#ifndef __XBOT_SYNCFLAGS__
#define __XBOT_SYNCFLAGS__

#include <XBotInterface/TypedefAndEnums.h>

namespace XBot {

    inline void parseFlags(bool& pos,
                           bool& vel,
                           bool& acc,
                           bool& eff,
                           bool& k,
                           bool& d,
                           bool& ft,
                           bool& imu,
                           Sync s = Sync::All){

        pos = (s == Sync::Position) || (s == Sync::All);
        vel = (s == Sync::Velocity) || (s == Sync::All);
        acc = (s == Sync::Acceleration) || (s == Sync::All);
        eff = (s == Sync::Effort) || (s == Sync::All);
        k = (s == Sync::Stiffness) || (s == Sync::Impedance) || (s == Sync::All);
        d = (s == Sync::Damping) || (s == Sync::Impedance) || (s == Sync::All);
        ft = (s == Sync::ForceTorque) || (s == Sync::Sensors) || (s == Sync::All);
        imu = (s == Sync::Imu) || (s == Sync::Sensors) || (s == Sync::All);

    }

    template <typename... Flags>
    inline void parseFlags(bool& pos,
                           bool& vel,
                           bool& acc,
                           bool& eff,
                           bool& k,
                           bool& d,
                           bool& ft,
                           bool& imu,
                           Sync s, Flags... rest){

        parseFlags(pos, vel, acc, eff, k, d, ft, imu, rest...);

        pos = pos || (s == Sync::Position);
        vel = vel || (s == Sync::Velocity);
        acc = acc || (s == Sync::Acceleration);
        eff = eff || (s == Sync::Effort);
        k = k || (s == Sync::Stiffness);
        d = d || (s == Sync::Damping);
        ft = ft || (s == Sync::ForceTorque);
        imu = imu || (s == Sync::Imu);

    }



    class FlagsParser {

    public:

        FlagsParser(){
            load_flags();
        }

        template <typename... Flags>
        void load_flags(Flags... flags)
        {
            _flags_map.clear();
            _flags_map["pos"] = false;
            _flags_map["vel"] = false;
            _flags_map["acc"] = false;
            _flags_map["eff"] = false;
            _flags_map["k"] = false;
            _flags_map["d"] = false;
            _flags_map["ft"] = false;
            _flags_map["imu"] = false;

            parseFlags(_flags_map["pos"],
                _flags_map["vel"],
                _flags_map["acc"],
                _flags_map["eff"],
                _flags_map["k"],
                _flags_map["d"],
                _flags_map["ft"],
                _flags_map["imu"],
                flags...
            );

        }

        bool position_flag() const { return _flags_map.at("pos"); }
        bool velocity_flag() const { return _flags_map.at("vel"); }
        bool acceleration_flag() const { return _flags_map.at("acc"); }
        bool effort_flag() const { return _flags_map.at("eff"); }
        bool stiffness_flag() const { return _flags_map.at("k"); }
        bool damping_flag() const { return _flags_map.at("d"); }
        bool ft_flag() const { return _flags_map.at("ft"); }
        bool imu_flag() const { return _flags_map.at("imu"); }

    private:

        std::map<std::string, bool> _flags_map;

    };

}
#endif