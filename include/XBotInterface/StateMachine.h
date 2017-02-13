/*
 * Copyright (C) 2017 IIT-ADVR
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


#ifndef __XBOT_FINITE_STATE_MACHINE_H__
#define __XBOT_FINITE_STATE_MACHINE_H__

#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Logger.hpp>

namespace XBot {

namespace FSM {

    /**
     * @brief Base class for defining custom events which states can react to.
     *
     */
    class Event {};

    /**
     * @brief Base class for defining custom messages that can be processed by
     * the states during the entry phase.
     *
     */
    class Message {};

    // Forward declaration for State class
    template <typename StateType> class State;


    /**
     * @brief Class implementing a finite state machine with states inheriting
     * from the user-defined class StateType, which must provided as the only
     * template parameter.
     */
    template <typename StateType>
    class StateMachine {

    public:

        friend class State<StateType>;

        StateMachine(): fsm_ptr(this), _is_fsm_init(false) {}

        bool init( const std::string& initial_state_name ){
            return init(initial_state_name, Message());
        }

        template <typename MessageType>
        bool init( const std::string& initial_state_name, const MessageType& msg )
        {
            auto it = _registered_states.find(initial_state_name);

            if( it == _registered_states.end() )
            {
                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << "! Initial state " <<
                initial_state_name <<
                " was not registered with this state machine! Call register_state()!" << std::endl;

                return false;
            }

            _current_state = _previous_state = it->second;
            _current_state->entry(msg);

            return true;
        }

        bool register_state( std::shared_ptr<StateType> state )
        {
            if( _registered_states.count(state->get_name()) ){
                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << "! State " <<
                state->get_name() <<
                " has already beed registered with this state machine!" << std::endl;

                return false;
            }

            state->_parent_fsm = fsm_ptr;
            _registered_states[state->get_name()] = state;

            return true;

        }

        template <typename EventType>
        bool send_event(const EventType& event)
        {
            if(!_is_fsm_init){
                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << "! State machine "  <<
                " has not been initialized! Call init()!" << std::endl;

                return false;
            }

            _current_state->react(event);
            return true;
        }

        bool run(double time, double period){

            if(!_is_fsm_init){
                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << "! State machine "  <<
                " has not been initialized! Call init()!" << std::endl;

                return false;
            }

            _current_state->run(time, period);
            return true;
        }

    protected:

    private:


        template <typename MessageType>
        bool transit( const std::string& next_state_name, const MessageType& msg )
        {
            auto it = _registered_states.find(next_state_name);

            if( it == _registered_states.end() )
            {
                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << "! Initial state " <<
                next_state_name <<
                " was not registered with this state machine! Call register_state()!" << std::endl;

                return false;
            }

            _current_state->exit();
            _previous_state = _current_state;
            _current_state = it->second;
            _current_state->entry(msg);

            return true;
        }

        std::shared_ptr<StateType> _current_state, _previous_state;

        StateMachine<StateType> * fsm_ptr;

        std::unordered_map<std::string, std::shared_ptr<StateType>> _registered_states;

        bool _is_fsm_init;


    };


    /**
     * @brief By inheriting from the State class, the user of the FSM defines
     * a base class for all custom states.  The only template parameter
     * is the base class for user-defined states, according to the Curiously
     * Recurring Template Pattern (CRTP).
     */
    template <typename StateType>
    class State {

    public:

        friend class StateMachine<StateType>;

        virtual std::string get_name() const = 0;

        virtual void run(double time, double period) {}
        virtual void exit() {}

        void react(const Event& e) {}


    protected:

        template <typename MessageType>
        bool transit( const std::string& next_state_name, const MessageType& msg )
        {
            return _parent_fsm->transit(next_state_name, msg);
        }

        bool transit( const std::string& next_state_name )
        {
            return _parent_fsm->transit(next_state_name, Message());
        }

        std::string get_previous_state_name() const
        {
            _parent_fsm->_previous_state->get_name();
        }

        template <typename EventType>
        void send_event(const EventType& e)
        {
            _parent_fsm->send_event(e);
        }



    private:

        StateMachine<StateType> * _parent_fsm;

    };

}

}

#endif