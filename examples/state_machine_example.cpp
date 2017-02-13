#include <XBotInterface/StateMachine.h>
#include <iostream>

using namespace XBot;

// ONE: define events by inheriting from FSM::Event ...
class LiftLeg : public FSM::Event {

public:

    LiftLeg(int leg_id, double liftoff_time): leg_id(leg_id),
                                              liftoff_time(liftoff_time)
    {}

    int leg_id;
    double liftoff_time;

};

class PlaceLeg : public FSM::Event {

public:

    PlaceLeg(int leg_id, double touchdown_time): leg_id(leg_id),
                                                 touchdown_time(touchdown_time)
    {}

    int leg_id;
    double touchdown_time;

};

class Walk : FSM::Event {

};

// ..and messages by inheriting from FSM::Message
class LiftLegMessage : public FSM::Message {

public:

    int leg_id;
    double time_to_liftoff;
};




// TWO: define the base class for all states of the FSM, by inheriting from FSM::State.
// The only template parameter is the derived class itself (we make use of the Curiously
// Recurring Template Pattern, CRTP).
// A virtual react() method must be defined
// for all events and an entry method for all messages.
// This is how we tell the state machine to listen to certain events/messages
class LocomotionState :  public FSM::State<LocomotionState> {

public:

    virtual void react(const LiftLeg& e) {}
    virtual void react(const PlaceLeg& e) {}
    virtual void react(const Walk& e) {}
    virtual void entry(const FSM::Message& msg) {}
    virtual void entry(const LiftLegMessage& msg) {}


};


// THREE: define the states by inheriting from the states base-class
class LiftingLeg : public LocomotionState {

public:

    virtual std::string get_name() const { return "LIFTING_LEG"; }
    virtual void run(double time, double period){
        std::cout << "Lifting leg " << leg_id << ", " << time_left << " left..." << std::endl;
        time_left -= period;
        if(time_left < 0) transit("IDLE");
    }

    virtual void entry(const LiftLegMessage& msg){
        leg_id = msg.leg_id;
        time_left = msg.time_to_liftoff;
    }

private:

    int leg_id;
    double time_left;

};

class Idle : public LocomotionState {

public:

    virtual std::string get_name() const { return "IDLE"; }
    virtual void run(double time, double period)
    {
        std::cout << "Idle..." << std::endl;
    }
    virtual void react(const LiftLeg& e)
    {
        LiftLegMessage msg;
        msg.leg_id = e.leg_id;
        msg.time_to_liftoff = e.liftoff_time;
        transit("LIFTING_LEG",  msg);
    }

};


int main(int argc, char** argv){

    // Instantiate the FSM, providing the state base class as template parameter
    FSM::StateMachine<LocomotionState> fsm;

    // Construct states and register states with the FSM
    fsm.register_state(std::make_shared<Idle>());
    fsm.register_state(std::make_shared<LiftingLeg>());

    // Initialize the FSM with the initial state
    fsm.init("IDLE");

    double time = 0;

    for( int i = 0; i < 500; i++ ){
        time += 0.01;
        fsm.run(time, 0.01);

       if( i == 200 ){
           fsm.send_event(LiftLeg(1,2));
       }

    }




}