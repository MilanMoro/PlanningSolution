#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Rand.hh>

#include "custom_messages.pb.h"

#include <gazebo/gazebo_client.hh>

#include "EgoController.h"

typedef const boost::shared_ptr<
        const custom_messages::WorldState>
        WorldStateRequestPtr;

typedef const boost::shared_ptr<
        const custom_messages::Statistics>
        StatisticsRequestPtr;

class Controller {
    private: gazebo::transport::NodePtr node;

    private: gazebo::transport::SubscriberPtr world_sub;
    private: gazebo::transport::SubscriberPtr statistics_sub;
    private: gazebo::transport::PublisherPtr pub;

    private: std::string worldTopicName = "~/world_state";
    private: std::string statisticsTopicName = "~/statistics";
    private: std::string commandTopicName = "~/client_command";

    private: int32_t simulation_round = 0;

    public: EgoController Control = EgoController({20,1.999,1.999}, 0.1, 46, 58, 0.05);

    public: void Init()
    {
        // Create our node for communication
        node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init();

        //Controller will replan in every time step
        Control.EnableAutoReplanning(0);

        // Subscribe to the topic, and register a callback
        this->world_sub = node->Subscribe(worldTopicName, &Controller::OnWorldStateReceived, this);

        this->statistics_sub = node->Subscribe(statisticsTopicName, &Controller::OnStatisticsReceived, this);

        // Publish to the velodyne topic
        this->pub = node->Advertise<custom_messages::Command>(commandTopicName);

        // Wait for a subscriber to connect to this publisher
        this->pub->WaitForConnection();
    }

    // Called when the simulator resets the world after reaching the goal, running out of time or crashing
    public: void ResetWorld()
    {
        std::cout << "New simulation round started, resetting world." << std::endl;
    }

    public: void PrintWorldStateMessage(WorldStateRequestPtr& msg) const
    {
        std::cout << "Simulation round: " << msg->simulation_round() << "; "
                  << "time: " << msg->time().sec() << "." << msg->time().nsec() << std::endl;
        std::cout << "  ego_car "
                  << " lane id: " << msg->ego_vehicle().lane_id()
                  << " p: (" << msg->ego_vehicle().position().x() << ", " << msg->ego_vehicle().position().y()
                  << ") v: (" << msg->ego_vehicle().velocity().x() << ", " << msg->ego_vehicle().velocity().y() << "); "
                  << std::endl;
        for (const auto& vehicle_msg : msg->vehicles())
        {
            std::cout << "  car id " << vehicle_msg.vehicle_id()
                      << " lane id: " << vehicle_msg.lane_id()
                      << " p: (" << vehicle_msg.position().x() << ", " << vehicle_msg.position().y()
                      << ") v: (" << vehicle_msg.velocity().x() << ", " << vehicle_msg.velocity().y() << "); "
                      << std::endl;
        }

        std::cout << std::endl;
    }

    public: void PrintStatisticsMessage(StatisticsRequestPtr& msg) const
    {
        std::cout << "Statistics from previous round: "
                  << "Success: " << msg->success() << ", "
                  << "collision: " << msg->collision_detected() << ", "
                  << "Time steps: "
                  << msg->simulation_time_steps_taken() << ", "
                  << "total acceleration: " << msg->total_acceleration() << ", "
                  << "acceleration/jerk limits respected: " << msg->limits_respected()
                  << std::endl;
    }

    // Called every time a new update is received from the simulator.
    public: void OnWorldStateReceived(WorldStateRequestPtr& msg)
    {
        PrintWorldStateMessage(msg);

        // If the simulation round is different then this is a whole new setting, reinitialize the world
        if (msg->simulation_round() != simulation_round)
        {
            ResetWorld();
            Control.Reset();
            simulation_round = msg->simulation_round();
            //In reset acceleration should be 0
            Control.SetState(msg->ego_vehicle().position().x()+50,msg->ego_vehicle().velocity().x(),0);
        }
        else
        {
        	//Otherwise it is calculated
        	Control.SetState(msg->ego_vehicle().position().x()+50,msg->ego_vehicle().velocity().x());
        }

        for (const auto& vehicle_msg : msg->vehicles())
        {
        	Control.AddCar(vehicle_msg.lane_id(), vehicle_msg.position().x(),vehicle_msg.position().y(),
            		vehicle_msg.velocity().x(), vehicle_msg.velocity().y());
        }

        // Calculate the next velocity for the ego car and send the response.
        custom_messages::Command response_msg;
        double vControl = Control.ActualVelocityControlValue();
        response_msg.set_ego_car_speed(vControl);
        response_msg.set_simulation_round(msg->simulation_round());
        this->pub->Publish(response_msg);
    }

    public: void OnStatisticsReceived(StatisticsRequestPtr& msg)
    {
        PrintStatisticsMessage(msg);
    }
};

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    Controller controller;
    controller.Init();

    // for (int i = 0; i < 100; ++i)
    while (true)
        gazebo::common::Time::MSleep(100);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
