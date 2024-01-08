/**
 * ESC.cpp
 *
 * @author    Ethan Foss
 * @date       09/26/2023
 */


#include <string>
#include <iostream>
#include <cstdlib>

//rclib
#ifdef __cplusplus
extern "C"
{
#endif
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#ifdef __cplusplus
}
#endif

#include "ros/ros.h" // ROS

//Msgs
//#include <std_msgs/Int32.h>
#include <lander_junior_ros/ActuatorCommands.h>

//Services
#include <std_srvs/SetBool.h>

class ESC
{
public:

    // Define Statements
    const double ESC_MAX_THROTTLE = 1.0; // Max Servo Angle [deg]
    const double ESC_MIN_THROTTLE = -.1; // Servo Speed [deg/s]
    const double THRUST_TO_THROTTLE = .0599628; // Thrust to Throttle [1/]
    const double ESC_FREQUENCY = 100; // ESC Update Frequency [Hz]

    ESC(ros::NodeHandle *nh)
    {

        // Set ESC Channels
        if (nh->getParam("esc_channel_1",escChannel1))
        {
            std::cout << "Failed to get ESC Channel 1" << std::endl;
            exit(1);
        }
        if (nh->getParam("esc_channel_2",escChannel2))
        {
            std::cout << "Failed to get ESC Channel 2" << std::endl;
            exit(1);
        }

        // Create Subscriber
        std::string esc_sub_name;
        if (nh->getParam("~topics/actuator_commands",esc_sub_name))
        {
            std::cout << "Failed to get ESC Sub Name" << std::endl;
            exit(1);
        }
        actuator_cmd_sub_ = nh->subscribe(esc_sub_name,10,&ESC::subscriberCallback,this);

        // Create Service Listener
        std::string esc_enable_service_name;
        if (nh->getParam("~services/EnableESC",esc_enable_service_name))
        {
            std::cout << "Failed to get ESC Sub Name" << std::endl;
            exit(1);
        }
        disable_esc_service = nh->advertiseService(esc_enable_service_name,&ESC::handleEnableService,this);

        // Create Service Listener
        std::string esc_disable_service_name;
        if (nh->getParam("~services/DisableESC",esc_disable_service_name))
        {
            std::cout << "Failed to get ESC Sub Name" << std::endl;
            exit(1);
        }
        enable_esc_service = nh->advertiseService(esc_disable_service_name,&ESC::handleDisableService,this);

        // Create Timer for Servo
        timer_ = nh->createTimer(ros::Duration(1.0/ESC_FREQUENCY),&ESC::timerCallback,this);

    }

    bool handleEnableService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        if(req.data)
        {
            res.success = enableESCs();
            res.message = "Servos Successfully Enabled";
            return res.success;
        }
        res.success = false;
        return false;
        
    }

    bool handleDisableService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
    {
        if(req.data)
        {
            res.success = disableESCs();
            res.message = "Servos Successfully Disabled";
            return res.success;
        }
        res.success = false;
        return false;
        
    }

private:

    int escChannel1;
    int escChannel2;
    double T1;
    double T2;
    bool enabled;

    ros::Timer timer_;
    ros::Subscriber actuator_cmd_sub_;
    ros::ServiceServer enable_esc_service;
    ros::ServiceServer disable_esc_service;

    bool enableESCs()
    {

        if(enabled)
        {
            std::cout << "ESCs already enabled!" << std::endl;
            return false;
        }

        // Check Battery Power
        if(rc_adc_init())
        {
            std::cout << "Failed to Initialize ADC for ESCs" << std::endl;
            exit(1);
            return false;
        }
        if(rc_adc_batt()<6.0)
        {
            std::cout << "Battery Power Insufficient to Drive ESCs" << std::endl;
            exit(1);
            return false;
        }
        rc_adc_cleanup();

        //initialize PRU
        if(rc_servo_init())
        {
            std::cout << "Failed to Intialize ESC" << std::endl;
            exit(1);
            return false;
        }
        if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US))
        {
            std::cout << "Failed to Intialize ESC" << std::endl;
            exit(1);
            return false;
        }

        // Power Rail
        std::cout << "Powering Servo Rail" << std::endl;
        rc_servo_power_rail_en(0);

        // Sleep 2 Seconds
        ros::Duration(2).sleep();

        // Enable:
        enabled = true;
        return true;
    }

    bool disableESCs()
    {
        if(!enabled)
        {
            std::cout << "ESCs already disabled!" << std::endl;
            return false;
        }

        enabled = false;

        rc_servo_send_esc_pulse_normalized(escChannel1,ESC_MIN_THROTTLE);
        rc_servo_send_esc_pulse_normalized(escChannel2,ESC_MIN_THROTTLE);
        rc_servo_power_rail_en(0);
        rc_servo_cleanup();
        return true;

    }

    void timerCallback(const ros::TimerEvent& event)
    {
        if(enabled)
        {
            rc_servo_send_esc_pulse_normalized(escChannel1,THRUST_TO_THROTTLE*T1);
            rc_servo_send_esc_pulse_normalized(escChannel2,THRUST_TO_THROTTLE*T2);
        }
        //else
        //{
        //    rc_servo_power_rail_en(0);
        //    rc_servo_cleanup();
        //    exit(1);
        //}
    }

    void subscriberCallback(const lander_junior_ros::ActuatorCommands& actuator_commands_msg)
    {
        T1 = actuator_commands_msg.T1;
        T2 = actuator_commands_msg.T2;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"esc_node");
    ros::NodeHandle nh;
    ESC esc = ESC(&nh);
    ros::spin();
    ros::shutdown();
    return 0;
}