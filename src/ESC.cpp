/**
 * servo.cpp
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
#include <msgs/ActuatorCommands.h>

class ESC
{
public:

    // Define Statements
    const double ESC_MAX_THROTTLE = 1.0; // Max Servo Angle [deg]
    const double ESC_MIN_THROTTLE = -.1; // Servo Speed [deg/s]
    const double THRUST_TO_THROTTLE = .0599628; // Thrust to Throttle [1/]

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

        // Check Battery Power
        if(rc_adc_init())
        {
            std::cout << "Failed to Initialize ADC for ESCs" << std::endl;
            exit(1);
        }
        if(rc_adc_batt()<6.0)
        {
            std::cout << "Battery Power Insufficient to Drive ESCs" << std::endl;
            exit(1);
        }
        rc_adc_cleanup();

        //initialize PRU
        if(rc_servo_init())
        {
            std::cout << "Failed to Intialize ESC" << std::endl;
            exit(1);
        }
        if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US,RC_ESC_DEFAULT_MAX_US))
        {
            std::cout << "Failed to Intialize ESC" << std::endl;
            exit(1);
        }

        // Power Rail
        std::cout << "Powering Servo Rail" << std::endl;
        rc_servo_power_rail_en(0);
        enabled = true;

        // Sleep 2 Seconds
        sleep(2);

        // Create Subscriber
        std::string esc_sub_name;
        if (nh->getParam("~topics/esc",esc_sub_name))
        {
            std::cout << "Failed to get Servo Sub Name" << std::endl;
            exit(1);
        }
        servo_cmd_sub_ = nh->subscribe(esc_sub_name,10,&ESC::subscriberCallback,this);

        // Create Timer for Servo
        timer_ = nh->createTimer(ros::Duration(1.0/SERVO_FREQUENCY),&ESC::timerCallback,this);

    }

private:

    int escChannel1;
    int escChannel2;
    double T1;
    double T2;
    bool enabled;

    ros::Timer timer_;
    ros::Subscriber servo_cmd_sub_;

    void timerCallback(const ros::TimerEvent& event)
    {
        if(enabled)
        {
            rc_servo_send_esc_pusle_normalized(escChannel1,THRUST_TO_THROTTLE*T1)
            rc_servo_send_esc_pusle_normalized(escChannel2,THRUST_TO_THROTTLE*T2)
        }
        //else
        //{
        //    rc_servo_power_rail_en(0);
        //    rc_servo_cleanup();
        //    exit(1);
        //}
    }

    void subscriberCallback(const msgs::ActuatorCommands& actuator_commands_msg)
    {
        T1 = actuator_commands_msg.T1;
        T2 = actuator_commands_msg.T2;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Hi" << std::endl;
    ros::init(argc,argv,"servo_node");
    ros::NodeHandle nh;
    Servo servo = Servo(&nh);
    ros::spin();
    ros::shutdown();
    return 0;
}