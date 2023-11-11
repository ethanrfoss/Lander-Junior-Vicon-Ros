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
#include <std_msgs/Int32.h>

class Servo
{
public:

    // Define Statements
    const double SERVO_MAX_ANGLE = 20.0; // Max Servo Angle [deg]
    const double SERVO_SPEED = 250.0; // Servo Speed [deg/s]
    const int SERVO_FREQUENCY = 100; // Servo Frequency [Hz]
    const double SERVO_MAX_COMMAND_DIFF = 5.0; // Servo Max Command Difference [deg]
    const double SERVO_DEG_TO_VAL = 0.011111111; // Servo Degrees to Command Value [deg]
    //const int RC_SERVO_CH_MIN = 1; // Channel Limits
    //const int RC_SERVO_CH_MAX = 8; // Channel Limits

    Servo(ros::NodeHandle *nh)
    {
        // Initialize Servo Position
        servoPos = 0;

        // Set Servo Channel
        if (nh->getParam("servo_channel",servoChannel))
        {
            std::cout << "Failed to get Servo Channel" << std::endl;
            exit(1);
        }

        // Check Battery Power
        if(rc_adc_init())
        {
            std::cout << "Failed to Initialize ADC for Servos" << std::endl;
            exit(1);
        }
        if(rc_adc_batt()<6.0)
        {
            std::cout << "Battery Power Insufficient to Drive Servos" << std::endl;
            exit(1);
        }
        rc_adc_cleanup();

        //initialize PRU
        if(rc_servo_init())
        {
            std::cout << "Failed to Intialize Servo" << std::endl;
            exit(1);
        }

        // Power Rail
        std::cout << "Powering Servo Rail" << std::endl;
        rc_servo_power_rail_en(1);
        enabled = true;

        // Sleep 2 Seconds
        sleep(2);

        // Create Subscriber
        std::string servo_sub_name;
        if (nh->getParam("~topics/servo",servo_sub_name))
        {
            std::cout << "Failed to get Servo Sub Name" << std::endl;
            exit(1);
        }
        servo_cmd_sub_ = nh->subscribe(servo_sub_name,10,&Servo::subscriberCallback,this);

        // Create Timer for Servo
        timer_ = nh->createTimer(ros::Duration(1.0/SERVO_FREQUENCY),&Servo::timerCallback,this);

    }

private:

    int servoChannel;
    double servoPos;
    double servoCommand;
    bool enabled;

    ros::Timer timer_;
    ros::Subscriber servo_cmd_sub_;

    void timerCallback(const ros::TimerEvent& event)
    {
        if(enabled)
        {
            if(abs(servoPos-servoCommand) < SERVO_MAX_COMMAND_DIFF*SERVO_DEG_TO_VAL)
            {
                servoPos = servoCommand;
            }
            else if(servoPos > servoCommand)
            {
                servoPos-=SERVO_SPEED/SERVO_FREQUENCY;
            }
            else
            {
                servoPos+=SERVO_SPEED/SERVO_FREQUENCY;
            }

            if(servoPos>SERVO_MAX_ANGLE)
            {
                servoPos = SERVO_MAX_ANGLE;
            }
            if(servoPos<-SERVO_MAX_ANGLE)
            {
                servoPos = -SERVO_MAX_ANGLE;
            }

            if(rc_servo_send_pulse_normalized(servoChannel,servoPos)==-1)
            {
                std::cout << "Failed to set Servo Position" << std::endl;
                exit(1);
            }
        }
        else
        {
            rc_servo_power_rail_en(0);
            rc_servo_cleanup();
            exit(1);
        }
    }

    void subscriberCallback(const std_msgs::Int32& servoMsg)
    {
        servoCommand = servoMsg.data;
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