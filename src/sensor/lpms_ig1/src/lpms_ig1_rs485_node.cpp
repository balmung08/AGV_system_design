
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"

#include "lpsensor/LpmsIG1I.h"
#include "lpsensor/SensorDataI.h"
#include "lpsensor/LpmsIG1Registers.h"

struct IG1Command
{
    short command;
    union Data {
        uint32_t i[64];
        float f[64];
        unsigned char c[256];
    } data;
    int dataLength;
};

class LpIG1Proxy
{
public:
    // Node handler
    ros::NodeHandle nh, private_nh;
    ros::Timer updateTimer;

    // Publisher
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher autocalibration_status_pub;

    // Service
    ros::ServiceServer autocalibration_serv;
    ros::ServiceServer autoReconnect_serv;
    ros::ServiceServer gyrocalibration_serv;
    ros::ServiceServer resetHeading_serv;
    ros::ServiceServer getImuData_serv;
    ros::ServiceServer setStreamingMode_serv;
    ros::ServiceServer setCommandMode_serv;

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    // Parameters
    std::string comportNo;
    int baudrate;
    int startupMode;
    bool autoReconnect;
    std::string frame_id;
    int rs485ControlPin;
    int rs485ControlPinToggleWaitMs;
    int rate;

    LpIG1Proxy(ros::NodeHandle h) : 
        nh(h),
        private_nh("~")
    {
        // Get node parameters
        private_nh.param<std::string>("port", comportNo, "/dev/ttyUSB0");
        private_nh.param("baudrate", baudrate, 115200);
        private_nh.param("startupmode", startupMode, SENSOR_MODE_STREAMING);
        private_nh.param("autoreconnect", autoReconnect, true);
        private_nh.param("rs485ControlPin", rs485ControlPin, -1);
        private_nh.param("rs485ControlPinToggleWaitMs", rs485ControlPinToggleWaitMs, 2);
        private_nh.param<std::string>("frame_id", frame_id, "imu");
        private_nh.param("rate", rate, 200);

        // Create LpmsIG1 object 
        sensor1 = IG1Factory();
        sensor1->setVerbose(VERBOSE_INFO);
        sensor1->setAutoReconnectStatus(autoReconnect);
        sensor1->setStartupSensorMode(startupMode);
        sensor1->setConnectionInterface(CONNECTION_INTERFACE_RS485);
        sensor1->setControlGPIOForRs485(rs485ControlPin);
        sensor1->setControlGPIOToggleWaitMs(rs485ControlPinToggleWaitMs); 

        ROS_INFO("Settings");
        ROS_INFO("Port: %s", comportNo.c_str());
        ROS_INFO("Baudrate: %d", baudrate);
        ROS_INFO("Startup mode: %s", (startupMode == 0)? "Command mode":"Streaming mode");
        ROS_INFO("Auto reconnect: %s", autoReconnect? "Enabled":"Disabled");
        ROS_INFO("rs485ControlPin: %d", rs485ControlPin);
        ROS_INFO("rs485ControlPinToggleWaitMs: %d", rs485ControlPinToggleWaitMs);

        imu_pub = nh.advertise<sensor_msgs::Imu>("data",1);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);
        autocalibration_status_pub = nh.advertise<std_msgs::Bool>("is_autocalibration_active", 1, true);

        autocalibration_serv = nh.advertiseService("enable_gyro_autocalibration", &LpIG1Proxy::setAutocalibration, this);
        autoReconnect_serv = nh.advertiseService("enable_auto_reconnect", &LpIG1Proxy::setAutoReconnect, this);
        gyrocalibration_serv = nh.advertiseService("calibrate_gyroscope", &LpIG1Proxy::calibrateGyroscope, this);
        resetHeading_serv = nh.advertiseService("reset_heading", &LpIG1Proxy::resetHeading, this);
        getImuData_serv = nh.advertiseService("get_imu_data", &LpIG1Proxy::getImuData, this);
        setStreamingMode_serv = nh.advertiseService("set_streaming_mode", &LpIG1Proxy::setStreamingMode, this);
        setCommandMode_serv = nh.advertiseService("set_command_mode", &LpIG1Proxy::setCommandMode, this);

         // Connects to sensor
        if (!sensor1->connect(comportNo, baudrate))
        {
            ROS_ERROR("Error connecting to sensor\n");
            sensor1->release();
            ros::Duration(3).sleep(); // sleep 3 s
        }

        do
        {
            ROS_INFO("Waiting for sensor to connect. Sensor status: %d", sensor1->getStatus());
            ros::Duration(1).sleep();
        } while(
            ros::ok() &&
            (
                !(sensor1->getStatus() == STATUS_CONNECTED) && 
                !(sensor1->getStatus() == STATUS_CONNECTION_ERROR)
            )
        );

        if (sensor1->getStatus() == STATUS_CONNECTED)
        {
            ROS_INFO("Sensor connected");
            ros::Duration(1).sleep();
            //sensor1->commandGotoStreamingMode();
        }
        else 
        {
            ROS_INFO("Sensor connection error: %d.", sensor1->getStatus());
            ros::shutdown();
        }
    }

    ~LpIG1Proxy(void)
    {
        sensor1->release();
    }

    void update(const ros::TimerEvent& te)
    {
        static bool runOnce = false;

        if (sensor1->getStatus() == STATUS_CONNECTED &&
                sensor1->hasImuData())
        {
            if (!runOnce)
            {
                publishIsAutocalibrationActive();
                runOnce = true;
            }
            IG1ImuDataI sd;
            sensor1->getImuData(sd);

            /* Fill the IMU message */

            // Fill the header
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;

            // Fill orientation quaternion
            imu_msg.orientation.w = sd.quaternion.data[0];
            imu_msg.orientation.x = -sd.quaternion.data[1];
            imu_msg.orientation.y = -sd.quaternion.data[2];
            imu_msg.orientation.z = -sd.quaternion.data[3];

            // Fill angular velocity data
            // - scale from deg/s to rad/s
            imu_msg.angular_velocity.x = sd.gyroIAlignmentCalibrated.data[0]*3.1415926/180;
            imu_msg.angular_velocity.y = sd.gyroIAlignmentCalibrated.data[1]*3.1415926/180;
            imu_msg.angular_velocity.z = sd.gyroIAlignmentCalibrated.data[2]*3.1415926/180;

            // Fill linear acceleration data
            imu_msg.linear_acceleration.x = -sd.accCalibrated.data[0]*9.81;
            imu_msg.linear_acceleration.y = -sd.accCalibrated.data[1]*9.81;
            imu_msg.linear_acceleration.z = -sd.accCalibrated.data[2]*9.81;

            /* Fill the magnetometer message */
            mag_msg.header.stamp = imu_msg.header.stamp;
            mag_msg.header.frame_id = frame_id;

            // Units are microTesla in the LPMS library, Tesla in ROS.
            mag_msg.magnetic_field.x = sd.magRaw.data[0]*1e-6;
            mag_msg.magnetic_field.y = sd.magRaw.data[1]*1e-6;
            mag_msg.magnetic_field.z = sd.magRaw.data[2]*1e-6;

            // Publish the messages
            imu_pub.publish(imu_msg);
            mag_pub.publish(mag_msg);
        }
    }

    void run(void)
    {
        // The timer ensures periodic data publishing
        updateTimer = ros::Timer(nh.createTimer(ros::Duration(1.0f/rate),
                                                &LpIG1Proxy::update,
                                                this));
    }

    void publishIsAutocalibrationActive()
    {
        std_msgs::Bool msg;
        IG1SettingsI settings;
        sensor1->getSettings(settings);
        msg.data = settings.enableGyroAutocalibration;
        autocalibration_status_pub.publish(msg);
    }


    ///////////////////////////////////////////////////
    // Service Callbacks
    ///////////////////////////////////////////////////
    bool setAutocalibration (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ROS_INFO("set_autocalibration");

        // clear current settings
        IG1SettingsI settings;
        sensor1->getSettings(settings);

        sensor1->commandSetGyroAutoCalibration(req.data);
        ros::Duration(0.2).sleep();

        double retryElapsedTime = 0;
        int retryCount = 0;
        while (!sensor1->hasSettings()) 
        {
            ros::Duration(0.1).sleep();
            ROS_INFO("set_autocalibration wait");

            retryElapsedTime += 0.1;
            if (retryElapsedTime > 2.0)
            {
                retryElapsedTime = 0;
                sensor1->commandGetGyroAutoCalibration();
                retryCount++;
            }

            if (retryCount > 5)
                break;
        }
        ROS_INFO("set_autocalibration done");

        // Get settings
        sensor1->getSettings(settings);

        std::string msg;
        if (settings.enableGyroAutocalibration == req.data) 
        {
            res.success = true;
            msg.append(std::string("[Success] autocalibration status set to: ") + (settings.enableGyroAutocalibration?"True":"False"));
        }
        else 
        {
            res.success = false;
            msg.append(std::string("[Failed] current autocalibration status set to: ") + (settings.enableGyroAutocalibration?"True":"False"));
        }

        ROS_INFO("%s", msg.c_str());
        res.message = msg;

        publishIsAutocalibrationActive();
        return res.success;
    }

    // Auto reconnect
    bool setAutoReconnect (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ROS_INFO("set_auto_reconnect");

        sensor1->setAutoReconnectStatus(req.data);
        
        res.success = true;
        std::string msg;
        msg.append(std::string("[Success] auto reconnection status set to: ") + (sensor1->getAutoReconnectStatus()?"True":"False"));
    
        ROS_INFO("%s", msg.c_str());
        res.message = msg;

        return res.success;
    }

    // reset heading
    bool resetHeading (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("reset_heading");

        sensor1->commandSetOffsetMode(LPMS_OFFSET_MODE_HEADING);

        res.success = true;
        res.message = "[Success] Heading resets";

        return true;
    }

    bool calibrateGyroscope (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds");
        
        sensor1->commandStartGyroCalibration();
        ros::Duration(4).sleep();
        res.success = true;
        res.message = "[Success] Gyroscope calibration procedure completed";
        ROS_INFO("calibrate_gyroscope: Gyroscope calibration procedure completed");

        //sensor1->commandGotoStreamingMode();
        return true;
    }

    bool getImuData (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        cmdGetImuData();
        res.success = true;
        res.message = "[Success] Get imu data";
        return true;
    }

    bool setStreamingMode (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        sensor1->commandGotoStreamingMode();
        res.success = true;
        res.message = "[Success] Set streaming mode";
        return true;
    }

    bool setCommandMode (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        sensor1->commandGotoCommandMode();
        res.success = true;
        res.message = "[Success] Set command mode";
        return true;
    }

    ///////////////////////////////////////////////////
    // Helpers
    ///////////////////////////////////////////////////

    void cmdGetImuData()
    {
        IG1Command cmd;
        cmd.command = GET_IMU_DATA;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }
    

 private:

    // Access to LPMS data
    IG1I* sensor1;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_ig1_node_rs485");
    ros::NodeHandle nh("imu");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    LpIG1Proxy lpIG1(nh);

    lpIG1.run();
    ros::waitForShutdown();

    return 0;
}
