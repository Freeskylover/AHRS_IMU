#include <iostream>
// ros
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
//for shared ptr
#include <memory>
//for matrix
#include <Eigen/Core>
#include <Eigen/Geometry>


#define GRAVITY_ACCELERATION 9.81
#define PAYLOAD_OFFSET 6
#define PI 3.1415926535898

class ImuData
{
public:
    ImuData()
    {
    }

    ~ImuData()
    {

    }

    virtual void getDataFromBuff(uint8_t  *const &  buffer )
    {
        Header1 = *buffer;
        Header2 = *(buffer + 1);
        MessageType = *(buffer + 2);
        identifier = *(buffer + 3);
        MessageLength = *(uint16_t * )(buffer+4);

        CheckSum = *(uint16_t * )(buffer + MessageLength);
    }

    virtual void pulishSensorMsg(ros::Publisher IMU_pub)
    {
    }


public:
    uint8_t Header1;
    uint8_t Header2;
    uint8_t MessageType;
    uint8_t identifier;
    uint16_t MessageLength;

    uint16_t CheckSum;

    bool bPublishRelativeRotation = true;
};


class HRData : public  ImuData
{
public:
    void getDataFromBuff( uint8_t * const  &  buffer ) override final
    {
        ImuData::getDataFromBuff(buffer );
        uint8_t * bufferCurse = buffer + PAYLOAD_OFFSET ;

        Heading = *(int * )bufferCurse;
        Pitch = *(int * )(bufferCurse + 4);
        Roll = * (int * )(bufferCurse + 8);

        GyroX = *(int * )(bufferCurse + 12);
        GyroY = *(int * )(bufferCurse + 16);
        GyroZ = *(int * )(bufferCurse + 20);

        AccX = *(int * )(bufferCurse + 24);
        AccY = *(int * )(bufferCurse + 28);
        AccZ = *(int * )(bufferCurse + 32);

        MagX = *(int16_t * )(bufferCurse + 36);
        MagY = *(int16_t * )(bufferCurse + 38);
        MagZ = *(int16_t * )(bufferCurse + 40);

        Reserved1 = *(int16_t * )(bufferCurse + 42);
        Reserved2 = *(int16_t * )(bufferCurse + 44);
        USW = *(uint16_t * )(bufferCurse + 46);
        Vinp = *(uint16_t * )(bufferCurse + 48);
        Temper = *(int16_t * )(bufferCurse + 50);

    }

    void pulishSensorMsg(ros::Publisher IMU_pub) override final
    {
        static Eigen::Quaterniond FirstRotation;
        static bool isFirst = true;

        sensor_msgs::Imu IMU_Msg;
        IMU_Msg.header.stamp = ros::Time::now() ;
        IMU_Msg.header.frame_id = "imu" ;

        Eigen::AngleAxisd roll(Roll /1000.0 * PI /180 , Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitch(Pitch /1000.0 * PI /180, Eigen::Vector3d::UnitX());
        double tmpYaw =  Heading /1000.0 ;
        if(tmpYaw >= 0 && tmpYaw < 180)
        {
            tmpYaw = -tmpYaw ;
        } else
        {
            tmpYaw = -tmpYaw + 360 ;
        }

        Eigen::AngleAxisd yaw(tmpYaw * PI /180, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond Rotation =  yaw * pitch * roll ;

        if(isFirst)
        {
            FirstRotation = Rotation;
            isFirst = false ;
        }

        if(bPublishRelativeRotation)
        {
            Rotation = FirstRotation.inverse() * Rotation ;
            Eigen::Vector3d euler = Rotation.toRotationMatrix().eulerAngles(2, 0, 1);
            Eigen::Vector3d first_euler = FirstRotation.toRotationMatrix().eulerAngles(2,0,1);

            std::cout<< "current : "<< euler.transpose() / 3.1415926 * 180;
            std::cout<<"--"<<"first : " << first_euler.transpose()/ 3.1415926 * 180 ;

        }
        std::cout<<" -----------  "<< tmpYaw <<"  "<< Pitch /1000. << "  " << Roll / 1000. <<std::endl;

        IMU_Msg.orientation.x = Rotation.x() ;
        IMU_Msg.orientation.y = Rotation.y() ;
        IMU_Msg.orientation.z = Rotation.z() ;
        IMU_Msg.orientation.w = Rotation.w() ;

        IMU_Msg.linear_acceleration.x = AccX * GRAVITY_ACCELERATION / 1e6 ;
        IMU_Msg.linear_acceleration.y = AccY * GRAVITY_ACCELERATION / 1e6 ;
        IMU_Msg.linear_acceleration.z = AccZ * GRAVITY_ACCELERATION / 1e6  ;

        IMU_Msg.angular_velocity.x = GyroX / 1e5 * 3.14 / 180 ;
        IMU_Msg.angular_velocity.y = GyroY / 1e5 * 3.14 / 180 ;
        IMU_Msg.angular_velocity.z = GyroZ / 1e5 * 3.14 / 180 ;

        IMU_pub.publish<sensor_msgs::Imu>(IMU_Msg);
    }

public :
    int Heading;
    int Pitch;
    int Roll;

    int AccX;
    int AccY;
    int AccZ;

    int GyroX;
    int GyroY;
    int GyroZ;

    int16_t MagX;
    int16_t MagY;
    int16_t MagZ;

    int16_t Reserved1;
    int16_t Reserved2;

    uint16_t USW;
    uint16_t Vinp;
    int16_t Temper;


    //static  uint8_t Stop[9] ;
    //static  uint8_t SendData[9];
};


uint8_t  SendData[9] = {0xaa , 0x55 , 0x00 , 0x00 , 0x07 , 0x00 , 0x81 , 0x88 , 0x00};
uint8_t  Stop[9]     = {0xaa , 0x55 , 0x00 , 0x00 , 0x07 , 0x00 , 0xFE , 0x05 , 0x01};




class SerialReader
{
private:
    SerialReader(){}
public:
    void init(const ros::NodeHandle & private_nh)
    {
        int time;
        private_nh.param<int>("Timeout", time, 100);
        private_nh.param<std::string>("SerialName", PortName, "/dev/ttyUSB1");
        private_nh.param<int>("BaudRate", buadRate, 115200);
        timeout = std::make_shared<serial::Timeout>(serial::Timeout::simpleTimeout(time));
        sp.setTimeout(*timeout);
        sp.setBaudrate(buadRate);
        sp.setPort(PortName);
    }

    bool open()
    {
        if(sp.isOpen())
        {
            ROS_ERROR_STREAM("Port is open !");
            sp.close();
        }
        try
        {
            sp.open();    //打开串口
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return false;
        }

        if(!sp.isOpen())
        {
            ROS_ERROR_STREAM("failed  to open port.");
            return false;
        }

        return true;
    }

    bool readData(ImuData * const & imuData)
    {
        if(sp.available()<60)
        {
            return false;
        }
        sp.read(buffer,60);

        imuData->getDataFromBuff(buffer);
        return true;
    }

    bool stop()
    {
        try
        {
            sp.write(Stop, sizeof(Stop));
            isInitial = false ;
            ROS_INFO("success to stop!");
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Fail to transmit stop command!");
            return false;
        }

        return true;
    }

    bool start()
    {
        stop();
        try
        {
            sp.write(SendData, sizeof(SendData));
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Fail to transmit sendData command!");
            return false;
        }

        // judge if reply from IMU has been transmitted.
        while(1)
        {
            if(sp.available()<500)
                continue;
            uint8_t bufferTmp[500];
            sp.read(bufferTmp, sizeof(bufferTmp));
            ROS_INFO("Success to transmit sendData command!");
            break;
        }

        //alignment that putting  Header  first byte the next time you read.
        sp.read(buffer ,60);
        for(int i = 0 ; i < 60 ; i ++ )
        {
            //Detecting Header
            if(buffer[i+1] == 0xaa && buffer[i+2] == 0x55)
            {
                sp.read(buffer,i+1);
                isInitial = true;
                ROS_INFO("Intialized completed!");
                break;
            }

        }
        return true;
    }

    void close()
    {
        sp.close();
    }




    static SerialReader *  getInstance()
    {
        if(serialReaderInstance == nullptr)
        {
            serialReaderInstance = new SerialReader();
            return serialReaderInstance ;
        }
        else
        {
            return serialReaderInstance;
        }

    }

private:
    //param used for set serial
    serial::Serial sp;
    std::shared_ptr<serial::Timeout> timeout;
    std::string PortName;
    int buadRate ;

    //    The singleton pattern
    static SerialReader * serialReaderInstance;

    uint8_t buffer[60]; //used for sto
    bool isInitial = false;
};

SerialReader *  SerialReader::serialReaderInstance = nullptr;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "AHRS_Imu");
    ros::NodeHandle nh ;
    ros::NodeHandle private_nh("~");
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("IMU",10);

    // initialize serial reader
    SerialReader * serial = SerialReader::getInstance() ;
    serial->init(private_nh);
    serial->open();
    serial->start();

    ros::Rate loop_rate(100);
    ImuData * imuData = new HRData();


    // publish sensor message
    while (ros::ok())
    {
        serial->readData(imuData);
        imuData->pulishSensorMsg(IMU_pub);
        loop_rate.sleep();
    }

    //close serial
    serial->stop();
    serial->close();


    return 0;
}