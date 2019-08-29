#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"

double x_last;
double p_last;
bool first_flag = true;
ros::Publisher pressure_pub;

double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R)
{
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;
    double x_mid = x_last;
    double x_now;
    double p_mid;
    double p_now;
    double kg;

    x_mid = x_last;                           //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q;                       //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    kg = p_mid / (p_mid + R);                 //kg为kalman filter，R为噪声
    x_now = x_mid + kg * (ResrcData - x_mid); //估计出的最优值

    p_now = (1 - kg) * p_mid; //最优值对应的covariance

    p_last = p_now; //更新covariance值
    x_last = x_now; //更新系统状态值

    return x_now;
}

void BarometerCallback(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    if (first_flag)
    {
        x_last = msg->fluid_pressure;
        first_flag = false;
    }

    double pressure = KalmanFilter(msg->fluid_pressure, 0.1, 6.0);
    printf("%.1f ---->%.1f----->%.1f\n", msg->fluid_pressure, pressure, msg->fluid_pressure - pressure);

    sensor_msgs::FluidPressure msg_new = *msg;
    msg_new.fluid_pressure = pressure;
    pressure_pub.publish(msg_new); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "barometer_test");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("barometer", 1, BarometerCallback);
    pressure_pub = n.advertise<sensor_msgs::FluidPressure>("barometer_filted", 1);

    ros::spin();
    return 0;
}
