#include "ros/ros.h"
#include "equation_solver/solve.h"
#include "cmath"
#include <std_msgs/Float32MultiArray.h>


ros::Publisher pub;
ros::Subscriber sub;
int root_cnt = 0;
const float EPS = 0.000001;

bool solve(equation_solver::solve::Request &req, equation_solver::solve::Response &res)
{
    std_msgs::Float32MultiArray root_arr;
    root_arr.layout.dim.push_back(std_msgs::MultiArrayDimension());
    float disct = req.b*req.b - 4*req.a*req.c; //D = b^2-4ac

    if (req.a == 0) //если уравнение не квадратное
    {
        if (req.b == 0) //если нет коэффициентов при x
        {
            root_arr.layout.dim[0].size = 1;
            root_arr.layout.dim[0].stride = 0;
            root_arr.data.clear();
            root_cnt = -1;
        }
        else
        {
            root_cnt = 1;
            res.root.push_back((-1*req.c)/(req.b));
            root_arr.layout.dim[0].size = 1;
            root_arr.layout.dim[0].stride = 1;
            root_arr.data.clear();
            root_arr.data.push_back(res.root[0]);
        }
    }
    else //уравнение квадратное
    {
        
        if (disct > -EPS) //D > 0
        {
            if ((disct > -EPS) && (disct < EPS)) //D = 0
            {
                root_cnt = 1;
                root_arr.layout.dim[0].size = 1;
                root_arr.layout.dim[0].stride = 1;
                root_arr.data.clear();
                res.root.push_back((-1*req.b + sqrtf(disct))/(2*req.a));
                root_arr.data.push_back(res.root[0]);
            }
            else 
            {
                root_cnt = 2;
                res.root.push_back((-1*req.b + sqrtf(disct))/(2*req.a));
                res.root.push_back((-1*req.b - sqrtf(disct))/(2*req.a));
                root_arr.layout.dim[0].size = 1;
                root_arr.layout.dim[0].stride = 2;
                root_arr.data.clear();

                root_arr.data.push_back(res.root[0]);
                root_arr.data.push_back(res.root[1]);
            }
            
        }
        else
        {
            
            root_cnt = 0;
        }
    }
    
    pub.publish(root_arr);
    return true;
}

void resCallback(const std_msgs::Float32MultiArray::ConstPtr& topic)
{
    switch (root_cnt)
    {
        case 0:
            ROS_INFO("No roots");
            break;
        case 1:
            ROS_INFO("x = %f", topic->data[0]);
            break;
        case 2:
            ROS_INFO("x1 = %f, x2 = %f", topic->data[0], topic->data[1]);
            break;
        default:
        {
            ROS_INFO("?");
            break;
        }
        
    }
    
}

int main(int argc, char **argv)
    {
        ros::init(argc, argv, "slv_server");

        ros::NodeHandle node;
        
        ros::ServiceServer srv = node.advertiseService("slv_server_srv",solve);
        ROS_INFO("Waiting for input data");

        sub = node.subscribe("/topic_name",10,resCallback);
        pub = node.advertise<std_msgs::Float32MultiArray>("/topic_name",10);
        
        ros::spin();
        return 0;

    }