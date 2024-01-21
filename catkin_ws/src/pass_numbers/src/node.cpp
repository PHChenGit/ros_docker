
#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <string>
#include <sstream>
#include <cmath>


class BaseNode {
public:
    std::string pub_topic = "";
    std::string sub_topic = "";
    std::string node_name = "";
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    virtual void pub() = 0;
    void sub() {
        ros::spin();
    }
};

class DoubleNode: BaseNode {
    private:
        std::stringstream ss;
        void sub_callback(const std_msgs::String::ConstPtr& msg)
        {
            ROS_INFO("I heard: [%s]", msg->data.c_str());
        }
    public:
        DoubleNode(int argc, char **argv) {
            pub_topic = "double_to_int";
            sub_topic = "str_to_double";
            node_name = "double_node";
            ss << std::setprecision(20) << M_PI;

            ros::init(argc, argv, node_name);
            ros::NodeHandle n;
            publisher = n.advertise<std_msgs::Float64>(pub_topic, 1000);

            ros::Subscriber sub = n.subscribe<std_msgs::String>(sub_topic, 1000, boost::bind(&DoubleNode::sub_callback, this, _1));
        }
        std::string getPiDigits(std::stringstream& ss, int n) {
            std::string result;
            char ch;
            int count = 0;

            // 重置 stringstream 的讀取位置
            ss.seekg(0);
            // 忽略小數點之前的數字和小數點本身
            ss.ignore(2);

            // 從小數點之後開始讀取
            while (count < n && ss.get(ch)) {
                result += ch;
                count++;
            }

            return "3." + result;
        }
        void sub() {
            ros::spin();
        }
        void pub() {
            ROS_INFO("%s", "double start publishing");
            ros::Rate loop_rate(10);
            int rounds = 0;
            int curr_pos = 1; // Get digits after dot.

            while (ros::ok())
            {
                double value;
                if (curr_pos+2 >= ss.str().length()) {
//                  ss >> value;
                    value = 3.141593;
//                    std::cout << "\n\nEnd!\n";
//                    break;
                } else {
                    std::string piDigits = getPiDigits(ss, curr_pos);
                    std::stringstream msg_ss (piDigits);
                    msg_ss >> value;
                    ++curr_pos;
                }

                std_msgs::Float64 msg;
                msg.data = value;

                ROS_INFO("Pub: %f", msg.data);

                publisher.publish(msg);

                ros::spinOnce();
                loop_rate.sleep();
                ++rounds;
            }
        }
};

class IntNode: BaseNode {
private:
    double received_val;
public:
    IntNode(int argc, char **argv) {
        pub_topic = "int_to_float";
        sub_topic = "double_to_int";
        node_name = "int_node";

        ros::init(argc, argv, node_name);
        ros::NodeHandle n;
        publisher = n.advertise<std_msgs::Int32>(pub_topic, 1000);

        ros::Subscriber sub = n.subscribe<std_msgs::Float64>(sub_topic, 1000, boost::bind(&IntNode::sub_callback, this, _1));
    }
    void sub() {
        ros::spin();
    }
    void pub() {
        ros::Rate loop_rate(10);
        int rounds = 0;

        while (ros::ok())
        {
            std_msgs::Int32 msg;
            msg.data = (int) received_val;

            ROS_INFO("Pub: %d", msg.data);

            publisher.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
            ++rounds;
        }
    }
    void sub_callback(const std_msgs::Float64::ConstPtr& msg) {
        received_val = msg->data;
        ROS_INFO("I heard: [%f]", received_val);
    }
};



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ROS_INFO("%d, %s", argc, argv[1]);
    if (argc > 1) {
        std::string node_type = argv[1];
        if (node_type == "double") {
            ROS_INFO("%s", "yes double");
            DoubleNode double_node(argc, argv);
            double_node.pub();
            double_node.sub();
        } else if (node_type == "int") {
            IntNode int_node(argc, argv);
            int_node.pub();
            int_node.sub();
        }
    } else {
        printf("Please choose a valid node type. The following list is valid node types: double, int, float, str");
    }

    return 0;
}
