
#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
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
};

class DoubleNode: BaseNode {
private:
    std::string received_val = "";
    std::stringstream ss;
    ros::NodeHandle nh_;
    void sub_callback(const std_msgs::String::ConstPtr& msg) {
        received_val = msg->data.c_str();
        ROS_INFO("I heard: [%s]", received_val);
    }
    public:
        DoubleNode(std::string name) {
            ROS_INFO("double node constructor");
            pub_topic = "double_to_int";
            sub_topic = "str_to_double";
            node_name = name;
            ss << std::setprecision(30) << M_PI;
            publisher = nh_.advertise<std_msgs::Float64>(pub_topic, 1000);
            subscriber = nh_.subscribe<std_msgs::String>(sub_topic, 1000, boost::bind(&DoubleNode::sub_callback, this, _1));
        }
        void pub() {}
        void pub(int curr_pos) {
            long double value;

            std::stringstream ss_double (ss.str().substr(0, curr_pos));
            ss_double >> value;

            std_msgs::Float64 msg;
            msg.data = value;

            ROS_INFO("Pub: %lf", msg.data);

            publisher.publish(msg);
        }
        std::string get_received_val() {
            return received_val;
        }
};

class IntNode: BaseNode {
private:
    double received_val = 0.0;
    ros::NodeHandle nh_;
    void sub_callback(const std_msgs::Float64::ConstPtr& msg) {
        received_val = msg->data;
        ROS_INFO("I heard: [%lf]", received_val);
    }
public:
    IntNode(std::string name) {
        pub_topic = "int_to_float";
        sub_topic = "double_to_int";
        node_name = name;

        publisher = nh_.advertise<std_msgs::Int32>(pub_topic, 1000);
        subscriber = nh_.subscribe<std_msgs::Float64>(sub_topic, 1000, boost::bind(&IntNode::sub_callback, this, _1));
    }
    void pub() {
        int val = (int) received_val;
        std_msgs::Int32 msg;
        msg.data = val;

        ROS_INFO("Pub: %d", msg.data);

        publisher.publish(msg);
    }
    double get_received_val() {
        return received_val;
    }
};

class FloatNode: BaseNode {
private:
    int received_val = 0;
    ros::NodeHandle nh_;
    void sub_callback(const std_msgs::Int32::ConstPtr& msg) {
        received_val = msg->data;
        ROS_INFO("I heard: [%d]", received_val);
    }
public:
    FloatNode(std::string name) {
        pub_topic = "float_to_str";
        sub_topic = "int_to_float";
        node_name = name;

        publisher = nh_.advertise<std_msgs::Float32>(pub_topic, 1000);
        subscriber = nh_.subscribe<std_msgs::Int32>(sub_topic, 1000, boost::bind(&FloatNode::sub_callback, this, _1));
    }
    void pub() {
        float val = (float) received_val;
        std_msgs::Float32 msg;
        msg.data = val;

        ROS_INFO("Pub: %f", msg.data);

        publisher.publish(msg);
    }
    int get_received_val() {
        return received_val;
    }
};

class StrNode: BaseNode {
private:
    float received_val;
    ros::NodeHandle nh_;
    void sub_callback(const std_msgs::Float32::ConstPtr& msg) {
        received_val = msg->data;
        ROS_INFO("I heard: [%f]", received_val);
    }
public:
    StrNode(std::string name) {
        pub_topic = "str_to_double";
        sub_topic = "float_to_str";
        node_name = name;

        publisher = nh_.advertise<std_msgs::String>(pub_topic, 1000);
        subscriber = nh_.subscribe<std_msgs::Float32>(sub_topic, 1000, boost::bind(&StrNode::sub_callback, this, _1));
    }
    void pub() {
        std::stringstream ss;
        ss << received_val;
        std_msgs::String msg;
        msg.data = ss.str();

        ROS_INFO("Pub: %s", msg.data.c_str());

        publisher.publish(msg);
    }
    float get_received_val() {
        return received_val;
    }
};


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ROS_INFO("%d, %s", argc, argv[1]);
    float ros_rate = 0.5;
    if (argc > 1) {
        std::string node_type = argv[1];
        if (node_type == "double") {
            ROS_INFO("%s", "yes double");
            ros::init(argc, argv, "double_node");
            DoubleNode double_node("double_node");
            ros::Rate loop_rate(ros_rate);
            int curr_pos = 3; // Get digits after dot.

            while (ros::ok())
            {
                double_node.pub(curr_pos);
                ++curr_pos;
                ros::spinOnce();
                loop_rate.sleep();
            }
        } else if (node_type == "int") {
            ros::init(argc, argv, "int_node");
            IntNode int_node("int_node");
            ros::Rate loop_rate(ros_rate);

            while (ros::ok) {
                ros::spinOnce();
                loop_rate.sleep();
                int_node.pub();
            }
        } else if (node_type == "float") {
            ros::init(argc, argv, "float_node");
            FloatNode float_node("float_node");
            ros::Rate loop_rate(ros_rate);

            while (ros::ok) {
                ros::spinOnce();
                loop_rate.sleep();
                float_node.pub();
            }
        } else if (node_type == "str") {
            ros::init(argc, argv, "str_node");
            StrNode str_node("str_node");
            ros::Rate loop_rate(ros_rate);

            while (ros::ok) {
                ros::spinOnce();
                loop_rate.sleep();
                str_node.pub();
            }
        }
    } else {
        printf("Please choose a valid node type. The following list is valid node types: double, int, float, str");
    }

    return 0;
}
