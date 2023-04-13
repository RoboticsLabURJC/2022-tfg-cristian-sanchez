#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>

class RosButton : public QWidget
{

public:
    RosButton(const std::string& msg) : nh()
    {
        initUI();
        pub = nh.advertise<std_msgs::String>("button_topic", 10);
        message.data = msg;

        // Start the subscriber in a separate thread
        std::thread subThread(&RosButton::startSub, this);
        subThread.detach();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    std_msgs::String message;

    QPushButton* button;

    void initUI()
    {
        setWindowTitle("ROS Noetic Button Example");
        setGeometry(100, 100, 200, 150);
        button = new QPushButton("Click me!", this);
        button->setToolTip("This is a ROS Noetic button!");
        connect(button, &QPushButton::clicked, this, &RosButton::buttonClicked);
        show();
    }

    void buttonClicked()
    {
        pub.publish(message);
    }

    void startSub()
    {
        ros::NodeHandle sub_nh;
        ros::Subscriber sub = sub_nh.subscribe("button_topic", 10, &RosButton::buttonCallback, this);

        // Start the spinner in a separate thread
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::waitForShutdown();
    }

    void buttonCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Received message: %s", msg->data.c_str());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "button_node");
    QApplication app(argc, argv);
    RosButton ex("Hello World!");
    return app.exec();
}