#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <stdio.h>

namespace nodelet_intraprocess_subscribe
{

	class Subscribe : public nodelet::Nodelet
	{
	public:
	  Subscribe()
	  {}

	private:
	  virtual void onInit()
	  {
		ros::NodeHandle& private_nh = getPrivateNodeHandle();
		m_sub= private_nh.subscribe<std_msgs::String>("intraprocess_topic", 10, boost::bind(&Subscribe::subscribeCallback, this, _1));
	  }

	  void subscribeCallback(const std_msgs::String::ConstPtr& input)
	  {
		ROS_INFO("(Subscriber)data:%s, address:%p, pid:%d", input->data.c_str(), &(input->data), getpid());
	  }

	  ros::Subscriber m_sub;
	};

	PLUGINLIB_EXPORT_CLASS(nodelet_intraprocess_subscribe::Subscribe, nodelet::Nodelet)
}
