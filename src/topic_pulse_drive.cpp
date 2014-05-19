#include <topic_tools/shape_shifter.h>
#include <topic_tools/parse.h>
#include <std_msgs/Float32.h>

using std::string;
using std::vector;
using namespace topic_tools;

ros::Publisher g_pub;
ros::Subscriber* g_sub;
ros::Subscriber t_sub;
ros::NodeHandle *g_node = NULL;
string g_input_topic;
string g_output_topic;
ros::TransportHints g_th;

bool publish_signal_receive_mode = false;
bool g_advertised = false;

void conn_cb(const ros::SingleSubscriberPublisher&)
{
}

void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event)
{
  if(publish_signal_receive_mode){
    publish_signal_receive_mode = false;

    boost::shared_ptr<ShapeShifter const> const &msg = msg_event.getConstMessage();
    boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();
    if(!g_advertised){
      bool latch = false;
      g_pub = msg->advertise(*g_node, g_output_topic, 10, latch, conn_cb);
      g_advertised = true;
    }
    ROS_INFO("in_cb PUBLISHED");
    g_pub.publish(msg);
  }
}

void service_in(std_msgs::Float32 signal)
{
  publish_signal_receive_mode = true;
}

void subscribe()
{
  g_sub = new ros::Subscriber(g_node->subscribe(g_input_topic, 10, &in_cb, g_th));
  t_sub = g_node->subscribe(std::string("debug_signal_float"), 1000, service_in);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("\nusage: topic_pulse_drive IN_TOPIC [OUT_TOPIC]\n\n");
    return 1;
  }

  std::string topic_name;
  if(!getBaseName(string(argv[1]), topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_pulse_drive"),
            ros::init_options::AnonymousName);
  if (argc == 2)
    g_output_topic = string(argv[1]) + string("_pulse_out");
  else // argc == 3
    g_output_topic = string(argv[2]);
  g_input_topic = string(argv[1]);

  ros::NodeHandle n;
  g_node = &n;

  subscribe();
  ros::spin();
  return 0;
}
