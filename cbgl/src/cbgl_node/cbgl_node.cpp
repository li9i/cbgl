#include <cbgl_node/cbgl.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cbgl_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  CBGL cbgl(nh, nh_private);
  ros::spin();
  return 0;
}
