#include "receiver_node.h"


ReceiverNode::ReceiverNode()
{
    n = ros::NodeHandle("~");

    // READ PARAMS
    //readParams();

    // Initialize all topics
    pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ReceiverNode::positionCallback, this);
    velocity_sub_ = n.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, &ReceiverNode::velocityCallback, this);
    state_sub_ = n.subscribe<mavros_msgs::State>("/mavros/state", 1, &ReceiverNode::stateCallback, this);

}


ReceiverNode::~ReceiverNode() {}


void ReceiverNode::printPose(geometry_msgs::PoseStamped pos)
{
    printf("Pose\n");
    printf("Position: x=%.3f y=%.3f z=%.3f\n", pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);
    printf("Orientation: x=%.3f y=%.3f z=%.3f w=%.3f\n", pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z, pos.pose.orientation.w);
    printf("-------------------------------------\n");
}


void ReceiverNode::printVelocity(geometry_msgs::TwistStamped vel) 
{
    printf("Current Velocity\n");
    printf("Linear: x=%.3f y=%.3f z=%.3f\n", vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z);
    printf("-------------------------------------\n");
}


void ReceiverNode::positionCallback(const geometry_msgs::PoseStamped msg) {

   printPose(msg);
}


void ReceiverNode::velocityCallback(const geometry_msgs::TwistStamped msg) {

   //printVelocity(msg);
}


void ReceiverNode::stateCallback(const mavros_msgs::State msg) {

    currently_armed_ = msg.armed;

    if(!currently_armed_) {
      if(msg.armed) {
        printf("Vehicle Armed\n");
        currently_armed_ = true;
      }
    } else {
      if(!msg.armed) {
        printf("Vehicle Disarmed\n");
        currently_armed_ = false;
      }
    }

    // if(msg.mode == "OFFBOARD"){

    //   if(currently_offboard_ == false){
    //     printf("Offboard Mode Activated\n");
    //   }
    //   currently_offboard_ = true;
      
    // }else{

    //   if(currently_offboard_ == true){
    //     printf("Offboard Mode Deactivated\n");
    //   }
    //   currently_offboard_ = false;

    // }

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "receiver_node");

  ReceiverNode receiver_node;

  ros::spin();

  return 0;
}
