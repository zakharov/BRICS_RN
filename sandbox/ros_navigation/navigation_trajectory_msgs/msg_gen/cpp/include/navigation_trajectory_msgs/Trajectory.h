/* Auto-generated by genmsg_cpp for file /home/alexey/workspace/BRICS_RN/sandbox/ros_navigation/navigation_trajectory_msgs/msg/Trajectory.msg */
#ifndef NAVIGATION_TRAJECTORY_MSGS_MESSAGE_TRAJECTORY_H
#define NAVIGATION_TRAJECTORY_MSGS_MESSAGE_TRAJECTORY_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"

namespace navigation_trajectory_msgs
{
template <class ContainerAllocator>
struct Trajectory_ {
  typedef Trajectory_<ContainerAllocator> Type;

  Trajectory_()
  : header()
  , trajectory()
  {
  }

  Trajectory_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , trajectory(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other >  _trajectory_type;
  std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other >  trajectory;


  ROS_DEPRECATED uint32_t get_trajectory_size() const { return (uint32_t)trajectory.size(); }
  ROS_DEPRECATED void set_trajectory_size(uint32_t size) { trajectory.resize((size_t)size); }
  ROS_DEPRECATED void get_trajectory_vec(std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other > & vec) const { vec = this->trajectory; }
  ROS_DEPRECATED void set_trajectory_vec(const std::vector< ::nav_msgs::Odometry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nav_msgs::Odometry_<ContainerAllocator> >::other > & vec) { this->trajectory = vec; }
private:
  static const char* __s_getDataType_() { return "navigation_trajectory_msgs/Trajectory"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "e6e1f5641a64a734bc86ffc4b38d8364"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
nav_msgs/Odometry[] trajectory\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, trajectory);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, trajectory);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(trajectory);
    return size;
  }

  typedef boost::shared_ptr< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Trajectory
typedef  ::navigation_trajectory_msgs::Trajectory_<std::allocator<void> > Trajectory;

typedef boost::shared_ptr< ::navigation_trajectory_msgs::Trajectory> TrajectoryPtr;
typedef boost::shared_ptr< ::navigation_trajectory_msgs::Trajectory const> TrajectoryConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace navigation_trajectory_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e6e1f5641a64a734bc86ffc4b38d8364";
  }

  static const char* value(const  ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe6e1f5641a64a734ULL;
  static const uint64_t static_value2 = 0xbc86ffc4b38d8364ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "navigation_trajectory_msgs/Trajectory";
  }

  static const char* value(const  ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
nav_msgs/Odometry[] trajectory\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.trajectory);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Trajectory_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::navigation_trajectory_msgs::Trajectory_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "trajectory[]" << std::endl;
    for (size_t i = 0; i < v.trajectory.size(); ++i)
    {
      s << indent << "  trajectory[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::nav_msgs::Odometry_<ContainerAllocator> >::stream(s, indent + "    ", v.trajectory[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_TRAJECTORY_MSGS_MESSAGE_TRAJECTORY_H

