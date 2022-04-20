// Generated by gencpp from file ur_msgs/SetSpeedSliderFractionRequest.msg
// DO NOT EDIT!


#ifndef UR_MSGS_MESSAGE_SETSPEEDSLIDERFRACTIONREQUEST_H
#define UR_MSGS_MESSAGE_SETSPEEDSLIDERFRACTIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ur_msgs
{
template <class ContainerAllocator>
struct SetSpeedSliderFractionRequest_
{
  typedef SetSpeedSliderFractionRequest_<ContainerAllocator> Type;

  SetSpeedSliderFractionRequest_()
    : speed_slider_fraction(0.0)  {
    }
  SetSpeedSliderFractionRequest_(const ContainerAllocator& _alloc)
    : speed_slider_fraction(0.0)  {
  (void)_alloc;
    }



   typedef double _speed_slider_fraction_type;
  _speed_slider_fraction_type speed_slider_fraction;





  typedef boost::shared_ptr< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetSpeedSliderFractionRequest_

typedef ::ur_msgs::SetSpeedSliderFractionRequest_<std::allocator<void> > SetSpeedSliderFractionRequest;

typedef boost::shared_ptr< ::ur_msgs::SetSpeedSliderFractionRequest > SetSpeedSliderFractionRequestPtr;
typedef boost::shared_ptr< ::ur_msgs::SetSpeedSliderFractionRequest const> SetSpeedSliderFractionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'ur_msgs': ['/home/robot/workspaces/ur5_controllers/src/universal_robot/ur_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "64134244ab4dfc72a3406fe06d580274";
  }

  static const char* value(const ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x64134244ab4dfc72ULL;
  static const uint64_t static_value2 = 0xa3406fe06d580274ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_msgs/SetSpeedSliderFractionRequest";
  }

  static const char* value(const ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
\n\
float64 speed_slider_fraction\n\
";
  }

  static const char* value(const ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.speed_slider_fraction);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetSpeedSliderFractionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_msgs::SetSpeedSliderFractionRequest_<ContainerAllocator>& v)
  {
    s << indent << "speed_slider_fraction: ";
    Printer<double>::stream(s, indent + "  ", v.speed_slider_fraction);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_MSGS_MESSAGE_SETSPEEDSLIDERFRACTIONREQUEST_H
