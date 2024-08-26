// Generated by gencpp from file arm_info/kinemaricsRequest.msg
// DO NOT EDIT!


#ifndef ARM_INFO_MESSAGE_KINEMARICSREQUEST_H
#define ARM_INFO_MESSAGE_KINEMARICSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace arm_info
{
template <class ContainerAllocator>
struct kinemaricsRequest_
{
  typedef kinemaricsRequest_<ContainerAllocator> Type;

  kinemaricsRequest_()
    : tar_x(0.0)
    , tar_y(0.0)
    , tar_z(0.0)
    , Roll(0.0)
    , Pitch(0.0)
    , Yaw(0.0)
    , cur_joint1(0.0)
    , cur_joint2(0.0)
    , cur_joint3(0.0)
    , cur_joint4(0.0)
    , cur_joint5(0.0)
    , cur_joint6(0.0)
    , kin_name()  {
    }
  kinemaricsRequest_(const ContainerAllocator& _alloc)
    : tar_x(0.0)
    , tar_y(0.0)
    , tar_z(0.0)
    , Roll(0.0)
    , Pitch(0.0)
    , Yaw(0.0)
    , cur_joint1(0.0)
    , cur_joint2(0.0)
    , cur_joint3(0.0)
    , cur_joint4(0.0)
    , cur_joint5(0.0)
    , cur_joint6(0.0)
    , kin_name(_alloc)  {
  (void)_alloc;
    }



   typedef double _tar_x_type;
  _tar_x_type tar_x;

   typedef double _tar_y_type;
  _tar_y_type tar_y;

   typedef double _tar_z_type;
  _tar_z_type tar_z;

   typedef double _Roll_type;
  _Roll_type Roll;

   typedef double _Pitch_type;
  _Pitch_type Pitch;

   typedef double _Yaw_type;
  _Yaw_type Yaw;

   typedef double _cur_joint1_type;
  _cur_joint1_type cur_joint1;

   typedef double _cur_joint2_type;
  _cur_joint2_type cur_joint2;

   typedef double _cur_joint3_type;
  _cur_joint3_type cur_joint3;

   typedef double _cur_joint4_type;
  _cur_joint4_type cur_joint4;

   typedef double _cur_joint5_type;
  _cur_joint5_type cur_joint5;

   typedef double _cur_joint6_type;
  _cur_joint6_type cur_joint6;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _kin_name_type;
  _kin_name_type kin_name;





  typedef boost::shared_ptr< ::arm_info::kinemaricsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_info::kinemaricsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct kinemaricsRequest_

typedef ::arm_info::kinemaricsRequest_<std::allocator<void> > kinemaricsRequest;

typedef boost::shared_ptr< ::arm_info::kinemaricsRequest > kinemaricsRequestPtr;
typedef boost::shared_ptr< ::arm_info::kinemaricsRequest const> kinemaricsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::arm_info::kinemaricsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::arm_info::kinemaricsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::arm_info::kinemaricsRequest_<ContainerAllocator1> & lhs, const ::arm_info::kinemaricsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.tar_x == rhs.tar_x &&
    lhs.tar_y == rhs.tar_y &&
    lhs.tar_z == rhs.tar_z &&
    lhs.Roll == rhs.Roll &&
    lhs.Pitch == rhs.Pitch &&
    lhs.Yaw == rhs.Yaw &&
    lhs.cur_joint1 == rhs.cur_joint1 &&
    lhs.cur_joint2 == rhs.cur_joint2 &&
    lhs.cur_joint3 == rhs.cur_joint3 &&
    lhs.cur_joint4 == rhs.cur_joint4 &&
    lhs.cur_joint5 == rhs.cur_joint5 &&
    lhs.cur_joint6 == rhs.cur_joint6 &&
    lhs.kin_name == rhs.kin_name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::arm_info::kinemaricsRequest_<ContainerAllocator1> & lhs, const ::arm_info::kinemaricsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace arm_info

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::arm_info::kinemaricsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::arm_info::kinemaricsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::arm_info::kinemaricsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "11d857e8542c0047afc9d3b13061446f";
  }

  static const char* value(const ::arm_info::kinemaricsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x11d857e8542c0047ULL;
  static const uint64_t static_value2 = 0xafc9d3b13061446fULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "arm_info/kinemaricsRequest";
  }

  static const char* value(const ::arm_info::kinemaricsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# request\n"
"float64 tar_x\n"
"float64 tar_y\n"
"float64 tar_z\n"
"float64 Roll\n"
"float64 Pitch\n"
"float64 Yaw\n"
"float64 cur_joint1\n"
"float64 cur_joint2\n"
"float64 cur_joint3\n"
"float64 cur_joint4\n"
"float64 cur_joint5\n"
"float64 cur_joint6\n"
"string  kin_name\n"
;
  }

  static const char* value(const ::arm_info::kinemaricsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tar_x);
      stream.next(m.tar_y);
      stream.next(m.tar_z);
      stream.next(m.Roll);
      stream.next(m.Pitch);
      stream.next(m.Yaw);
      stream.next(m.cur_joint1);
      stream.next(m.cur_joint2);
      stream.next(m.cur_joint3);
      stream.next(m.cur_joint4);
      stream.next(m.cur_joint5);
      stream.next(m.cur_joint6);
      stream.next(m.kin_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct kinemaricsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_info::kinemaricsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::arm_info::kinemaricsRequest_<ContainerAllocator>& v)
  {
    s << indent << "tar_x: ";
    Printer<double>::stream(s, indent + "  ", v.tar_x);
    s << indent << "tar_y: ";
    Printer<double>::stream(s, indent + "  ", v.tar_y);
    s << indent << "tar_z: ";
    Printer<double>::stream(s, indent + "  ", v.tar_z);
    s << indent << "Roll: ";
    Printer<double>::stream(s, indent + "  ", v.Roll);
    s << indent << "Pitch: ";
    Printer<double>::stream(s, indent + "  ", v.Pitch);
    s << indent << "Yaw: ";
    Printer<double>::stream(s, indent + "  ", v.Yaw);
    s << indent << "cur_joint1: ";
    Printer<double>::stream(s, indent + "  ", v.cur_joint1);
    s << indent << "cur_joint2: ";
    Printer<double>::stream(s, indent + "  ", v.cur_joint2);
    s << indent << "cur_joint3: ";
    Printer<double>::stream(s, indent + "  ", v.cur_joint3);
    s << indent << "cur_joint4: ";
    Printer<double>::stream(s, indent + "  ", v.cur_joint4);
    s << indent << "cur_joint5: ";
    Printer<double>::stream(s, indent + "  ", v.cur_joint5);
    s << indent << "cur_joint6: ";
    Printer<double>::stream(s, indent + "  ", v.cur_joint6);
    s << indent << "kin_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.kin_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARM_INFO_MESSAGE_KINEMARICSREQUEST_H
