// Generated by gencpp from file lidar_localization/saveMapResponse.msg
// DO NOT EDIT!


#ifndef LIDAR_LOCALIZATION_MESSAGE_SAVEMAPRESPONSE_H
#define LIDAR_LOCALIZATION_MESSAGE_SAVEMAPRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_localization
{
template <class ContainerAllocator>
struct saveMapResponse_
{
  typedef saveMapResponse_<ContainerAllocator> Type;

  saveMapResponse_()
    : succeed(false)  {
    }
  saveMapResponse_(const ContainerAllocator& _alloc)
    : succeed(false)  {
  (void)_alloc;
    }



   typedef uint8_t _succeed_type;
  _succeed_type succeed;





  typedef boost::shared_ptr< ::lidar_localization::saveMapResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_localization::saveMapResponse_<ContainerAllocator> const> ConstPtr;

}; // struct saveMapResponse_

typedef ::lidar_localization::saveMapResponse_<std::allocator<void> > saveMapResponse;

typedef boost::shared_ptr< ::lidar_localization::saveMapResponse > saveMapResponsePtr;
typedef boost::shared_ptr< ::lidar_localization::saveMapResponse const> saveMapResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_localization::saveMapResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_localization::saveMapResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_localization::saveMapResponse_<ContainerAllocator1> & lhs, const ::lidar_localization::saveMapResponse_<ContainerAllocator2> & rhs)
{
  return lhs.succeed == rhs.succeed;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_localization::saveMapResponse_<ContainerAllocator1> & lhs, const ::lidar_localization::saveMapResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_localization

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::saveMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::saveMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::saveMapResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d9c3b918a0afafe09791ef8d7853918";
  }

  static const char* value(const ::lidar_localization::saveMapResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d9c3b918a0afafeULL;
  static const uint64_t static_value2 = 0x09791ef8d7853918ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_localization/saveMapResponse";
  }

  static const char* value(const ::lidar_localization::saveMapResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool succeed\n"
;
  }

  static const char* value(const ::lidar_localization::saveMapResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.succeed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct saveMapResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_localization::saveMapResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_localization::saveMapResponse_<ContainerAllocator>& v)
  {
    s << indent << "succeed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.succeed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_LOCALIZATION_MESSAGE_SAVEMAPRESPONSE_H
