// Generated by gencpp from file leo_driving/Interrupt.msg
// DO NOT EDIT!


#ifndef LEO_DRIVING_MESSAGE_INTERRUPT_H
#define LEO_DRIVING_MESSAGE_INTERRUPT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace leo_driving
{
template <class ContainerAllocator>
struct Interrupt_
{
  typedef Interrupt_<ContainerAllocator> Type;

  Interrupt_()
    : enableEvent(0)
    , event_x1(0.0)
    , event_y1(0.0)
    , event_z1(0.0)
    , event_x2(0.0)
    , event_y2(0.0)
    , event_z2(0.0)
    , enableDocking(0)
    , numStation(0)
    , station_x(0.0)
    , station_y(0.0)
    , station_z(0.0)  {
    }
  Interrupt_(const ContainerAllocator& _alloc)
    : enableEvent(0)
    , event_x1(0.0)
    , event_y1(0.0)
    , event_z1(0.0)
    , event_x2(0.0)
    , event_y2(0.0)
    , event_z2(0.0)
    , enableDocking(0)
    , numStation(0)
    , station_x(0.0)
    , station_y(0.0)
    , station_z(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _enableEvent_type;
  _enableEvent_type enableEvent;

   typedef float _event_x1_type;
  _event_x1_type event_x1;

   typedef float _event_y1_type;
  _event_y1_type event_y1;

   typedef float _event_z1_type;
  _event_z1_type event_z1;

   typedef float _event_x2_type;
  _event_x2_type event_x2;

   typedef float _event_y2_type;
  _event_y2_type event_y2;

   typedef float _event_z2_type;
  _event_z2_type event_z2;

   typedef int32_t _enableDocking_type;
  _enableDocking_type enableDocking;

   typedef int32_t _numStation_type;
  _numStation_type numStation;

   typedef float _station_x_type;
  _station_x_type station_x;

   typedef float _station_y_type;
  _station_y_type station_y;

   typedef float _station_z_type;
  _station_z_type station_z;





  typedef boost::shared_ptr< ::leo_driving::Interrupt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::leo_driving::Interrupt_<ContainerAllocator> const> ConstPtr;

}; // struct Interrupt_

typedef ::leo_driving::Interrupt_<std::allocator<void> > Interrupt;

typedef boost::shared_ptr< ::leo_driving::Interrupt > InterruptPtr;
typedef boost::shared_ptr< ::leo_driving::Interrupt const> InterruptConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::leo_driving::Interrupt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::leo_driving::Interrupt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::leo_driving::Interrupt_<ContainerAllocator1> & lhs, const ::leo_driving::Interrupt_<ContainerAllocator2> & rhs)
{
  return lhs.enableEvent == rhs.enableEvent &&
    lhs.event_x1 == rhs.event_x1 &&
    lhs.event_y1 == rhs.event_y1 &&
    lhs.event_z1 == rhs.event_z1 &&
    lhs.event_x2 == rhs.event_x2 &&
    lhs.event_y2 == rhs.event_y2 &&
    lhs.event_z2 == rhs.event_z2 &&
    lhs.enableDocking == rhs.enableDocking &&
    lhs.numStation == rhs.numStation &&
    lhs.station_x == rhs.station_x &&
    lhs.station_y == rhs.station_y &&
    lhs.station_z == rhs.station_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::leo_driving::Interrupt_<ContainerAllocator1> & lhs, const ::leo_driving::Interrupt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace leo_driving

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::leo_driving::Interrupt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::leo_driving::Interrupt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leo_driving::Interrupt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leo_driving::Interrupt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leo_driving::Interrupt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leo_driving::Interrupt_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::leo_driving::Interrupt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4c83ca8933f6b8086033f46e3805c2b0";
  }

  static const char* value(const ::leo_driving::Interrupt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4c83ca8933f6b808ULL;
  static const uint64_t static_value2 = 0x6033f46e3805c2b0ULL;
};

template<class ContainerAllocator>
struct DataType< ::leo_driving::Interrupt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "leo_driving/Interrupt";
  }

  static const char* value(const ::leo_driving::Interrupt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::leo_driving::Interrupt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 enableEvent\n"
"float32 event_x1\n"
"float32 event_y1\n"
"float32 event_z1\n"
"float32 event_x2\n"
"float32 event_y2\n"
"float32 event_z2\n"
"int32 enableDocking\n"
"int32 numStation\n"
"float32 station_x\n"
"float32 station_y\n"
"float32 station_z\n"
;
  }

  static const char* value(const ::leo_driving::Interrupt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::leo_driving::Interrupt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.enableEvent);
      stream.next(m.event_x1);
      stream.next(m.event_y1);
      stream.next(m.event_z1);
      stream.next(m.event_x2);
      stream.next(m.event_y2);
      stream.next(m.event_z2);
      stream.next(m.enableDocking);
      stream.next(m.numStation);
      stream.next(m.station_x);
      stream.next(m.station_y);
      stream.next(m.station_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Interrupt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::leo_driving::Interrupt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::leo_driving::Interrupt_<ContainerAllocator>& v)
  {
    s << indent << "enableEvent: ";
    Printer<int32_t>::stream(s, indent + "  ", v.enableEvent);
    s << indent << "event_x1: ";
    Printer<float>::stream(s, indent + "  ", v.event_x1);
    s << indent << "event_y1: ";
    Printer<float>::stream(s, indent + "  ", v.event_y1);
    s << indent << "event_z1: ";
    Printer<float>::stream(s, indent + "  ", v.event_z1);
    s << indent << "event_x2: ";
    Printer<float>::stream(s, indent + "  ", v.event_x2);
    s << indent << "event_y2: ";
    Printer<float>::stream(s, indent + "  ", v.event_y2);
    s << indent << "event_z2: ";
    Printer<float>::stream(s, indent + "  ", v.event_z2);
    s << indent << "enableDocking: ";
    Printer<int32_t>::stream(s, indent + "  ", v.enableDocking);
    s << indent << "numStation: ";
    Printer<int32_t>::stream(s, indent + "  ", v.numStation);
    s << indent << "station_x: ";
    Printer<float>::stream(s, indent + "  ", v.station_x);
    s << indent << "station_y: ";
    Printer<float>::stream(s, indent + "  ", v.station_y);
    s << indent << "station_z: ";
    Printer<float>::stream(s, indent + "  ", v.station_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEO_DRIVING_MESSAGE_INTERRUPT_H
