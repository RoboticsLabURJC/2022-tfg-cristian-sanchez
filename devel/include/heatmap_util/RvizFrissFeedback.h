// Generated by gencpp from file heatmap_util/RvizFrissFeedback.msg
// DO NOT EDIT!


#ifndef HEATMAP_UTIL_MESSAGE_RVIZFRISSFEEDBACK_H
#define HEATMAP_UTIL_MESSAGE_RVIZFRISSFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace heatmap_util
{
template <class ContainerAllocator>
struct RvizFrissFeedback_
{
  typedef RvizFrissFeedback_<ContainerAllocator> Type;

  RvizFrissFeedback_()
    : feedback(false)  {
    }
  RvizFrissFeedback_(const ContainerAllocator& _alloc)
    : feedback(false)  {
  (void)_alloc;
    }



   typedef uint8_t _feedback_type;
  _feedback_type feedback;





  typedef boost::shared_ptr< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct RvizFrissFeedback_

typedef ::heatmap_util::RvizFrissFeedback_<std::allocator<void> > RvizFrissFeedback;

typedef boost::shared_ptr< ::heatmap_util::RvizFrissFeedback > RvizFrissFeedbackPtr;
typedef boost::shared_ptr< ::heatmap_util::RvizFrissFeedback const> RvizFrissFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator1> & lhs, const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.feedback == rhs.feedback;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator1> & lhs, const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace heatmap_util

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f1f168a39479bedb24dba7a087426182";
  }

  static const char* value(const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf1f168a39479bedbULL;
  static const uint64_t static_value2 = 0x24dba7a087426182ULL;
};

template<class ContainerAllocator>
struct DataType< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "heatmap_util/RvizFrissFeedback";
  }

  static const char* value(const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"bool feedback\n"
;
  }

  static const char* value(const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RvizFrissFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::heatmap_util::RvizFrissFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::heatmap_util::RvizFrissFeedback_<ContainerAllocator>& v)
  {
    s << indent << "feedback: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HEATMAP_UTIL_MESSAGE_RVIZFRISSFEEDBACK_H
