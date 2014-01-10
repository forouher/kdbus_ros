#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <boost/interprocess/detail/config_begin.hpp>
#include <boost/interprocess/detail/workaround.hpp>
#include <ros/boost_container.h>

/*
template<class T>
class my_algorithm
{
    void* next_free;


   /// @cond
   private:

   //Self type
   typedef my_algorithm<T>   self_t;

   //Pointer to void
   typedef void*  aux_pointer_t;

   //Typedef to const void pointer
   typedef typename
      boost::pointer_to_other
         <aux_pointer_t, const void>::type   cvoid_ptr;

   //Pointer to the allocator
//   typedef typename boost::pointer_to_other
//      <cvoid_ptr>::type     alloc_ptr_t;

   //Not assignable from related allocator
   template<class T2>
   my_algorithm& operator=(const my_algorithm<T2>&);

   //Not assignable from other allocator
   my_algorithm& operator=(const my_algorithm&);

   //Pointer to the allocator
//   alloc_ptr_t mp_mngr;
   /// @endcond


   public:

    typedef void * void_pointer;
    // typedef offset_ptr<void> void_pointer; // ???

   typedef T                                    value_type;
   typedef typename boost::pointer_to_other
      <cvoid_ptr, T>::type                      pointer;
   typedef typename boost::
      pointer_to_other<pointer, const T>::type  const_pointer;
   typedef typename detail::add_reference
                     <value_type>::type         reference;
   typedef typename detail::add_reference
                     <const value_type>::type   const_reference;
   typedef std::size_t                          size_type;
   typedef std::ptrdiff_t                       difference_type;

   //!Obtains an allocator that allocates
   //!objects of type T2
   template<class T2>
   struct rebind
   {
      typedef my_algorithm<T2>     other;
   };


   //!The pointer type to be used by the rest of Interprocess framework
//   typedef implementation_defined   void_pointer;

   //!Constructor. "size" is the total size of the managed memory segment,
   //!"extra_hdr_bytes" indicates the extra bytes after the sizeof(my_algorithm)
   //!that the allocator should not use at all.
   my_algorithm (std::size_t size, std::size_t extra_hdr_bytes) { next_free = this; };
    // extra_hdr_bytes --> sizeof(std_msgs::String_<...>)

   //!Obtains the minimum size needed by the algorithm
   static std::size_t get_min_size (std::size_t extra_hdr_bytes) { return 0; };

   //!Allocates bytes, returns 0 if there is not more memory
   void* allocate (std::size_t nbytes) { return NULL; };

   //!Deallocates previously allocated bytes
   void  deallocate (void *adr) { };

   //!Returns the size of the memory segment
   std::size_t get_size()  const {  return 0; };

   //!Increases managed memory in extra_size bytes more
   void grow(std::size_t extra_size) {

   }
};
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "Hi there!";

    // TODO: copy msg into a shared memory location.
    ros::malloc_allocator<char> alloc3;
//    std_msgs::String_<ros::malloc_allocator<void> > msg2(alloc3);
//    msg2.data = msg.data;

    ros::messages::types::basic_string<char, std::char_traits<char> > u;
    ros::messages::types::basic_string<char, std::char_traits<char>, ros::malloc_allocator<char> > s(alloc3);
    ros::messages::types::basic_string<char, std::char_traits<char>, ros::malloc_allocator<char> > t(alloc3);
    t = u;
//    u = t;
//    s = t;

//    ROS_INFO("foo: %s", msg2.data.c_str());
//    chatter_pub.publish(msg2);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
