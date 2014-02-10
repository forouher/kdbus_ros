///////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright Ion Gaztanaga 2005-2012. Distributed under the Boost
// Software License, Version 1.0. (See accompanying file
// LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/interprocess for documentation.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ROS_ALLOCATOR_H
#define ROS_ALLOCATOR_H

#include <boost/intrusive/pointer_traits.hpp>

#include <boost/interprocess/interprocess_fwd.hpp>
#include <boost/interprocess/containers/allocation_type.hpp>
#include <boost/container/detail/multiallocation_chain.hpp>
#include <boost/interprocess/allocators/detail/allocator_common.hpp>
#include <boost/interprocess/detail/utilities.hpp>
#include <boost/interprocess/containers/version_type.hpp>
#include <boost/interprocess/exceptions.hpp>
#include <boost/assert.hpp>
#include <boost/utility/addressof.hpp>
#include <boost/interprocess/detail/type_traits.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>

#include <memory>
#include <new>
#include <algorithm>
#include <cstddef>
#include <stdexcept>

//!\file
//!Describes an allocator that allocates portions of fixed size
//!memory buffer (shared memory, mapped file...)

namespace boost {
namespace interprocess {


//!An STL compatible allocator that uses a segment manager as
//!memory source. The internal pointer type will of the same type (raw, smart) as
//!"typename SegmentManager::void_pointer" type. This allows
//!placing the allocator in shared memory, memory mapped-files, etc...
template<class T, class SegmentManager>
class ros_allocator
{
   public:
   //Segment manager
   typedef SegmentManager                                segment_manager;
   typedef typename SegmentManager::void_pointer         void_pointer;

   /// @cond
   private:

   //Self type
   typedef ros_allocator<T, SegmentManager>   self_t;

   //Pointer to void
   typedef typename segment_manager::void_pointer  aux_pointer_t;

   //Typedef to const void pointer
   typedef typename boost::intrusive::
      pointer_traits<aux_pointer_t>::template
         rebind_pointer<const void>::type          cvoid_ptr;

   //Pointer to the allocator
   typedef typename boost::intrusive::
      pointer_traits<cvoid_ptr>::template
         rebind_pointer<segment_manager>::type          alloc_ptr_t;

   //Not assignable from related allocator
   template<class T2, class SegmentManager2>
   ros_allocator& operator=(const ros_allocator<T2, SegmentManager2>&);

   //Not assignable from other allocator
   ros_allocator& operator=(const ros_allocator&);

   //Pointer to the allocator
   alloc_ptr_t mp_mngr;
   /// @endcond

   public:
   typedef T                                    value_type;
   typedef typename boost::intrusive::
      pointer_traits<cvoid_ptr>::template
         rebind_pointer<T>::type                pointer;
   typedef typename boost::intrusive::
      pointer_traits<pointer>::template
         rebind_pointer<const T>::type          const_pointer;
   typedef typename ipcdetail::add_reference
                     <value_type>::type         reference;
   typedef typename ipcdetail::add_reference
                     <const value_type>::type   const_reference;
   typedef typename segment_manager::size_type               size_type;
   typedef typename segment_manager::difference_type         difference_type;

//   typedef boost::interprocess::version_type<ros_allocator, 2>   version;

   /// @cond

   //Experimental. Don't use.
   typedef boost::container::container_detail::transform_multiallocation_chain
      <typename SegmentManager::multiallocation_chain, T>multiallocation_chain;
   /// @endcond

   //!Obtains an allocator that allocates
   //!objects of type T2
   template<class T2>
   struct rebind
   {
      typedef ros_allocator<T2, SegmentManager>     other;
   };

   //!Returns the segment manager.
   //!Never throws
   segment_manager* get_segment_manager()const
   {  return ipcdetail::to_raw_pointer(mp_mngr);   }

   //!Constructor from the segment manager.
   //!Never throws
   ros_allocator(segment_manager *segment_mngr)
      : mp_mngr(segment_mngr) { }

   ros_allocator()
      : mp_mngr(NULL) { }

   //!Constructor from other allocator.
   //!Never throws
   ros_allocator(const ros_allocator &other)
      : mp_mngr(other.get_segment_manager()){ }

   //!Constructor from related allocator.
   //!Never throws
   template<class T2>
   ros_allocator(const ros_allocator<T2, SegmentManager> &other)
      : mp_mngr(other.get_segment_manager()){}

   //!Allocates memory for an array of count elements.
   //!Throws boost::interprocess::bad_alloc if there is no enough memory
   pointer allocate(size_type count, cvoid_ptr hint = 0)
   {
      (void)hint;
      if(size_overflows<sizeof(T)>(count)){
         throw bad_alloc();
      }
      if (mp_mngr)
        return pointer(static_cast<value_type*>(mp_mngr->allocate(count*sizeof(T))));
      else{
        return pointer(std::allocator<T>().allocate(count, boost::addressof(hint)));
      }
   }

   //!Deallocates memory previously allocated.
   //!Never throws
   void deallocate(const pointer &ptr, size_type size)
   { if (mp_mngr)
	 mp_mngr->deallocate((void*)ipcdetail::to_raw_pointer(ptr));
     else
        std::allocator<T>().deallocate(ipcdetail::to_raw_pointer(ptr),size);
}

   //!Returns the number of elements that could be allocated.
   //!Never throws
   size_type max_size() const
   {  if (mp_mngr)
	return mp_mngr->get_size()/sizeof(T);
      else
	return std::allocator<T>().max_size();
    }

   //!Swap segment manager. Does not throw. If each allocator is placed in
   //!different memory segments, the result is undefined.
   friend void swap(self_t &alloc1, self_t &alloc2)
   {  ipcdetail::do_swap(alloc1.mp_mngr, alloc2.mp_mngr);   }

   //!Returns maximum the number of objects the previously allocated memory
   //!pointed by p can hold. This size only works for memory allocated with
   //!allocate, allocation_command and allocate_many.
   size_type size(const pointer &p) const
   {
      if (mp_mngr)
         return (size_type)mp_mngr->size(ipcdetail::to_raw_pointer(p))/sizeof(T);
      else
	return std::allocator<T>().size();
   }

   //!Returns address of mutable object.
   //!Never throws
   pointer address(reference value) const
   {  return pointer(boost::addressof(value));  }

   //!Returns address of non mutable object.
   //!Never throws
   const_pointer address(const_reference value) const
   {  return const_pointer(boost::addressof(value));  }

   //!Constructs an object
   //!Throws if T's constructor throws
   //!For backwards compatibility with libraries using C++03 allocators
   template<class P>
   void construct(const pointer &ptr, BOOST_FWD_REF(P) p)
   {  ::new((void*)ipcdetail::to_raw_pointer(ptr)) value_type(::boost::forward<P>(p));  }

   //!Destroys object. Throws if object's
   //!destructor throws
   void destroy(const pointer &ptr)
   {  BOOST_ASSERT(ptr != 0); (*ptr).~value_type();  }

};

//!Equality test for same type
//!of allocator
template<class T, class SegmentManager> inline
bool operator==(const ros_allocator<T , SegmentManager>  &alloc1,
                const ros_allocator<T, SegmentManager>  &alloc2)
   {  return alloc1.get_segment_manager() == alloc2.get_segment_manager(); }

//!Inequality test for same type
//!of allocator
template<class T, class SegmentManager> inline
bool operator!=(const ros_allocator<T, SegmentManager>  &alloc1,
                const ros_allocator<T, SegmentManager>  &alloc2)
   {  return alloc1.get_segment_manager() != alloc2.get_segment_manager(); }

}  //namespace interprocess {

/// @cond

template<class T>
struct has_trivial_destructor;

template<class T, class SegmentManager>
struct has_trivial_destructor
   <boost::interprocess::ros_allocator <T, SegmentManager> >
{
   static const bool value = true;
};
/// @endcond

}  //namespace boost {

namespace ros {

template <typename value_type>
struct alloc
{
    typedef boost::interprocess::ros_allocator< value_type, boost::interprocess::managed_external_buffer::segment_manager> type;
};

typedef alloc<void>::type void_allocator;
}

#endif   //ROS_ALLOCATOR_H

