//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright Ion Gaztanaga 2005-2012. Distributed under the Boost
// Software License, Version 1.0. (See accompanying file
// LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/interprocess for documentation.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef DFO_INTERPROCESS_FILE_MAPPING_HPP
#define DFO_INTERPROCESS_FILE_MAPPING_HPP

#include <boost/interprocess/detail/config_begin.hpp>
#include <boost/interprocess/detail/workaround.hpp>

#include <boost/interprocess/interprocess_fwd.hpp>
#include <boost/interprocess/exceptions.hpp>
#include <boost/interprocess/detail/utilities.hpp>
#include <boost/interprocess/creation_tags.hpp>
#include <boost/interprocess/detail/os_file_functions.hpp>
#include <boost/move/move.hpp>
#include <string>    //std::string

//!\file
//!Describes file_mapping and mapped region classes

namespace boost {
namespace interprocess {

/*
template
      <
         class CharType,
         class AllocationAlgorithm,
         template<class IndexConfig> class IndexType
      >
class basic_managed_memfd_memory
   : public ipcdetail::basic_managed_memory_impl
      <CharType, AllocationAlgorithm, IndexType
      ,ipcdetail::shmem_open_or_create<AllocationAlgorithm>::type::ManagedOpenOrCreateUserOffset>
   , private ipcdetail::shmem_open_or_create<AllocationAlgorithm>::type
{
   /// @cond
   typedef ipcdetail::basic_managed_memory_impl
      <CharType, AllocationAlgorithm, IndexType,
      ipcdetail::shmem_open_or_create<AllocationAlgorithm>::type::ManagedOpenOrCreateUserOffset>   base_t;
   typedef typename ipcdetail::shmem_open_or_create<AllocationAlgorithm>::type                     base2_t;

   typedef ipcdetail::create_open_func<base_t>        create_open_func_t;

   basic_managed_memfd_memory *get_this_pointer()
   {  return this;   }

   public:
   typedef shared_memory_object                    device_type;
   typedef typename base_t::size_type              size_type;

   private:
   typedef typename base_t::char_ptr_holder_t   char_ptr_holder_t;
   BOOST_MOVABLE_BUT_NOT_COPYABLE(basic_managed_memfd_memory)
   /// @endcond

   public: //functions

   //!Destroys *this and indicates that the calling process is finished using
   //!the resource. The destructor function will deallocate
   //!any system resources allocated by the system for use by this process for
   //!this resource. The resource can still be opened again calling
   //!the open constructor overload. To erase the resource from the system
   //!use remove().
   ~basic_managed_memfd_memory()
   {}

   //!Default constructor. Does nothing.
   //!Useful in combination with move semantics
   basic_managed_memfd_memory()
   {}

   //!Creates shared memory and creates and places the segment manager.
   //!This can throw.
   basic_managed_memfd_memory(int fd, create_only_t, const char *name,
                             size_type size, const void *addr = 0, const permissions& perm = permissions())
      : base_t()
      , base2_t(create_only, name, size, read_write, addr,
                create_open_func_t(get_this_pointer(), ipcdetail::DoCreate), perm)
   {}

   //!Creates shared memory and creates and places the segment manager if
   //!segment was not created. If segment was created it connects to the
   //!segment.
   //!This can throw.
   basic_managed_memfd_memory (open_or_create_t,
                              const char *name, size_type size,
                              const void *addr = 0, const permissions& perm = permissions())
      : base_t()
      , base2_t(open_or_create, name, size, read_write, addr,
                create_open_func_t(get_this_pointer(),
                ipcdetail::DoOpenOrCreate), perm)
   {}

   //!Connects to a created shared memory and its segment manager.
   //!in copy_on_write mode.
   //!This can throw.
   basic_managed_memfd_memory (open_copy_on_write_t, const char* name,
                                const void *addr = 0)
      : base_t()
      , base2_t(open_only, name, copy_on_write, addr,
                create_open_func_t(get_this_pointer(),
                ipcdetail::DoOpen))
   {}

   //!Connects to a created shared memory and its segment manager.
   //!in read-only mode.
   //!This can throw.
   basic_managed_memfd_memory (open_read_only_t, const char* name,
                                const void *addr = 0)
      : base_t()
      , base2_t(open_only, name, read_only, addr,
                create_open_func_t(get_this_pointer(),
                ipcdetail::DoOpen))
   {}

   //!Connects to a created shared memory and its segment manager.
   //!This can throw.
   basic_managed_memfd_memory (open_only_t, const char* name,
                                const void *addr = 0)
      : base_t()
      , base2_t(open_only, name, read_write, addr,
                create_open_func_t(get_this_pointer(),
                ipcdetail::DoOpen))
   {}

   //!Moves the ownership of "moved"'s managed memory to *this.
   //!Does not throw
   basic_managed_memfd_memory(BOOST_RV_REF(basic_managed_memfd_memory) moved)
   {
      basic_managed_memfd_memory tmp;
      this->swap(moved);
      tmp.swap(moved);
   }

   //!Moves the ownership of "moved"'s managed memory to *this.
   //!Does not throw
   basic_managed_memfd_memory &operator=(BOOST_RV_REF(basic_managed_memfd_memory) moved)
   {
      basic_managed_memfd_memory tmp(boost::move(moved));
      this->swap(tmp);
      return *this;
   }

   //!Swaps the ownership of the managed shared memories managed by *this and other.
   //!Never throws.
   void swap(basic_managed_memfd_memory &other)
   {
      base_t::swap(other);
      base2_t::swap(other);
   }

   //!Tries to resize the managed shared memory object so that we have
   //!room for more objects.
   //!
   //!This function is not synchronized so no other thread or process should
   //!be reading or writing the file
   static bool grow(const char *shmname, size_type extra_bytes)
   {
      return base_t::template grow
         <basic_managed_memfd_memory>(shmname, extra_bytes);
   }

   //!Tries to resize the managed shared memory to minimized the size of the file.
   //!
   //!This function is not synchronized so no other thread or process should
   //!be reading or writing the file
   static bool shrink_to_fit(const char *shmname)
   {
      return base_t::template shrink_to_fit
         <basic_managed_memfd_memory>(shmname);
   }
   /// @cond

   //!Tries to find a previous named allocation address. Returns a memory
   //!buffer and the object count. If not found returned pointer is 0.
   //!Never throws.
   template <class T>
   std::pair<T*, size_type> find  (char_ptr_holder_t name)
   {
      if(base2_t::get_mapped_region().get_mode() == read_only){
         return base_t::template find_no_lock<T>(name);
      }
      else{
         return base_t::template find<T>(name);
      }
   }

   /// @endcond
};
*/

template
      <  
         class CharType,
         class MemoryAlgorithm,
         template<class IndexConfig> class IndexType
      >
class basic_managed_memfd_memory : public segment_manager<CharType, MemoryAlgorithm, IndexType>
{
   typedef segment_manager<CharType, MemoryAlgorithm, IndexType> Base;

   int fd_;

   public:
   typedef MemoryAlgorithm                memory_algorithm;
   typedef typename Base::void_pointer    void_pointer;
   typedef typename Base::size_type       size_type;
   typedef typename Base::difference_type difference_type;
   typedef CharType                       char_type;

    explicit basic_managed_memfd_memory(const int fd, size_type segment_size)
	: Base(segment_size), fd_(fd) {};

};

typedef basic_managed_memfd_memory<char,rbtree_best_fit<mutex_family>,iset_index> managed_memfd_memory;


//!A class that wraps a file-mapping that can be used to
//!create mapped regions from the mapped files
class kdbus_mapping
{
   /// @cond
   BOOST_MOVABLE_BUT_NOT_COPYABLE(kdbus_mapping)
   /// @endcond

   public:
   //!Constructs an empty file mapping.
   //!Does not throw
   kdbus_mapping();

   //!Opens a file mapping of file "filename", starting in offset
   //!"file_offset", and the mapping's size will be "size". The mapping
   //!can be opened for read-only "read_only" or read-write "read_write"
   //!modes. Throws interprocess_exception on error.
   kdbus_mapping(const int fd, mode_t mode);

   //!Moves the ownership of "moved"'s file mapping object to *this.
   //!After the call, "moved" does not represent any file mapping object.
   //!Does not throw
   kdbus_mapping(BOOST_RV_REF(kdbus_mapping) moved)
      :  m_handle(file_handle_t(ipcdetail::invalid_file()))
      ,  m_mode(read_only)
   {  this->swap(moved);   }

   //!Moves the ownership of "moved"'s file mapping to *this.
   //!After the call, "moved" does not represent any file mapping.
   //!Does not throw
   kdbus_mapping &operator=(BOOST_RV_REF(kdbus_mapping) moved)
   {
      kdbus_mapping tmp(boost::move(moved));
      this->swap(tmp);
      return *this;
   }

   //!Swaps to kdbus_mappings.
   //!Does not throw.
   void swap(kdbus_mapping &other);

   //!Returns access mode
   //!used in the constructor
   mode_t get_mode() const;

   //!Obtains the mapping handle
   //!to be used with mapped_region
   mapping_handle_t get_mapping_handle() const;

   //!Destroys the file mapping. All mapped regions created from this are still
   //!valid. Does not throw
   ~kdbus_mapping();

   //!Returns the name of the file
   //!used in the constructor.
//   const char *get_name() const;

   //!Removes the file named "filename" even if it's been memory mapped.
   //!Returns true on success.
   //!The function might fail in some operating systems if the file is
   //!being used other processes and no deletion permission was shared.
   static bool remove(const int fd);

   /// @cond
   private:
   //!Closes a previously opened file mapping. Never throws.
   void priv_close();
   file_handle_t  m_handle;
   mode_t         m_mode;
   int 		  m_fd;
   /// @endcond
};

inline kdbus_mapping::kdbus_mapping()
   :  m_handle(file_handle_t(ipcdetail::invalid_file()))
   ,  m_mode(read_only)
{}

inline kdbus_mapping::~kdbus_mapping()
{  this->priv_close(); }

//inline const char *kdbus_mapping::get_name() const
//{  return m_filename.c_str(); }

inline void kdbus_mapping::swap(kdbus_mapping &other)
{
   std::swap(m_handle, other.m_handle);
   std::swap(m_mode, other.m_mode);
   std::swap(m_fd, other.m_fd);
}

inline mapping_handle_t kdbus_mapping::get_mapping_handle() const
{  return ipcdetail::mapping_handle_from_file_handle(m_handle);  }

inline mode_t kdbus_mapping::get_mode() const
{  return m_mode; }

inline kdbus_mapping::kdbus_mapping
   (const int fd, mode_t mode)
   :  m_fd(fd)
{
   //Check accesses
   if (mode != read_write && mode != read_only){
      error_info err = other_error;
      throw interprocess_exception(err);
   }

   //Open file
   //m_handle = ipcdetail::open_existing_file(filename, mode);

   //Check for error
   if(m_handle == ipcdetail::invalid_file()){
      error_info err = system_error_code();
      this->priv_close();
      throw interprocess_exception(err);
   }
   m_mode = mode;
}

inline bool kdbus_mapping::remove(const int fd)
{  /*return ipcdetail::delete_file(filename);*/  }

///@cond

inline void kdbus_mapping::priv_close()
{
   if(m_handle != ipcdetail::invalid_file()){
      ipcdetail::close_file(m_handle);
      m_handle = ipcdetail::invalid_file();
   }
}

///@endcond

}

}

#include <boost/interprocess/detail/config_end.hpp>

#endif   //DFO_INTERPROCESS_kdbus_mapping_HPP
