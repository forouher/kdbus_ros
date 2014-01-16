#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <string>
#include <cstdlib> //std::system
#include "kdbus_mapping.hpp"
#include "managed_memfd_file.hpp"

// msg_inner
namespace msgs_test {

template <class ContainerAllocator>
struct inner_ {

    // this is used by magic outside. not sure how
    typedef ContainerAllocator allocator_type;

    typedef boost::container::basic_string<char, std::char_traits< char >, typename ContainerAllocator::template rebind<char>::other > MyString;
    MyString s;

    typedef boost::container::scoped_allocator_adaptor<typename ContainerAllocator::template rebind<MyString>::other> AllocScoped;
    typedef boost::container::vector<MyString, AllocScoped> MyVector;
    MyVector vi;

    int i;
    float f;

    inner_(const ContainerAllocator& alloc)
	: vi(alloc), s(alloc)
    { }

    inner_(const inner_& i, const ContainerAllocator& alloc)
	: vi(i.vi, alloc), s(i.s, alloc)
    { }

};

//typedef ::msgs_test::inner_<boost::interprocess::allocator< void, boost::interprocess::managed_memfd_memory> > inner;
//typedef ::msgs_test::inner_<boost::interprocess::allocator< void, boost::interprocess::managed_shared_memory::segment_manager> > inner;
typedef ::msgs_test::inner_<boost::interprocess::allocator< void, boost::interprocess::managed_external_buffer::segment_manager> > inner;


// msg_outer
template <class ContainerAllocator>
struct outer_ {

    // this is used by magic outside. not sure how
    typedef ContainerAllocator allocator_type;

    typedef boost::container::scoped_allocator_adaptor<typename ContainerAllocator::template rebind<inner>::other> AllocScoped;
    typedef boost::container::vector<inner, AllocScoped> MyVectorV;
    MyVectorV v;
    MyVectorV v2;

    int i;
    float f;

    outer_(const ContainerAllocator& alloc)
	: v(alloc), v2(alloc)
    { }

    outer_(const outer_& o, const ContainerAllocator& alloc)
	: v(o.v, alloc), v2(o.v, alloc)
    { }

};

// ros::segment_manager (base class)
// ros::simple_segment_manager : ros::segment_manager (behaves like std::allocator)
// ros::kdbus_segment_manager : ros::segment_manager (stores in kdbus fd)

//typedef ::msgs_test::outer_<boost::interprocess::allocator< void, boost::interprocess::managed_memfd_memory> > outer;
typedef ::msgs_test::outer_<boost::interprocess::allocator< void, boost::interprocess::managed_external_buffer::segment_manager> > outer;
//typedef ::msgs_test::outer_<boost::interprocess::allocator< void, boost::interprocess::managed_shared_memory::segment_manager> > outer;

}

//Main function. For parent process argc == 1, for child process argc == 2
int main(int argc, char *argv[])
{
   if(argc == 1){ //Parent process
      //Remove shared memory on construction and destruction
      struct shm_remove
      {
         shm_remove() { boost::interprocess::shared_memory_object::remove("MySharedMemory"); }
         ~shm_remove(){ boost::interprocess::shared_memory_object::remove("MySharedMemory"); }
      } remover;

//      int fd = 45;
//      boost::interprocess::kdbus_mapping m_file(fd, boost::interprocess::read_write);
//      mapped_region region(m_file, read_write);

//      boost::interprocess::managed_memfd_file segment(boost::interprocess::create_only,"./demo.db", 50ul<<20); // "50Gb ought to be enough for anyone"
//      msgs_test::outer::AllocScoped alloc_inst (segment.get_segment_manager());
//      msgs_test::outer *msg3 = segment.construct<msgs_test::outer>("MyVector")(alloc_inst);

      char* buffer[65536];

      //Create a new segment with given name and size
      boost::interprocess::managed_shared_memory segment2(boost::interprocess::create_only, "MySharedMemory", 65536);
      boost::interprocess::managed_external_buffer segment(boost::interprocess::create_only, buffer, 65536);

//      boost::interprocess::managed_memfd_memory segment(fd, 65536);
      //msgs_test::outer::AllocScoped alloc_inst (&segment);

      //Initialize shared memory STL-compatible allocator
      msgs_test::outer::AllocScoped alloc_inst (segment.get_segment_manager());

      //Construct a vector named "msg" in shared memory with argument alloc_inst
      msgs_test::outer *msg = segment.construct<msgs_test::outer>("MyVector")(alloc_inst);

      const char* test = "oierwuiofjoihedfsjfigjisjijiogfiieejiogvejiogfejfigivdjiofjdifvodjdjvdviodjiodsfjwfjwiowjfiowj";
      const char* testvi = "11111111111111111111111111111111111111111ejfigivdjiofjdifvodjdjvdviodjiodsfjwfjwiowjfiowj";

      msgs_test::inner foo(alloc_inst);
      msgs_test::inner foo2(alloc_inst);
      foo.vi.resize(1);
      foo.vi[0] = testvi;
      foo2 = foo;

      msgs_test::outer bar(alloc_inst);
      msgs_test::outer bar2(alloc_inst);
      bar2 = bar;

      msg->v.resize(1);
      msg->v[0] = foo2;
      msg->v.at(0).vi.resize(5);
      for(int i = 0; i < 5; ++i)  //Insert data in the vector
	msg->v[0].vi[i] = test;

      for(int i = 0; i < 10; ++i)  //Insert data in the vector
         msg->v[0].vi.push_back(msgs_test::inner::MyString(test, alloc_inst));

      msg->v[0].vi.push_back(foo.vi[0]);

      //Launch child process
      std::string s(argv[0]); s += " child ";
      if(0 != std::system(s.c_str()))
         return 1;

      //Check child has destroyed the vector
      if(segment.find<msgs_test::outer>("MyVector").first)
         return 1;
   }
   else{ //Child process
      //Open the managed segment
      boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "MySharedMemory");
//      boost::interprocess::managed_memfd_file segment(boost::interprocess::open_only,"./demo.db"); // "50Gb ought to be enough for anyone"

      //Find the vector using the c-string name
//      msgs_test::outer *msg = segment.find<msgs_test::outer>("MyVector").first;

//      for(int i = 0; i < msg->v.at(0).vi.size(); ++i)  //Insert data in the vector
//          printf("%i: got string: ->%s<-\n",i, msg->v.at(0).vi.at(i).c_str());

      //When done, destroy the vector from the segment
      segment.destroy<msgs_test::outer>("MyVector");
   }

   return 0;
};
