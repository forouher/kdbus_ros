#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <string>
#include <cstdlib> //std::system

// msg_inner
namespace msgs_test {

struct inner {

    typedef boost::interprocess::allocator< char, boost::interprocess::managed_shared_memory::segment_manager>  AllocS;
    typedef boost::container::basic_string<char, std::char_traits< char >,AllocS > MyString;

    typedef boost::interprocess::allocator< MyString, boost::interprocess::managed_shared_memory::segment_manager>  AllocV;
    typedef boost::container::scoped_allocator_adaptor< AllocV> AllocScoped;
    typedef boost::container::vector<MyString, AllocScoped> MyVector;

    inner(const AllocScoped& alloc)
	: s(alloc)
    { }

    MyString s;
};

// msg_outer
struct outer {

    typedef boost::interprocess::allocator< inner::MyVector, boost::interprocess::managed_shared_memory::segment_manager>  AllocV;
    typedef boost::container::scoped_allocator_adaptor< AllocV> AllocScoped;
    typedef boost::container::vector<inner::MyVector, AllocScoped> MyVectorV;

    outer(const AllocScoped& alloc)
	: v(alloc)
    { }

    MyVectorV v;

};

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

      //Create a new segment with given name and size
      boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, "MySharedMemory", 65536);

      //Initialize shared memory STL-compatible allocator
      const msgs_test::outer::AllocScoped alloc_inst (segment.get_segment_manager());

      //Construct a vector named "MyVector" in shared memory with argument alloc_inst
      msgs_test::outer *myvector = segment.construct<msgs_test::outer>("MyVector")(alloc_inst);

      const char* test = "oierwuiofjoihedfsjfigjisjijiogfiieejiogvejiogfejfigivdjiofjdifvodjdjvdviodjiodsfjwfjwiowjfiowj";

      msgs_test::inner foo(alloc_inst);
      msgs_test::inner foo2(alloc_inst);
      foo.s = test;
      foo2 = foo;

      msgs_test::outer bar(alloc_inst);
      msgs_test::outer bar2(alloc_inst);
      bar2 = bar;

      myvector->v.resize(1);
      myvector->v.at(0).resize(5);
      for(int i = 0; i < 5; ++i)  //Insert data in the vector
	myvector->v[0][i] = test;

      for(int i = 0; i < 10; ++i)  //Insert data in the vector
         myvector->v[0].push_back(msgs_test::inner::MyString(test, alloc_inst));

      myvector->v[0].push_back(foo.s);

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

      //Find the vector using the c-string name
      msgs_test::outer *myvector = segment.find<msgs_test::outer>("MyVector").first;

      for(int i = 0; i < myvector->v.at(0).size(); ++i)  //Insert data in the vector
          printf("%i: got string: ->%s<-\n",i, myvector->v.at(0).at(i).c_str());

      //Use vector in reverse order
      //std::sort(myvector->rbegin(), myvector->rend());

      //When done, destroy the vector from the segment
      segment.destroy<msgs_test::outer>("MyVector");
   }

   return 0;
};
