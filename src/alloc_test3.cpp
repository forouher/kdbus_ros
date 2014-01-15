#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <string>
#include <cstdlib> //std::system

typedef boost::interprocess::allocator< char, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocatorS;
typedef boost::container::basic_string<char, std::char_traits< char >,ShmemAllocatorS > MyString;

typedef boost::interprocess::allocator< MyString, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocatorV;
typedef boost::container::scoped_allocator_adaptor< ShmemAllocatorV> ShmemAllocatorScopedV;
typedef boost::container::vector<MyString, ShmemAllocatorScopedV> MyVector;

typedef boost::interprocess::allocator< MyVector, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocatorVV;
typedef boost::container::scoped_allocator_adaptor< ShmemAllocatorVV> ShmemAllocatorScopedVV;
//typedef boost::container::scoped_allocator_adaptor< ShmemAllocatorVV, ShmemAllocatorV> ShmemAllocatorScopedVV;
typedef boost::container::vector<MyVector, ShmemAllocatorScopedVV> MyVectorV;

namespace boost {
namespace container {

//template <class T, class Allocator >
//struct constructible_with_allocator_suffix<basic_string<T,std::char_traits< T >, Allocator> > : ::boost::true_type { };

//template <class T, class Allocator >
////template <class T, class Allocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> >
//struct constructible_with_allocator_suffix<vector<T,Allocator> > : ::boost::true_type { };

}
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
      const ShmemAllocatorScopedVV alloc_inst (segment.get_segment_manager());

      //Construct a vector named "MyVector" in shared memory with argument alloc_inst
      MyVectorV *myvector = segment.construct<MyVectorV>("MyVector")(alloc_inst);

      const char* test = "oierwuiofjoihedfsjfigjisjijiogfiieejiogvejiogfejfigivdjiofjdifvodjdjvdviodjiodsfjwfjwiowjfiowj";
      //MyString test2 = test;

      myvector->resize(1);
      myvector->at(0).resize(5);
      for(int i = 0; i < 5; ++i)  //Insert data in the vector
	(*myvector)[0][i] = test;

      for(int i = 0; i < 10; ++i)  //Insert data in the vector
         (*myvector)[0].push_back(MyString(test, alloc_inst));

      //Launch child process
      std::string s(argv[0]); s += " child ";
      if(0 != std::system(s.c_str()))
         return 1;

      //Check child has destroyed the vector
      if(segment.find<MyVector>("MyVector").first)
         return 1;
   }
   else{ //Child process
      //Open the managed segment
      boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "MySharedMemory");

      //Find the vector using the c-string name
      MyVectorV *myvector = segment.find<MyVectorV>("MyVector").first;

      for(int i = 0; i < myvector->at(0).size(); ++i)  //Insert data in the vector
          printf("%i: got string: ->%s<-\n",i, myvector->at(0).at(i).c_str());

      //Use vector in reverse order
      //std::sort(myvector->rbegin(), myvector->rend());

      //When done, destroy the vector from the segment
      segment.destroy<MyVector>("MyVector");
   }

   return 0;
};
