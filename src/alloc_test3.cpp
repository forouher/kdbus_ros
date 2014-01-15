#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <string>
#include <cstdlib> //std::system

using namespace boost::interprocess;


//Define an STL compatible allocator of ints that allocates from the managed_shared_memory.
//This allocator will allow placing containers in the segment
typedef allocator< char, managed_shared_memory::segment_manager>  ShmemAllocatorS;
typedef boost::container::basic_string<char, std::char_traits< char >,ShmemAllocatorS > MyString;

typedef allocator< MyString, managed_shared_memory::segment_manager>  ShmemAllocatorV;


//Alias a vector that uses the previous STL-like allocator so that allocates
//its values from the segment
typedef vector<MyString, ShmemAllocatorV> MyVector;

//Main function. For parent process argc == 1, for child process argc == 2
int main(int argc, char *argv[])
{
   if(argc == 1){ //Parent process
      //Remove shared memory on construction and destruction
      struct shm_remove
      {
         shm_remove() { shared_memory_object::remove("MySharedMemory"); }
         ~shm_remove(){ shared_memory_object::remove("MySharedMemory"); }
      } remover;

      //Create a new segment with given name and size
      managed_shared_memory segment(create_only, "MySharedMemory", 65536);

      //Initialize shared memory STL-compatible allocator
      const ShmemAllocatorV alloc_inst (segment.get_segment_manager());

      //Construct a vector named "MyVector" in shared memory with argument alloc_inst
      MyVector *myvector = segment.construct<MyVector>("MyVector")(alloc_inst);

//	myvector->resize(15);
      for(int i = 0; i < 10; ++i)  //Insert data in the vector
//         myvector->push_back("oierwuiofjoihedfsjfigjisjijiogfiieejiogvejiogfejfigivdjiofjdifvodjdjvdviodjiodsfjwfjwiowjfiowj");
         myvector->push_back(MyString("oierwuiofjoihedfsjfigjisjijiogfiieejiogvejiogfejfigivdjiofjdifvodjdjvdviodjiodsfjwfjwiowjfiowj", alloc_inst));

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
      managed_shared_memory segment(open_only, "MySharedMemory");

      //Find the vector using the c-string name
      MyVector *myvector = segment.find<MyVector>("MyVector").first;

      for(int i = 0; i < 10; ++i)  //Insert data in the vector
          printf("got string: ->%s<-\n", myvector->at(i).c_str());

      //Use vector in reverse order
      //std::sort(myvector->rbegin(), myvector->rend());

      //When done, destroy the vector from the segment
      segment.destroy<MyVector>("MyVector");
   }

   return 0;
};
