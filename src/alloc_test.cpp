#include "std_msgs/String.h"
#include <ros/boost_container.h>

void dump_memory(char* data, size_t len)
{
    size_t i;
    printf("Data in [%p..%p): ",data,data+len);
    for (i=0;i<len;i++)
        printf("%02X ", ((unsigned char*)data)[i] );
    printf("\n");
}

size_t offset_alloc = sizeof(boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager>);
size_t offset_struct = sizeof(std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> >);

void send(char* shmem) {
    printf("sizeof(boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager>): %li\n", sizeof(boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager>));


    // user creates msg in own code
    std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> > msg_pub;
    //msg_pub.data = "Hello world";
//    msg_pub.data = "Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdkfohdkhopdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdkhop";

    printf("Sende -->%s<--\n", msg_pub.data.c_str());

    // message is then converted to kdbus-compatible object
    boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager>* my_alloc;// = new(shmem) boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager>(shmem+offset_alloc+offset_struct);
    std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> >* msg_kdbus = new(shmem+offset_alloc) std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> >(*my_alloc);
    *msg_kdbus = msg_pub;

}

void receive(char* shmem2) {

    std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> >* msg_kdbus2 = reinterpret_cast<std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> >* >(shmem2+offset_alloc);
    
    // on subscriber, message is casted back to user-friendly msg 
    std_msgs::String_<boost::interprocess::ros_allocator< void, boost::interprocess::managed_external_buffer::segment_manager> > msg;
    msg = *msg_kdbus2;

  printf("Empfangen -->%s<--\n", msg.data.c_str());

}

int main(int argc, char **argv)
{
    char* shmem = (char*)malloc(1000);
    send(shmem);

    char* shmem2 = (char*)malloc(1000);

    memcpy(shmem2, shmem,1000);
    memset(shmem,0, 1000);
    dump_memory(shmem, 100);
    free(shmem);
    dump_memory(shmem2, 100);

    receive(shmem2);


  return 0;
}
