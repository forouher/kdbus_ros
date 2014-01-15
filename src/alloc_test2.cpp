#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include <ros/boost_container.h>

void dump_memory(char* data, size_t len)
{
    size_t i;
    printf("Data in [%p..%p): ",data,data+len);
    for (i=0;i<len;i++)
        printf("%02X ", ((unsigned char*)data)[i] );
    printf("\n");
}

size_t offset_alloc = sizeof(ros::allocator<void>);
size_t offset_struct = sizeof(tf2_msgs::TFMessage_<ros::allocator<void> >);

void send(char* shmem) {
    printf("sizeof(ros::allocator): %li\n", sizeof(ros::allocator<void>));

    printf("Creating std_alloc\n");
    char* process_mem = (char*)malloc(1000);
    ros::allocator<void>* std_alloc = new ros::allocator<void>(process_mem);

    // user creates msg in own code
    printf("Creating msg_pub\n");
    tf2_msgs::TFMessage_<ros::allocator<void> > msg_pub(*std_alloc);
    printf("Resize msg_pub\n");
    msg_pub.transforms.resize(3);
    printf("setting frame_id 1\n");
    msg_pub.transforms[0].header.frame_id = boost::container::basic_string<char, std::char_traits<char>, ros::allocator<char> >("", *std_alloc);
    printf("setting frame_id 2\n");
    msg_pub.transforms[1].header.frame_id = "1Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
    printf("setting frame_id 3\n");
    msg_pub.transforms[2].header.frame_id = "2Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
    printf("setting other transform fields\n");
    msg_pub.transforms[0].transform.translation.x = 05;
    msg_pub.transforms[1].transform.translation.x = 15;
    msg_pub.transforms[2].transform.translation.x = 25;

    // message is then converted to kdbus-compatible object
    printf("Creating new allocator\n");
    ros::allocator<void>* my_alloc = new(shmem) ros::allocator<void>(shmem+offset_alloc+offset_struct);
    printf("Creating message in shmem\n");
    tf2_msgs::TFMessage_<ros::allocator<void> >* msg_kdbus = new(shmem+offset_alloc) tf2_msgs::TFMessage_<ros::allocator<void> >(*my_alloc);
    printf("Copy to msg_kdbus\n");
    *msg_kdbus = msg_pub;

}

void receive(char* shmem2) {

    tf2_msgs::TFMessage_<ros::allocator<void> >* msg_kdbus2 = reinterpret_cast<tf2_msgs::TFMessage_<ros::allocator<void> >* >(shmem2+offset_alloc);
    
    // on subscriber, message is casted back to user-friendly msg 
    tf2_msgs::TFMessage_<ros::allocator<void> > msg;
    msg = *msg_kdbus2;

  //printf("Empfangen -->%s<--\n", msg.data.c_str());

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

//    receive(shmem2);


  return 0;
}
