#include "ros/ros.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>

#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"

extern "C" 
{

#include "kdbus-util.h"
#include "kdbus-enum.h"

}

void dump_memory(char* data, size_t len)
{
    size_t i;
    printf("Data in [%p..%p): ",data,data+len);
    for (i=0;i<len;i++)
        printf("%02X ", ((unsigned char*)data)[i] );
    printf("\n");
}


int msg_send_dis(const struct conn *conn, const char *name, uint64_t cookie, uint64_t dst_id);

size_t offset_alloc = sizeof(ros::allocator<void>);
size_t offset_struct = sizeof(std_msgs::String_<ros::allocator<void> >);

int msg_send(const struct conn *conn,
		    const char *name,
		    uint64_t cookie,
		    uint64_t dst_id)
{
	struct kdbus_msg *msg;
	const char ref1[1024 * 1024 + 3] = "0123456789_0";
	const char ref2[] = "0123456789_1";
	struct kdbus_item *item;
	uint64_t size;
	int memfd = -1;
	int ret;

	size = sizeof(struct kdbus_msg);
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_vec));
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_vec));
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_vec));

	if (dst_id == KDBUS_DST_ID_BROADCAST)
		size += KDBUS_ITEM_HEADER_SIZE + 64;
	else {
		ret = ioctl(conn->fd, KDBUS_CMD_MEMFD_NEW, &memfd);
		if (ret < 0) {
			fprintf(stderr, "KDBUS_CMD_MEMFD_NEW failed: %m\n");
			return EXIT_FAILURE;
		}

		fprintf(stderr,"1\n");
		char* address_fd = (char*)mmap(NULL, 1000,PROT_WRITE,MAP_SHARED,memfd,0);
		if (MAP_FAILED == address_fd) {
			fprintf(stderr, "mmap() to memfd failed: %m\n");
			return EXIT_FAILURE;
		}

		memset(address_fd, 0xAA, 1000);

//		std_msgs::String_<ros::allocator<void> > msg_pub;
//		msg_pub.data = "0Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";

		tf2_msgs::TFMessage_<ros::allocator<void> > msg_pub;
		msg_pub.transforms.resize(3);
//		msg_pub.transforms[0].header.frame_id = "Foo1";
//		msg_pub.transforms[1].header.frame_id = "Foo2";
//		msg_pub.transforms[2].header.frame_id = "Foo3";
		msg_pub.transforms[0].header.frame_id = "0Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
		msg_pub.transforms[1].header.frame_id = "1Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
		msg_pub.transforms[2].header.frame_id = "2Hello worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
		msg_pub.transforms[0].transform.translation.x = 05;
		msg_pub.transforms[1].transform.translation.x = 15;
		msg_pub.transforms[2].transform.translation.x = 25;

//		printf("msg_pub.transforms.alloc()==msg_pub.transforms[0].header.frame_id.alloc(): %i\n", msg_pub.transforms.get_stored_allocator()==msg_pub.transforms[0].header.frame_id.get_allocator());

		ros::allocator<void>* my_alloc = new(address_fd) ros::allocator<void>(address_fd+offset_alloc+offset_struct);
		tf2_msgs::TFMessage_<ros::allocator<void> >* msg_kdbus = new(address_fd+offset_alloc) tf2_msgs::TFMessage_<ros::allocator<void> >(*my_alloc);
//		std_msgs::String_<ros::allocator<void> >* msg_kdbus = new(address_fd+offset_alloc) std_msgs::String_<ros::allocator<void> >(*my_alloc);
		*msg_kdbus = msg_pub;

		msg_pub.transforms[0].header.frame_id = "000llo worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
		msg_pub.transforms[1].header.frame_id = "111llo worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";
		msg_pub.transforms[2].header.frame_id = "222llo worldjfigpdjsjhiopdfsdhjopdfhodpdfophkoddofhdhdopkhdophkdophkdhopdkhopdkhopdhkdophkdhopdkhopdkhopdhpdEND";

		printf("ROS: orig  size=%li\n", msg_pub.transforms.size());
		printf("ROS: kdbus size=%li\n", msg_kdbus->transforms.size());
		printf("some string %s\n", msg_kdbus->transforms[0].header.frame_id.c_str());

		dump_memory(address_fd, 1000);

		munmap(address_fd,1000);

		ret = ioctl(memfd, KDBUS_CMD_MEMFD_SEAL_SET, true);
		if (ret < 0) {
			fprintf(stderr, "memfd sealing failed: %m\n");
			return EXIT_FAILURE;
		}

		size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_memfd));
	}

	if (name)
		size += KDBUS_ITEM_SIZE(strlen(name) + 1);

	msg = (kdbus_msg*)malloc(size);
	if (!msg) {
		fprintf(stderr, "unable to malloc()!?\n");
		return EXIT_FAILURE;
	}

	memset(msg, 0, size);
	msg->size = size;
	msg->src_id = conn->id;
	msg->dst_id = name ? 0 : dst_id;
	msg->cookie = cookie;
	msg->payload_type = KDBUS_PAYLOAD_DBUS;

	item = msg->items;

	if (name) {
		item->type = KDBUS_ITEM_DST_NAME;
		item->size = KDBUS_ITEM_HEADER_SIZE + strlen(name) + 1;
		strcpy(item->str, name);
		item = KDBUS_ITEM_NEXT(item);
	}

	item->type = KDBUS_ITEM_PAYLOAD_VEC;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_vec);
	item->vec.address = (uintptr_t)&ref1;
	item->vec.size = sizeof(ref1);
	item = KDBUS_ITEM_NEXT(item);

	/* data padding for ref1 */
	item->type = KDBUS_ITEM_PAYLOAD_VEC;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_vec);
	item->vec.address = (uintptr_t)NULL;
	item->vec.size =  KDBUS_ALIGN8(sizeof(ref1)) - sizeof(ref1);
	item = KDBUS_ITEM_NEXT(item);

	item->type = KDBUS_ITEM_PAYLOAD_VEC;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_vec);
	item->vec.address = (uintptr_t)&ref2;
	item->vec.size = sizeof(ref2);
	item = KDBUS_ITEM_NEXT(item);

	if (dst_id == KDBUS_DST_ID_BROADCAST) {
		item->type = KDBUS_ITEM_BLOOM;
		item->size = KDBUS_ITEM_HEADER_SIZE + 64;
	} else {
		item->type = KDBUS_ITEM_PAYLOAD_MEMFD;
		item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_memfd);
		item->memfd.size = 16;
		item->memfd.fd = memfd;
	}
	item = KDBUS_ITEM_NEXT(item);

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_SEND, msg);
	if (ret < 0) {
		fprintf(stderr, "error sending message: %d err %d (%m)\n", ret, errno);
		return EXIT_FAILURE;
	}

	if (memfd >= 0)
		close(memfd);
	free(msg);

	return 0;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_writer");
    ros::NodeHandle n;
    
	struct {
		struct kdbus_cmd_make head;

		/* bloom size item */
		struct {
			uint64_t size;
			uint64_t type;
			uint64_t bloom_size;
		} bs;

		/* name item */
		uint64_t n_size;
		uint64_t n_type;
		char name[64];
	} bus_make;

	int fdc, ret, cookie;
	char *bus;
	struct conn *conn_a, *conn_b;
	struct pollfd fds[2];
	int count;
	int r;

	printf("-- opening /dev/kdbus/control\n");
	fdc = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
	if (fdc < 0) {
		fprintf(stderr, "--- error %d (%m)\n", fdc);
		return EXIT_FAILURE;
	}

	memset(&bus_make, 0, sizeof(bus_make));
	bus_make.bs.size = sizeof(bus_make.bs);
	bus_make.bs.type = KDBUS_ITEM_BLOOM_SIZE;
	bus_make.bs.bloom_size = 64;

	snprintf(bus_make.name, sizeof(bus_make.name), "%u-topicY", getuid());
	bus_make.n_type = KDBUS_ITEM_MAKE_NAME;
	bus_make.n_size = KDBUS_ITEM_HEADER_SIZE + strlen(bus_make.name) + 1;

	bus_make.head.size = sizeof(struct kdbus_cmd_make) +
			     sizeof(bus_make.bs) +
			     bus_make.n_size;

/*
	printf("-- creating bus '%s'\n", bus_make.name);
	ret = ioctl(fdc, KDBUS_CMD_BUS_MAKE, &bus_make);
	if (ret) {
		fprintf(stderr, "--- error %d (%m)\n", ret);
		return EXIT_FAILURE;
	}
*/

	if (asprintf(&bus, "/dev/kdbus/%s/bus", bus_make.name) < 0)
		return EXIT_FAILURE;

	conn_a = connect_to_bus(bus, 0);
	conn_b = connect_to_bus(bus, 0);
	if (!conn_a || !conn_b)
		return EXIT_FAILURE;

	r = upload_policy(conn_a->fd, "foo.bar2.test");
	if (r < 0)
		return EXIT_FAILURE;
	r = upload_policy(conn_a->fd, "foo.bar2.baz");
	if (r < 0)
		return EXIT_FAILURE;

	r = name_acquire(conn_a, "foo.bar2.test", KDBUS_NAME_ALLOW_REPLACEMENT);
	if (r < 0)
		return EXIT_FAILURE;
	r = name_acquire(conn_a, "foo.bar2.baz", 0);
	if (r < 0)
		return EXIT_FAILURE;
	r = name_acquire(conn_b, "foo.bar2.baz", KDBUS_NAME_QUEUE);
	if (r < 0)
		return EXIT_FAILURE;

	name_list(conn_b, KDBUS_NAME_LIST_UNIQUE|
			  KDBUS_NAME_LIST_NAMES|
			  KDBUS_NAME_LIST_QUEUED|
			  KDBUS_NAME_LIST_ACTIVATORS);

	add_match_empty(conn_a->fd);
	add_match_empty(conn_b->fd);

	cookie = 0;

	msg_send(conn_b, NULL, 0xc0000000 | cookie, 1);

	printf("-- closing bus connections\n");
	close(conn_a->fd);
	close(conn_b->fd);
	free(conn_a);
	free(conn_b);


	printf("-- closing bus master\n");
	close(fdc);
	free(bus);

	return EXIT_SUCCESS;
}
