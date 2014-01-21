#include <string>
#include <cstdlib>

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

extern "C" 
{

#include "kdbus-util.h"
#include "kdbus-enum.h"
}

#include "tf2_msgs/TFMessage.h"

#include <boost/interprocess/managed_external_buffer.hpp>

#define KBUILD_MODNAME "kdbus"

void msg_dump(const struct conn *conn, const struct kdbus_msg *msg);

int msg_send(const struct conn *conn,
		    const char *name,
		    uint64_t cookie,
		    uint64_t dst_id,
		    tf2_msgs::TFMessage& m)
{
	int ret;

	int memfd = -1;
	ret = ioctl(conn->fd, KDBUS_CMD_MEMFD_NEW, &memfd);
	if (ret < 0) {
		fprintf(stderr, "KDBUS_CMD_MEMFD_NEW failed: %m\n");
		return EXIT_FAILURE;
	}

	const int memfd_size = 200000;

	char* address_fd = (char*)mmap(NULL, memfd_size,PROT_WRITE,MAP_SHARED,memfd,0);
	if (MAP_FAILED == address_fd) {
		fprintf(stderr, "mmap() to memfd failed: %m\n");
		return EXIT_FAILURE;
	}

	//Create a new segment with given name and size
	boost::interprocess::managed_external_buffer segment(boost::interprocess::create_only, address_fd, memfd_size);
	tf2_msgs::TFMessage::allocator alloc (segment.get_segment_manager());
	segment.construct<tf2_msgs::TFMessage>("DATA")(m,alloc);

	munmap(address_fd,memfd_size);

	ret = ioctl(memfd, KDBUS_CMD_MEMFD_SEAL_SET, true);
	if (ret < 0) {
		fprintf(stderr, "memfd sealing failed: %m\n");
		return EXIT_FAILURE;
	}

	uint64_t size = sizeof(struct kdbus_msg);
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_memfd));

	if (name)
		size += KDBUS_ITEM_SIZE(strlen(name) + 1);

	struct kdbus_msg *msg = (kdbus_msg*)malloc(size);
	if (!msg) {
		fprintf(stderr, "unable to malloc()!?\n");
		return EXIT_FAILURE;
	}

	// create new kdbus message
	memset(msg, 0, size);
	msg->size = size;
	msg->src_id = conn->id;
	msg->dst_id = name ? 0 : dst_id;
	msg->cookie = cookie;
	msg->payload_type = KDBUS_PAYLOAD_DBUS;

	// start with first item
	struct kdbus_item *item = msg->items;

	// start with name
	if (name) {
		item->type = KDBUS_ITEM_DST_NAME;
		item->size = KDBUS_ITEM_HEADER_SIZE + strlen(name) + 1;
		strcpy(item->str, name);
		item = KDBUS_ITEM_NEXT(item);
	}

	// next is our memfd
	item->type = KDBUS_ITEM_PAYLOAD_MEMFD;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_memfd);
	item->memfd.size = 16;
	item->memfd.fd = memfd;
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

int msg_recv(struct conn *conn)
{
	uint64_t off;
	struct kdbus_msg *msg;
	int ret;

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_RECV, &off);
	if (ret < 0) {
		fprintf(stderr, "error receiving message: %d (%m)\n", ret);
		return EXIT_FAILURE;
	}

	msg = (struct kdbus_msg *)(conn->buf + off);
	msg_dump(conn, msg);

	ret = ioctl(conn->fd, KDBUS_CMD_FREE, &off);
	if (ret < 0) {
		fprintf(stderr, "error free message: %d (%m)\n", ret);
		return EXIT_FAILURE;
	}

	return 0;
}

void msg_dump(const struct conn *conn, const struct kdbus_msg *msg)
{
	const struct kdbus_item *item = msg->items;
	char buf_src[32];
	char buf_dst[32];
	uint64_t timeout = 0;
	uint64_t cookie_reply = 0;

	KDBUS_ITEM_FOREACH(item, msg, items) {
		if (item->size <= KDBUS_ITEM_HEADER_SIZE) {
			printf("  +%s (%llu bytes) invalid data record\n", enum_MSG(item->type), item->size);
			break;
		}

		switch (item->type) {

		case KDBUS_ITEM_PAYLOAD_MEMFD: {
			char *buf;
			uint64_t size;

			buf = (char*)mmap(NULL, 200000, PROT_READ, MAP_SHARED, item->memfd.fd, 0);
			if (buf == MAP_FAILED) {
				printf("mmap() fd=%i failed:%m", item->memfd.fd);
				break;
			}

			if (ioctl(item->memfd.fd, KDBUS_CMD_MEMFD_SIZE_GET, &size) < 0) {
				fprintf(stderr, "KDBUS_CMD_MEMFD_SIZE_GET failed: %m\n");
				break;
			}

		      boost::interprocess::managed_external_buffer segment(boost::interprocess::open_only, buf, 200000); // "50Gb ought to be enough for anyone"
    		       tf2_msgs::TFMessage *msg = segment.find<tf2_msgs::TFMessage>("DATA").first;

		          printf("%i: size of array ->%lu<-\n",0, msg->transforms.size());
		          printf("%i: got seq: ->%i<-\n",0, msg->transforms[0].header.seq);
		          printf("%i: got child string: ->%s<-\n",0, msg->transforms[0].child_frame_id.c_str());
		          printf("%i: got string: ->%s<-\n",0, msg->transforms[0].header.frame_id.c_str());


			break;
		}

		case KDBUS_ITEM_NAME: {
			printf("  +%s (%llu bytes) '%s' (%zu) flags=0x%08llx\n",
			       enum_MSG(item->type), item->size, item->name.name, strlen(item->name.name),
			       item->name.flags);
			break;
		}

		default:
			printf("  +%s (%llu bytes)\n", enum_MSG(item->type), item->size);
			break;
		}
	}

	if ((char *)item - ((char *)msg + msg->size) >= 8)
		printf("invalid padding at end of message\n");

	printf("\n");
}



//Main function. For parent process argc == 1, for child process argc == 2
int main(int argc, char *argv[])
{
   if(argc == 1){ //Parent process

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
	struct conn *conn_b;
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

	snprintf(bus_make.name, sizeof(bus_make.name), "%u-ros", getuid());
	bus_make.n_type = KDBUS_ITEM_MAKE_NAME;
	bus_make.n_size = KDBUS_ITEM_HEADER_SIZE + strlen(bus_make.name) + 1;

	bus_make.head.size = sizeof(struct kdbus_cmd_make) +
			     sizeof(bus_make.bs) +
			     bus_make.n_size;

	if (asprintf(&bus, "/dev/kdbus/%s/bus", bus_make.name) < 0)
		return EXIT_FAILURE;

	conn_b = connect_to_bus(bus, 0);
	if (!conn_b)
		return EXIT_FAILURE;

	r = upload_policy(conn_b->fd, "node_pub");
	if (r < 0)
		return EXIT_FAILURE;

	r = name_acquire(conn_b, "node_pub", KDBUS_NAME_QUEUE);
	if (r < 0)
		return EXIT_FAILURE;

	name_list(conn_b, KDBUS_NAME_LIST_UNIQUE|
			  KDBUS_NAME_LIST_NAMES|
			  KDBUS_NAME_LIST_QUEUED|
			  KDBUS_NAME_LIST_ACTIVATORS);

	add_match_empty(conn_b->fd);

	cookie = 0;

	tf2_msgs::TFMessage tfm;
	tfm.transforms.resize(1);
	tfm.transforms[0].header.frame_id = "foo1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].child_frame_id = "bar1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].header.seq = 42;

	msg_send(conn_b, NULL, 0xc0000000 | cookie, 1, tfm);

	printf("-- closing bus connections\n");
	close(conn_b->fd);
	free(conn_b);

	printf("-- closing bus master\n");
	close(fdc);
	free(bus);

	return EXIT_SUCCESS;

   }
   else{ //Child process

	struct {
		struct kdbus_cmd_make head;

		struct {
			uint64_t size;
			uint64_t type;
			uint64_t bloom_size;
		} bs;

		uint64_t n_size;
		uint64_t n_type;
		char name[64];
	} bus_make;

	int fdc, ret, cookie;
	char *bus;
	struct conn *conn_a, *conn_b;
	struct pollfd fds[1];
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

	snprintf(bus_make.name, sizeof(bus_make.name), "%u-ros", getuid());
	bus_make.n_type = KDBUS_ITEM_MAKE_NAME;
	bus_make.n_size = KDBUS_ITEM_HEADER_SIZE + strlen(bus_make.name) + 1;

	bus_make.head.size = sizeof(struct kdbus_cmd_make) +
			     sizeof(bus_make.bs) +
			     bus_make.n_size;

	printf("-- creating bus '%s'\n", bus_make.name);
	ret = ioctl(fdc, KDBUS_CMD_BUS_MAKE, &bus_make);
	if (ret) {
		fprintf(stderr, "--- error %d (%m)\n", ret);
		return EXIT_FAILURE;
	}

	if (asprintf(&bus, "/dev/kdbus/%s/bus", bus_make.name) < 0)
		return EXIT_FAILURE;

	conn_a = connect_to_bus(bus, 0);
	if (!conn_a)
		return EXIT_FAILURE;

	r = upload_policy(conn_a->fd, "node_sub");
	if (r < 0)
		return EXIT_FAILURE;
	r = name_acquire(conn_a, "node_sub", 0);
	if (r < 0)
		return EXIT_FAILURE;

	add_match_empty(conn_a->fd);

	fds[0].fd = conn_a->fd;

	printf("-- starting poll ...\n");

	{
		int i, nfds = 1;

		fds[0].events = POLLIN | POLLPRI | POLLHUP;
		fds[0].revents = 0;

		ret = poll(fds, nfds, 1000000);

		if (ret <= 0)
			return 0;

		if (fds[0].revents & POLLIN) {
			if (count > 2)
				name_release(conn_a, "node_sub");

			msg_recv(conn_a);
		}

	}

	printf("-- closing bus connections\n");
	close(conn_a->fd);
	free(conn_a);


	printf("-- closing bus master\n");
	close(fdc);
	free(bus);

	return EXIT_SUCCESS;

   }

   return 0;
};

