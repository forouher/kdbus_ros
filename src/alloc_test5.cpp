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
#include "kdbus_transport.h"

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

	ros::KDBusTransport t("1000-ros");
	t.open_connection("publisher");

	int cookie = 0;

	tf2_msgs::TFMessage tfm;
	tfm.transforms.resize(1);
	tfm.transforms[0].header.frame_id = "foo1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].child_frame_id = "bar1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].header.seq = 42;

	msg_send(t.conn, NULL, 0xc0000000 | cookie, 1, tfm);

	t.close_connection();

	return EXIT_SUCCESS;

   }
   else{ //Child process

	ros::KDBusTransport t("1000-ros");
	t.create_bus();
	t.open_connection("subscriber");

	int fdc, ret;
	struct pollfd fds[1];
	int count;
	int r;
	fds[0].fd = t.conn->fd;

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
				name_release(t.conn, "node_sub");

			msg_recv(t.conn);
		}

	}

	t.close_connection();

	t.destroy_bus();

	return EXIT_SUCCESS;

   }

   return 0;
};

