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
#include "ros/transport/kdbus-util.h"
#include "ros/transport/kdbus-enum.h"
}

#include <boost/interprocess/managed_external_buffer.hpp>

#define KBUILD_MODNAME "kdbus"

#include "ros/transport/kdbus_transport.h"

namespace ros {

KDBusTransport::KDBusTransport(const std::string& name) {

    bus = name;
    buspath = "/dev/kdbus/"+name+"/bus";

}

int KDBusTransport::create_bus() {

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

	int ret;

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

	strncpy(bus_make.name, bus.c_str(),bus.length());
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

	return 0;

}

int KDBusTransport::destroy_bus() {
	close(fdc);
}

int KDBusTransport::open_connection(const std::string& name) {
	int r, ret;

        if (!fdc) {
            printf("-- opening /dev/kdbus/control\n");
            fdc = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
            if (fdc < 0) {
                fprintf(stderr, "--- error %d (%m)\n", fdc);
                return EXIT_FAILURE;
            }
        }

	conn = connect_to_bus(buspath.c_str(), 0);
	if (!conn)
		return EXIT_FAILURE;

	r = upload_policy(conn->fd, name.c_str());
	if (r < 0)
		return EXIT_FAILURE;
	r = name_acquire(conn, name.c_str(), 0);
	if (r < 0)
		return EXIT_FAILURE;

	add_match_empty(conn->fd);

	return 0;

}

int KDBusTransport::close_connection() {
	close(conn->fd);
	free(conn);

}

KDBusMessage KDBusTransport::createMessage() {

        int ret;

	printf("-- opening /dev/kdbus/control\n");
	int fdc2 = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
	if (fdc2 < 0) {
		fprintf(stderr, "--- error %d (%m)\n", fdc2);
		return EXIT_FAILURE;
	}

        int memfd = -1;
        ret = ioctl(fdc2, KDBUS_CMD_MEMFD_NEW, &memfd);
        if (ret < 0) {
                fprintf(stderr, "KDBUS_CMD_MEMFD_NEW failed: %m\n");
//                return EXIT_FAILURE;
        }

        const int memfd_size = 200000;

        char* address_fd = (char*)mmap(NULL, memfd_size,PROT_WRITE,MAP_SHARED,memfd,0);
        if (MAP_FAILED == address_fd) {
                fprintf(stderr, "mmap() to memfd failed: %m\n");
//                return EXIT_FAILURE;
        }

	KDBusMessage m(memfd, address_fd, memfd_size);

	close(fdc2);

	return m;
}

int KDBusTransport::sendMessage(KDBusMessage& msg, const std::string& receiver) {

	const char* name = receiver.c_str();

        munmap(msg.address_fd,msg.size);

        int ret = ioctl(msg.memfd, KDBUS_CMD_MEMFD_SEAL_SET, true);
        if (ret < 0) {
                fprintf(stderr, "memfd sealing failed: %m\n");
                return EXIT_FAILURE;
        }

	uint64_t size = sizeof(struct kdbus_msg);
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_memfd));

	if (name)
		size += KDBUS_ITEM_SIZE(strlen(name) + 1);

	struct kdbus_msg *kmsg = (kdbus_msg*)malloc(msg.size);
	if (!kmsg) {
		fprintf(stderr, "unable to malloc()!?\n");
		return EXIT_FAILURE;
	}

	// create new kdbus message
	memset(kmsg, 0, size);
	kmsg->size = size;
	kmsg->src_id = conn->id;
	kmsg->dst_id = 0; // name defines destination
	kmsg->cookie = 0;
	kmsg->payload_type = KDBUS_PAYLOAD_DBUS; // TODO??

	// start with first item
	struct kdbus_item *item = kmsg->items;

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
	item->memfd.fd = msg.memfd;
	item = KDBUS_ITEM_NEXT(item);

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_SEND, kmsg);
	if (ret < 0) {
		fprintf(stderr, "error sending message: %d err %d (%m)\n", ret, errno);
		return EXIT_FAILURE;
	}

	close(msg.memfd);

	free(kmsg);

	return 0;

}

KDBusMessage KDBusTransport::receiveMessage() {


	uint64_t off;
	struct kdbus_msg *msg;
	int ret;

	KDBusMessage retm;

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_RECV, &off);
	if (ret < 0) {
		fprintf(stderr, "error receiving message: %d (%m)\n", ret);
//		return EXIT_FAILURE;
	}

	msg = (struct kdbus_msg *)((char*)(conn->buf) + off);
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

			retm = KDBusMessage(0, buf, 100000);

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


	ret = ioctl(conn->fd, KDBUS_CMD_FREE, &off);
	if (ret < 0) {
		fprintf(stderr, "error free message: %d (%m)\n", ret);
//		return EXIT_FAILURE;
	}

	return retm;

}

} // namespace ros

