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

extern "C" 
{

#include "kdbus-util.h"
#include "kdbus-enum.h"
}

#define KBUILD_MODNAME "kdbus"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_reader");
    ros::NodeHandle n;
    
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

	snprintf(bus_make.name, sizeof(bus_make.name), "%u-topicY", getuid());
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
	//conn_b = connect_to_bus(bus, 0);
	if (!conn_a)
		return EXIT_FAILURE;

	r = upload_policy(conn_a->fd, "test_reader");
	if (r < 0)
		return EXIT_FAILURE;
	r = name_acquire(conn_a, "test_reader", 0);
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
				name_release(conn_a, "test_reader");

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
