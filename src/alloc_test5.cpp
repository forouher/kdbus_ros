#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/container/vector.hpp>
#include <boost/container/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <string>
#include <cstdlib> //std::system
//#include "kdbus_mapping.hpp"
//#include "managed_memfd_file.hpp"
//#include "allocator.hpp"


#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>

//#include "ros/ros.h"
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

//#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include "ros/allocator.h"
#include "ros/boost_container.h"

#define KBUILD_MODNAME "kdbus"




void dump_memory(char* data, size_t len)
{
    size_t i;
    printf("Data in [%p..%p): ",data,data+len);
    for (i=0;i<len;i++)
        printf("%02X ", ((unsigned char*)data)[i] );
    printf("\n");
}

void msg_dump(const struct conn *conn, const struct kdbus_msg *msg);

int msg_send_dis(const struct conn *conn, const char *name, uint64_t cookie, uint64_t dst_id);

int msg_send(const struct conn *conn,
		    const char *name,
		    uint64_t cookie,
		    uint64_t dst_id,
		    tf2_msgs::TFMessage& m)
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

{
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

		memset(address_fd, 0xAA, memfd_size);

      //Create a new segment with given name and size
      boost::interprocess::managed_external_buffer segment(boost::interprocess::create_only, address_fd, memfd_size);
      tf2_msgs::TFMessage::allocator alloc (segment.get_segment_manager());
      segment.construct<tf2_msgs::TFMessage>("MyVector")(m,alloc);

		munmap(address_fd,memfd_size);

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

/*	if (dst_id == KDBUS_DST_ID_BROADCAST) {
		item->type = KDBUS_ITEM_BLOOM;
		item->size = KDBUS_ITEM_HEADER_SIZE + 64;
	} else {*/
		item->type = KDBUS_ITEM_PAYLOAD_MEMFD;
		item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_memfd);
		item->memfd.size = 16;
		item->memfd.fd = memfd;
//	}
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

	if (msg->flags & KDBUS_MSG_FLAGS_EXPECT_REPLY)
		timeout = msg->timeout_ns;
	else
		cookie_reply = msg->cookie_reply;

	printf("MESSAGE: %s (%llu bytes) flags=0x%08llx, %s → %s, cookie=%llu, timeout=%llu cookie_reply=%llu\n",
		enum_PAYLOAD(msg->payload_type), (unsigned long long)msg->size,
		(unsigned long long)msg->flags,
		msg_id(msg->src_id, buf_src), msg_id(msg->dst_id, buf_dst),
		(unsigned long long)msg->cookie, (unsigned long long)timeout, (unsigned long long)cookie_reply);

	KDBUS_ITEM_FOREACH(item, msg, items) {
		if (item->size <= KDBUS_ITEM_HEADER_SIZE) {
			printf("  +%s (%llu bytes) invalid data record\n", enum_MSG(item->type), item->size);
			break;
		}

		switch (item->type) {
		case KDBUS_ITEM_PAYLOAD_OFF: {
			const char *s;

			if (item->vec.offset == ~0ULL)
				s = "[\\0-bytes]";
			else
				s = (char *)conn->buf + item->vec.offset;

			printf("  +%s (%llu bytes) off=%llu size=%llu '%s'\n",
			       enum_MSG(item->type), item->size,
			       (unsigned long long)item->vec.offset,
			       (unsigned long long)item->vec.size, s);
			break;
		}

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

			dump_memory(buf, 1000);

//			size_t offset_alloc = sizeof(ros::allocator<void>);

		      boost::interprocess::managed_external_buffer segment(boost::interprocess::open_only, buf, 200000); // "50Gb ought to be enough for anyone"

		      //Find the vector using the c-string name
//    		       std_msgs::String *msg = segment.find<std_msgs::String>("MyVector").first;
    		       tf2_msgs::TFMessage *msg = segment.find<tf2_msgs::TFMessage>("MyVector").first;

		          printf("%i: size of array ->%lu<-\n",0, msg->transforms.size());

		          printf("%i: got seq: ->%i<-\n",0, msg->transforms[0].header.seq);
		          printf("%i: got child string: ->%s<-\n",0, msg->transforms[0].child_frame_id.c_str());
		          printf("%i: got string: ->%s<-\n",0, msg->transforms[0].header.frame_id.c_str());
//			for(int i = 0; i < msg->v.at(0).vi.size(); ++i)  //Insert data in the vector
//		          printf("%i: got string: ->%s<-\n",i, msg->v.at(0).vi.at(i).c_str());


//			printf("ROS: tf0=%lf  tf1=%lf tf2=%lf \n",  msg_kdbus2->transforms[0].transform.translation.x,  msg_kdbus2->transforms[1].transform.translation.x,  msg_kdbus2->transforms[2].transform.translation.x);
//			printf("ROS: size kdbus=%li\n",  msg_kdbus2->transforms.size()); // msg_kdbus2->transforms[0].header.frame_id.c_str()
/*			printf("  +%s (%llu bytes) fd=%i size=%llu filesize=%llu '%s'\n",
			       enum_MSG(item->type), item->size, item->memfd.fd,
			       (unsigned long long)item->memfd.size, (unsigned long long)size, msg_kdbus2->transforms[0].header.frame_id.c_str());
*/			break;
		}

		case KDBUS_ITEM_CREDS:
			printf("  +%s (%llu bytes) uid=%lld, gid=%lld, pid=%lld, tid=%lld, starttime=%lld\n",
				enum_MSG(item->type), item->size,
				item->creds.uid, item->creds.gid,
				item->creds.pid, item->creds.tid,
				item->creds.starttime);
			break;

		case KDBUS_ITEM_PID_COMM:
		case KDBUS_ITEM_TID_COMM:
		case KDBUS_ITEM_EXE:
		case KDBUS_ITEM_CGROUP:
		case KDBUS_ITEM_SECLABEL:
		case KDBUS_ITEM_DST_NAME:
			printf("  +%s (%llu bytes) '%s' (%zu)\n",
			       enum_MSG(item->type), item->size, item->str, strlen(item->str));
			break;

		case KDBUS_ITEM_NAME: {
			printf("  +%s (%llu bytes) '%s' (%zu) flags=0x%08llx\n",
			       enum_MSG(item->type), item->size, item->name.name, strlen(item->name.name),
			       item->name.flags);
			break;
		}

		case KDBUS_ITEM_CMDLINE: {
			size_t size = item->size - KDBUS_ITEM_HEADER_SIZE;
			const char *str = item->str;
			int count = 0;

			printf("  +%s (%llu bytes) ", enum_MSG(item->type), item->size);
			while (size) {
				printf("'%s' ", str);
				size -= strlen(str) + 1;
				str += strlen(str) + 1;
				count++;
			}

			printf("(%d string%s)\n", count, (count == 1) ? "" : "s");
			break;
		}

		case KDBUS_ITEM_AUDIT:
			printf("  +%s (%llu bytes) loginuid=%llu sessionid=%llu\n",
			       enum_MSG(item->type), item->size,
			       (unsigned long long)item->data64[0],
			       (unsigned long long)item->data64[1]);
			break;

		case KDBUS_ITEM_CAPS: {
			int n;
			const uint32_t *cap;
			int i;

			printf("  +%s (%llu bytes) len=%llu bytes\n",
			       enum_MSG(item->type), item->size,
			       (unsigned long long)item->size - KDBUS_ITEM_HEADER_SIZE);

			cap = item->data32;
			n = (item->size - KDBUS_ITEM_HEADER_SIZE) / 4 / sizeof(uint32_t);

			printf("    CapInh=");
			for (i = 0; i < n; i++)
				printf("%08x", cap[(0 * n) + (n - i - 1)]);

			printf(" CapPrm=");
			for (i = 0; i < n; i++)
				printf("%08x", cap[(1 * n) + (n - i - 1)]);

			printf(" CapEff=");
			for (i = 0; i < n; i++)
				printf("%08x", cap[(2 * n) + (n - i - 1)]);

			printf(" CapInh=");
			for (i = 0; i < n; i++)
				printf("%08x", cap[(3 * n) + (n - i - 1)]);
			printf("\n");
			break;
		}

		case KDBUS_ITEM_TIMESTAMP:
			printf("  +%s (%llu bytes) realtime=%lluns monotonic=%lluns\n",
			       enum_MSG(item->type), item->size,
			       (unsigned long long)item->timestamp.realtime_ns,
			       (unsigned long long)item->timestamp.monotonic_ns);
			break;

		case KDBUS_ITEM_REPLY_TIMEOUT:
			printf("  +%s (%llu bytes) cookie=%llu\n",
			       enum_MSG(item->type), item->size, msg->cookie_reply);
			break;

		case KDBUS_ITEM_NAME_ADD:
		case KDBUS_ITEM_NAME_REMOVE:
		case KDBUS_ITEM_NAME_CHANGE:
			printf("  +%s (%llu bytes) '%s', old id=%lld, new id=%lld, old_flags=0x%llx new_flags=0x%llx\n",
				enum_MSG(item->type), (unsigned long long) item->size,
				item->name_change.name, item->name_change.old.id,
				item->name_change.new2.id, item->name_change.old.flags,
				item->name_change.new2.flags);
			break;

		case KDBUS_ITEM_ID_ADD:
		case KDBUS_ITEM_ID_REMOVE:
			printf("  +%s (%llu bytes) id=%llu flags=%llu\n",
			       enum_MSG(item->type), (unsigned long long) item->size,
			       (unsigned long long) item->id_change.id,
			       (unsigned long long) item->id_change.flags);
			break;

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
//    ros::init(argc, argv, "test_reader");
//    ros::NodeHandle n;

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

	tf2_msgs::TFMessage tfm;
	tfm.transforms.resize(1);
	tfm.transforms[0].header.frame_id = "foo1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].child_frame_id = "bar1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].header.seq = 42;

	msg_send(conn_b, NULL, 0xc0000000 | cookie, 1, tfm);

	printf("-- closing bus connections\n");
	close(conn_a->fd);
	close(conn_b->fd);
	free(conn_a);
	free(conn_b);


	printf("-- closing bus master\n");
	close(fdc);
	free(bus);
/*
      //Launch child process
      std::string s(argv[0]); s += " child ";
      if(0 != std::system(s.c_str()))
         return 1;
*/
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


      //Open the managed segment
//      boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "MySharedMemory");
//      boost::interprocess::managed_memfd_file segment(boost::interprocess::open_only,"./demo.db"); // "50Gb ought to be enough for anyone"

      //Find the vector using the c-string name
//      msgs_test::outer *msg = segment.find<msgs_test::outer>("MyVector").first;

//      for(int i = 0; i < msg->v.at(0).vi.size(); ++i)  //Insert data in the vector
//          printf("%i: got string: ->%s<-\n",i, msg->v.at(0).vi.at(i).c_str());

      //When done, destroy the vector from the segment
//      segment.destroy<msgs_test::outer>("MyVector");
   }

   return 0;
};

