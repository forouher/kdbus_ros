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

#include <boost/interprocess/managed_external_buffer.hpp>

extern "C"
{

#include "kdbus-util.h"
#include "kdbus-enum.h"
}

namespace ros {

class KDBusMessage {

public:

    int memfd;
    char* address_fd;
    size_t size;

    KDBusMessage() {
	memfd = -1;
	address_fd = NULL;
	size = 0;
    }

    KDBusMessage(int m, char* a, size_t s) {
	memfd = m;
	address_fd = a;
	size = s;
    }

    template<class T>
    void fillWithMessage(const T& msg) {
        boost::interprocess::managed_external_buffer segment(boost::interprocess::create_only, address_fd, size);
        typename T::allocator alloc (segment.get_segment_manager());
        segment.construct<T>("DATA")(msg,alloc);
    }

    template<class T>
    T* extractMessage() {
	boost::interprocess::managed_external_buffer segment(boost::interprocess::open_only, address_fd, size);
	T *msg = segment.find<T>("DATA").first;
	return msg;
    }

    void close();

};

class KDBusTransport {

public:

    KDBusTransport(const std::string& name);

    int create_bus();

    int destroy_bus();

    int open_connection(const std::string& name);

    int close_connection();

    KDBusMessage createMessage();

    int sendMessage(KDBusMessage& msg, const std::string& receiver);

    KDBusMessage receiveMessage();

    // connection
    struct conn *conn;

private:

    // bus
    std::string bus;
    std::string buspath;
    int fdc;

};

}

