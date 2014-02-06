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
#include <ros/transport/kdbus_transport.h>

#include "tf2_msgs/TFMessage.h"

//Main function. For parent process argc == 1, for child process argc == 2
int main(int argc, char *argv[])
{
   if(argc == 1){ //Parent process

	ros::KDBusTransport t("1000-ros");
	t.open_connection("publisher");

	tf2_msgs::TFMessage tfm;
	tfm.transforms.resize(1);
	tfm.transforms[0].header.frame_id = "foo1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].child_frame_id = "bar1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF!!!!!!";
	tfm.transforms[0].header.seq = 42;

	ros::KDBusMessage m = t.createMessage();
	m.fillWithMessage<tf2_msgs::TFMessage>(tfm);
	t.sendMessage(m, "subscriber");

	t.close_connection();

	return EXIT_SUCCESS;

   }
   else{ //Child process

	ros::KDBusTransport t("1000-ros");
	t.create_bus();
	t.open_connection("subscriber");

	struct pollfd fds[1];
	fds[0].fd = t.conn->fd;

	printf("-- starting poll ...\n");

	{
		int i, nfds = 1;

		fds[0].events = POLLIN | POLLPRI | POLLHUP;
		fds[0].revents = 0;

		int ret = poll(fds, nfds, 1000000);

		if (ret <= 0)
			return 0;

		if (fds[0].revents & POLLIN) {

			ros::KDBusMessage m = t.receiveMessage();
			tf2_msgs::TFMessage* msg = m.extractMessage<tf2_msgs::TFMessage>();

		          printf("%i: size of array ->%lu<-\n",0, msg->transforms.size());
		          printf("%i: got seq: ->%i<-\n",0, msg->transforms[0].header.seq);
		          printf("%i: got child string: ->%s<-\n",0, msg->transforms[0].child_frame_id.c_str());
		          printf("%i: got string: ->%s<-\n",0, msg->transforms[0].header.frame_id.c_str());

		}

	}

	t.close_connection();
	t.destroy_bus();

	return EXIT_SUCCESS;

   }

   return 0;
};

