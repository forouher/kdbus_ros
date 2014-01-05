Introduction
============

This is an experimental new transport mechanism for roscpp.

It builds upon the memfd and kdbus kernel API to transport
ROS messages between roscpp nodes without the need of
serialization or data copy.

The motivation is to speed up the message transport between roscpp nodes on the same host.
This transport method is, depending on the message size, significantly faster than the
TCP based transport method.

This change is split over multiple packages

* roscpp
* cpp_common
* roscpp_serialization
* roscpp_traits
* gencpp

Due to some API modifications, changes in other packages are also necessary. We've made those changes
to some packages, but not the full desktop-full list of packages. They are included in the rosinstall
file included in this repository.

Concept
=======

The core idea is to change the memory structure of a ROS message, so that all its content is contained
within a well-known memory area. That memory area can then be shared across processes.

Making sure that a complex C++ class like a ROS message is completely contained within a defined memory
segment is non-trivial. Most of the complexity of this patch-set stems from the changes necessary to
guarantee that.

A good analysis of the problems one encounters with sharing data structures across process is explained
in the Boost.Interprocess documentation: http://www.boost.org/doc/libs/1_56_0/doc/html/interprocess/allocators_containers.html

The main points are:
* The message object has to be created with a special function. Using new() is not enough. We provide
  function ros::make_shared<T>() for that. It works the same way as boost::make_shared<T>();
* The message has to be given a custom memory allocator. Allocators are used by containers like vector
  or string to dynamically allocate memory. We ended up creating a  custom allocator "ros_allocator"
  for this.
* The use of standard STL containers is not possible. Special Boost.Container types have to be used.
  The reason for this is that STL containers internally use absolute pointers T* to store references to
  dynamically allocated memory. Absolute pointers won't work with shared memory segments (the memory
  segment might be mapped to a different address in each process). Therefore special "offset_ptr" have
  to be used, which is only possible with Boost.Container types.

The last point is the most intrusive one. Unfortunately, it brakes API in some (rare) situations.
We've tried to minimize the effects by wrapping the Boost.Container types into own ROS types (e.g.
ros::messages::types::vector<T>). With dynamic casting, those types can transform themself into STL
types in many cases (or pretend to be one).

Dependencies
============

* Ubuntu 14.04 amd64
* Kernel 3.17 (e.g. http://kernel.ubuntu.com/~kernel-ppa/mainline/v3.17-utopic/)
* Kdbus Kernel module (https://github.com/gregkh/kdbus)
* Boost 1.56 (optional but recommended, due to improved performance of Boost.Container)

Install
=======

1. Install the dependencies given above
2. Download the rosinstall within this github repository
3. Use it to create a ROS workspace. (wstool init ...)
4. As long as the kdbus module is loaded, roscpp traffic should now run via kdbus
5. The package kdbus_tests contains a simple publisher/subscriber example.

Caveats
=======

This code is *very* experimental. While it will probably not eat your kittens, it will very likely
crash. The simple example given in kdbus_tests works, but I wouldn't expect more complex nodes to
work.

Just a few of the things that need to be addressed:

* Topic latching is currently broken
* Having two kdbus-capable roscpp nodes on seperate hosts will fail
* Starting and stopping of nodes is unreliable
* Some changes to roscpp are somewhat hacky and need to be reworked (e.g. TopicManager::publish)
* Probably a lot more...

Links
=====

* http://www.boost.org/doc/libs/1_56_0/doc/html/interprocess.html
* http://www.boost.org/doc/libs/1_56_0/doc/html/container.html
* https://github.com/gregkh/kdbus
* https://lwn.net/Articles/602332/
* https://lwn.net/Articles/593918/
