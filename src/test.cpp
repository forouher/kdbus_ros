#include <sys/time.h>
#include <boost/container/vector.hpp>
#include <vector>

void test()
{
  int size = 2000000;
  {
    struct timeval start, end;
    gettimeofday(&start, NULL);

    std::vector<char, std::allocator<char> > vec(size);

    gettimeofday(&end, NULL);
    long long time =(end.tv_sec * (unsigned int)1e6 +   end.tv_usec) -
                    (start.tv_sec * (unsigned int)1e6 + start.tv_usec);
    printf("std::vector %llu us\n", time);
  }
  {
    struct timeval start, end;
    gettimeofday(&start, NULL);

    boost::container::vector<char, std::allocator<char> > vec(size);

    gettimeofday(&end, NULL);
    long long time =(end.tv_sec * (unsigned int)1e6 +   end.tv_usec) -
                    (start.tv_sec * (unsigned int)1e6 + start.tv_usec);
    printf("boost::vector %llu us\n", time);
  }
}

int main(int argc, char **argv)
{
  while(true) {
    test();
    usleep(100000);
  }
}
