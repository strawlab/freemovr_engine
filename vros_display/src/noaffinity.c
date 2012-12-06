#include <pthread.h>
#include <stdio.h>
#include <sched.h>

int pthread_setaffinity_np(pthread_t thread, size_t cpusetsize,
                           const cpu_set_t *cpuset) {
  printf("Call to pthread_setaffinity_np() intercepted.\n");
  return 0;
}

int sched_setaffinity( pid_t pid, size_t cpusetsize,
                       cpu_set_t *mask ) {
  printf("Call to sched_setaffinity() intercepted.\n");
  return 0;
}
