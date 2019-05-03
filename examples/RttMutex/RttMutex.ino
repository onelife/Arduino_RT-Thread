#include <rtt.h>

// NOTES: "CONFIG_USING_SPISD" in "rtconfig.h" may be turned off to save memory

#define THREAD_NUM 3
#define STACK_SIZE 256

struct rt_mutex lock;

// user thread entry function
void thread_entry(void* parameter) {
  unsigned int i = (unsigned int)parameter;

  rt_kprintf("Thread %d started\n", i);

  rt_kprintf("Thread %d tries to take mutex\n", i);
  rt_mutex_take(&lock, RT_WAITING_FOREVER);
  rt_kprintf("Thread %d took mutex\n", i);

  rt_kprintf("Thread %d goes to sleep\n", i);
  rt_thread_sleep(RT_TICK_PER_SECOND);
  rt_kprintf("Thread %d woken up\n", i);

  rt_mutex_release(&lock);
  rt_kprintf("Thread %d released mutex\n", i);

  rt_kprintf("Thread %d exits\n", i);
  rt_thread_yield();
}

// RT-Thread function called by "RT_T.begin()"
void rt_setup(void) {
  unsigned int i;
  char name[] = "thread_x";
  rt_thread_t tid[THREAD_NUM];

  // dynamically initialize threads
  for (i = 0; i < THREAD_NUM; i++) {
    name[7] = '1' + i;
    if (RT_NULL == (tid[i] = rt_thread_create(
        name,                       // [in] thread name
        thread_entry,               // [in] thread entry function
        (void *)i,                  // [in] thread parameter (for entry function)
        STACK_SIZE,                 // [in] thread stack size
        10,                         // [in] thread priority
        20))) {                     // [in] thread ticks
      rt_kprintf("Failed to create thread %d!\n", i);
      return;
    }
  }

  // statically initialize mutex
  if (RT_EOK != rt_mutex_init(&lock, "lock", RT_IPC_FLAG_FIFO)) {
    rt_kprintf("Failed to initialize mutex!\n");
    return;
  }

  // start threads
  for (i = 0; i < THREAD_NUM; i++) {
    rt_thread_startup(tid[i]);
  }
}

void setup() {
  RT_T.begin();
  // no code here as RT_T.begin() never return
}

// this function will be called by "Arduino" thread
void loop() {
  // may put some code here that will be run repeatedly
}
