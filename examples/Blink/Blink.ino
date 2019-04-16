#include <rtt.h>

// NOTES: "CONFIG_USING_FINSH" and "CONFIG_USING_SPISD" in "rtconfig.h" may be turned off to save memory

struct rt_thread blink_thread;
byte blink_thread_stack[1024];
bool led_state = false;

// user thread entry function
void blink_thread_entry(void* parameter) {
  (void)parameter;

  rt_kprintf("Start Blink\n");

  // the loop is here
  while (true) {
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
    rt_thread_sleep(RT_TICK_PER_SECOND);
  }
}

// RT-Thread function called by "RT_T.begin()"
void rt_setup(void) {
  // statically initialize user thread
  if (RT_EOK != rt_thread_init(
      &blink_thread,              // [in/out] thread descriptor
      "blink",                    // [in] thread name
      blink_thread_entry,         // [in] thread entry function
      RT_NULL,                    // [in] thread parameter (for entry function)
      blink_thread_stack,         // [in] thread stack
      sizeof(blink_thread_stack), // [in] thread stack size
      10,                         // [in] thread priority
      20)) {                      // [in] thread ticks
    rt_kprintf("Failed to initialize user thread!\n");
  } else {
    // start user thread
    rt_thread_startup(&blink_thread);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  RT_T.begin();
}

// this function will be called by "Arduino" thread
void loop() {
  // may put some code here that will be run repeatedly
}
