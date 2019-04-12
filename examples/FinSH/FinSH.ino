#include <rtt.h>

// ATTENTION: To compile this Sketch, please copy the following line to "shell_cmd.h" (without //)
// ADD_SHELL_CMD(led, Turn on/off builtin LED, led, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)

// ATTENTION: To compile this Sketch, please also copy the following 2 lines to "shell_var.h" (without //)
// ADD_SHELL_VAR(id, LED ID, led_id, finsh_type_uint)
// ADD_SHELL_VAR(state, LED state, led_state, finsh_type_uchar)

// After upload, send the following command through "Serial Monitor" and check the result.
// led(0, 1)
// led(0, 0)
// led(id, state)
// state
// state=0
// led(id, state)

extern "C" {

  rt_uint32_t led_id = 0;
  rt_uint8_t led_state = 1;

  rt_uint32_t led(rt_uint32_t id, rt_uint8_t state) {
    rt_kprintf("led%d=%d\n", id, state);
    if (id != 0) {
      return 1;
    }
    if (state) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
    return 0;
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
