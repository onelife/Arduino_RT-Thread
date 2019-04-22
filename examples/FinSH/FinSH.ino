#include <rtt.h>

// ATTENTION: Please append the new shell command and variable to "shell_cmd.h" and "shell_var.h"

// NOTES: When using FinSH without MSH (CONFIG_USING_MSH == 0)
// - please append the following line to "shell_cmd.h" (without //)
// ADD_FINSH_CMD(led, Turn on/off builtin LED, led, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
//
// - and append the following 2 lines to "shell_var.h" (without //)
// ADD_SHELL_VAR(id, LED ID, led_id, finsh_type_uint)
// ADD_SHELL_VAR(state, LED state, led_state, finsh_type_uchar)
//
// After upload, send the following command through "Serial Monitor" and check the result.
// led(0, 1)
// led(0, 0)
// led(id, state)
// state
// state=0
// led(id, state)

// NOTES: When using FinSH with MSH (CONFIG_USING_MSH == 1)
// - please append the following line to "shell_cmd.h" (without //)
// ADD_MSH_CMD(led, Turn on/off builtin LED, led, rt_uint32_t, rt_uint32_t id, rt_uint8_t state)
// Due to MSH doesn't support shell variables, "ADD_SHELL_VAR" has no effect
//
// After upload, send the following command through "Serial Monitor" and check the result.
// led 0, 1, 2
// led 1, 1
// led 0, 1

extern "C" {

#if !CONFIG_USING_MSH

  rt_uint32_t led_id = 0;
  rt_uint32_t led_state = 1;

  rt_uint32_t led(rt_uint32_t id, rt_uint32_t state) {
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

#else /* !CONFIG_USING_MSH */

  rt_uint32_t led(int argc, char **argv) {
    // argc - the number of arguments
    // argv[0] - command name, e.g. "led"
    // argv[n] - nth argument in the type of char array

    rt_uint32_t id;
    rt_uint32_t state;

    if (argc != 3) {
        rt_kprintf("Usage: led <id> <state>\n");
        return 1;
    }

    rt_kprintf("led%s=%s\n", argv[1], argv[2]);
    // convert arguments to their specific types
    sscanf(argv[1], "%u", &id);
    sscanf(argv[2], "%u", &state);
    if (id != 0) {
      rt_kprintf("Error: Invalid led ID\n");
      return 1;
    }
    if (state) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
    return 0;
  }

#endif

}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  RT_T.begin();
}

// this function will be called by "Arduino" thread
void loop() {
  // may put some code here that will be run repeatedly
}
