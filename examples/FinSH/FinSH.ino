/***************************************************************************//**
 * @file    FinSH.ino
 * @brief   Arduino RT-Thread library "FinSH" example
 * @author  onelife <onelife.real[at]gmail.com>
 ******************************************************************************/
#include <rtt.h>

/* ATTENTION
    - Please enable "CONFIG_USING_FINSH" in "rtconfig.h"
    - Please add new shell commands to "shell_cmd.h".
 */

/* NOTES
    - Using "MSH_CMD_EXPORT_ALIAS" macro to export shell command with the following format:
      MSH_CMD_EXPORT_ALIAS(function_name, command_name, command description)
    - Using "ADD_MSH_CMD" macro to add shell command to system with the following format:
      ADD_MSH_CMD(function_name)
    - Please insert the following line to "shell_cmd.h":
      ADD_MSH_CMD(led)

    After uploaded, please send the following command through "Serial Monitor" and observe the output:
    led 0 1 2
    led 1 1
    led 0 1
 */

extern "C" {

  int led_set(int argc, char **argv) {
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
}
MSH_CMD_EXPORT_ALIAS(led_set, led, Turn on/off builtin LED.);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  RT_T.begin();
  // no code here as RT_T.begin() never return
}

// this function will be called by "Arduino" thread
void loop() {
  // may put some code here that will be run repeatedly
}
