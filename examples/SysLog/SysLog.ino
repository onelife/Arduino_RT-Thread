#include <rtt.h>
#define LOG_LVL LOG_LVL_DBG
#define LOG_TAG "EXAMPLE"
#include <log.h>

// NOTES: "LOG_LVL" and "LOG_TAG" must be defined before "#include <log.h>"
// Available LOG_LVL:
// - LOG_LVL_ASSERT
// - LOG_LVL_ERROR
// - LOG_LVL_WARNING
// - LOG_LVL_INFO
// - LOG_LVL_DBG

// NOTES: "CONFIG_USING_FINSH" and "CONFIG_USING_SPISD" in "rtconfig.h" may be turned off to save memory


// RT-Thread function called by "RT_T.begin()"
void rt_setup(void) {
}

void setup() {
  RT_T.begin();
}

// this function will be called by "Arduino" thread
void loop() {
  char msg[] = "message here";
  char hex[] = "this is a hex message: \x01\x02\x03\x0a\x0b\x0c";

  LOG_E("Error %s", msg);
  LOG_W("Warning %s", msg);
  LOG_I("Information %s", msg);
  LOG_D("Debug %s", msg); 
  LOG_RAW("Raw %s, without makeup\n", msg);
  LOG_HEX("hex_dump", 16, (rt_uint8_t *)hex, sizeof(hex));

  LOG_I("System will stop after ASSERT failed");
  ASSERT(1 == 0); // the system will stop here
  LOG_I("Never reach here");
}
