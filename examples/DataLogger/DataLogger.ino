#include <rtt.h>

// RT-Thread function called by "RT_T.begin()"
void rt_setup(void) {
}

void setup() {
  RT_T.begin();
}

// this function will be called by "Arduino" thread
void loop() {
  char buf[15];
  int sensorVal[3];
  int dataFile = -1;
  const unsigned int maxCount = 10;
  static unsigned int count = 0;

  // read sensors once per second for 10 times
  if (count < maxCount) {
    count++;

    // read three sensors and convert the result to string:
    for (int analogPin = 0; analogPin < 3; analogPin++) {
      sensorVal[analogPin] = analogRead(analogPin);
    }
    rt_sprintf(buf, "%03d,%03d,%03d\n", sensorVal[0], sensorVal[1], sensorVal[2]);
  
    // open log file (create if doesn't exist or append if already existed):
    dataFile = open("datalog.txt", O_CREAT | O_RDWR | O_ACCMODE | O_APPEND);
  
    // if file opened successfully, then write to it:
    if (dataFile >= 0) {
      write(dataFile, buf, rt_strlen(buf));
      if (maxCount == count) {
        // flush data to SD:
        fsync(dataFile);
      }
      close(dataFile);
      // print to the serial port too:
      rt_kprintf(buf);
    } else {
      // if the file isn't open, pop up an error:
      rt_kprintf("*** error opening \"datalog.txt\" ***\n");
    }
  }

  rt_thread_sleep(RT_TICK_PER_SECOND);
}
