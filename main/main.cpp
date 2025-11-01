/*
  The main entry point for this program
*/

#include <stdio.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>

#include "p_scanner.h"

extern "C" void app_main(void) {
  printf("Start\n");

  // P_Scanner contains the actual logic of this program
  P_Scanner* scanner;
  // Using Heap memory to avoid stack overflow
  scanner = new P_Scanner();
  
  scanner->init();
  
  printf("Scanner Initialized\n");

  vTaskDelay(pdMS_TO_TICKS(101));
  scanner->init_Buttons();
  vTaskDelay(pdMS_TO_TICKS(100));

  printf("Buttons Initialized\n");

  // While loop to allow main task to do something instead of nothing
  while(1)
    vTaskDelay(pdMS_TO_TICKS(100));
}
