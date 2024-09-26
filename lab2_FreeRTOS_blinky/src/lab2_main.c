/**
 * Yall Here is a good video series that helps a ton
 *
 * https://www.youtube.com/watch?v=OPrcpbKNSjU (this is oart 3 but if you want
 * can start with part 1)
 *
 */

// Include FreeRTOS headers.
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "lib_ee152.h"
#include "task.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

bool do_blink_red = 1, do_blink_grn = 1;
bool blink_alt = 0;
int mode = 0;

#define BLINK_RED_DELAY (500 / portTICK_PERIOD_MS)
// Keep blinking as long as do_blink_red==true.
void task_blink_red(void *pvParameters) {
  // Hook up the red LED to Nano D12.
  pinMode(D12, "OUTPUT");

  // write this code
  const int cycle_time = 1000; // 1000ms = 1 second
  const int red_half_period = cycle_time / 4; // ~250 (2 cycles per second)

  for (;;) {
    if (do_blink_red) {
      // int mode = *(int)*pvParameters;   // casting void pointer as int type, setting to 
      
      switch (mode)
      {
      case 0:
        // Both case
        toggle_red_LED();
        vTaskDelay(BLINK_RED_DELAY);
        break;
      case 1:
        // Alternate case
        digitalWrite(D12, !blink_alt);
        vTaskDelay(BLINK_RED_DELAY);
        break;
      case 2:
        // 2 on 3 case
        toggle_red_LED();
        vTaskDelay(red_half_period / portTICK_PERIOD_MS);
        break;
      default:
        break;
      }
    }
  }

  // From  the video (just terminates task if some how loop exits and is good
  // practice in every thread function)
  osThreadTerminate(NULL);
}

// Keep blinking as long as do_blink_grn==true.
#define BLINK_GRN_DELAY (500 / portTICK_PERIOD_MS)
void task_blink_grn(void *pvParameters) {

  // The green LED is at Nano D13, or PB3.
  pinMode(D13, "OUTPUT");

  // write this code
  const int cycle_time = 1000; // 1000ms = 1 second
  const int green_half_period = cycle_time / 6; // ~167ms (3 cycles per second)
  // int mode = *(int*)pvParameters;
  

  
  
  for (;;) {

    if (do_blink_grn) {
      // serial_write(USART2, "green on");
      // digitalWrite (D13, 1);
      // vTaskDelay (BLINK_GRN_DELAY);
      // serial_write(USART2, "green off");
      // digitalWrite (D13, 0);
      // vTaskDelay (BLINK_GRN_DELAY);
      // int mode = *(int*)pvParameters;   // casting void pointer as int type, setting to 
      // Debugging
      // char mode_str[50];
      // sprintf(mode_str, "Green mode value: %d\n", mode);
      // serial_write(USART2, mode_str);
      
      switch (mode)
      {
      case 0:
        // Both case
        toggle_grn_LED();
        vTaskDelay(BLINK_GRN_DELAY);
        break;
      case 1:
        // Alternate case
        digitalWrite(D13, blink_alt);
        blink_alt = !blink_alt;
        vTaskDelay(BLINK_GRN_DELAY);
        break;
      case 2:
        // 2 on 3 case
        toggle_grn_LED();
        vTaskDelay(green_half_period / portTICK_PERIOD_MS);
        break;
      default:
        break;
      }
      
    }
  }

  // From  the video (just terminates task if some how loop exits and is good
  // practice in every thread function)
  vTaskDelay(0);
  // osThreadTerminate(NULL);
}

// This task keeps reading the UART forever. It sets the globals
// do_blink_grn and do_blink_red to communicate with the other tasks.
void task_uart(void *pvParameters) {
  const char prompt[] = "R=red, G=green, B=both, N=neither: ";
  char rxByte, buf[40];
  while (1) {
    serial_write(USART2, prompt);
    rxByte = serial_read(USART2);
    int red = (rxByte == 'R' || rxByte == 'r');
    int grn = (rxByte == 'G' || rxByte == 'g');
    int both = (rxByte == 'B' || rxByte == 'b');
    do_blink_red = red || both;
    do_blink_grn = grn || both;
    strcpy(buf, "Red=");
    strcat(buf, (do_blink_red ? "on" : "off"));
    strcat(buf, ", grn=");
    strcat(buf, (do_blink_grn ? "on\n\r" : "off\n\r"));
    serial_write(USART2, buf);
  }
}

int main() {
  // The default clock is 4MHz, which is more than fast enough for LEDs.
  // clock_setup_16MHz();		// 16 MHz
  clock_setup_80MHz(); // 80 MHz
  serial_begin(USART2);
  serial_write(USART2, "In main()\r\n"); // TODO why doesnt this shit print

  // Create tasks.
  /* COMMENT BACK IN FOR UART
  TaskHandle_t task_handle_uart = NULL;
  BaseType_t OK =
      xTaskCreate(task_uart, "Decide which LEDs to blink",
                  100,  // stack size in words
                  NULL, // parameter passed into task, e.g. "(void *) 1"
                  // tskIDLE_PRIORITY+1, // priority
                  tskIDLE_PRIORITY, &task_handle_uart);
  if (OK != pdPASS)
    for (;;)
      ;
  */

  TaskHandle_t task_handle_red = NULL;
  // TODO: Ask why are parameter passing isnt working ie *(int*)pvParameters
  // const int mode_int = 0;
  // ... create task_blink_red ...
  // BaseType_t OK = xTaskCreate(task_blink_red, "Blink Red LED", 100, (void*)&mode_int, tskIDLE_PRIORITY, &task_handle_red);
  BaseType_t OK = xTaskCreate(task_blink_red, "Blink Red LED", 100, NULL, tskIDLE_PRIORITY, &task_handle_red);
  if (OK != pdPASS)
    for (;;)
      ;

  TaskHandle_t task_handle_grn = NULL;
  // ... create task_blink_red ...
  // OK = xTaskCreate ( ... );
  // if (OK != pdPASS) for ( ;; );
  OK = xTaskCreate(task_blink_grn, "Blink GRN LED", 100, NULL , tskIDLE_PRIORITY,
                   &task_handle_grn);
  if (OK != pdPASS)
    for (;;)
      ;

  vTaskStartScheduler();
}
