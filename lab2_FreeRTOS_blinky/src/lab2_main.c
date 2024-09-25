/**
 * Yall Here is a good video series that helps a ton
 * 
 * https://www.youtube.com/watch?v=OPrcpbKNSjU (this is oart 3 but if you want can start with part 1)
 * 
 */


// Include FreeRTOS headers.
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "lib_ee152.h"

bool do_blink_red=1, do_blink_grn=1;

#define BLINK_RED_DELAY ( 500 / portTICK_PERIOD_MS )
// Keep blinking as long as do_blink_red==true.
void task_blink_red( void * pvParameters )
{
    // Hook up the red LED to Nano D12.
    pinMode(D12, "OUTPUT");

    // write this code

    for ( ;; ) {
	if (do_blink_red) {
	    // We want to us osDelay() 
    }

    // From 

}

// Keep blinking as long as do_blink_grn==true.
#define BLINK_GRN_DELAY ( 500 / portTICK_PERIOD_MS )
void task_blink_grn( void * pvParameters )
{
    // The green LED is at Nano D13, or PB3.
    pinMode(D13, "OUTPUT");

  
}

// This task keeps reading the UART forever. It sets the globals
// do_blink_grn and do_blink_red to communicate with the other tasks.
void task_uart (void *pvParameters) {
    const char prompt[] = "R=red, G=green, B=both, N=neither: ";
    char rxByte, buf[40];
    while (1) {
	serial_write(USART2, prompt);
	rxByte = serial_read (USART2);
	int red  = (rxByte == 'R' || rxByte == 'r');
	int grn  = (rxByte == 'G' || rxByte == 'g');
	int both = (rxByte == 'B' || rxByte == 'b');
	do_blink_red = red || both;
	do_blink_grn = grn || both;
	strcpy (buf, "Red=");   strcat (buf, (do_blink_red?"on":"off"));
	strcat (buf, ", grn=");	strcat (buf, (do_blink_grn?"on\n\r":"off\n\r"));
	serial_write(USART2, buf);
    }
}

int main()
{
    // The default clock is 4MHz, which is more than fast enough for LEDs.
    //clock_setup_16MHz();		// 16 MHz
    clock_setup_80MHz();		// 80 MHz
    serial_begin (USART2);

    // Create tasks.
    TaskHandle_t task_handle_uart = NULL;
    BaseType_t OK = xTaskCreate (
	    task_uart,
	    "Decide which LEDs to blink",
	    100, // stack size in words
	    NULL, // parameter passed into task, e.g. "(void *) 1"
	    tskIDLE_PRIORITY+1, // priority
	    &task_handle_uart);
    if (OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_red = NULL;
    ... create task_blink_red ...
    OK = xTaskCreate ( ... );
    if (OK != pdPASS) for ( ;; );

    TaskHandle_t task_handle_grn = NULL;
    ... create task_blink_red ...
    OK = xTaskCreate ( ... );
    if (OK != pdPASS) for ( ;; );

    vTaskStartScheduler();
}
