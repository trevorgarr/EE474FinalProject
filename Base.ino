/*
 * Trevor Garrood and Dylan Hylander
 * EE 474 Lab 4
 */

#include <Arduino_FreeRTOS.h>

// Task 1 and 2 defines
#define DELAY_TASK_1 250
#define BIT_3 1<<3
#define BIT_2 1<<2
#define BIT_1 1<<1
#define BIT_0 1<<0
#define CLR_2 ~(1<<2)
#define CLR_1 ~(1<<1)
#define CLR_0 ~(1<<0)
#define CLEAR 0b00000000
#define DELAY_TASK_2 1000

// Music note defines
#define NOTE_d4 6802 // 293 Hz
#define NOTE_e4 6079 // 329 Hz
#define NOTE_c4 7662 // 261 Hz
#define NOTE_c3 15385 // 130 Hz
#define NOTE_g3 10204 // 196 Hz
#define NO_NOTE 0 // pause between notes
#define D4 293 // 293 Hz
#define E4 329 // 329 Hz
#define C4 261 // 261 Hz
#define C3 130 // 130 Hz
#define G3 196 // 196 Hz

void TaskBlink( void *pvParameters );
void TaskSongPlay( void *pvParameters );

void setup() {
  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskSongPlay
    ,  "SongPlay"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  //  (note how the above comment is WRONG!!!)
  vTaskStartScheduler();
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters) {
  //pinMode(47, OUTPUT);
  DDRL |= BIT_2; // pin 47

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(47, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100ms
    digitalWrite(47, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for 200ms
  }
}

void TaskSongPlay(void *pvParameters) {
  DDRL |= BIT_3; // pin 46 -> OC5A
  //pinMode(46, OUTPUT);

  int taskTwoPlays = 0;

  //setup for clock for speaker
  TCCR5A &= CLEAR;
  TCCR5B &= CLEAR;
  //Set WGM4 bits 3:0 to 0b1111 corresponding to "Fast PWM" with Top set to the value stored in OCR5A
  TCCR5A |= 0b00000011;
  TCCR5B |= 0b00011000;
  //Set timer 5 prescaler to 8?
  TCCR5B |= 0b00000010;
  // Set COM5A0 and COM5A1 to 0b01 to select the compare mode: "Toggle OC5A on Compare Match"
  // 5A corresponds to OC5A, which is pin 64 on the mega 
  TCCR5A |= 0b01000000;
  for (;;) {
    OCR5A = NOTE_d4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_e4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_c4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_c3;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NOTE_g3;
    vTaskDelay( (DELAY_TASK_2 * 4) / portTICK_PERIOD_MS ); // wait for 200ms
    OCR5A = NO_NOTE;
    vTaskDelay( (DELAY_TASK_2 * 1.5) / portTICK_PERIOD_MS ); // wait for 200ms
    taskTwoPlays++;
    if (taskTwoPlays == 3) {
      vTaskSuspend(NULL);
    }
  }
}  



/*--------------------------------------------------*/
/*---------------- Helper Functions ----------------*/
/*--------------------------------------------------*/

/* @brief synchronize the loop to wait for d ms.
 *    Assumes ISR is called every 1ms. 
 *    @note all tasks need to complete in < 1ms for d
 *       to be accurate. 
 */
//void synch_sched(int d){
//  int j=0; 
//  while(d>0) {
//    while(isr_flag) { // isr should break this loop
//      j = j + 1 ;   // burn some time
//      j = j * 23 ;
//      j = j / 8; 
//      j = j + 1 ;
//      j = j + 1 ;
//     }
//     isr_flag = 1; 
//     d--;
//  }
//}
