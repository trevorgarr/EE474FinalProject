/**
 *   University of Washington
 *   ECE/CSE 474,  5/19/2021
 *   
 *   @file   RT_03.ino
 *   @author    Dylan Hylander, Trevor Garrood
 *   @date      9-June-2021
 *   @brief   EE 474 Lab 4 RT-03
 *   
 *  This file combines blinking LED light ------------
 *  
 *  Acknowledgements: "kosme" on GitHub for arduinoFFT example code
 *  https://github.com/kosme/arduinoFFT/blob/master/Examples/FFT_02/FFT_02.ino
 */

#include <stdio.h>
#include <stdlib.h>
#include <Arduino_FreeRTOS.h>
#include <arduinoFFT.h>
#include <queue.h>

// Task 1/2 defines
#define BIT_3 1<<3
#define BIT_2 1<<2
#define BIT_1 1<<1
#define BIT_0 1<<0
#define CLR_2 ~(1<<2)
#define CLR_1 ~(1<<1)
#define CLR_0 ~(1<<0)
#define CLEAR 0b00000000
#define DELAY_TASK_2 1000
#define DELAY_50 50
#define DELAY_100 100
#define DELAY_150 150
#define DELAY_200 200

// Music note defines
#define NOTE_d4 6802 ///< 293 Hz
#define NOTE_e4 6079 ///< 329 Hz
#define NOTE_c4 7662 ///< 261 Hz
#define NOTE_c3 15385 ///< 130 Hz
#define NOTE_g3 10204 ///< 196 Hz
#define NO_NOTE 0 ///< pause between notes
#define SONG_STOP 3  // play song 3 times before stopping

// Task 3/4 defines
#define ARRAY_SIZE 64  // Size of random int array
#define SERIAL_BITS 19200  // number of bits per second for serial communication
#define TOTAL_FFT 5  // total number of FFT's to calculate

/*
   Task 3/4: These values can be changed in order to evaluate the functions
*/
const double sampling = 40;
const uint8_t amplitude = 4;
uint8_t exponent;
const double startFrequency = 2;
const double stopFrequency = 16.4;
const double step_size = 0.1;

/*
  Task 3/4: These are the input and output vectors
            Input vectors receive computed results from FFT
*/
double vReal[ARRAY_SIZE];
double vImag[ARRAY_SIZE];

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
static double (*arrayPtr)[ARRAY_SIZE];  // pointer to random int array
static long *timePtr;  // pointer to avg time for a FFT

static int numBlinks = 0;  // number of blink cycles
static int numSongs = 0;  // number of song cycles
static int numFFT = 0;  // number of FFT calculations 

// define two tasks for Blink & SongPlay
void TaskBlink( void *pvParameters );
void TaskSongPlay( void *pvParameters );

//  declare the Queue handles
static QueueHandle_t    queue1;
static QueueHandle_t    queue2;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 19200 bits per second:
  Serial.begin(SERIAL_BITS);
  while (!Serial) ;  // wait for it to comeup

  // Task 1 Setup
  DDRL |= BIT_2;  //pinMode(47, OUTPUT);

  // Task 2 Setup
  DDRL |= BIT_3;  // pin 46 -> OC5A //pinMode(46, OUTPUT);

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
  
  // FFT setup
  srand(0);  // Set rand seed
  // initialize array in memory all set to random doubles
  double d[ARRAY_SIZE] = {0};
  arrayPtr = &d;  // intialize global array pointer to array d of random ints
  
  // Set FFT array values to random integers
  for(int i = 0 ; i < ARRAY_SIZE ; i++ ) {
    d[i] = rand();
    // Serial.print("i: ");  Serial.print(i);  Serial.print("   num: ");  Serial.println(d[i]);
  }

  exponent = FFT.Exponent(ARRAY_SIZE);

  //  Set up Queues
  queue1 = xQueueCreate(ARRAY_SIZE, ARRAY_SIZE * sizeof(double));
  queue2 = xQueueCreate(ARRAY_SIZE, ARRAY_SIZE * sizeof(double));

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  2000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //task pointer "task tag"

  xTaskCreate(
    TaskSongPlay
    ,  "SongPlay"
    ,  2000  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  xTaskCreate(
    TaskSetupFFT
    ,  "SetupFFT"
    ,  1000  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  xTaskCreate(
    TaskFFT
    ,  "FFT"
    ,  1000  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  NULL );

  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*---------------------- Tasks ---------------------*/

/*
  Blink
  Turns on an LED on for 100 ms, then off for 200 ms, repeatedly.
*/
void TaskBlink(void *pvParameters) { // This is a task.
  vTaskDelay( DELAY_50 / portTICK_PERIOD_MS ); // wait for 50 ms
  
  for (;;) {  // A Task shall never return or exit.
    numBlinks++;
    digitalWrite(47, HIGH);// PORTL |= BIT_2;   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( DELAY_100 / portTICK_PERIOD_MS ); // wait for 100 ms
    PORTL &= CLR_2;    // turn the LED off by making the voltage LOW
    vTaskDelay( DELAY_200 / portTICK_PERIOD_MS ); // wait for 200 ms
    // ------------------------DELETE-----------------------------------------------------------------------------------------------------//
    if (numBlinks == 50) {  // blink LED 10 times before stopping
      vTaskSuspend(NULL);
    }
     // -----------------------------------------------------------
  }
}

/*
// Test HWM for blink == 50 w/ 128 stack size
/*
void TaskBlink(void *pvParameters) {
  vTaskDelay( 50 / portTICK_PERIOD_MS ); // wait for 50 ms
  UBaseType_t stackHighH20, SH2;
  // (void) pvParameters;  // allocate stack space for params
  
  for (;;) {  // A Task shall never return or exit.
    numBlinks++;  // count the blink cycles
    for(int i = 0; i < 2; i++) {
      digitalWrite(47, HIGH);   // turn the LED on (HIGH is the voltage level)
      vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 ms
      digitalWrite(47, LOW);    // turn the LED off by making the voltage LOW
      vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for 200 ms
    }
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for 1000 ms
    if (numBlinks > 2)  break;
  }

  vTaskDelay( 250 / portTICK_PERIOD_MS ); // wait for 250 ms
  SH2 = uxTaskGetStackHighWaterMark(NULL);

  Serial.print("Blink Stack HWM: "); ; Serial.println(SH2);
  numBlinks = -1;    // a signal to other tasks to report their stack HWM. 
  for(;;)  {  // get stuck here with solid LED
      digitalWrite(47, HIGH);  // solid light
      vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 ms
  }
}
*/

/*
  SongPlay
  Plays close encounters of the third kind theme
*/
void TaskSongPlay(void *pvParameters) {
  vTaskDelay( DELAY_100 / portTICK_PERIOD_MS ); // wait for 100 ms
    
  for (;;) {
    numSongs++;  // count the number of times song is played
    OCR5A = NOTE_d4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_e4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_c4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_c3;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_g3;
    vTaskDelay( (DELAY_TASK_2 * 4) / portTICK_PERIOD_MS ); // wait for 4s
    OCR5A = NO_NOTE;
    vTaskDelay( (DELAY_TASK_2 * 1.5) / portTICK_PERIOD_MS ); // wait for 1.5s
    if (numSongs == 3) {  // play theme 3 times before stopping
      vTaskSuspend(NULL);
    }
  }
}

// Test HWM for song play == 50 w 128 stack size
/*
void TaskSongPlay(void *pvParameters) {
  vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 ms
  UBaseType_t stackHighH20, SH2;
  
  for (;;) {
    numSongs++;  // count the songs 
    OCR5A = NOTE_d4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_e4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_c4 / 2;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_c3;
    vTaskDelay( DELAY_TASK_2 / portTICK_PERIOD_MS ); // wait for 1s
    OCR5A = NOTE_g3;
    vTaskDelay( (DELAY_TASK_2 * 4) / portTICK_PERIOD_MS ); // wait for 4s
    OCR5A = NO_NOTE;
    vTaskDelay( (DELAY_TASK_2 * 1.5) / portTICK_PERIOD_MS ); // wait for 1.5s
    taskTwoPlays++;
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for 1000 ms
    if (numSongs > 2)  break; // break after 3 songs;
  }

  vTaskDelay( 250 / portTICK_PERIOD_MS ); // wait for 250 ms
  SH2 = uxTaskGetStackHighWaterMark(NULL);

  Serial.print("Song Stack HWM: "); ; Serial.println(SH2);
  numSongs = -1;    // a signal to other tasks to report their stack HWM. 
  for(;;)  {  // get stuck here with no sound
      OCR5A = NO_NOTE;  // no sound
      vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 ms
  }
}
*/

/*
  SetupFFT
  Creates an array of random integers and a FreeRTOS Queue
*/
void TaskSetupFFT(void *pvParameters) {
  vTaskDelay( DELAY_150 / portTICK_PERIOD_MS ); // wait for 150 ms
  
  // put into queue
  xQueueSendToBack(queue1, arrayPtr, 0);

  // take out from queue
  xQueueReceive(queue2, timePtr, 0);

  long avgTime = *timePtr / TOTAL_FFT;
  Serial.print("Average Wall Clock Time Per FFT: ");  Serial.println(avgTime);  // Prints time to calculate FFT's
}

/*
  FFT
  Caclculates 5 FFT's for an array of random integers from a FreeRTOS Queue
*/
void TaskFFT(void *pvParameters) {
  vTaskDelay( DELAY_200 / portTICK_PERIOD_MS ); // wait for 200 ms
  long startTicks = xTaskGetTickCount();  // gets number of ticks since scheduler started
  for (;;) {
    numFFT++;  // number of FFT's calculated
    xQueueReceive(queue1, arrayPtr, 0);
    for (int i = 0; i < TOTAL_FFT; i++) {  // stop after 5 FFT's
      for(double frequency = startFrequency; frequency<=stopFrequency; frequency+=step_size)
      {
        // Build raw data 
        double cycles = (((ARRAY_SIZE-1) * frequency) / sampling);
        for (uint16_t i = 0; i < ARRAY_SIZE; i++)
        {
          vReal[i] = *arrayPtr[i];
          vImag[i] = 0; //Reset the imaginary values vector for each new frequency
        }
        FFT.Windowing(vReal, ARRAY_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data
        FFT.Compute(vReal, vImag, ARRAY_SIZE, exponent, FFT_FORWARD); // Compute FFT 
        FFT.ComplexToMagnitude(vReal, vImag, ARRAY_SIZE); // Compute magnitudes
        double x = FFT.MajorPeak(vReal, ARRAY_SIZE, sampling);
      }
    }
    long taskTime = xTaskGetTickCount() - startTicks;  // number of ticks since scheduler started
    timePtr = &taskTime;
    xQueueSendToBack(queue2, timePtr, 0);
  }
}
