/**
 *   University of Washington
 *   ECE/CSE 474,  5/19/2021
 *   
 *   @file   final_project.ino
 *   @author    Dylan Hylander, Trevor Garrood
 *   @date      9-June-2021
 *   @brief   EE 474 Lab 4 Final Project
 *   
 *  This file creates a banking device using a Membrane switch module and LCD display as well as a servo for the user.
 *  
 */

// --------- Define Statements --------
#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal.h>
#include <Keypad.h>

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
#define NOTE_d4 6802 ///< 293 Hz
#define NOTE_e4 6079 ///< 329 Hz
#define NOTE_c4 7662 ///< 261 Hz
#define NOTE_c3 15385 ///< 130 Hz
#define NOTE_g3 10204 ///< 196 Hz
#define NO_NOTE 0 ///< pause between notes
#define D4 293 // 293 Hz
#define E4 329 // 329 Hz
#define C4 261 // 261 Hz
#define C3 130 // 130 Hz
#define G3 196 // 196 Hz

// Seven Segment Defines
#define DIGITAL_PIN_THREE 3
#define DIGITAL_PIN_TWO 2
#define DIGITAL_PIN_ONE 1
#define DIGITAL_PIN_ZERO 0
#define LED_INTERVAL 4
#define LED_INTERVAL_OFFSET 1
#define MAX_TIME 100
#define DIGITAL_LEN 4
#define SEG_LEN 8
#define SEV_SEG_LEN 7

// ATM Defines
#define INIT_STATE 0
#define PSWD_IN_STATE 1
#define MENU_STATE 2
#define SELECT_STATE 3
#define DEPOSIT_MENU_STATE 4
#define DEPOSIT_STATE 5
#define WTHDRW_MENU_STATE 6
#define WTHDRW_STATE 7
#define PSWD_MENU_STATE 8
#define PSWD_STATE 9
#define RESET_STATE 10
#define EXIT_MENU_STATE 11
#define EXIT_STATE 12
#define IDLE_STATE 13

// --------------- Variables --------------------

// Keypad Variables
const byte ROWS = 4; ///< four rows for the beypad
const byte COLS = 4; ///< four columns for the beypad
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {25, 27, 29, 31}; ///< connect to the row pinouts of the keypad
byte colPins[COLS] = {33, 35, 37, 39}; ///< connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

// ATM Variables
byte password[4] = {0, 0, 0, 0};
int passwordLoc = 0;
int keyInputCount = 0;
bool correctPassword = true; // prone to attacks
bool withdrawal = false;
bool deposit = false;
int currentState = IDLE_STATE;
int balance = 0;
String depositAmount = "";
int depositLoc = 0;
String withdrawAmount = "";
int withdrawLoc = 0;

const int rs = 32, en = 30, d4 = 28, d5 = 26, d6 = 24, d7 = 22;  ///< LCD Variables
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Seven Segment Variables
byte seven_seg_digits[10][7] = { { 1,1,1,1,1,1,0 },  // = 0
                                 { 0,1,1,0,0,0,0 },  // = 1
                                 { 1,1,0,1,1,0,1 },  // = 2
                                 { 1,1,1,1,0,0,1 },  // = 3
                                 { 0,1,1,0,0,1,1 },  // = 4
                                 { 1,0,1,1,0,1,1 },  // = 5
                                 { 1,0,1,1,1,1,1 },  // = 6
                                 { 1,1,1,0,0,0,0 },  // = 7
                                 { 1,1,1,1,1,1,1 },  // = 8
                                 { 1,1,1,0,0,1,1 }   // = 9
                               };
byte digitPins[] = {14, 11, 12, 13}; ///< Digit pins: D1, D2, D3, D4
byte segmentPins[] = {9, 2, 3, 5, 6, 8, 7, 4}; ///< Segment pins: A, B, C, D, E, F, G, DP
byte numberArray[] = {0, 0, 0, 0}; // Start at 0000

char TCNT2init= 130; ///< ISR Variable

// Task Initializers
void TaskBlink( void *pvParameters );
void TaskSongPlay( void *pvParameters );
void TaskKeypad( void *pvParameters );
void TaskServo( void *pvParameters );


/**
 * void setup()
 * @brief Arduino setup
 * @author Trevor Garrood, Dylan Hylander
 * 
 * Sets the output pins for this application depending on the connected device, including the seven segment display,
 * ISR, timer, LED, and speaker.
 */
void setup() {
  // Setup RT-3
  //randomSeed(l);
  
  //Setup for Seven Segment Display
  for (int i = 0; i < DIGITAL_LEN; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], HIGH); // common cathode: HIGH is off
  }
  for (int j = 0; j < SEG_LEN; j++) {
    pinMode(segmentPins[j], OUTPUT);
    digitalWrite(segmentPins[j], LOW); // common cathode: LOW is off
  }

  // setup for ISR
  DDRD |= 0b0000100;
  TCCR2A &= CLEAR;
  TCCR2B &= CLEAR;
  //Set WGM2 bits 3:0 to 0b1111 corresponding to "Fast PWM" with Top set to the value stored in OCR2A
  TCCR2A |= 0b00000011;
  TCCR2B |= 0b00011000;
  // Prescale 128 (Timer/Counter started)
  TCCR2B |= 0b00000101;
  // Set COM2A0 and COM2A1 to 0b01 to select the compare mode: "Toggle OC2A on Compare Match"
  TCCR2A |= 0b01000000;
  TIMSK2 |= (1<<TOIE2);
  TCNT2 = TCNT2init;

  // Create task for blinking an LED with a stack size of 128 and priority of 1.
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  // Create task for playing a song with a stack size of 128 and priority of 1.
  xTaskCreate(
    TaskSongPlay
    ,  "SongPlay"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Create task for an I/O keypad with a stack size of 800 and priority of 1.
  xTaskCreate(
    TaskKeypad
    ,  "Keypad"
    ,  800  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Create task for moving a servo with a stack size of 128 and priority of 1.
  xTaskCreate(
    TaskServo
    ,  "Servo"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  delay(500);
  
  vTaskStartScheduler();  // Starts the task tscheduler 
}

/**
 * void loop()
 * @author Trevor Garrood, Dylan Hylander
 * 
 * Does nothing as things are done in tasks.
 */
void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

/**
 * void TaskBlink(void *pvParameters)
 * @brief blinks an LED: on for 100 ms and off for 200 ms
 * @author Trevor Garrood, Dylan Hylander
 * @param {void *} pvParameters - parameters of the task
 * 
 * Blinks an off-board LED for 100 ms ON and 200 ms OFF. Uses pin 47 on the board.
 */
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

/**
 * void TaskSongPlay(void *pvParameters)
 * @brief plays a song 3 times
 * @author Trevor Garrood, Dylan Hylander
 * @param {void *} pvParameters - parameters of the task
 * 
 * Plays the theme from close encounters of the third kind 3 times before stopping the task. Uses pin 46 on the board and
 * uses a timer and Fast PWM as a counter.
 */
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
    if (taskTwoPlays == 3) {  // halts task if played theme 3 times
      currentState = INIT_STATE;
      vTaskSuspend(NULL);
    }
  }
}  

/**
 * void TaskKeypad(void *pvParameters)
 * @brief utilizes user input on a external keypad to change states
 * @author Trevor Garrood, Dylan Hylander
 * @param {void *} pvParameters - parameters of the task
 * 
 * Uses an external membrane switch to change between internal states in order to simulate a password-locked lock. It
 * uses 4 different states: deposit menu, withdraw menu, password menu, and reset state. It also uses an LCD display
 * in order to display money values and other inputted information.
 */
void TaskKeypad(void *pvParameters) {
  lcd.begin(16, 2);

  for (;;) {
    char key = keypad.getKey(); // Read the key
    if (currentState == IDLE_STATE) {  // if idle, do nothing
      
    } else if (currentState == INIT_STATE) {  // if initialized, ask for password
      lcd.print("Enter Pswd: ");
      currentState = PSWD_IN_STATE;
    } else if (currentState == PSWD_IN_STATE) {  // if password types, checks if it's correct
      if (key){
        lcd.print("*");
        if (password[keyInputCount] != (key - '0')) {
          correctPassword = false;
        }
        keyInputCount++;
        vTaskDelay(250 / portTICK_PERIOD_MS);
        if (keyInputCount == 4) {
          lcd.clear();
          if (correctPassword) {  // if correct password, changes state to menu
            lcd.print("Welcome");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            currentState = MENU_STATE;
          } else {
            lcd.print("Incorrect Pswd");  // if wrong password, resets back to beginning
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            currentState = RESET_STATE;
          }
        }
      }
    } else if (currentState == MENU_STATE) {  // if currently in menu state, asks for deposit or withdrawl or exit
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("A) Add B) Wthdrw");
      lcd.setCursor(0, 1);
      lcd.print("C) Pswd D) Exit");
      currentState = SELECT_STATE;
    } else if (currentState == SELECT_STATE) {  // if in selecting state, checks for which state to enter next
      if (key) {
        if (key == 'A') {
          currentState = DEPOSIT_MENU_STATE;
        } else if (key == 'B') {
          currentState = WTHDRW_MENU_STATE;
        } else if (key == 'C') {
          currentState = PSWD_MENU_STATE;
        } else if (key == 'D') {
          currentState = RESET_STATE;
        } else {
          lcd.clear();
          lcd.print("Invalid Input");
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          currentState = MENU_STATE;
        }
      }
    } else if (currentState == DEPOSIT_MENU_STATE) {  // if in deposit menu state, prints out deposit:
      lcd.clear();
      lcd.print("Deposit: ");
      currentState = DEPOSIT_STATE;
    } else if (currentState == DEPOSIT_STATE) {  // if in deposit state, adds input to current balance
      if (key){
        lcd.print(key);
        if (depositLoc == 6 || key == '#') {
          char buf[depositAmount.length()+1];
          depositAmount.toCharArray(buf, depositAmount.length()+1);
          int depositNum = atoi(buf);
          balance += depositNum;
          lcd.clear();
          lcd.print("Thank you");
          Serial.print("Current Balance: ");
          Serial.println(balance);
          deposit = true;
          vTaskDelay(1500 / portTICK_PERIOD_MS);
          currentState = EXIT_MENU_STATE;
          depositAmount = "";
          depositLoc = 0;
        } else if (depositLoc < 6) {
          depositAmount += key;
          depositLoc++;
        }
      }
    } else if (currentState == WTHDRW_MENU_STATE) {  // if in withdraw menu state, prints withdraw:
      lcd.clear();
      lcd.print("Withdraw: ");
      currentState = WTHDRW_STATE;
    } else if (currentState == WTHDRW_STATE) {  // if in withdraw state, subtracts given amount from balance
      if (key){
        lcd.print(key);
        if (withdrawLoc == 6 || key == '#') {
          char buf[withdrawAmount.length()+1];
          withdrawAmount.toCharArray(buf, withdrawAmount.length()+1);
          int withdrawNum = atoi(buf);
          if (withdrawNum <= balance) {
            balance -= withdrawNum;
            lcd.clear();
            lcd.print("Thank you");
            Serial.print("Current Balance: ");
            Serial.println(balance);
            withdrawal = true;
          } else {
            lcd.clear();
            lcd.print("Insufficient $");
          }
          vTaskDelay(1500 / portTICK_PERIOD_MS);
          currentState = EXIT_MENU_STATE;
          withdrawAmount = "";
          withdrawLoc = 0;
        } else if (withdrawLoc < 6) {
          withdrawAmount += key;
          withdrawLoc++;
        }
      }
    } else if (currentState == PSWD_MENU_STATE) {  // if in password menu state, prints out new pswd:
      lcd.clear();
      lcd.print("New Pswd: ");
      currentState = PSWD_STATE;
    } else if (currentState == PSWD_STATE) {  // if in password state, changes the password to given input and gives exit menu
      if (key){
        if (passwordLoc < 4) {
          lcd.print(key);
          password[passwordLoc] = key - '0';
          passwordLoc++;
        } else if (passwordLoc == 4 && key == '#') {
         lcd.clear();
         lcd.print("Password Set");
         vTaskDelay(1500 / portTICK_PERIOD_MS);
         currentState = EXIT_MENU_STATE;
         passwordLoc = 0;
        }
      }
    } else if (currentState == EXIT_MENU_STATE) {  // if in exit menu state, asks if should exit menu
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("A) Continue");
      lcd.setCursor(0, 1);
      lcd.print("B) Exit");
      currentState = EXIT_STATE;
    } else if (currentState == EXIT_STATE) {  // if in exit state, checks given input and either goes to menu or resets
      if (key){
        if (key == 'A') {
          currentState = MENU_STATE;
        } else if (key == 'B') {
          currentState = RESET_STATE;
        }
      }
    } else if (currentState == RESET_STATE) {  // if in reset state, sets deposit/withdraw to 0 and goes to init state
      lcd.clear();
      keyInputCount = 0;
      correctPassword = true;
      currentState = INIT_STATE;
      depositAmount = "";
      depositLoc = 0;
      withdrawAmount = "";
      withdrawLoc = 0;
      passwordLoc = 0;
    }
  }
}

/**
 * void TaskServo(void *pvParameters)
 * @brief controls an external servo
 * @author Trevor Garrood, Dylan Hylander
 * @param {void *} pvParameters - parameters of the task
 * 
 * Uses an external servo connected to port 53 and "opens" (high for 1.5s, low for 1.5s) if either deposits or withdrawing.
 */
void TaskServo(void *pvParameters) {
  int pos = 0;
  pinMode(53, OUTPUT);
  for (;;) {
    if (deposit || withdrawal) {
      digitalWrite(53, HIGH);
      vTaskDelay(1500 / portTICK_PERIOD_MS);
      digitalWrite(53, LOW);
      vTaskDelay(1500 / portTICK_PERIOD_MS);
      deposit = false;  // resets deposit and withdrawl back to false to essentially change internal state
      withdrawal = false;
    }
  }
}

/*
 * Sets up ISR vector for TIMER2
 */
volatile static int isrCount;
byte digitsPos[4] = {DIGITAL_PIN_THREE, DIGITAL_PIN_TWO, DIGITAL_PIN_ONE, DIGITAL_PIN_ZERO};

ISR(TIMER2_OVF_vect) { 
  TCNT2 = TCNT2init;
  static int digitPos;
  static int state;
  // changes balance into an array
  intToNumber(balance, numberArray);
  if (isrCount == 0) {
    if (++digitPos >= 4) {
      digitPos = 0;
    }
    showDigit(numberArray[digitPos], digitsPos[digitPos]);  // checks digit position
  }
  // hides all digits if isr count is 3
  if (isrCount == 3) {
    hideDigit(DIGITAL_PIN_ZERO);
    hideDigit(DIGITAL_PIN_ONE);
    hideDigit(DIGITAL_PIN_TWO);
    hideDigit(DIGITAL_PIN_THREE);
  }
  if (isrCount == 4) {
    isrCount = -1;
  }
  isrCount++;
};

/*--------------------------------------------------*/
/*---------------- Helper Functions ----------------*/
/*--------------------------------------------------*/

/**
 * void showDigit(int number, int digitPin)
 * @brief Displays a number on the Seg7 display
 * @author Trevor Garrood, Dylan Hylander
 * 
 * Displays a number on the Seg7 display at a specific digitPin location
 * @params {integer} number - a number 0-9 to be displayed on the seg7
 * @params {integer} digitPin - specific digit pin 0-3 to be written to
 */
void showDigit(int number, int digitPin) {
  digitalWrite(digitPins[digitPin], LOW);
  for (int i = 0; i < SEV_SEG_LEN; i++) {
    digitalWrite(segmentPins[i], seven_seg_digits[number][i]);
  }
}

/**
 * void hideDigit(int digit)
 * @brief Hides a digit and allows for the others to be written to
 * @author Trevor Garrood, Dylan Hylander
 * 
 * Hides a digit and allows for the others to be written to. HIGH is OFF
 * @params {integer} digit - a digit 0-3 designating which digit on seg7 to turn off
 */
void hideDigit(int digit) {
  digitalWrite(digitPins[digit], HIGH);
}

/**
 * void intToNumber(int num, byte numberArray[])
 * @brief converts and integer number to an array of digits
 * @author Trevor Garrood, Dylan Hylander
 * 
 * converts and integer number to an array of digits
 * @params {integer} num - a number 0-9999 to be converted to an array
 * @params {byte} numberArray - an array of digits converted from the number passed in 
 */
void intToNumber(int num, byte numberArray[]) {
  for (int i = 0; i < 4; i++) {
    numberArray[i] = num % 10;
    num /= 10;
  }
}
