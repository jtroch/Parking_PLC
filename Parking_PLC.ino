//test
#include <Controllino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <ArduinoJson.h>
//#include ".\FreeRTOSConfig.h"
#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <avr/wdt.h>

#define MAX_REPLY_LENGTH 400
#define MAX_BODY_LENGTH  300

#define MSG_TYPE_ENTRY        0
#define MSG_TYPE_EXIT         1
#define MSG_TYPE_SETTINGS     2
#define MSG_TYPE_ID           3

#define MSG_IDKEY_API_PATH       "/api.domain.com/idkey"

// REST client to server @ fixed IP address
EthernetClient httpClient;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte local_ip[] = { 192, 168, 0, 2 };
const char* serverIP = "192.168.0.1";
char serverName[] = "www.google.com";
const int serverPort = 3000;

int  ApiId=0;
int  ApiKey=0;
char httpApiPathBase[50];

// Global volatile variables, communication between tasks
// Are safe because the are 8-bit type and volatile

volatile bool bAccessAllowed=false;        // true if reply form server is received
volatile bool bEntrySequence=false;        // true during 'entry' event, necessary to trigger timeout
volatile bool bExitSequence=false;         // true during 'exit'  event, necessary to trigger timeout
volatile bool bOpenBarrier=false;          // true when barrier should be opened
volatile bool bReplyReceived=false;        // true when barrier should be opened
volatile bool GotApiIdKey=false;       // true when ID and key are exchanged

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;        // mutex to lock Serial line
SemaphoreHandle_t xTxRxSemaphore;          // semaphore to ync send and receive tasks
SemaphoreHandle_t xSettingsMutex;          // mutex to lock settings struct

// define Tasks 
void TaskRunningLeds(void *pvParameters);          // Task showing running LED
void TaskHttpReceive(void *pvParameters);          // Task receiving msg from server
void TaskVehicleDetection(void *pvParameters);     // Task detecting vehicles
void TaskBarrierControl(void *pvParameters);       // Task for opening the barrier
void TaskHttpGetSettings(void *pvParameters);         // Task for getting settings from server
void TaskHttpGetIdKey(void *pvParameters);               // Task for getting ID en key

// Timer and callbacks
TimerHandle_t xEntryLoopTimer;
void fEntryTimerCallback(TimerHandle_t pxTimer);              // callback executed to debounce entry loop
TimerHandle_t xExitLoopTimer;
void fExitTimerCallback(TimerHandle_t pxTimer);               // callback executed to debounce entry loop
TimerHandle_t xHttpEntryResponseTimer;
void xHttpEntryResponseTimerCallback(TimerHandle_t pxTimer);  // callback executed to check timeout on entry loop
TimerHandle_t xHttpExitResponseTimer;
void xHttpExitResponseTimerCallback(TimerHandle_t pxTimer);   // callback executed to check timeout on exit loop
TimerHandle_t xBarrierOpenTimer;
void fBarrierOpenCallback(TimerHandle_t pxTimer);             // callback to open barrier for a predefined time

/*--------------------------------------------------*/
/*-------------------- Structs- --------------------*/
/*--------------------------------------------------*/

// tasks
typedef enum 
{
  PARKINGTASK_LEDS,
  PARKINGTASK_DETECTION,
  PARKINGTASK_BARRIER,
  PARKINGTASK_HTTP_RX,
  PARKINGTASK_HTTP_SETTINGS,
  NUM_PARKINGTASKS
} ParkingTasks_t;
TaskHandle_t tasks[NUM_PARKINGTASKS];

// different types of POST messages
typedef enum 
{
    MSG_ENTRY,
    MSG_EXIT,
    MSG_SETTINGS,
    MSG_ID
} MessageType_t;

// Settings variables, communicated by SETTINGS POST
typedef struct 
{
    int barrierOpen;
    int autonomous;
    int loopTimeout;
    int replyTimeout;
    int barrierPulseLength;
    int outputR5;
} SWSettings_t;

SWSettings_t Settings = {0, 0, 2000, 3000, 1000, 0};

/*--------------------------------------------------*/
/*----------  Auxiliary Functions ------------------*/
/*--------------------------------------------------*/

static void printWatermark(TaskHandle_t task)
{
  UBaseType_t mark = uxTaskGetStackHighWaterMark(task);
  Serial.print("T");
  Serial.println((int)mark);
}

void debugPrint(char * text)
{
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 0) == pdTRUE )
    {
        Serial.print(text);
        xSemaphoreGive(xSerialSemaphore);
    }     
}

void debugPrintln(char * text)
{
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 0) == pdTRUE )
    {
        Serial.println(text);
        xSemaphoreGive(xSerialSemaphore);
    }     
}

void debugPrint(int num)
{
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 0) == pdTRUE )
    {
        Serial.print(num);
        xSemaphoreGive(xSerialSemaphore);
    }     
}

void debugPrintln(int num)
{
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t) 0) == pdTRUE )
    {
        Serial.println(num);
        xSemaphoreGive(xSerialSemaphore);
    }     
}

char * timeString(void)
{
   int d, mo, y, h, mi, s;
   char timestring[30];
  
   d = Controllino_GetDay( ); 
   mo = Controllino_GetMonth();   
   y = Controllino_GetYear(); 
   h = Controllino_GetHour(); 
   mi = Controllino_GetMinute();
   s = Controllino_GetSecond(); 
 
   sprintf(timestring, "%02i/%02i/%02i %02i:%02i:%02i", d, mo, y, h, mi, s);
   debugPrint(timestring);

   return timestring;
} 

long int getTimeValue(void)
{
   long int  h, mi, s;
   long int getTimeValue;
   char buf[6];
  
   h = Controllino_GetHour(); 
   mi = Controllino_GetMinute();
   s = Controllino_GetSecond(); 

   getTimeValue = (h*10000) + (mi*100) + s;

   return getTimeValue;
} 

long int getDateValue(void)
{
   long int  y, mo, d;
   long int getDateValue;
  
   y = Controllino_GetYear(); 
   mo = Controllino_GetMonth();
   d = Controllino_GetDay(); 

   getDateValue = (d*10000) + (mo*100) + y;

   return getDateValue;
} 

int updateTime(long int tvalue, long int dvalue)
{
   long int  y, mo, d, h, mi, s;
   char buf[10];

   d  = dvalue / 10000;
   mo = (dvalue / 100) % 100;
   y  = dvalue % 100;

   h  = tvalue / 10000;
   mi = (tvalue / 100) % 100;
   s  = tvalue % 100;

   //Serial.print("# "); Serial.print(d); Serial.print("/"); Serial.print(mo); Serial.print("/"); Serial.print(y); Serial.print(" ");
   //                    Serial.print(h); Serial.print(":"); Serial.print(mi); Serial.print(":"); Serial.print(s); Serial.println(" #");
   
   if ((y>=0) && (y<100) && (mo>0) && (mo<13) && (d>0) && (d<32) && (h>=0) && (h<24) && (mi>=0) && (mi<60) && (s>=0) && (s<60))
   {
       Controllino_SetTimeDate(d, 1, mo, y, h, mi, s);
   }

   return 1;
} 

bool isAutonomous(void)
{
    bool autonomous=false;
    
    if (xSemaphoreTake(xSettingsMutex, (TickType_t) 0) == pdTRUE )
    {
        if (digitalRead(CONTROLLINO_A8) || Settings.autonomous) autonomous=true;
        xSemaphoreGive(xSettingsMutex);
    }
    return autonomous;
}

bool barrierContinuouslyOpen(void)
{
     bool barrieropen=false;
     
     if (xSemaphoreTake(xSettingsMutex, (TickType_t) 0) == pdTRUE )
     {
        if (digitalRead(CONTROLLINO_A8) || Settings.autonomous) barrieropen=true;
        xSemaphoreGive(xSettingsMutex);
     }
     return barrieropen;
}

int updateSettings(
    int barrierOpen, 
    int autonomous,
    int loopTimeout,
    int replyTimeout,
    int barrierPulseLength,
    int outputR5)
{
    int ret=1;

    if (xSemaphoreTake(xSettingsMutex, (TickType_t) 0) == pdTRUE )
    {
        digitalWrite(CONTROLLINO_R5, outputR5);
        Settings.outputR5=outputR5; 
        Settings.barrierOpen=barrierOpen; 
        Settings.autonomous=autonomous;   
       
        if ((loopTimeout<=5000) && (loopTimeout>=100)) 
        {
             Settings.loopTimeout=loopTimeout; 
             if( xTimerChangePeriod( xEntryLoopTimer, loopTimeout / portTICK_PERIOD_MS, 50) == pdFAIL ) return 0;
             if( xTimerChangePeriod( xExitLoopTimer , loopTimeout / portTICK_PERIOD_MS, 50) == pdFAIL ) return 0;
        } else ret=0;
        
        if ((replyTimeout<=5000) && (replyTimeout>=100)) 
        {
             Settings.replyTimeout=replyTimeout; 
             if( xTimerChangePeriod( xHttpEntryResponseTimer, replyTimeout / portTICK_PERIOD_MS, 50) == pdFAIL ) return 0;
             if( xTimerChangePeriod( xHttpExitResponseTimer , replyTimeout / portTICK_PERIOD_MS, 50) == pdFAIL ) return 0;
        } else ret=0;
    
        if ((barrierPulseLength<=5000) && (barrierPulseLength>=100))
        {
             Settings.barrierPulseLength=barrierPulseLength; 
             if( xTimerChangePeriod( xBarrierOpenTimer, barrierPulseLength / portTICK_PERIOD_MS, 50) == pdFAIL ) return 0;
        } else return 0;

        xSemaphoreGive(xSettingsMutex);
    }

    return ret;
}

void resetPLC()
{
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }     
}

void composeApiPathBase(int id, int key)
{
    //sprintf(httpApiPathBase, "/api.domain.com/%i/%i", id, key);
    sprintf(httpApiPathBase, "/api.domain.com/");
    return;
}

// the setup function runs once when you press reset or power the board
void setup() {

  // disable watchdog
  wdt_disable();
  
  // Running LED s
  pinMode(CONTROLLINO_D0, OUTPUT); pinMode(CONTROLLINO_D1, OUTPUT); pinMode(CONTROLLINO_D2, OUTPUT); pinMode(CONTROLLINO_D3, OUTPUT);
  pinMode(CONTROLLINO_D4, OUTPUT); pinMode(CONTROLLINO_D5, OUTPUT); pinMode(CONTROLLINO_D6, OUTPUT); pinMode(CONTROLLINO_D7, OUTPUT);
  // Inputs
  pinMode(CONTROLLINO_IN0, INPUT); // Entry loop
  pinMode(CONTROLLINO_IN1, INPUT); // Exit  loop
  pinMode(CONTROLLINO_A7, INPUT);  // Barrier continously open by HW
  pinMode(CONTROLLINO_A8, INPUT);  // LPR inactive by HW = Controllino works autonomous
  //Diagnostic outputs
  pinMode(CONTROLLINO_R0, OUTPUT); // Entry Loop
  pinMode(CONTROLLINO_R1, OUTPUT); // Exit loop
  pinMode(CONTROLLINO_R2, OUTPUT); // Timeout duration Http reply
  pinMode(CONTROLLINO_R4, OUTPUT); // Settings message
  pinMode(CONTROLLINO_R5, OUTPUT); // Settings output 
  pinMode(CONTROLLINO_R6, OUTPUT); // ID/key exchanged
  // To open barrier
  pinMode(CONTROLLINO_R3, OUTPUT); // Output to barrier

  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  Serial.println("Serial started");

  // start the Ethernet connection 
  
    
  Ethernet.begin(mac,local_ip);
  Serial.println(Ethernet.localIP());
  
  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.

  xSerialSemaphore   = xSemaphoreCreateMutex(); 
  xTxRxSemaphore     = xSemaphoreCreateCounting(50,0);
  xSemaphoreTake(xTxRxSemaphore, (TickType_t)0);
  xSettingsMutex    = xSemaphoreCreateMutex(); 

  // Now set Tasks to run independently.

  xTaskCreate(
    TaskVehicleDetection
    ,  (const portCHAR *) "TaskVehicleDetection"
    ,  255  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &tasks[PARKINGTASK_DETECTION] );
  
  xTaskCreate(
    TaskHttpReceive
    ,  (const portCHAR *) "TaskHttpReceive"
    ,  2047  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  &tasks[PARKINGTASK_HTTP_RX] ); 

  xTaskCreate(
    TaskBarrierControl
    ,  (const portCHAR *) "TaskBarrierControl"
    ,  255  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &tasks[PARKINGTASK_BARRIER] );

  xTaskCreate(
    TaskHttpGetSettings
    ,  (const portCHAR *) "TaskHttpGetSettings"
    ,  511  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  &tasks[PARKINGTASK_HTTP_RX] ); 

  xTaskCreate(
    TaskHttpGetId
    ,  (const portCHAR *) "TaskHttpGetId"
    ,  511  // Stack size
    ,  NULL
    ,  0  // Priority
    ,  &tasks[PARKINGTASK_HTTP_RX] ); 

  xTaskCreate(
    TaskRunningLeds
    ,  (const portCHAR *)"TaskRunningLeds"  // A name just for humans
    ,  255  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &tasks[PARKINGTASK_LEDS] );
    
  xEntryLoopTimer = xTimerCreate(    
    "Timer"
    , (Settings.loopTimeout / portTICK_PERIOD_MS)
    , pdFALSE
    , ( void * ) 0
    , fEntryTimerCallback);

  xExitLoopTimer = xTimerCreate(    
    "Timer"
    , (Settings.loopTimeout / portTICK_PERIOD_MS)
    , pdFALSE
    , ( void * ) 0
    , fExitTimerCallback);

  xHttpEntryResponseTimer = xTimerCreate(    
    "xHttpEntryResponseTimer"
    , (Settings.replyTimeout / portTICK_PERIOD_MS)
    , pdFALSE
    , ( void * ) 0
    , fHttpEntryResponseTimerCallback);

  xHttpExitResponseTimer = xTimerCreate(    
    "xHttpExitResponseTimer"
    , (Settings.replyTimeout / portTICK_PERIOD_MS)
    , pdFALSE
    , ( void * ) 0
    , fHttpExitResponseTimerCallback);

  xBarrierOpenTimer = xTimerCreate(    
    "xBarrierOpenTimer"
    , (Settings.barrierPulseLength / portTICK_PERIOD_MS)
    , pdFALSE
    , ( void * ) 0
    , fBarrierOpenTimerCallback);

  Controllino_RTC_init(0);
  Serial.println("RTC initialized");
  
  /* This function sets the time and dae to the connected RTC chip (RV-2123). Return code -1 means RTC chip was not properly initialized before. */
  //Controllino_SetTimeDate(13, 2, 12, 16, 9, 0, 0);

  Serial.println("Starting tasks ...");
  Serial.print("TIME: ");
  char * a = timeString(); 
  Serial.println();

  Serial.println("Exchanging ID and PW ...");

  //httpClient.print("POST "); httpClient.print(msgEntryApiPath); httpClient.println( " HTTP/1.1");          
  //httpClient.println("Connection: close"); httpClient.println();     

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started
}

void loop()
{
    //static uint16_t iterations = 0;
    // Empty. Things are done in Tasks.

    /*
    if(iterations==0)
    {
       for(int nTask = 0; nTask < NUM_PARKINGTASKS; ++nTask)
       {
          printWatermark(tasks[nTask]);
      }
    }
    ++iterations;
    */
}

/*--------------------------------------------------*/
/*---------- -- Timer Callbacks --------------------*/
/*--------------------------------------------------*/

// Callback, called when timer of entry loop ends
// Time during which vehicle should be on entry loop
// Triggers the http send task ot send POST API/ENTRY message
void fEntryTimerCallback(TimerHandle_t pxTimer)
{
   bool bOnLoop;

   xTimerStop(xEntryLoopTimer, 10);
   bOnLoop = digitalRead(CONTROLLINO_IN0);

   if (bOnLoop)
   {
      //debugPrint("I detected ");
      digitalWrite(CONTROLLINO_R0, 1);
      timeString();     
      if( !isAutonomous())
      {
          debugPrint(" Http POST I "); 
          sendHttpMsg(MSG_ENTRY);        
          // start entry sequence and timer, to check timeout later
          xTimerStart(xHttpEntryResponseTimer, 5);
          bEntrySequence= true; 
      }
      else
      {
          bOpenBarrier=true;
      }
   }
}

// Callback, called when timer of exit loop ends
// Time during which vehicle should be on entry loop
// Triggers the http send task ot send POST API/EXIT message
void fExitTimerCallback(TimerHandle_t pxTimer)
{
   bool bOnLoop;

   xTimerStop(xExitLoopTimer, 5);
   bOnLoop = digitalRead(CONTROLLINO_IN1);

   if (bOnLoop)
   {
      //debugPrint("O detected ");
      digitalWrite(CONTROLLINO_R1, 1);
      timeString();
      if( !isAutonomous())
      {
          debugPrint(" Http POST O "); 
          sendHttpMsg(MSG_EXIT);        
          // start entry sequence and timer, to check timeout later
          xTimerStart(xHttpExitResponseTimer, 5);
          bExitSequence= true; 
      }
      else
      {
          bOpenBarrier=true;
      }
   }
}

// Callback, called when timeout of Http receive of POST/ENTRY ends
// Http reply must be received within time
void fHttpEntryResponseTimerCallback(TimerHandle_t pxTimer)
{
   xTimerStop(xHttpEntryResponseTimer, 5);
}

// Callback, called when timeout of Http receive of POST/EXIT ends
// Http reply must be received within time
void fHttpExitResponseTimerCallback(TimerHandle_t pxTimer)
{
   xTimerStop(xHttpExitResponseTimer, 5);
}

// Callback, called when timer of barrier ends
// Defines pulsewidth of barrier open pulse
void fBarrierOpenTimerCallback(TimerHandle_t pxTimer)
{
   xTimerStop(xBarrierOpenTimer, 5);
   if (!barrierContinuouslyOpen()) // do not close if set to be continuously open bij HW pin
   {
       digitalWrite(CONTROLLINO_R3, 0);
   }
}

/*--------------------------------------------------*/
/*------------------- Tasks ------------------------*/
/*--------------------------------------------------*/

// Detects vehicles on entry and exit loop
// Starts timer to check if vehicle is a predefined time on loop
void TaskVehicleDetection(void *pvParameters __attribute__((unused)))
{
  bool bOnEntryLoop=false;
  bool bWasOnEntryLoop=false;
  bool bOnExitLoop=false;
  bool bWasOnExitLoop=false;
  
  while(1) 
  {
      if (GotApiIdKey)
      {
          bOnEntryLoop = digitalRead(CONTROLLINO_IN0);
          bOnExitLoop = digitalRead(CONTROLLINO_IN1);
    
          // entry loop
          if (bOnEntryLoop && !bWasOnEntryLoop) // rising edge detection
          {
              bWasOnEntryLoop = true;
              xTimerStart(xEntryLoopTimer, 5);
          } 
          else if (!bOnEntryLoop) // reset when vehicle leaves loop
          {
              digitalWrite(CONTROLLINO_R0, false);
              bWasOnEntryLoop = false;
          }
          // exit loop
          if (bOnExitLoop && !bWasOnExitLoop) // rising egde detection
          {
              bWasOnExitLoop = true;
              xTimerStart(xExitLoopTimer, 10);
          } 
          else if (!bOnExitLoop) // reset when vehicle leaves loop
          {
              digitalWrite(CONTROLLINO_R1, false);
              bWasOnExitLoop = false;
          }
      }
      vTaskDelay(5); 
   }
}

// Update Settings task
void TaskHttpGetSettings(void *pvParameters __attribute__((unused))) 
{
    for (;;) 
    {    
        if (GotApiIdKey)
        {
            timeString();
            debugPrint(" Http GET S ");         
            sendHttpMsg(MSG_SETTINGS);
        }
        vTaskDelay(1000);
    }
}

// Task getting ID and key from server
void TaskHttpGetId(void *pvParameters __attribute__((unused))) 
{
    for (;;) 
    {    
        if (!GotApiIdKey)
        {
            timeString();
            debugPrint(" Http GET ID ");         
            sendHttpMsg(MSG_ID);
            vTaskDelay(100);
        }
    }
}

// Send the different types of POST messages
int sendHttpMsg(MessageType_t type) 
{
    char path[20];
    int len;
    
    if (!httpClient.connect(serverIP, serverPort))
    {
         debugPrintln("(Tx connection error) ");
         return 0;
    } 
    else
    {
        switch(type)
        {
            case MSG_ENTRY:
                httpClient.print("POST "); httpClient.print(httpApiPathBase); httpClient.print("entry"); httpClient.println( " HTTP/1.1");          
                httpClient.println("Connection: close"); httpClient.println();                         
                break;
            case MSG_EXIT:
                httpClient.print("POST "); httpClient.print(httpApiPathBase); httpClient.print("exit"); httpClient.println( " HTTP/1.1");          
                httpClient.println("Connection: close"); httpClient.println();
                break;
            case MSG_SETTINGS: 
                httpClient.print("GET ");  httpClient.print(httpApiPathBase); httpClient.print("settings"); httpClient.println( " HTTP/1.1");          
                httpClient.println("Connection: close"); httpClient.println();
                break; 
            case MSG_ID: 
                httpClient.print("GET "); httpClient.print(MSG_IDKEY_API_PATH); httpClient.println( " HTTP/1.1");          
                httpClient.println("Connection: close"); httpClient.println();
                break; 
            default:
                debugPrint("error: unkown message type ");
                return 0;
        }
        // signal receive task
        xSemaphoreGive(xTxRxSemaphore);
        
        return 1;
    }
}

// Try to receive server reply's
// Important: Reading the whole reply message, even if received in several TCP packets
// Reading is finished when the last } is received
int receiveHttpResponse(char * reply)
{
    int i;
    char c;
    int openBraces = 0;
    bool hasBraces = false;
   
    i=0;
    // clear string
    for (i=0; i<sizeof(reply); i++) reply[i]=0;
    
    i=0;
    // it is possible to have available() but the connection is already closed by the server !!!
    // Therefore we need to check both connected() or available()
    bool receiving = httpClient.connected() || httpClient.available();
    bool eof = false;
    while (receiving) 
    { 
        if(!httpClient.available()) vTaskDelay(1);
        bool bufferFree = (i<MAX_REPLY_LENGTH-1);// to prevent overrun of the reply[MAX_REPLY_LENGTH] array
        while (httpClient.available() && bufferFree && !eof) 
        {
            c = httpClient.read();
            reply[i++]=c;
            switch(c)
            {
                case '{' : {
                    ++openBraces;
                    hasBraces = true;
                } break;
                case '}' : {
                    if(openBraces>0) --openBraces;
                } break;
            }
            bufferFree = (i<MAX_REPLY_LENGTH-1);
            eof = hasBraces && (openBraces<=0); // last open { was closed
            //if(eof) debugPrintln("@");
        }
        receiving = (httpClient.connected() || httpClient.available()) && bufferFree && !eof;
    }
    const bool hasData = i>0;
    if(hasData)
    {
        reply[i]='\0';
    }
    httpClient.stop();
    return hasData;
}

// Parse Http reply messages
int parseHttpResponse(char * reply, int * msgtype, bool * allowed)
{
    int i=0; 
    int j=0;
    char c;
    char *body = reply;
    char buf[MAX_BODY_LENGTH];
    StaticJsonBuffer<MAX_REPLY_LENGTH> jsonBuffer;
    int statuscode;
    int m, a, o, lt, rt, bpl, u, am, r, or5, id, key;
    int ret=0;
  
    //debugPrintln(reply);

    // 1. Parse status code
    reply[12]= '\0';
    statuscode = atoi(&reply[9]);

    if (statuscode!=200)
    {
         debugPrint("(error: http reply code:"); debugPrint(statuscode); debugPrintln(") ");
         return 0;
    }

    // 2. Skip headers
    i=13;
    while ((reply[i]!='\0') && (reply[i]!='{') && (i<MAX_REPLY_LENGTH-1)) i++;
    body = &reply[i];

    // 3. parse body
    JsonObject& root = jsonBuffer.parseObject(body);
    if (root.success())
    {
        int m = root["message-type"];   // extract message type
        
        *msgtype=m;
        switch (m)
        {
            case MSG_ENTRY:
            case MSG_EXIT:
                a = root["access-allowed"]; // extract if access allowed
                sprintf(buf, "{message-type:%i,access-allowed:%i}", m, a);
                debugPrint(buf); 
                *allowed=a; 
                break;

            case MSG_SETTINGS:
                o    = root["open-barrier"];     
                am   = root["autonomous-mode"];     
                lt   = root["loop-timeout"];   
                rt   = root["reply-timeout"];  
                bpl  = root["barrier-pulse-length"]; 
                r    = root["reset"]; 
                or5  = root["output-r5"]; 

                sprintf(buf, "{open-barrier:%i,autonomous-mode:%i,loop-timeout:%i,reply-timeout:%i,barrier-pulse-length:%i,reset:%i,output-r5:%i}", o, am, lt, rt, bpl, r, or5);
                debugPrint(buf); 
                //root.printTo(Serial);
                //Serial.println();

                if (r==1) resetPLC();
                if (updateSettings(o, a, lt, rt, bpl, or5)) debugPrintln(" (settings updated) ");
                else debugPrintln(" (error: settings update) ");
                break;
            
            case MSG_ID:
                id  = root["id"]; // extract ID
                key = root["key"]; // extract key
                
                sprintf(buf, "{id:%i,key:%i}", id, key);
                debugPrintln(buf); 
                composeApiPathBase(id, key);
                Serial.println("ID and PW exchanged");
                digitalWrite(CONTROLLINO_R6, 1);
                GotApiIdKey=true;
                break;
             default:
                debugPrintln(" (error: message type) ");
                return 0;
        }   
    } else 
    {
        debugPrintln(" (error: parsing) ");
        return 0;
    }

    return 1;
}

// Task receives http responses from server
// Triggered by httpsend task

void TaskHttpReceive(void *pvParameters __attribute__((unused))) 
{   
    int  statusCode=0;
    int  ret=0;
    bool allowed;
    char reply[MAX_REPLY_LENGTH];
    int  msgType;
    
    for (;;) 
    {   
        if (xSemaphoreTake(xTxRxSemaphore, portMAX_DELAY) == pdTRUE )
        {
             if (receiveHttpResponse(reply))
             {
                 debugPrint("Http Rx "); 
                 if (parseHttpResponse(reply, &msgType, &allowed))
                 {
                      switch (msgType)
                      {
                          case MSG_ENTRY:
                               bAccessAllowed = allowed; // barrier can be opened if true
                               bReplyReceived = true;    // trigger barrier control task 
                               break;
                          case MSG_EXIT:
                               bAccessAllowed = allowed; // barrier can be opened if true
                               bReplyReceived = true;    // trigger barrier control task 
                               break;
                          case MSG_SETTINGS:
                          case MSG_ID:
                               break;
                          default:
                               break;
                      }              
                 }
                 else
                 {
                      bReplyReceived=false; // will trigger timeout in barrier control task
                 } 
             }
             else
             {
                 debugPrintln(" (Rx connection error) ");
                 bReplyReceived=false; // will trigger timeout in barrier control task
             }
        }
    } 
}

// Task opens barrier based on:
// * replies received from http receive task
// * response timeouts
// * SW or HW settings (continue open, close)
void TaskBarrierControl(void *pvParameters __attribute__((unused)))
{
    while(1) 
    {
        const bool rxEntryResponseTimeout = xTimerIsTimerActive(xHttpEntryResponseTimer) == pdFALSE;
        const bool rxExitResponseTimeout = xTimerIsTimerActive(xHttpExitResponseTimer) == pdFALSE;

        if (bEntrySequence && !rxEntryResponseTimeout && bReplyReceived) // message received, no timeout
        {
             bEntrySequence=false;
             bReplyReceived=false;
             if (bAccessAllowed) 
             { 
                  bAccessAllowed=false;
                  bOpenBarrier=true;
                  debugPrint(" (accepted) ");
             }
             else debugPrintln(" (not allowed) ");
    
        } 
        else if (bEntrySequence && rxEntryResponseTimeout) // no message received, timeout 
        { 
             bOpenBarrier=true;
             bEntrySequence=false;
             debugPrint(" (timeout) ");
        }

        if (bExitSequence && !rxExitResponseTimeout && bReplyReceived) // message received, no timeout
        { 
             bExitSequence=false;
             bReplyReceived=false;
             if (bAccessAllowed) 
             { 
                  bAccessAllowed=false;
                  bOpenBarrier=true;
                  debugPrint(" (accepted) ");
             }
             else debugPrintln(" (not allowed) ");
             
        } 
        else if (bExitSequence && rxExitResponseTimeout) // no message received, no timeout  
        { 
             bOpenBarrier=true;
             bExitSequence=false;
             debugPrint(" (timeout) ");
        }

        if (barrierContinuouslyOpen())
        {
             digitalWrite(CONTROLLINO_R3, 1);
        }
        else if (bOpenBarrier) 
        {
             bOpenBarrier=false;
             debugPrintln(" barrier opened");
             digitalWrite(CONTROLLINO_R3, 1);
             xTimerStart(xBarrierOpenTimer, 5);
        }
        
        vTaskDelay(5); 
     }
}

// Task runs leds, t show real-time activity of Controllino
void TaskRunningLeds(void *pvParameters __attribute__((unused))) 
{
    int led[6] = {CONTROLLINO_D0,CONTROLLINO_D1,CONTROLLINO_D2, CONTROLLINO_D3, CONTROLLINO_D4, CONTROLLINO_D5};
    bool on[6], _on[6];
    
    for (int i=0; i<6; i++) on[i]=false; on[0] = true;
    
    for (;;) 
    {    
        for (int i=0; i<5; i++) _on[i+1]=on[i]; _on[0]=on[5];
        for (int i=0; i<6; i++) 
        {
            digitalWrite(led[i], _on[i]);
            on[i]=_on[i];
        }
        vTaskDelay(5); 
    }
}
