#include <Arduino.h>
#include <ESP32Encoder.h>

#define ASF_PIN 2
#define PE_PIN 4
#define FEED_IN_PIN 23
#define FEED_OUT_PIN 22
#define encoderA 18
#define encoderB 19

#define LED 32
#define BTN_READY 33
#define BTN_IN 34
#define BTN_OUT 35

#define ASF_CH 0
#define ASF_FREQ 5
#define ASF_RES 8
#define ASF_DUTY 128

#define Ready 1
#define Standby 0

long oldPosition = -999;
long newPosition;
long encoder_active = 0;
int offsetnewPosition = 50;

long arr_stop[20];
int chk_stop = 0;
int offset_chk_stop = 3000;
uint8_t indexof_arr_stop = 0;

uint8_t PE_state = 0;
uint8_t btn_pe_state = 0;

uint8_t last_value0 = 0, last_value1 = 0, last_value2 = 0;
uint8_t cur_ready = 0, cur_in = 0, cur_out = 0;

ESP32Encoder epsonEncoder;

enum state
{
  p_standby,
  p_readytoprint,
  p_print,
  p_printfinish,
  p_stepfinish
};

enum state process_state = p_standby;

void ASF_Emulator()
{
  ledcSetup(ASF_CH, ASF_FREQ, ASF_RES);
  ledcAttachPin(ASF_PIN, ASF_CH);
  ledcWrite(ASF_CH, ASF_DUTY);
}

void chk_finish()
{
  arr_stop[indexof_arr_stop] = newPosition;
  indexof_arr_stop++;
  // Serial.print("index0 ");
  // Serial.println(arr_stop[0]);
  // Serial.print("index20 ");
  // Serial.println(arr_stop[19]);

  if (indexof_arr_stop > 20)
  {
    // Serial.println("-----------------------------20 ");
    indexof_arr_stop = 0;
    chk_stop = arr_stop[19] - arr_stop[0];
    // Serial.print("-----------------------------chk_stop ");
    // Serial.println(chk_stop);
  }
}

void Encoder_Task(void *p)
{
  while (1)
  {
    newPosition = epsonEncoder.getCount() / 2;
    if (newPosition != oldPosition)
    {
      oldPosition = newPosition;
      chk_finish();
      Serial.println(newPosition);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Ready_To_Print()
{
  switch (process_state)
  {
  case p_standby:

    break;

  case p_readytoprint:
    if (newPosition >= encoder_active + offsetnewPosition)
    {
      Serial.println("Start Print");
      digitalWrite(PE_PIN, false);
      digitalWrite(LED, true);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      process_state = p_print;
    }

    break;

  case p_print:

    if (chk_stop > offset_chk_stop)
    {
      Serial.println("Stop Print");
      btn_pe_state = Standby;
      process_state = p_standby;

      digitalWrite(PE_PIN, true);
      digitalWrite(LED, false);
    }

    break;
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

void BTN_task(void *p)
{
  while (1)
  {
    cur_ready = digitalRead(BTN_READY);
    cur_in = digitalRead(BTN_IN);
    cur_out = digitalRead(BTN_OUT);

    if (cur_ready != last_value0)
    {
      if (cur_ready == 0 && btn_pe_state == Standby)
      {
        btn_pe_state = Ready;
        encoder_active = newPosition;
        process_state = p_readytoprint;
        digitalWrite(LED, true);
      }
      else if (cur_ready == 0 && btn_pe_state == Ready)
      {
        btn_pe_state = Standby;
        process_state = p_standby;
        digitalWrite(PE_PIN, true);
        digitalWrite(LED, false);
      }
      last_value0 = cur_ready;
    }

    if (cur_in != last_value1)
    {
      if (cur_in == 0 && btn_pe_state == Standby)
      {
        digitalWrite(FEED_IN_PIN, false);
      }
      else
      {
        digitalWrite(FEED_IN_PIN, true);
      }
      last_value1 = cur_in;
    }

    if (cur_out != last_value2)
    {
      if (cur_out == 0 && btn_pe_state == Standby)
      {
        digitalWrite(FEED_IN_PIN, false);
        digitalWrite(FEED_OUT_PIN, false);
      }
      else
      {
        digitalWrite(FEED_IN_PIN, true);
        digitalWrite(FEED_OUT_PIN, true);
      }
      last_value2 = cur_out;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void set_init()
{
  Serial.begin(9600);

  pinMode(BTN_IN, INPUT);
  pinMode(BTN_OUT, INPUT);
  pinMode(BTN_READY, INPUT);
  pinMode(LED, OUTPUT);

  pinMode(ASF_PIN, OUTPUT);
  pinMode(PE_PIN, OUTPUT);
  pinMode(FEED_IN_PIN, OUTPUT);
  pinMode(FEED_OUT_PIN, OUTPUT);
  digitalWrite(PE_PIN, true);

  epsonEncoder.attachHalfQuad(encoderA, encoderB);
  epsonEncoder.setCount(0);
}

void setup()
{
  set_init();
  ASF_Emulator();
  xTaskCreate(&BTN_task, "BTN_task", 2048, NULL, 10, NULL);
  xTaskCreate(&Encoder_Task, "Encoder_Task", 2048, NULL, 10, NULL);
}

void loop()
{
  Ready_To_Print();
}