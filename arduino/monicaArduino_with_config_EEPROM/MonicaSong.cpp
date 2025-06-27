#include <Arduino.h>
#include <pitches.h>
#include "MonicaSong.h"

#define   MAX_SONG   5

static int melody_bear[] = {
  // 곰세마리
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5,           //도도도도도
  NOTE_E5, NOTE_G5, NOTE_G5, NOTE_E5, NOTE_C5,           //미솔솔미도
  NOTE_G5, NOTE_G5, NOTE_E5, NOTE_G5, NOTE_G5, NOTE_E5,  //솔솔미솔솔미
  NOTE_C5, NOTE_C5, NOTE_C5,                             //도도도
};

static int melody_birthday[] = {
  NOTE_C4, NOTE_C4, 
  NOTE_D4, NOTE_C4, NOTE_F4,
  NOTE_E4, NOTE_C4, NOTE_C4, 
  NOTE_D4, NOTE_C4, NOTE_G4,
  NOTE_F4, NOTE_C4, NOTE_C4,
  
  NOTE_C5, NOTE_A4, NOTE_F4, 
  NOTE_E4, NOTE_D4, NOTE_AS4, NOTE_AS4,
  NOTE_A4, NOTE_F4, NOTE_G4,
  NOTE_F4
};

static int melody_pink[] = {
  REST, REST, REST, NOTE_DS4,
  NOTE_E4, REST, NOTE_FS4, NOTE_G4, REST, NOTE_DS4,
  NOTE_E4, NOTE_FS4, NOTE_G4, NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_B4,
  NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_E4, NOTE_D4,
};

static int melody_star[] = {
  NOTE_AS4, NOTE_AS4, NOTE_AS4,
  NOTE_F5, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5, NOTE_C5,
  NOTE_F5, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
};

static int melody_tetris[] = {
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5,
};


static int durations_bear[] = {
  4, 8, 8, 4, 4,     //1
  4, 8, 8, 4, 4,     //2
  8, 8, 4, 8, 8, 4,  //3
  4, 4, 2,           //4

};

static int durations_birthday[] = {
  4, 8, 
  4, 4, 4,
  2, 4, 8, 
  4, 4, 4,
  2, 4, 8,
  
  4, 4, 4, 
  4, 4, 4, 8,
  4, 4, 4,
  2
};

static int durations_pink[] = {
  2, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8,
  2, 16, 16, 16, 16,
};

static int durations_star[] = {
  8, 8, 8,
  2, 2,
  8, 8, 8, 2, 4,
  8, 8, 8, 2, 4,
  8, 8, 8, 2, 8, 8, 8,
  2, 2,
  8, 8, 8, 2, 4,
};

static int durations_tetris[] = {
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 4, 4,
  4, 4, 8, 4, 8, 8,
};

static int *durArray[] = { durations_pink, durations_bear, durations_birthday, durations_star, durations_tetris };
static int *melArray[] = { melody_pink, melody_bear, melody_birthday, melody_star, melody_tetris };
static int sizeArray[] = { 23, 19, 25, 29, 22 };





MonicaSong::MonicaSong(int buzzer_pin, int buzzer_channel)
{
  set_buzzer_pin(buzzer_pin);
  set_buzzer_channel(buzzer_channel);

  song_ = 0;
  size_ = 0;
  note_ = 0;
  duration_ = 0;
  start_time_ms_ = 0;
  state_ = SONG_STANDBY;
}

MonicaSong::~MonicaSong()
{
}

void MonicaSong::Initialize()
{
  //ledcSetup(buzzer_channel_, 1000, 8);  //enB, channel: 1, 1000Hz, 8bits = 256(0 ~ 255)
  //ledcSetup(buzzer_channel_, 1E5, 12);
  ledcSetup(buzzer_channel_, 5000, 10);
  ledcAttachPin(buzzer_pin_, buzzer_channel_);
  ledcWrite(buzzer_channel_, 0);
}

void MonicaSong::play()
{
  switch (state_)
  {
  case SONG_STANDBY:
    /*skip*/
    break;

  case SONG_MELODY:
  {
    ledcWriteTone(buzzer_channel_, melArray[song_][note_]);
    duration_ = 1000 / durArray[song_][note_];

    start_time_ms_ = millis();
    delay_time_ = duration_;
    state_ = SONG_MELODY_DELAY;
  }
    break;

  case SONG_MELODY_DELAY:
  {
    if ((millis() - start_time_ms_) >= delay_time_)
    {
      state_ = SONG_PAUSE;
    }
  }
    break;

  case SONG_PAUSE:
  {
    ledcWrite(buzzer_channel_, 0);

    start_time_ms_ = millis();
    delay_time_ = duration_ * 1.3;
    state_ = SONG_PAUSE_DELAY;
  }
    break;

  case SONG_PAUSE_DELAY:
  {
    if ((millis() - start_time_ms_) >= delay_time_)
    {
      state_ = SONG_CHECK_NEXT;
    }
  }
    break;

  case SONG_CHECK_NEXT:
  {
    note_++;

    if (note_ < size_)
    {
      state_ = SONG_MELODY;
    }
    else
    {
      state_ = SONG_STANDBY;
    }
  }
    break;
  
  default:
    break;
  }
}

void MonicaSong::stop()
{
  ledcWrite(buzzer_channel_, 0);

  song_ = 0;
  size_ = 0;
  note_ = 0;
  duration_ = 0;
  start_time_ms_ = 0;
  state_ = SONG_STANDBY;
}

void MonicaSong::select(int song)
{
  song_ = song % MAX_SONG;
  size_ = sizeArray[song];
  note_ = 0;
  state_ = SONG_MELODY;
}
