#ifndef _MONICA_SONG_H_
#define _MONICA_SONG_H_

typedef enum
{
    SONG_STANDBY,
    SONG_MELODY,
    SONG_MELODY_DELAY,
    SONG_PAUSE,
    SONG_PAUSE_DELAY,
    SONG_CHECK_NEXT,

} SONG_STATES;

class MonicaSong
{
public:
    MonicaSong(int buzzer_pin, int buzzer_channel);
    ~MonicaSong();

public:
    void Initialize();
    void play();
    void stop();
    void select(int song);

public:
    void set_buzzer_pin(int buzzer_pin) { buzzer_pin_=buzzer_pin; }
    void set_buzzer_channel(int buzzer_channel) { buzzer_channel_=buzzer_channel; }

private:
    int song_;
    int size_;
    int note_;
    int duration_;
    int delay_time_;
    unsigned long start_time_ms_;
    SONG_STATES state_;

    int buzzer_pin_;
    int buzzer_channel_;

};
#endif /*_MONICA_SONG_H_*/
