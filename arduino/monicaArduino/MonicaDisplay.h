#ifndef _MONICA_DISPLAY_H_
#define _MONICA_DISPLAY_H_

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

typedef enum
{
    DISPLAY_STANDBY,
    DISPLAY_SHOW,
    DISPLAY_DELAY,
    DISPLAY_CHECK_NEXT,

} DISPLAY_STATES;

class MonicaDisplay
{
public:
    MonicaDisplay();
    ~MonicaDisplay();

public:
    void Initialize();
    void select(int index);
    void clear();
    void show();


private:
    Adafruit_SSD1306* display_;
    int index_;
    int count_;
    int frame_;
    int max_frame_;
    unsigned long start_time_ms_;
    DISPLAY_STATES state_;

};
#endif /*_MONICA_DISPLAY_H_*/
