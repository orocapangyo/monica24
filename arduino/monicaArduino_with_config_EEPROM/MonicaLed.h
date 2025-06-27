#ifndef _MONICA_LED_H_
#define _MONICA_LED_H_

typedef enum
{
    LED_LEFT_ON=0,  //rear left on
    LED_RIGHT_ON,   //rear right on 
    LED_REAR_ON,    //rear left and right on
    LED_FRONT_ON,   //front right on
    LED_ALL_ON,     //all on
    LED_ALL_OFF,    //all off

} LED_ACTIVE;

class MonicaLed
{
public:
    MonicaLed(int led_left_pin,
             int led_right_pin,
             int led_front_pin);
    ~MonicaLed();

public:
    void Initialize();
    void RGB(int ledbehav);

private:
    int led_left_pin_;
    int led_right_pin_;
    int led_front_pin_;
};
#endif /*_MONICA_LED_H_*/
