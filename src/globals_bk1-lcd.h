// data trasfer variables
union u_tag {
    uint8_t b[4];
    int32_t i;
};


// push buttons
#define BTN_NUM 6

// lcd  menu
#define MENU_MAX 3
extern bool menuOn;
extern uint8_t menu;
extern uint8_t menuselect[MENU_MAX+1][2];

extern union u_tag pwm_data[8];
extern union u_tag enc_data[8];
extern union u_tag tmp_data[8];

extern double lcdTimer;
extern double lcdBacklightTimer;
extern bool lcdBacklightStatus;

extern bool lidarMotorStatus;
extern bool lcdBacklightStatus;

extern bool btnStatus;
extern double btnTimer;
extern uint8_t btnState[BTN_NUM];

