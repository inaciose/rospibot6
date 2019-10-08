#include <ros/ros.h>
#include <sys/sysinfo.h>

#include "system.h"
#include "nokia5110.h"
#include "nokia5110_lcdmenu.h"
#include "nokia5110_lcdmenu_def.h"
#include "nokia5110_lcdmenu_globals.h"
#include "globals.h"
#include "commands_def.h"
#include "mcu.h"

// i2c
#include "I2Cdev.h"

// global lcd variables
bool lcdBacklightStatus = false;

// global buttons variables
bool btnStatus = false;

// local buttons variables
uint8_t btnState[BTN_NUM];

// local lcd menu variables
bool menuOn = false;
uint8_t menu = 0;
uint8_t menuselect[MENU_MAX+1][2] = {{0,3}, {0,3}, {0,2}, {0,1}}; // selected item , total_itens

//
// LCD SETUP
//

//
// BUTTONS SETUP
//

void lcdmenu_lcd_setup() {
    // set the nokia 5110 lcd pins on bcm2835
    lcdCreate(LCD_PIN_RESET, LCD_PIN_SCE, LCD_PIN_DC, LCD_PIN_SDIN, LCD_PIN_SCLK, LCD_PIN_BL);
    // start the lcd
    lcdInit();
    // disable backlight
    lcdBackLight (1);
}

void lcdmenu_buttons_setup() {
    // Set RPI pins to be an input
    bcm2835_gpio_fsel(BTN_PIN_BUP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BDN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BLF, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BRT, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BNO, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BTN_PIN_BOK, BCM2835_GPIO_FSEL_INPT);

    // set pins with a pullup
    bcm2835_gpio_set_pud(BTN_PIN_BUP, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BDN, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BLF, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BRT, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BNO, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BTN_PIN_BOK, BCM2835_GPIO_PUD_UP);
}

//
// PROCESS BUTTONS
//

int lcdmenu_buttons_process() {

        int ttl_buttons_pressed = 0;

        btnState[BTN_BUP] = !bcm2835_gpio_lev(BTN_PIN_BUP);
        btnState[BTN_BDN] = !bcm2835_gpio_lev(BTN_PIN_BDN);
        btnState[BTN_BLF] = !bcm2835_gpio_lev(BTN_PIN_BLF);
        btnState[BTN_BRT] = !bcm2835_gpio_lev(BTN_PIN_BRT);
        btnState[BTN_BNO] = !bcm2835_gpio_lev(BTN_PIN_BNO);
        btnState[BTN_BOK] = !bcm2835_gpio_lev(BTN_PIN_BOK);

        //printf("%d %d %d %d %d %d\n", btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BL$

        // buttons imediate actions
        if(menuOn) {
            if(btnState[BTN_BNO]) {
                lcdBackLight(0);
                lcdBacklightStatus = true;
                lcdBacklightTimer = ros::Time::now().toSec() + LCD_BACKLIGHT_TIMER;
            }
            if(btnState[BTN_BOK]) {
                menuOn = false;
            }
        } else {
            if(btnState[BTN_BNO]) {
                lcdBackLight(0);
                lcdBacklightStatus = true;
                lcdBacklightTimer = ros::Time::now().toSec() + LCD_BACKLIGHT_TIMER;
            }
            if(btnState[BTN_BOK]) {
                menuOn = true;
            }
        }

        // check for any button pressed
        ttl_buttons_pressed = 0;
        for(int f = 0; f < BTN_NUM; f++) {
            ttl_buttons_pressed += btnState[f];
        }

	if(ttl_buttons_pressed > 0) {
            btnStatus = true;
	}

	return ttl_buttons_pressed;

}

//
// PROCESS MENU & LCD UPDATE
//

void lcdmenu_set_backlight(bool state) {
        lcdBackLight(!state);
        lcdBacklightStatus = state;
}

void lcdmenu_update() {

        if(menuOn) {
            switch(menu) {
                case 0: lcdMenu(); break;
                case 1: lcdMenu1(); break;
                case 2: lcdMenu2(); break;
            }
        } else {
            lcdHome();
            //bcm2835_delay(800);
        }

        //reset button status
        btnStatus = false;
        //for(int f = 0; f < BTN_NUM; f++) {
        //    btnState[f] = 0;
        //}
}

//
// HOME DISPLAY
//

void lcdHome() {
    // lcd output vars
    char lcdOutString1[24];
    char lcdOutString2[24];
    char lcdOutString3[24];

    // lcd uptime
    struct sysinfo info;
    int uptimeDays = 0;
    int uptimeHours = 0;
    int uptimeMins = 0;
    int uptimeSecs = 0;


        // get the ip addresses
        getIfAddress("eth0", lcdOutString2);
        getIfAddress("wlx0013efcb0cbc", lcdOutString3);

        // get uptime (from system info)
        // we can also get the load
        sysinfo(&info);

        // convert to human standart
        uptimeMins = info.uptime / 60;
        uptimeSecs = info.uptime - uptimeMins * 60;

        uptimeHours = uptimeMins / 60;
        uptimeMins = uptimeMins - uptimeHours * 60;

        uptimeDays = uptimeHours / 24;
        uptimeHours = uptimeHours - uptimeDays * 24;

        // debug
        //printf("Uptime = %ld %d %02d:%02d:%02d\n", info.uptime, uptimeDay

        // display on lcd
        lcdClear();
        sprintf(lcdOutString1, "IP & uptime ");
        lcdGotoXY(0,0);
        lcdString(lcdOutString1);

        lcdGotoXY(0,1);
        lcdString(lcdOutString2);

        lcdGotoXY(0,3);
        lcdString(lcdOutString3);

        lcdGotoXY(0,5);
        sprintf(lcdOutString1,"%d %02d:%02d:%02d", uptimeDays, uptimeHours, uptimeMins, uptimeSecs);
        lcdString(lcdOutString1);
}

//
// MENU HELPERS
//

void lcdMenuShowItem(uint8_t x, uint8_t y, const char text[20], bool sel) {
        char outString[24];
        if(sel) {
                sprintf(outString, "> %s", text);
        } else {
                sprintf(outString, "  %s", text);
        }

        lcdGotoXY(x,y);
        lcdString(outString);
}

//
// DISPLAY & PROCESS MENUS SELECTIONS
//

void lcdMenu1() {

        // menu config
        uint8_t menuid = 1;
        int selected = menuselect[menuid][0];

        // lcd output vars
        char outString[20];

        // mcu command
        uint8_t res;
        uint8_t* dataptr = (uint8_t*)tmp_data;
        dataptr++;

//        printf("+%d %d %d %d %d %d %d %d %d\n", menuOn, menu, selected, btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BLF], btnState[BTN_BRT], btnState[BTN_BNO], $

        // up/down button pressed
        if(btnState[BTN_BUP] && (selected > 0)) selected--;
        if(btnState[BTN_BDN] && (selected < menuselect[menuid][1]-1)) selected++;

        if(btnState[BTN_BUP] || btnState[BTN_BDN]) {
            menuselect[menuid][0] = selected;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            printf("-%d %d %d\n", menuOn, menu, selected);
        }

        // back button pressed
        if(btnState[BTN_BLF]) {
            menu = 0;
            menuselect[menuid][0] = 0;
            //menuOn = false;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            return;
        }

        // select buttom pressed
        if(btnState[BTN_BRT]) {
            switch(selected) {
                case 0:
                    menu = 2;
                    return;
                    break;
                case 1:
                    //  send lidar command
                    if(lidarMotorStatus) {
                        ROS_INFO("send lidar off mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_OFF, 3, dataptr);
                        lidarMotorStatus = false;
                    } else {
                        ROS_INFO("send lidar on mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_ON, 3, dataptr);
                        lidarMotorStatus = true;
                    }
                    break;
                case 2:
                    ROS_INFO("shutdown");
                    break;
            }

            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // display on lcd
        lcdClear();
        sprintf(outString, "--- EXTRA ---");
        lcdGotoXY(0,0);
        lcdString(outString);

        lcdMenuShowItem(0,1, "Shutdown", (selected == 0) ? 1 : 0);
        lcdMenuShowItem(0,2, "Lidar", (selected == 1) ? 1 : 0);
        lcdMenuShowItem(0,3, "Outro", (selected == 2) ? 1 : 0);
}


void lcdMenu2() {
        // menu config
        uint8_t menuid = 2;
        int selected = menuselect[menuid][0];

        // lcd output vars
        char outString[20];

        // mcu command
        uint8_t res;
        uint8_t* dataptr = (uint8_t*)tmp_data;
        dataptr++;

        // up/down button pressed
        if(btnState[BTN_BUP] && (selected > 0)) selected--;
        if(btnState[BTN_BDN] && (selected < menuselect[menuid][1]-1)) selected++;

        if(btnState[BTN_BUP] || btnState[BTN_BDN]) {
            menuselect[menuid][0] = selected;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // back button pressed
        if(btnState[BTN_BLF]) {
            menu = 0;
            menuselect[menuid][0] = 0;
            //menuOn = false;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            return;
        }

        // select buttom pressed
        if(btnState[BTN_BRT]) {
            switch(selected) {
                case 0:
                    ROS_INFO("shutdown");
                    //sync();
                    //reboot(RB_POWER_OFF);
                    system("shutdown -P now");
                    break;
                case 1:
                    ROS_INFO("reboot");
                    //sync();
                    //reboot(RB_AUTOBOOT);
                    system("reboot");
                    break;
            }
            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // display on lcd
        lcdClear();
        sprintf(outString, "- ON/OFF --");
        lcdGotoXY(0,0);
        lcdString(outString);

        lcdMenuShowItem(0,1, "Shutdown", (selected == 0) ? 1 : 0);
        lcdMenuShowItem(0,2, "Reboot", (selected == 1) ? 1 : 0);

}

void lcdMenu() {

        // menu config
        uint8_t menuid = 0;
        int selected = menuselect[menuid][0];

        // lcd output vars
        char outString[20];

        // mcu command
        uint8_t res;
        uint8_t* dataptr = (uint8_t*)tmp_data;
        dataptr++;

        printf("+%d %d %d %d %d %d %d %d %d\n", menuOn, menu, selected, btnState[BTN_BUP], btnState[BTN_BDN], btnState[BTN_BLF], btnState[BTN_BRT], btnState[BTN_BNO], btnState[BTN_BOK]);

        // up/down button pressed
        if(btnState[BTN_BUP] && (selected > 0)) selected--;
        if(btnState[BTN_BDN] && (selected < menuselect[menuid][1]-1)) selected++;

        if(btnState[BTN_BUP] || btnState[BTN_BDN]) {
            menuselect[menuid][0] = selected;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            printf("-%d %d %d\n", menuOn, menu, selected);
        }


        // back button pressed
        if(btnState[BTN_BLF]) {
            menuselect[menuid][0] = 0;
            menuOn = false;
            lcdTimer = ros::Time::now().toSec(); // set timer for now
            return;
        }

        // select buttom pressed
        if(btnState[BTN_BRT]) {
            switch(selected) {
                case 0:
                    menu = 1;
                    return;
                    break;
                case 1:
                    //  send lidar command
                    if(lidarMotorStatus) {
                        ROS_INFO("send lidar off mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_OFF, 3, dataptr);
                        lidarMotorStatus = false;
                    } else {
                        ROS_INFO("send lidar on mcu cmd");
                        res = I2Cdev::writeBytes(I2CADDR_STM32, CMD_MCU_LIDAR_MOTOR_ON, 3, dataptr);
                        lidarMotorStatus = true;
                    }
                    break;
                case 2:
                    ROS_INFO("shutdown");
                    menu = 2;
                    return;
                    break;
            }
            lcdTimer = ros::Time::now().toSec(); // set timer for now
        }

        // display on lcd
        lcdClear();
        sprintf(outString, "--- Home ---");
        lcdGotoXY(0,0);
        lcdString(outString);

        lcdMenuShowItem(0,1, "Menu 1", (selected == 0) ? 1 : 0);
        lcdMenuShowItem(0,2, "Lidar", (selected == 1) ? 1 : 0);
        lcdMenuShowItem(0,3, "Shutdown", (selected == 2) ? 1 : 0);
}

