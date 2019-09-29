void lcdBackLight (bool state);
void lcdGotoXY(int x, int y);
void lcdClear(void);
void lcdCreate(unsigned char rst, unsigned char ce, unsigned char dc, unsigned char din, unsigned char clk, unsigned char bl);
void lcdInit(void);
void lcdString(char *characters);
void lcdStringln(char *characters);
//void lcdScroll(char message[]);
