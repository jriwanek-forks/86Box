void pic_init(void);
void pic2_init(void);
void pic_reset(void);

void picint(uint16_t num);
void picintlevel(uint16_t num);
void picintc(uint16_t num);
uint8_t picinterrupt(void);
void picclear(int num);
