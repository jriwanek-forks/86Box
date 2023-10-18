/* Copyright holders: Sarah Walker, Tenshi
   see COPYING for more details
*/
extern void (*keyboard_send)(uint8_t val);
extern void (*keyboard_poll)(void);
extern int keyboard_scan;

extern int pcem_key[272];
extern uint8_t mode;
void keyboard_process(void);

extern int set3_flags[272];
extern uint8_t set3_all_repeat;
extern uint8_t set3_all_break;
