/* Copyright holders: Sarah Walker, Tenshi
   see COPYING for more details
*/
#define UNICODE
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#define BITMAP WINDOWS_BITMAP
#include <windows.h>
#undef BITMAP
#include "plat-keyboard.h"
#include "win.h"
#include "video.h"

extern "C" int recv_key[272];

extern "C" void fatal(const char *format, ...);
extern "C" void pclog(const char *format, ...);

extern "C" void keyboard_init(void);
extern "C" void keyboard_close(void);
extern "C" void keyboard_poll(void);

int recv_key[272];

void keyboard_init(void)
{
        atexit(keyboard_close);
        
        memset(recv_key, 0, sizeof(recv_key));
	pclog("Keyboard initialized!\n");
}

void keyboard_close(void)
{
}

void keyboard_poll_host(void)
{
#if 0
        int c;

        for (c = 0; c < 272; c++)
		recv_key[c] = rawinputkey[c];

         if ((rawinputkey[0x1D] || rawinputkey[0x9D]) && 
             (rawinputkey[0x38] || rawinputkey[0xB8]) && 
             (rawinputkey[0x51] || rawinputkey[0xD1]) &&
              video_fullscreen)
                 leave_fullscreen();

         if ((rawinputkey[0x1D] || rawinputkey[0x9D]) && 
//             (rawinputkey[0x38] || rawinputkey[0xB8]) && 
             (rawinputkey[0x57] || rawinputkey[0x57]))
	{
		pclog("Taking screenshot...\n");
                 take_screenshot();
	}
#endif
}
