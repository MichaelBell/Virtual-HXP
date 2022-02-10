#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "terminal.h"
#include "VHXPData.h"

#define LED_PIN PICO_DEFAULT_LED_PIN // 25

static VHXPData sHxpData;

int main() {
    // Call this first as it changes the system clock
    terminal_start();

    stdio_init_all();
    puts("\x1b[2J\x1b[H");

    sHxpData.Init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    sleep_ms(500);
    gpio_put(LED_PIN, 0);

    char buf[80];

		while (1) 
    {
      sHxpData.ReadData();
      sHxpData.ComputeLegPositions();
      for (int i = 0; i < 6; ++i)
      {
        RDMVec legPos = sHxpData.GetLegPosition(i);
        sprintf(buf, "Leg %d: (%5.1f, %5.1f, %5.1f)", i, legPos.x, legPos.y, legPos.z);
        puts(buf);
      }
      puts("\x1b[H");
      sleep_ms(1);
    }

    return 0;
}
