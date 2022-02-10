#include <i2c_fifo.h>
#include <i2c_slave.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "terminal.h"

static const uint I2C_SLAVE_ADDRESS = 0x40;
static const uint I2C_SLAVE2_ADDRESS = 0x41;
static const uint I2C_BAUDRATE = 1000000; // 1000 kHz

#define LED_PIN PICO_DEFAULT_LED_PIN // 25

static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5

static const uint I2C_SLAVE2_SDA_PIN = 6;
static const uint I2C_SLAVE2_SCL_PIN = 7;

static float sCenteredAngle[18] = { 1.602f, 1.453f, 1.681f, 1.429f, 1.343f, 1.728f, 1.319f, 1.367f, 1.728f, 1.210f, 1.319f, 1.581f, 1.257f, 1.312f, 1.602f, 1.335f, 1.367f, 1.539f};

#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI      6.283185307179586476925286766559f
#define DEG_TO_RAD  0.017453292519943295769236907684886f
#define RAD_TO_DEG  57.295779513082320876798154814105f

// The slave implements a 256 byte memory. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct I2CMemContext
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context[2];

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
  uint i2c_index = i2c_hw_index(i2c);
  struct I2CMemContext* cxt = &context[i2c_index];
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!cxt->mem_address_written) {
            // writes always start with the memory address
            cxt->mem_address = i2c_read_byte(i2c);
            cxt->mem_address_written = true;
        } else {
            // save into memory
            cxt->mem[cxt->mem_address] = i2c_read_byte(i2c);
            cxt->mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte(i2c, cxt->mem[cxt->mem_address]);
        cxt->mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        cxt->mem_address_written = false;
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    gpio_init(I2C_SLAVE2_SDA_PIN);
    gpio_set_function(I2C_SLAVE2_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE2_SDA_PIN);

    gpio_init(I2C_SLAVE2_SCL_PIN);
    gpio_set_function(I2C_SLAVE2_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE2_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);
    i2c_slave_init(i2c1, I2C_SLAVE2_ADDRESS, &i2c_slave_handler);
}

typedef struct
{
  float x;
  float y;
  float z;
} leg_t;
static leg_t legs[6];
static float servo_angle[18];

void compute_leg_positions()
{
  const float L[3] = { 137.29f, 93.1f, 45.1f };

  for (int i = 0; i < 6; ++i)
  {
    float* angle = &servo_angle[i*3];
    leg_t* leg = &legs[i];
    float alpha = PI - angle[1] - angle[0];
    leg->z = cosf(angle[1]) * L[1] - cosf(alpha) * L[0];
    float u = sinf(angle[1]) * L[1] + sinf(alpha) * L[0] + L[2];
    leg->x = sinf(angle[2]) * u;
    leg->y = cosf(angle[2]) * u;
  }
}

int main() {
    // Call this first as it changes the system clock
    terminal_start();

    stdio_init_all();
    puts("\x1b[2J\x1b[H");

    setup_slave();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    sleep_ms(500);
    gpio_put(LED_PIN, 0);

    char buf[80];

    const float fSERVO_FREQ = 50;
    const float pulseDuration = 1000.0f / (fSERVO_FREQ * 4096);

		while (1) 
    {
      for (int i = 0; i < 9; ++i)
      {
        uint16_t on = context[0].mem[6+4*i] + (((uint16_t)context[0].mem[6+4*i+1]) << 8);
        uint16_t off = context[0].mem[6+4*i+2] + (((uint16_t)context[0].mem[6+4*i+3]) << 8);
        float pulseInMilliseconds = off * pulseDuration;
        float angle = (pulseInMilliseconds - 1.5f) * HALF_PI + sCenteredAngle[i];
        servo_angle[i] = angle;
        angle *= RAD_TO_DEG;
        sprintf(buf, "Servo %d: %04hu %04hu %4.0f", i, on, off, angle);
        puts(buf);
        //terminal_puts(0, i, buf);
      }

      for (int i = 1; i < 10; ++i)
      {
        uint16_t on = context[1].mem[6+4*i] + (((uint16_t)context[1].mem[6+4*i+1]) << 8);
        uint16_t off = context[1].mem[6+4*i+2] + (((uint16_t)context[1].mem[6+4*i+3]) << 8);
        float pulseInMilliseconds = off * pulseDuration;
        float angle = (pulseInMilliseconds - 1.5f) * HALF_PI + sCenteredAngle[18-i];
        servo_angle[18-i] = angle;
        angle *= RAD_TO_DEG;
        sprintf(buf, "Servo %02d: %04hu %04hu %4.0f", 18-i, on, off, angle);
        puts(buf);
        //terminal_puts(32, 9-i, buf);
      }

      compute_leg_positions();
      for (int i = 0; i < 6; ++i)
      {
        sprintf(buf, "Leg %d: (%5.1f, %5.1f, %5.1f)", i, legs[i].x, legs[i].y, legs[i].z);
        puts(buf);
      }
      puts("\x1b[H");
      sleep_ms(1);
    }

    return 0;
}
