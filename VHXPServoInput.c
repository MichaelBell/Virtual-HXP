#include <i2c_fifo.h>
#include <i2c_slave.h>
#include <pico/stdlib.h>

static const uint I2C_SLAVE_ADDRESS = 0x40;
static const uint I2C_SLAVE2_ADDRESS = 0x41;
static const uint I2C_BAUDRATE = 1000000; // 1000 kHz

static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5

static const uint I2C_SLAVE2_SDA_PIN = 6;
static const uint I2C_SLAVE2_SCL_PIN = 7;

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

void servo_input_start() {
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

uint16_t servo_get_on_duration(int board, int idx)
{
  return context[board].mem[6+4*idx] + (((uint16_t)context[board].mem[6+4*idx+1]) << 8);
}

uint16_t servo_get_off_duration(int board, int idx)
{
  return context[board].mem[6+4*idx+2] + (((uint16_t)context[board].mem[6+4*idx+3]) << 8);
}

