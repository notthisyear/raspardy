#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

/* Pin mapping */
const uint kUartTxPin = 0U;
const uint kUartRxPin = 1U;

const uint kPlayer1InputGpioPin = 2U;
const uint kPlayer2InputGpioPin = 3U;
const uint kPlayer3InputGpioPin = 4U;
const uint kPlayer4InputGpioPin = 5U;
const uint kPlayer5InputGpioPin = 6U;
const uint kPlayer6InputGpioPin = 7U;

const uint kPlayer1OutputGpioPin = 21U;
const uint kPlayer2OutputGpioPin = 20U;
const uint kPlayer3OutputGpioPin = 19U;
const uint kPlayer4OutputGpioPin = 18U;
const uint kPlayer5OutputGpioPin = 17U;
const uint kPlayer6OutputGpioPin = 16U;

/* Command IDs */
typedef enum 
{
    PING = 1,
    LIGHT_ON = 2,
    LIGHT_OFF = 3,
    ENABLE_INPUT = 4,
    DISABLE_INPUT = 5
} CommandId;

/* UART settings */
#define UART_ID uart0
const uint kBaudRate = 115200U;
const uint kDataBits = 8U;
const uint kStopBits = 1U;
const uart_parity_t kParity = UART_PARITY_NONE;
const bool kUseClearToSend = false;
const bool kUseRequestToSend = false;

/* Program variables */
#define SERIAL_TX_QUEUE_LENGTH 32
typedef struct
{
    char *data;
    size_t length;
} SerialMsgData;
    
static queue_t serial_msg_tx_queue;
static uint actual_baudrate;

#define NUMBER_OF_IO_PINS 6
static uint player_index_to_output_pin_map[NUMBER_OF_IO_PINS];
static uint64_t last_high_input_received[NUMBER_OF_IO_PINS];

#define INPUT_QUEUE_LENGTH 128
typedef struct
{ 
    uint8_t event;
    uint8_t player_idx;
} InputEventData;

static queue_t input_queue;
static timer_hw_t *default_timer;
static bool input_allowed;
const uint64_t kMinHighTimeBeforeNewInputUs = 40000U;

SerialMsgData* get_serial_msg_data(size_t length)
{
    SerialMsgData* msg = malloc(sizeof(SerialMsgData));
    msg->data = malloc(sizeof(char) * length);
    msg->length = length;
    return msg;
}

void add_response_to_tx_queue(char data)
{
    SerialMsgData* msg = get_serial_msg_data(1);
    msg->data[0] = data;
    (void)queue_try_add(&serial_msg_tx_queue, msg);
    free(msg);
}

int get_player_index_for_input_pin(uint gpio_pin)
{
    switch (gpio_pin)
    {
    case kPlayer1InputGpioPin:
        return 0;
    case kPlayer2InputGpioPin:
        return 1;
     case kPlayer3InputGpioPin:
        return 2;
     case kPlayer4InputGpioPin:
        return 3;
     case kPlayer5InputGpioPin:
        return 4;
     case kPlayer6InputGpioPin:
        return 5;
    default:
        return -1;
    }
}

/* GPIO functions */
void on_gpio_input(uint gpio_pin, uint32_t events)
{
    int player_idx = get_player_index_for_input_pin(gpio_pin);
    if (player_idx == -1)
    {
        return;
    }

    InputEventData event_data = {.event = (uint8_t)events, .player_idx = (uint8_t)player_idx};
    queue_add_blocking(&input_queue, &event_data);
}

/* Command handling */
void handle_command(CommandId cmd)
{
    printf("received command id = %d\n", cmd);
    switch (cmd)
    {
    case PING:
        add_response_to_tx_queue((char)0xff);
        break;
    case LIGHT_ON:
    case LIGHT_OFF:
        uint output_pin = player_index_to_output_pin_map[(size_t)uart_getc(UART_ID)];
        printf("setting output pin %d to %d\n", output_pin, cmd == LIGHT_ON ? 1 : 0);
        gpio_put(output_pin, cmd == LIGHT_ON ? 1 : 0);
        break;
    case ENABLE_INPUT:
        input_allowed = true;
        break;
    case DISABLE_INPUT:
        input_allowed = false;
        break;
    }
}

/* UART functions */
void on_uart_rx()
{
    while (uart_is_readable(UART_ID))
    {
        handle_command((CommandId)uart_getc(UART_ID));
    }
}

void send_message(SerialMsgData* msg)
{
    for (size_t i = 0; i < msg->length; ++i)
    {
        uart_putc_raw(UART_ID, msg->data[i]);
    }
    free(msg->data);
}

/* Setup functions */
void setup_uart()
{
    // Set UART parameters
    actual_baudrate = uart_init(UART_ID, kBaudRate);
    uart_set_hw_flow(UART_ID, kUseClearToSend, kUseRequestToSend);
    uart_set_format(UART_ID, kDataBits, kStopBits, kParity);

    // Set UART RX/TX pins
    gpio_set_function(kUartTxPin, UART_FUNCSEL_NUM(UART_ID, kUartTxPin));
    gpio_set_function(kUartRxPin, UART_FUNCSEL_NUM(UART_ID, kUartRxPin));

    // Set UART IRQ handler and enable interrupts
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    // Set the binary fixed pin info for the UART
    bi_decl(bi_2pins_with_func(kUartRxPin, kUartTxPin, GPIO_FUNC_UART));
}

void setup_io_pins()
{
    uint const input_pins[] = {
        kPlayer1InputGpioPin,
        kPlayer2InputGpioPin,
        kPlayer3InputGpioPin,
        kPlayer4InputGpioPin,
        kPlayer5InputGpioPin,
        kPlayer6InputGpioPin
    };
    uint const output_pins[] = {
        kPlayer1OutputGpioPin,
        kPlayer2OutputGpioPin,
        kPlayer3OutputGpioPin,
        kPlayer4OutputGpioPin,
        kPlayer5OutputGpioPin,
        kPlayer6OutputGpioPin
    };

    for (size_t i = 0; i < NUMBER_OF_IO_PINS; i++)
    {
        gpio_init(input_pins[i]);
        gpio_init(output_pins[i]);

        gpio_set_dir(input_pins[i], GPIO_IN);
        gpio_set_dir(output_pins[i], GPIO_OUT);

        // Set input pins as pull-up and attach callback
        gpio_pull_up(input_pins[i]);
        gpio_set_irq_enabled_with_callback(input_pins[i], GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &on_gpio_input);
        
        // Ensure that all input timers are zeroed out
        last_high_input_received[i] = 0U;
        
        // Store the output pin
        player_index_to_output_pin_map[i] = output_pins[i];
    }

    bi_decl(bi_pin_mask_with_names(0x0F << 4, "Input (P1 - P4)"));
    bi_decl(bi_pin_mask_with_names(0x03 << 8, "Input (P5 - P6)"));
    bi_decl(bi_pin_mask_with_names(0x03 << 21, "Output (P6 - P5)"));
    bi_decl(bi_pin_mask_with_names(0x0F << 24, "Output (P4 - P1)"));
}

void input_thread_main()
{
    InputEventData entry;
    while (1)
    {
        queue_remove_blocking(&input_queue, &entry);

        uint64_t t = timer_time_us_64(default_timer);
        if ((entry.event & GPIO_IRQ_EDGE_FALL) > 0)
        {
            uint64_t diff = t - last_high_input_received[entry.player_idx];
            printf("FALL for IDX %d at %lld (diff: %lld)\n", entry.player_idx, t, diff);
            if (diff > kMinHighTimeBeforeNewInputUs)
            {
                printf("--- KEY PRESS PLAYER IDX %d ---\n",entry.player_idx);
                add_response_to_tx_queue((char)(entry.player_idx));
            }
        }        

        if ((entry.event & GPIO_IRQ_EDGE_RISE) > 0)
        {
            last_high_input_received[entry.player_idx] = t;
            printf("RISE for IDX %d at %lld\n", entry.player_idx, t);
        }
    }
}

int main()
{
    // Initialize inputs as allowed
    input_allowed = true;

    // Set binary infomration
    bi_decl(bi_program_description("The software for the Jeopardy controller"));
    bi_decl(bi_program_version_string("1.0.0"));

    // Store default timer
    default_timer = PICO_DEFAULT_TIMER_INSTANCE();
    
    // Setup input queue and lauch input thread on second core
    queue_init(&input_queue, sizeof(InputEventData), INPUT_QUEUE_LENGTH);
    queue_init(&serial_msg_tx_queue, sizeof(SerialMsgData), SERIAL_TX_QUEUE_LENGTH);
    multicore_launch_core1(input_thread_main);

    // Setup UART
    setup_uart();

    // Setup IO
    setup_io_pins();

    // Init stdio (debug)
    stdio_init_all();

    while (1)
    {
        SerialMsgData msg;
        while (queue_try_remove(&serial_msg_tx_queue, &msg))
        {
            send_message(&msg);
        }
        tight_loop_contents();
    }

    return 0;
}