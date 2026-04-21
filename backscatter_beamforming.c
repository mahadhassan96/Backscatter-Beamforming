/*
    Modified code, group 4
    Beamforming Backscatter main file
*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"

#include "pico/util/queue.h"
#include "pico/binary_info.h"
#include "pico/util/datetime.h"
#include "hardware/spi.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "backscatter.h"
#include "carrier_CC2500.h"
#include "receiver_CC2500.h"
#include "packet_generation.h"

#define TRIGGER_PIN             15
#define RADIO_SPI             spi0
#define RADIO_MISO              16
#define RADIO_MOSI              19
#define RADIO_SCK               18

#define TX_DURATION            100 // send a packet every 250ms (when changing baud-rate, ensure that the TX delay is larger than the transmission time)
#define RECEIVER              1352 // define the receiver board either 2500 or 1352
#define PIN_TX1                  6
#define PIN_TX2                 27
#define CLOCK_DIV0              20 // larger
#define CLOCK_DIV1              18 // smaller
#define DESIRED_BAUD        100000
#define TWOANTENNAS          true

#define CARRIER_FEQ     2450000000

// Initialize the GPIO for the LED
void pico_led_init(void) {
#ifdef PICO_DEFAULT_LED_PIN
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
}

int main() {
    /*Set up LED*/
    stdio_init_all();
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0); // Start LOW

    sleep_ms(5000);

    /* setup backscatter state machine */
    PIO pio = pio0;
    uint sm = 3;
    struct backscatter_config backscatter_conf;
    backscatter_program_init(pio, sm, PIN_TX1, PIN_TX2, CLOCK_DIV0, CLOCK_DIV1, DESIRED_BAUD, &backscatter_conf);

    static uint8_t message[buffer_size(PAYLOADSIZE+2, HEADER_LEN)*4] = {0};  // include 10 header bytes
    static uint32_t buffer[1] = {0xffffffff}; // initialize the buffer
    static uint8_t seq = 0;
    uint8_t *header_tmplate = packet_hdr_template(RECEIVER);
    uint8_t tx_payload_buffer[PAYLOADSIZE];
    
    for(int i = 0; i < PAYLOADSIZE;i++){tx_payload_buffer[i] = i;}
    /* loop */
    while (true) {
        //generate_data(tx_payload_buffer, PAYLOADSIZE, true);
        //for(int i = 0; i < PAYLOADSIZE; i++){printf("TX Palyoad %d: %d\n ",i, tx_payload_buffer[i]);}
        //printf("\n");
        /* add header (10 byte) to packet */
        //add_header(&message[0], seq, header_tmplate);
        /* add payload to packet */
        //memcpy(&message[HEADER_LEN], tx_payload_buffer, PAYLOADSIZE);

        //int buffersize = buffer_size(PAYLOADSIZE, HEADER_LEN);
        //printf("Buffer size: %d\n", buffersize);
        /* casting for 32-bit fifo */
        /*for (uint8_t i=0; i < buffer_size(PAYLOADSIZE, HEADER_LEN); i++) {
            buffer[i] = ((uint32_t) message[4*i+3]) | (((uint32_t) message[4*i+2]) << 8) | (((uint32_t) message[4*i+1]) << 16) | (((uint32_t)message[4*i]) << 24);
            printf("Buffer %d: %08x\n", i, buffer[i]);
        }*/
        /* put the data to FIFO (start backscattering) */
        backscatter_send(pio,0,buffer,1);
        backscatter_send(pio,1,buffer,1);
        //sleep_ms(ceil((((double) buffer_size(PAYLOADSIZE, HEADER_LEN))*8000.0)/((double) DESIRED_BAUD))+3); // wait transmission duration (+3ms)
        /* increase seq number*/ 
        seq++;
        sleep_ms(TX_DURATION);

        //break;
    }

    /* stop carrier and receiver - never reached */
    RX_stop_listen();
    stopCarrier();
}
