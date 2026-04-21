/*
 * Tobias Mages & Wenqing Yan
 * Course: Wireless Communication and Networked Embedded Systems, Project VT2023
 * Backscatter PIO
 * 29-March-2023
 */

#include "backscatter.h"

const uint16_t nop = 0xa342; //101 00011 010 00 010 MOV delay 3 Y -> Y
const uint16_t extra_instr = 1;
const int length = 21;
uint16_t instr1[21] = {     
    63489, 24768, 24640, 24609, 0x208F,   //001 (WAI) 00000 1(POL) 00(GPIO) 01111(GPIO15)
    44 + extra_instr,
    40994, 65281, 63489, 63232,
    70 + extra_instr,
    64257,
    3,
    40998,
    65281, 63745, 63232, 61440,
    77 + extra_instr,
    64769,
    3
};

uint16_t instr2[20] = {     
    63489, 24768, 24640, 24609,   //001 (WAI) 00000 1(POL) 00(GPIO) 01111(GPIO15)
    44 + extra_instr,
    40994, 0xf701, 0xf001, 0xff00,
    70 + extra_instr,
    64257,
    3,
    40998,
    0xf701, 0xf101, 0xff00, 0xf800,
    77 + extra_instr,
    64769,
    3
};

/*uint16_t instr2[21] = {nop, 0xf801, 0x60c0, 0x6040, 0x6021, 0x002c, 0xa022, 0xff01, 0xf801, 
                        0xf700, 0x0046, 0xfb01, 0x0003, 0xa026, 0xff01, 0xf901, 0xf700, 
                        0xf000, 0x004d, 0xfd01, 0x0003};*/

// repeat the instruction until the desired delay has past
int16_t repeat(uint16_t* instructionBuffer, int16_t delay, uint32_t asm_instr, uint8_t *length, uint16_t max_delay){
    while(delay > 0){
        uint8_t delay_part = min(max_delay, delay) - 1;
        instructionBuffer[(*length)] = asm_instr | (((max_delay-1) & delay_part) << 8);
        delay = delay - (delay_part + 1);
        (*length)++;
    }
}

// how many instructions are needed to create this delay?
uint8_t instructionCount(uint16_t delay, uint16_t max_delay){
    if (delay % max_delay == 0){
        return delay/max_delay;
    }else{
        return delay/max_delay + 1;
    }
}

/* 
    - based on d0/d1/baud, the modulation parameters will be computed and returned in the struct backscatter_config 
    - pin2 is ignored if twoAntennas==falsev
*/
void backscatter_program_init(PIO pio, uint sm, uint pin1, uint pin2, uint16_t d0, uint16_t d1, uint32_t baud, struct backscatter_config *config){
    pio_set_sm_mask_enabled(pio, sm, false); // stop state machine if running
    // print warning at invalid settings
    if(d0 % 2 != 0){
        printf("WARNING: the clock divider d0 has to be an even integer. The state-machine may not function correctly");
    }
    if(d1 % 2 != 0){
        printf("WARNING: the clock divider d1 has to be an even integer. The state-machine may not function correctly");
    }
    // correct baud-rate
    if(((uint32_t) (CLKFREQ*pow(10,6))) % baud != 0){
        uint32_t baud_new = round(((uint32_t) (CLKFREQ*pow(10,6))) / round(((double) CLKFREQ*pow(10,6)) / ((double) baud)));
        printf("WARNING: a baudrate of %d Baud is not achievable with a %d MHz clock.\nTherefore, the closest achievable baud-rate %d Baud will be used.\n", baud, CLKFREQ, baud_new);
        baud = baud_new;
    }
    // generate pio-program
    struct pio_program backscatter_program1;
    struct pio_program backscatter_program2;
    
    for(int i = 0; i < 20; i++){
        instr1[i] = instr1[i] & 0xE7FF;
        instr2[i] = instr2[i] & 0xE7FF;
        printf("%x\n", instr1[i]);
    }
    uint offset = 0;
    
    backscatter_program1.instructions = instr1;
    backscatter_program1.length = length+1;
    backscatter_program1.origin = -1;

    backscatter_program2.instructions = instr1;
    backscatter_program2.length = length+1;
    backscatter_program2.origin = -1;

    pio_add_program_at_offset(pio, &backscatter_program1, offset);
    pio_add_program_at_offset(pio, &backscatter_program2, offset);
    /* print state-machine instructions */
    /*printf("state-machine length: %d\n", backscatter_program.length);
    for (uint16_t t = 0; t < backscatter_program.length; t++){
        printf("0x%04x\n",backscatter_program.instructions[t]);
    }*/

    // configure the state-machine
    pio_gpio_init(pio, pin1);
    pio_sm_set_consecutive_pindirs(pio, 0, pin1, 1, true);
    pio_gpio_init(pio, pin2);
    pio_sm_set_consecutive_pindirs(pio, 1, pin2, 1, true);
    
    // setup default state-machine config
    pio_sm_config c1 = pio_get_default_sm_config();
    pio_sm_config c2 = pio_get_default_sm_config();
    sm_config_set_wrap(&c1, offset, offset + backscatter_program1.length-1); 
    sm_config_set_wrap(&c2, offset, offset + backscatter_program2.length-1); 
    // setup specific state-machine config
    sm_config_set_set_pins(&c1, pin1, 1);
    sm_config_set_set_pins(&c2, pin2, 1);
    
    sm_config_set_fifo_join(&c1, PIO_FIFO_JOIN_TX); // We only need TX, so get an 8-deep FIFO (join RX and TX FIFO)
    sm_config_set_fifo_join(&c2, PIO_FIFO_JOIN_TX);

    sm_config_set_out_shift(&c1, false, true, 32);  // OUT shifts to left (MSB first), autopull after every 32 bit
    sm_config_set_out_shift(&c2, false, true, 32);

    uint sm0 = 0;
    uint sm1 = 1;
    pio_sm_init(pio, sm0, offset, &c1);
    pio_sm_init(pio, sm1, offset, &c2);
    
    pio_enable_sm_mask_in_sync(pio, sm);

    uint32_t reps0 = ((CLKFREQ*1000000/baud - 4) / d0) - 1; // (((125*1000000/100000)-4) / 20) - 1 = 61
    uint32_t reps1 = ((CLKFREQ*1000000/baud - 4) / d1) - 1; // ((125*1000000/100000 - 4) / 18) - 1 = 68
    //printf("Reps0: %d\n", reps0);
    //printf("Reps1: %d\n", reps1);
    pio_sm_put_blocking(pio, 0, reps0); // -1 is requried since JMP 0-- is still true
    pio_sm_put_blocking(pio, 0, reps1); // -1 is required since JMP 0-- is still true
    pio_sm_put_blocking(pio, 1, reps0); // -1 is requried since JMP 0-- is still true
    pio_sm_put_blocking(pio, 1, reps1); // -1 is required since JMP 0-- is still true

    // compute configuration parameters
    uint32_t fcenter    = (CLKFREQ*1000000/d0 + CLKFREQ*1000000/d1)/2;
    uint32_t fdeviation = abs(round((((double) CLKFREQ*1000000)/((double) d1)) - ((double) fcenter)));
    config->baudrate    = baud;
    config->center_offset = round(fcenter);
    config->deviation   = round(fdeviation);
    config->minRxBw     = round((baud + 2*fdeviation));
    
    if (fdeviation > 380000){
        printf("WARNING: the deviation is too large for the CC2500\n");
    }
    if (fdeviation > 1000000){
        printf("WARNING: the deviation is too large for the CC1352\n");
    }
    if (d0 < d1){
        printf("WARNING: symbol 0 has been assigned to larger frequncy than symbol 1\n");
    }

    printf("Computed baseband settings: \n- baudrate: %d\n- Center offset: %d\n- deviation: %d\n- RX Bandwidth: %d\n", config->baudrate, config->center_offset, config->deviation, config->minRxBw);
}

void backscatter_send(PIO pio, uint sm, uint32_t *message, uint32_t len) {
    /* Implement mutex here!!! */
    for(uint32_t i = 0; i < len; i++){
        pio_sm_put_blocking(pio, sm, message[i]); // set pin back to low
        pio_sm_put_blocking(pio, sm, message[i]);
        gpio_put(15, 1);
        sleep_ms(5);
        gpio_put(15, 0);
    }
    sleep_ms(1); // wait for transmission to finish
}
