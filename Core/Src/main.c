/* =====================================================================
 * LAB 2 - USART COMMAND PROTOCOL (8 LEDs over serial)
 * Board: NUCLEO-L053R8 (Cortex-M0+, bare-metal)
 *
 * WHAT THIS DOES:
 *   PC sends single-letter commands over USART2 @ 19200 baud; the board
 *   acts on 8 LEDs and replies. Commands:
 *     E<text> echo    N all-on    F all-off
 *     P<0-255> pattern (decimal)  Q report current LED byte (binary, MSB first)
 *   RX is interrupt-driven into a ring buffer; the main loop assembles
 *   lines and dispatches them.
 * ===================================================================== */

#include <stdint.h>
#include "stm32l0xx.h"

/* ---------------------------------------------------------------------
 * 8-LED array. led_pin[i] = the GPIOB pin carrying BIT i of the value.
 * Index 0 = LSB, index 7 = MSB. The pins are NON-CONTIGUOUS because the
 * others are taken by debug/USART/etc., so a lookup table maps logical
 * bit -> physical pin.
 * ------------------------------------------------------------------- */
static const uint8_t led_pin[8] = { 0, 1, 2, 5, 13, 8, 9, 10 };

/* Mask of all 8 LED pin bits: bits 0,1,2,5,8,9,10,13.
 * Work it out: 0x001|0x002|0x004|0x020|0x100|0x200|0x400|0x2000 = 0x2727.
 * Used to change only LED bits and leave the rest of port B alone. */
#define LED_MASK   0x2727U

/* ---------------------------------------------------------------------
 * leds_init - clock on, all 8 pins to output, all off.
 * The loop configures each pin generically using its NUMBER p:
 *   (2*p)      : each pin owns a 2-bit MODER field, so its field starts
 *                at bit 2*p (pin 0->bit0, pin 5->bit10, pin 13->bit26).
 *   3U<<(2*p)  : 3 = 0b11 marks BOTH bits of that field; shift to place.
 *                Clearing with &= ~ sets the field to 00 (input).
 *   1U<<(2*p)  : sets the low bit only -> field = 01 = output.
 * WHY 3 NOT 2? A field is 2 bits; I must clear both. 2 (0b10) would clear
 * only the high bit and leave the low bit intact - wrong.
 * ------------------------------------------------------------------- */
static void leds_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;         /* clock-gate GPIOB on */

    for (int i = 0; i < 8; i++) {
        uint32_t p = led_pin[i];
        GPIOB->MODER &= ~(3U << (2 * p));      /* clear the 2 mode bits -> 00 */
        GPIOB->MODER |=  (1U << (2 * p));      /* 01 = general-purpose output */
    }
    GPIOB->ODR &= ~LED_MASK;                   /* all LEDs off */
}

/* ---------------------------------------------------------------------
 * byte_to_pins - spread a logical byte across the scattered LED pins.
 * For each set bit i in v, set bit led_pin[i] in the output word.
 * This is the translation from "LED number" to "physical GPIO bit".
 * ------------------------------------------------------------------- */
static uint32_t byte_to_pins(uint8_t v)
{
    uint32_t out = 0;
    for (int i = 0; i < 8; i++) {
        if (v & (1U << i)) {                   /* is logical bit i on? */
            out |= (1U << led_pin[i]);         /* then light its real pin */
        }
    }
    return out;
}

/* ---------------------------------------------------------------------
 * pins_to_byte - the inverse: read the LED pins back into a logical byte.
 * Used by the Q command to report current state. Reads ODR (the level I
 * drove) and rebuilds the byte using the same mapping.
 * ------------------------------------------------------------------- */
static uint8_t pins_to_byte(void)
{
    uint32_t odr = GPIOB->ODR;
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) {
        if (odr & (1U << led_pin[i])) {        /* is this physical pin high? */
            v |= (1U << i);                    /* set the matching logical bit */
        }
    }
    return v;
}

/* Write a byte to the LEDs: clear only LED bits, OR in the remapped value. */
static void leds_write(uint8_t v)
{
    GPIOB->ODR = (GPIOB->ODR & ~LED_MASK) | byte_to_pins(v);
}

/* ---------------------------------------------------------------------
 * clock_init_hsi16 - switch SYSCLK to the 16 MHz internal oscillator.
 * WHY NEEDED? The L0 boots on MSI at ~2.1 MHz. My USART BRR value assumes
 * 16 MHz, so I must move to HSI16 first or the baud rate would be wrong.
 *   HSION      : turn HSI16 on.
 *   wait HSIRDY: don't switch until it's stable.
 *   CFGR SW=HSI: select HSI16 as system clock.
 *   wait SWS=HSI: confirm the hardware actually switched.
 * ------------------------------------------------------------------- */
static void clock_init_hsi16(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { }
}

/* ---------------------------------------------------------------------
 * USART2 setup.
 * BRR = fCK / baud = 16 000 000 / 19 200 = 833.33 -> 833 (integer).
 * Actual baud = 16e6/833 = 19207.7 -> 0.04% error, far inside the ~2-3%
 * a UART tolerates, so it works fine. (Common viva question.)
 * ------------------------------------------------------------------- */
#define BAUD_BRR   833U

static void usart2_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;        /* PA2/PA3 live on port A */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;      /* USART2 clock on */

    /* PA2=TX, PA3=RX. Set MODER field to 10 = alternate function so the
     * USART peripheral (not GPIO logic) drives these pins. MODEx_1 is the
     * "10" constant (the HIGH bit of the field). */
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    GPIOA->MODER |=  (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

    /* Pick WHICH alternate function: AF4 = USART2 on PA2/PA3 for the L0
     * family. (On F4 parts this is AF7 - family-specific, from the AF
     * table in the datasheet.) AFR[0] covers pins 0-7; each pin uses a
     * 4-bit nibble at 4*pin. Clear then set to 4. */
    GPIOA->AFR[0] &= ~((0xFU << (4 * 2)) | (0xFU << (4 * 3)));
    GPIOA->AFR[0] |=  ((4U   << (4 * 2)) | (4U   << (4 * 3)));

    USART2->CR1 = 0;                 /* disable while configuring */
    USART2->BRR = BAUD_BRR;          /* baud rate divisor */
    USART2->CR2 = 0;                 /* 1 stop bit (default) */
    USART2->CR3 = 0;                 /* no hardware flow control */

    /* TE=transmitter, RE=receiver, RXNEIE=interrupt when a byte arrives. */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* Enable the USART2 interrupt line in the NVIC.
     * Cortex-M0+ has only 4 priority levels (0-3); 1 is fine. */
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

    /* UE = USART enable, set LAST so the peripheral starts fully configured.
     * (Setting UE before config can latch half-set state.) */
    USART2->CR1 |= USART_CR1_UE;
}

/* Blocking single-char send: wait until TXE (transmit register empty),
 * then write the byte. TXE=1 means the data register can take a new byte. */
static void usart_send_char(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE)) { }
    USART2->TDR = (uint8_t)c;
}

static void usart_send_str(const char *s)
{
    while (*s) { usart_send_char(*s++); }
}

/* ---------------------------------------------------------------------
 * RX RING BUFFER, filled by the ISR, drained by main.
 * head = where the ISR writes next; tail = where main reads next.
 * Decoupling IRQ from main this way means no bytes are lost while main is
 * busy handling a command.
 * ------------------------------------------------------------------- */
#define RXBUF_SIZE 64
static volatile char    rx_buf[RXBUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

/* Kept global (not local) so I can watch them live in the debugger. */
volatile uint32_t int_val    = 0;
volatile uint8_t  usart_data = 0;

/* ISR: runs automatically whenever a byte arrives. */
void USART2_IRQHandler(void)
{
    /* Read the RXNE flag by masking the status register. */
    int_val = USART2->ISR & USART_ISR_RXNE;

    if (int_val) {
        /* Reading RDR returns the byte AND clears RXNE automatically. */
        usart_data = (uint8_t)(USART2->RDR & 0xFFU);

        /* Compute next head; % wraps it around the circular buffer. */
        uint8_t next = (uint8_t)((rx_head + 1U) % RXBUF_SIZE);
        if (next != rx_tail) {                 /* if not full... */
            rx_buf[rx_head] = (char)usart_data; /* store byte */
            rx_head = next;                     /* advance head */
        }                                       /* else: drop byte (full) */
    }

    /* OVERRUN handling: if a byte arrived before I read the last one, ORE
     * latches and would STALL all further reception until cleared. RXNE
     * self-clears on RDR read, but ORE does NOT - I must clear it via ICR. */
    if (USART2->ISR & USART_ISR_ORE) {
        USART2->ICR = USART_ICR_ORECF;         /* overrun clear flag */
    }
}

/* main pulls one byte from the ring buffer; returns 0 if empty. */
static int rx_get(char *c)
{
    if (rx_head == rx_tail) { return 0; }      /* empty */
    *c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) % RXBUF_SIZE);
    return 1;
}

/* ---------------------------------------------------------------------
 * COMMAND DISPATCH - first char selects the action.
 * ------------------------------------------------------------------- */
static void handle_line(const char *line)
{
    if (line[0] == '\0') { return; }           /* ignore empty line */

    switch (line[0]) {

    case 'E': case 'e':                        /* ECHO the whole packet */
        usart_send_str(line);
        usart_send_str("\r\n");
        break;

    case 'N': case 'n':                        /* all LEDs ON (0xFF) */
        leds_write(0xFFU);
        usart_send_str(line);
        usart_send_str("\r\n");
        break;

    case 'F': case 'f':                        /* all LEDs OFF (0x00) */
        leds_write(0x00U);
        usart_send_str(line);
        usart_send_str("\r\n");
        break;

    case 'P': case 'p': {                      /* PATTERN, decimal 0-255 */
        uint32_t value  = 0;
        int      digits = 0;
        const char *p   = line + 1;            /* skip the 'P' */

        /* Parse ASCII digits into a number. Reject any non-digit. */
        while (*p) {
            if (*p < '0' || *p > '9') { digits = -1; break; }
            value = value * 10U + (uint32_t)(*p - '0');  /* '5'-'0' = 5 */
            digits++;
            p++;
        }

        if (digits > 0 && value <= 255U) {     /* valid byte */
            leds_write((uint8_t)value);
            usart_send_str(line);
            usart_send_str("\r\n");
        } else {
            usart_send_str("ERR\r\n");         /* bad input */
        }
        break;
    }

    case 'Q': case 'q': {                      /* REPORT LED byte, MSB first */
        uint8_t v = pins_to_byte();
        char out[11];

        /* Build 8 chars, bit 7 down to bit 0. WHY (7-i) AND LEFT SHIFT?
         * The protocol wants MSB first: out[0] must be bit 7, out[7] bit 0.
         * (1U<<(7-i)) tests bit 7,6,...,0 in that order. Using right shift
         * (v>>i) would print LSB first - reversed. */
        for (int i = 0; i < 8; i++) {
            out[i] = (v & (1U << (7 - i))) ? '1' : '0';
        }
        out[8]  = '\r';
        out[9]  = '\n';
        out[10] = '\0';                        /* C-string terminator */

        usart_send_str(out);
        break;
    }

    default:
        usart_send_str("ERR\r\n");             /* unknown command */
        break;
    }
}

/* ---------------------------------------------------------------------
 * MAIN - line assembler + dispatcher.
 * ------------------------------------------------------------------- */
#define LINE_MAX 32

int main(void)
{
    char    line[LINE_MAX];
    uint8_t len = 0;
    char    c;

    clock_init_hsi16();     /* 16 MHz first, so BRR is correct */
    leds_init();            /* 8 outputs, all off */
    usart2_init();          /* serial up, RX interrupt armed */

    usart_send_str("\r\nSTM32 ready\r\n");

    while (1) {
        if (rx_get(&c)) {                      /* got a byte from the ISR? */

            if (c == '\r' || c == '\n') {      /* CR/LF = end of packet */
                line[len] = '\0';              /* terminate the string */
                if (len > 0) { handle_line(line); }
                len = 0;                        /* reset for next line */
            }
            else if (len < (LINE_MAX - 1)) {   /* room left? */
                line[len++] = c;               /* store char */
            }
            else {
                len = 0;                        /* overlong -> discard */
                usart_send_str("ERR\r\n");
            }
        }
    }
}
