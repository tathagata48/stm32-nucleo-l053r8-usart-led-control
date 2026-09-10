/* =====================================================================
 * USART command/response protocol driving an 8-LED array
 * NUCLEO-L053R8, bare metal (no HAL)
 *
 * USART2 @ 19200 8N1, routed to the ST-LINK virtual COM port (PA2/PA3).
 * RXNE interrupt enabled; bytes are buffered in the ISR and parsed in
 * the main loop.
 *
 * 8-LED array on GPIOB: PB0, PB1, PB2, PB5, PB13, PB8, PB9, PB10
 * (PB3 = SWO and PB4 = NJTRST are avoided - they are debug pins.)
 *
 * Protocol - one command per line, terminated by CR or LF:
 *   E<text>   echo the whole packet back
 *   N         all LEDs on            -> repeats the command
 *   F         all LEDs off           -> repeats the command
 *   P<0-255>  set pattern to decimal -> repeats the command
 *   Q         query LED state        -> replies with 8 binary digits, MSB first
 * ===================================================================== */

#include <stdint.h>
#include "stm32l0xx.h"

/* ---------------------------------------------------------------------
 * 8-LED array
 * led_pin[i] gives the GPIOB pin number carrying bit i of the value.
 * Index 0 is the LSB, index 7 the MSB.
 * ------------------------------------------------------------------- */
static const uint8_t led_pin[8] = { 0, 1, 2, 5, 13, 8, 9, 10 };
#define LED_MASK   0x2727U        /* bits 0,1,2,5,8,9,10,13 */

static void leds_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    for (int i = 0; i < 8; i++) {
        uint32_t p = led_pin[i];
        GPIOB->MODER &= ~(3U << (2 * p));      /* clear the two mode bits */
        GPIOB->MODER |=  (1U << (2 * p));      /* '01' = general purpose output */
    }
    GPIOB->ODR &= ~LED_MASK;                   /* all off */
}

/* Spread a byte across the non-contiguous LED pins */
static uint32_t byte_to_pins(uint8_t v)
{
    uint32_t out = 0;
    for (int i = 0; i < 8; i++) {
        if (v & (1U << i)) {
            out |= (1U << led_pin[i]);
        }
    }
    return out;
}

/* Gather the LED pins back into a byte */
static uint8_t pins_to_byte(void)
{
    uint32_t odr = GPIOB->ODR;
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) {
        if (odr & (1U << led_pin[i])) {
            v |= (1U << i);
        }
    }
    return v;
}

static void leds_write(uint8_t v)
{
    /* OUTPUT via the Output Data Register */
    GPIOB->ODR = (GPIOB->ODR & ~LED_MASK) | byte_to_pins(v);
}

/* ---------------------------------------------------------------------
 * Clock
 * The L0 boots on MSI at roughly 2.1 MHz. The BRR value below assumes
 * 16 MHz, so switch SYSCLK to the HSI16 internal oscillator first.
 * ------------------------------------------------------------------- */
static void clock_init_hsi16(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { }
}

/* ---------------------------------------------------------------------
 * USART2
 * ------------------------------------------------------------------- */
#define BAUD_BRR   833U    /* 16 000 000 / 19 200 = 833.33 -> 833 */

static void usart2_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = USART2_TX, PA3 = USART2_RX, alternate function mode '10' */
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    GPIOA->MODER |=  (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

    /* Alternate function 4 selects USART2 on PA2/PA3 (datasheet AF table) */
    GPIOA->AFR[0] &= ~((0xFU << (4 * 2)) | (0xFU << (4 * 3)));
    GPIOA->AFR[0] |=  ((4U   << (4 * 2)) | (4U   << (4 * 3)));

    USART2->CR1 = 0;                 /* disable while configuring */
    USART2->BRR = BAUD_BRR;          /* (b) baud rate */
    USART2->CR2 = 0;                 /* 1 stop bit */
    USART2->CR3 = 0;                 /* no flow control */

    /* (a) transmitter and receiver, (c) RXNE interrupt generation */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* (d) enable the interrupt vector in the NVIC.
     * Cortex-M0+ supports priority levels 0-3 only. */
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

    /* (e) enable the peripheral */
    USART2->CR1 |= USART_CR1_UE;
}

static void usart_send_char(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE)) { }
    USART2->TDR = (uint8_t)c;
}

static void usart_send_str(const char *s)
{
    while (*s) {
        usart_send_char(*s++);
    }
}

/* ---------------------------------------------------------------------
 * Receive ring buffer, filled from the interrupt handler
 * ------------------------------------------------------------------- */
#define RXBUF_SIZE 64
static volatile char    rx_buf[RXBUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

/* Kept as globals rather than locals so both can be inspected live in the
 * debugger while bytes arrive. */
volatile uint32_t int_val    = 0;
volatile uint8_t  usart_data = 0;

void USART2_IRQHandler(void)
{
    /* Read the interrupt flag from the status register by masking the bit */
    int_val = USART2->ISR & USART_ISR_RXNE;

    if (int_val) {
        /* Reading RDR clears the RXNE flag */
        usart_data = (uint8_t)(USART2->RDR & 0xFFU);

        uint8_t next = (uint8_t)((rx_head + 1U) % RXBUF_SIZE);
        if (next != rx_tail) {                 /* drop the byte if full */
            rx_buf[rx_head] = (char)usart_data;
            rx_head = next;
        }
    }

    /* An overrun latches until cleared and would otherwise stall reception */
    if (USART2->ISR & USART_ISR_ORE) {
        USART2->ICR = USART_ICR_ORECF;
    }
}

static int rx_get(char *c)
{
    if (rx_head == rx_tail) {
        return 0;
    }
    *c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) % RXBUF_SIZE);
    return 1;
}

/* ---------------------------------------------------------------------
 * Command handling
 * ------------------------------------------------------------------- */
static void handle_line(const char *line)
{
    if (line[0] == '\0') {
        return;
    }

    switch (line[0]) {

    case 'E': case 'e':                        /* echo the packet */
        usart_send_str(line);
        usart_send_str("\r\n");
        break;

    case 'N': case 'n':                        /* all LEDs on */
        leds_write(0xFFU);
        usart_send_str(line);
        usart_send_str("\r\n");
        break;

    case 'F': case 'f':                        /* all LEDs off */
        leds_write(0x00U);
        usart_send_str(line);
        usart_send_str("\r\n");
        break;

    case 'P': case 'p': {                      /* pattern, decimal 0-255 */
        uint32_t value  = 0;
        int      digits = 0;
        const char *p   = line + 1;

        while (*p) {
            if (*p < '0' || *p > '9') {
                digits = -1;
                break;
            }
            value = value * 10U + (uint32_t)(*p - '0');
            digits++;
            p++;
        }

        if (digits > 0 && value <= 255U) {
            leds_write((uint8_t)value);
            usart_send_str(line);
            usart_send_str("\r\n");
        } else {
            usart_send_str("ERR\r\n");
        }
        break;
    }

    case 'Q': case 'q': {                      /* report LED state */
        uint8_t v = pins_to_byte();
        char out[11];

        for (int i = 0; i < 8; i++) {
            out[i] = (v & (1U << (7 - i))) ? '1' : '0';   /* MSB first */
        }
        out[8]  = '\r';
        out[9]  = '\n';
        out[10] = '\0';

        usart_send_str(out);
        break;
    }

    default:
        usart_send_str("ERR\r\n");
        break;
    }
}

/* ---------------------------------------------------------------------
 * Main
 * ------------------------------------------------------------------- */
#define LINE_MAX 32

int main(void)
{
    char    line[LINE_MAX];
    uint8_t len = 0;
    char    c;

    clock_init_hsi16();
    leds_init();
    usart2_init();

    usart_send_str("\r\nSTM32 ready\r\n");

    while (1) {
        if (rx_get(&c)) {

            if (c == '\r' || c == '\n') {      /* end of packet */
                line[len] = '\0';
                if (len > 0) {
                    handle_line(line);
                }
                len = 0;
            }
            else if (len < (LINE_MAX - 1)) {
                line[len++] = c;
            }
            else {
                len = 0;                       /* overlong line, discard */
                usart_send_str("ERR\r\n");
            }
        }
    }
}
