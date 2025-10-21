#include "comm.h"
#include "dma.h"
#include "uart.h"
#include "nvic_table.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include <stdio.h>
#include <string.h>

static mxc_uart_req_t rx_req;
static uint8_t rx_byte;
static frame last_frame;
static bool msg_ready = false;

// Parser buffer
static uint8_t buf[128];
static uint8_t count = 0;

static inline uint8_t checksum(const uint8_t *b, uint8_t len_wo_chk) {
    uint8_t s = 0;
    for (uint8_t i = 0; i < len_wo_chk; i++) s += b[i];
    return (uint8_t)(-s);
}

static void parse_byte(uint8_t byte) {
    // adds new byte to buff when count is smaller than buf size
    if (count < sizeof(buf)) buf[count++] = byte;
    // if count gets bigger than total buff size drop oldest byte
    else { memmove(buf, buf + 1, --count); buf[count++] = byte; }
    // needs at least 3 bytes to read len and nl len
    while (count >= 3) {
        // 
        uint8_t L = buf[1], nL = buf[2];
        if ((L ^ nL) != 0xFF || L < 5) { memmove(buf, buf + 1, --count); continue; }
        if (count < L) break;
        // checks if message is complete
        if (checksum(buf, L - 1) != buf[L - 1]) { memmove(buf, buf + 1, --count); continue; }

        // Valid frame
        last_frame.addr = buf[0];
        last_frame.len = L;
        last_frame.nlen = nL;
        last_frame.plen = L - 4;
        memcpy(last_frame.payload, &buf[3], last_frame.plen);
        last_frame.chk = buf[L - 1];
        msg_ready = true;

        count -= L;
        if (count) memmove(buf, buf + L, count);
    }
}

// non blocking callback when one byte is received
static void rx_cb(mxc_uart_req_t *req, int error) {
    if (error == E_NO_ERROR) parse_byte(rx_byte);
    rx_req.uart = MXC_UART1;
    rx_req.rxData = &rx_byte;
    rx_req.rxLen = 1;
    rx_req.callback = rx_cb;
    MXC_UART_TransactionAsync(&rx_req);
}

// handler function for uart 
void UART1_Handler(void) { MXC_UART_AsyncHandler(MXC_UART1); }

// initialize uart whith baud rate
int comm_init(uint32_t baud) {
    int e = MXC_UART_Init(MXC_UART1, baud, MXC_UART_APB_CLK);
    if (e != E_NO_ERROR) return e;

    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_DisableIRQ(UART1_IRQn);
    MXC_NVIC_SetVector(UART1_IRQn, UART1_Handler);
    NVIC_SetPriority(UART1_IRQn,1);
    NVIC_EnableIRQ(UART1_IRQn);
    return E_NO_ERROR;
}

// start reading the message
int comm_start(void) {
    rx_req.uart = MXC_UART1;
    rx_req.rxData = &rx_byte;
    rx_req.rxLen = 1;
    rx_req.callback = rx_cb;
    return MXC_UART_TransactionAsync(&rx_req);
}
// used for polling
bool comm_message_ready(void) { return msg_ready; }


bool comm_get_message(frame *out) {
    if (!msg_ready) return false;
    *out = last_frame;
    msg_ready = false;
    return true;
}

// sending the message over uart
int comm_send(uint8_t a, uint8_t c, const uint8_t *d, uint8_t l) {
    uint8_t L = 5 + l;
    uint8_t f[255];
    f[0] = a; f[1] = L; f[2] = ~L; f[3] = c;
    memcpy(&f[4], d, l);
    f[L - 1] = checksum(f, L - 1);


    mxc_uart_req_t tx = { .uart = MXC_UART1, .txData = f, .txLen = L };
    return (MXC_UART_Transaction(&tx) == E_NO_ERROR) ? L : 0;
}