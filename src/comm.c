#include "comm.h"
#include "dma.h"
#include "uart.h"
#include "nvic_table.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include <stdio.h>
#include <string.h>

// ---------- RX/TX request objects ----------
static mxc_uart_req_t rx_req;
static mxc_uart_req_t tx_req;

// ---------- RX state ----------
static uint8_t rx_byte;
static frame    last_frame;
static bool     msg_ready = false;

// ---------- Parser buffer ----------
static uint8_t buf[128];
static uint8_t count = 0;

// ---------- TX state ----------
static volatile bool tx_busy = false;
static uint8_t tx_buf[255];    // persistent buffer for async TX

// ---------- Helpers ----------
static inline uint8_t checksum(const uint8_t *b, uint8_t len_wo_chk) {
    uint8_t s = 0;
    for (uint8_t i = 0; i < len_wo_chk; i++) s += b[i];
    return (uint8_t)(-s);
}

// Hardened parser: guards length and caps payload copy
static void parse_byte(uint8_t byte) {
    if (count < sizeof(buf)) buf[count++] = byte;
    else { memmove(buf, buf + 1, --count); buf[count++] = byte; }

    while (count >= 3) {
        uint8_t L  = buf[1];
        uint8_t nL = buf[2];

        // Basic structural guards
        if ((L ^ nL) != 0xFF || L < 5 || L > sizeof(buf)) {
            memmove(buf, buf + 1, --count);
            continue;
        }
        if (count < L) break;

        // Checksum
        if (checksum(buf, L - 1) != buf[L - 1]) {
            memmove(buf, buf + 1, --count);
            continue;
        }

        // Valid frame -> copy safely
        last_frame.addr = buf[0];
        last_frame.len  = L;
        last_frame.nlen = nL;

        uint8_t total_plen = L - 4; // cmd + data
        uint8_t copy_len   = total_plen;
        if (copy_len > sizeof(last_frame.payload)) {
            copy_len = sizeof(last_frame.payload); // cap
            // Optional: set an overflow flag in your frame type if you have one
        }
        last_frame.plen = copy_len;
        memcpy(last_frame.payload, &buf[3], copy_len);

        last_frame.chk  = buf[L - 1];
        msg_ready = true;

        count -= L;
        if (count) memmove(buf, buf + L, count);
    }
}

// ---------- IRQ handler ----------
void UART1_Handler(void) { MXC_UART_AsyncHandler(MXC_UART1); }

// ---------- RX callback: parse + re-arm 1 byte ----------
static void rx_cb(mxc_uart_req_t *req, int error) {
    if (error == E_NO_ERROR) {
        parse_byte(rx_byte);
    } else {
        // If you want to be defensive, you can clear flags/FIFOs here.
        // For most MSDKs, re-arming is enough on transient errors.
    }

    rx_req.uart     = MXC_UART1;
    rx_req.rxData   = &rx_byte;
    rx_req.rxLen    = 1;
    rx_req.callback = rx_cb;
    (void)MXC_UART_TransactionAsync(&rx_req);
}

// ---------- TX callback: mark free ----------
static void tx_cb(mxc_uart_req_t *req, int error) {
    (void)req; (void)error;
    tx_busy = false;
}

// ---------- Init ----------
int comm_init(uint32_t baud) {
    int e = MXC_UART_Init(MXC_UART1, baud, MXC_UART_APB_CLK);
    if (e != E_NO_ERROR) return e;

    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_DisableIRQ(UART1_IRQn);
    MXC_NVIC_SetVector(UART1_IRQn, UART1_Handler);
    NVIC_SetPriority(UART1_IRQn, 1);
    NVIC_EnableIRQ(UART1_IRQn);

    return E_NO_ERROR;
}

// ---------- Start async RX (1 byte) ----------
int comm_start(void) {
    rx_req.uart     = MXC_UART1;
    rx_req.rxData   = &rx_byte;
    rx_req.rxLen    = 1;
    rx_req.callback = rx_cb;
    return MXC_UART_TransactionAsync(&rx_req);
}

// ---------- Polling helpers ----------
bool comm_message_ready(void) { return msg_ready; }

bool comm_get_message(frame *out) {
    if (!msg_ready) return false;
    *out = last_frame;
    msg_ready = false;
    return true;
}

// ---------- Send a frame (async TX) ----------
// full-duplex: RX stays armed; we use a persistent TX buffer + busy flag.
int comm_send(uint8_t a, uint8_t c, const uint8_t *d, uint8_t l) {
    uint8_t L = 5 + l;
    if (L > sizeof(tx_buf)) return 0;          // cap length
    if (tx_busy) return 0;                     // already sending

    // Build the frame into persistent buffer
    tx_buf[0] = a;
    tx_buf[1] = L;
    tx_buf[2] = (uint8_t)~L;
    tx_buf[3] = c;
    if (l && d) memcpy(&tx_buf[4], d, l);
    tx_buf[L - 1] = checksum(tx_buf, L - 1);

    // Prepare async TX request
    tx_busy        = true;
    tx_req         = (mxc_uart_req_t){0};
    tx_req.uart    = MXC_UART1;
    tx_req.txData  = tx_buf;
    tx_req.txLen   = L;
    tx_req.callback= tx_cb; // mark not-busy on completion

    int rc = MXC_UART_TransactionAsync(&tx_req);

    if (rc != E_NO_ERROR) {
        // If your MSDK does not allow concurrent RX+TX requests (some don't),
        // fall back to: abort RX, do TX, then re-arm RX.
        tx_busy = false;

        // --- Fallback path: half-duplex driver ---
        // MXC_UART_AbortAsync(&rx_req); // or MXC_UART_AbortAsync(MXC_UART1) on older SDKs
        // rc = MXC_UART_Transaction(& (mxc_uart_req_t){ .uart=MXC_UART1, .txData=tx_buf, .txLen=L } );
        // // Re-arm RX:
        // rx_req.uart     = MXC_UART1;
        // rx_req.rxData   = &rx_byte;
        // rx_req.rxLen    = 1;
        // rx_req.callback = rx_cb;
        // (void)MXC_UART_TransactionAsync(&rx_req);
        return 0;
    }

    return L; // scheduled successfully
}