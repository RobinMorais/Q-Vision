#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <stdbool.h>


/* --------------------------------------------------------------------------
   Qbus frame format used by comm.c

   addr[1] | len[1] | nlen[1] | payload(cmd + data)[P] | chk[1]

   - len  : TOTAL bytes of the frame (including addr..chk)
   - nlen : bitwise complement of len (nlen == ~len)
   - chk  : 2's-complement of the sum of all prior bytes in the frame
            i.e., (uint8_t)(- (sum(addr..payload-last))) == chk
   - P = len - 4 (because addr,len,nlen,chk take 4 bytes)
   -------------------------------------------------------------------------- */

#define COMM_MAX_FRAME_BYTES   255u
#define COMM_MAX_PAYLOAD       (COMM_MAX_FRAME_BYTES - 4u)

/* Public frame type returned by comm_get_message(). 
   payload[0] is the command byte; payload[1..plen-1] are data bytes. */
typedef struct {
    uint8_t  addr;
    uint8_t  len;
    uint8_t  nlen;
    uint8_t  chk;
    uint16_t plen;                           /* payload length = len - 4 */
    uint8_t  payload[COMM_MAX_PAYLOAD];      /* cmd + data[...] */
} frame;

/* --------------------------------------------------------------------------
   Public API
   -------------------------------------------------------------------------- */


int  comm_init(uint32_t baud);

/** Start asynchronous, one-byte-at-a-time RX on UART1. */
int  comm_start(void);

/** True if a complete frame has been parsed and is available to read. */
bool comm_message_ready(void);

/** Retrieve the last parsed frame (single-slot buffer).
 *  Returns true on success; false if no frame is ready.
 *  Reading the frame clears the ready flag.
 */
bool comm_get_message(frame *out);

/** Build and send a Qbus frame on UART2.
 *  @param a  destination address
 *  @param c  command byte (will be payload[0])
 *  @param d  pointer to data bytes to follow the command (may be NULL if l==0)
 *  @param l  length of data bytes (not counting the command)
 *  @return total bytes sent (L = 5 + l) on success, 0 on error.
 */
int  comm_send(uint8_t a, uint8_t c, const uint8_t *d, uint8_t l);

#ifdef __cplusplus
}
#endif

#endif /* COMM_H */