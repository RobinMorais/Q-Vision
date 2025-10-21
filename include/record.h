#ifndef _RECORD_H_
#define _RECORD_H_

#define EMBEDDING_SIZE 64

#define SHOW_START_X (TFT_WIDTH - HEIGHT_ID) / 2
#define SHOW_START_Y (TFT_HEIGHT - WIDTH_ID) / 2

int record(uint8_t id);
void init_cnn_from_flash(void);

#endif // _RECORD_H_
