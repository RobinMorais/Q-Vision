#ifndef _FACEID_H_
#define _FACEID_H_

#define USE_BOX_ONLY
//#define UNNORMALIZE_RECORD // Do not normalize the recorded embeddings

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

#define CAPTURE_X 100
#define CAPTURE_Y 300

#define HEIGHT_ID 112
#define WIDTH_ID 112
#define THICKNESS 1 //4

#define FRAME_ORANGE 0xFD20

#define MAX_X_OFFSET 23 //(WIDTH_DET - WIDTH)/2 // 24 pixels
#define MAX_Y_OFFSET 31 //(HEIGHT_DET - HEIGHT)/2 // 32 pixels

// Data input: HWC (little data): 160x120x3
#define DATA_SIZE_IN_ID (112 * 112 * 3)

void set_fname(char n);
char get_fname(void);

int face_id(void);

#endif // _FACEID_H_
