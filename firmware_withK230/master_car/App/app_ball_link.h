#ifndef __APP_BALL_LINK_H
#define __APP_BALL_LINK_H

#include <stdint.h>

/* K230 main_k230_ball_rect_fast.py BALL_PACKET_V1. */
#define BALL_LINK_FRAME_SIZE       14U
#define BALL_LINK_HEADER_0         0xAAU
#define BALL_LINK_HEADER_1         0x55U
#define BALL_LINK_VERSION          0x01U
#define BALL_LINK_MESSAGE_BALL     0x21U

#define BALL_LINK_FLAG_BALL_VALID  0x01U
#define BALL_LINK_FLAG_PIPE_VALID  0x02U

typedef struct
{
    uint16_t sequence;
    uint8_t flags;
    uint8_t confidence;
    uint16_t ballXPixel;
    uint16_t positionCentiCm;
    uint32_t receiveTimeMs;
    uint8_t transportValid;
    uint8_t ballValid;
    uint8_t pipeValid;
} BallLinkFrame_t;

void App_BallLink_Init(void);
void App_BallLink_Reset(void);
void App_BallLink_Task10ms(void);

uint8_t App_BallLink_GetLatest(BallLinkFrame_t *frame);
uint8_t App_BallLink_HasNewFrame(void);
uint32_t App_BallLink_GetFrameAgeMs(uint32_t nowMs);
uint32_t App_BallLink_GetValidFrameCount(void);
uint32_t App_BallLink_GetCrcErrorCount(void);
uint32_t App_BallLink_GetRxOverflowCount(void);

#endif
