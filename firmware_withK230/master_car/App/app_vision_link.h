#ifndef __APP_VISION_LINK_H
#define __APP_VISION_LINK_H

#include <stdint.h>

#define VISION_RX_FRAME_SIZE  64U

typedef struct
{
    uint16_t seq;

    int16_t  lateralErrorQ1000;
    int16_t  headingErrorDeciDeg;
    uint16_t laneWidthQ1000;

    uint8_t  flags;
    uint16_t junctionDistanceMm;
    uint8_t  confidence;

    uint32_t receiveTimeMs;
    uint8_t  frameValid;
} VisionTrackFrame_t;

void App_VisionLink_Init(void);
void App_VisionLink_Task10ms(void);
void App_VisionLink_Reset(void);

uint8_t  App_VisionLink_GetLatest(VisionTrackFrame_t *frame);
uint8_t  App_VisionLink_HasNewFrame(void);
uint8_t  App_VisionLink_IsFresh(uint32_t maxAgeMs);
uint32_t App_VisionLink_GetFrameAgeMs(void);

uint32_t App_VisionLink_GetValidFrameCount(void);
uint32_t App_VisionLink_GetParseErrorCount(void);
uint32_t App_VisionLink_GetDuplicateFrameCount(void);
uint32_t App_VisionLink_GetUnknownFrameCount(void);
uint32_t App_VisionLink_GetRxOverflowCount(void);

void App_VisionLink_SendTrackMode(void);
void App_VisionLink_SendIdleMode(void);

#endif
