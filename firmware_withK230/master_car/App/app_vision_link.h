#ifndef __APP_VISION_LINK_H
#define __APP_VISION_LINK_H

#include <stdint.h>

#define VISION_BINARY_FRAME_SIZE       24U
#define VISION_BINARY_HEADER_0         0xAAU
#define VISION_BINARY_HEADER_1         0x55U
#define VISION_BINARY_VERSION          0x02U
#define VISION_BINARY_MESSAGE_ROAD     0x10U

typedef struct
{
    uint16_t sequence;
    uint32_t k230TimestampMs;

    uint8_t  mode;
    uint8_t  statusFlags;

    int16_t  lateralErrorDeciMm;
    int16_t  headingErrorCentiDeg;
    uint16_t roadWidthDeciMm;

    uint8_t  junctionStage;
    uint8_t  junctionDistanceLevel;
    uint8_t  confidence;
    uint8_t  anomalyFlags;

    uint32_t receiveTimeMs;

    uint8_t  transportValid;
    uint8_t  visionValid;
} VisionTrackFrame_t;

void App_VisionLink_Init(void);
void App_VisionLink_Task10ms(void);
void App_VisionLink_Reset(void);

uint8_t  App_VisionLink_GetLatest(VisionTrackFrame_t *frame);
uint8_t  App_VisionLink_HasNewFrame(void);
uint32_t App_VisionLink_GetFrameAgeMs(void);

uint32_t App_VisionLink_GetValidFrameCount(void);
uint32_t App_VisionLink_GetCrcErrorCount(void);
uint32_t App_VisionLink_GetHeaderSyncLossCount(void);
uint32_t App_VisionLink_GetVersionErrorCount(void);
uint32_t App_VisionLink_GetMessageTypeErrorCount(void);
uint32_t App_VisionLink_GetDuplicateFrameCount(void);
uint32_t App_VisionLink_GetRxOverflowCount(void);

void App_VisionLink_SendTrackMode(void);
void App_VisionLink_SendIdleMode(void);

#endif
