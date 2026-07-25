#ifndef __APP_VISION_LINK_H
#define __APP_VISION_LINK_H

#include <stdint.h>

#define VISION_BINARY_FRAME_SIZE       24U
#define VISION_BINARY_HEADER_0         0xAAU
#define VISION_BINARY_HEADER_1         0x55U
#define VISION_BINARY_VERSION          0x02U
#define VISION_BINARY_MESSAGE_ROAD     0x10U

/*
 * K230 binary protocol status byte (frame[11]) bit definitions.
 * Verified against k230_road_vision firmware v0.2 protocol.
 */
#define VISION_STATUS_VALID                 0x01U
#define VISION_STATUS_DEGRADED              0x02U
#define VISION_STATUS_LEFT_BOUNDARY_VALID   0x04U
#define VISION_STATUS_RIGHT_BOUNDARY_VALID  0x08U
#define VISION_STATUS_LEFT_BRANCH           0x10U
#define VISION_STATUS_RIGHT_BRANCH          0x20U
#define VISION_STATUS_INTERSECTION          0x40U

#define VISION_MODE_IDLE       0U
#define VISION_MODE_TRACK      1U
#define VISION_MODE_TURNING    2U
#define VISION_MODE_REACQUIRE  3U
#define VISION_MODE_FAULT      4U
#define VISION_MODE_NUMBER     5U

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
    uint8_t  degraded;
    uint8_t  leftBoundaryValid;
    uint8_t  rightBoundaryValid;
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
