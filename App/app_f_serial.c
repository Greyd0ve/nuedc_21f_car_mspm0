#include "app_f_serial.h"
#include "app_f_car.h"
#include "Serial.h"
#include <stdint.h>

static void FSerial_SendOk(const char *name)
{
    Serial_Printf("[f,%s,ok]\r\n", name);
}

static void FSerial_SendFail(const char *name, const char *reason)
{
    Serial_Printf("[f,%s,fail,%s]\r\n", name, reason);
}

static int32_t FSerial_ParseInt(const char *str, const char **endp)
{
    int32_t val = 0;
    while (*str >= '0' && *str <= '9')
    {
        val = val * 10 + (int32_t)(*str - '0');
        str++;
    }
    if (endp)
    {
        *endp = str;
    }
    return val;
}

static uint8_t FSerial_Match(const char *buf, const char *pattern)
{
    while (*pattern)
    {
        if (*buf != *pattern)
        {
            return 0U;
        }
        buf++;
        pattern++;
    }
    return 1U;
}

static void FSerial_ParseCommand(const char *buf)
{
    if (FSerial_Match(buf, "[f,room,"))
    {
        int32_t room = FSerial_ParseInt(buf + 8, NULL);
        if (room >= 1 && room <= 8)
        {
            FCarState_t state = FCar_GetState();
            if (state == F_CAR_IDLE || state == F_CAR_WAIT_ROOM_ID)
            {
                FCar_SetTargetRoom((uint8_t)room);
                FSerial_SendOk("room");
            }
            else
            {
                FSerial_SendFail("room", "busy");
            }
        }
        else
        {
            FSerial_SendFail("room", "range");
        }
    }
    else if (FSerial_Match(buf, "[f,start]"))
    {
        FCar_Start();
        if (FCar_GetState() == F_CAR_DELIVER_START)
        {
            FSerial_SendOk("start");
        }
        else
        {
            FSerial_SendFail("start", "not_ready");
        }
    }
    else if (FSerial_Match(buf, "[f,stop]"))
    {
        FCar_Stop();
        FSerial_SendOk("stop");
    }
    else if (FSerial_Match(buf, "[f,reset]"))
    {
        FCar_Reset();
        FSerial_SendOk("reset");
    }
    else if (FSerial_Match(buf, "[f,status]"))
    {
        Serial_Printf("[f,status,state,%u,room,%u,time,%lu]\r\n",
            (unsigned int)FCar_GetState(),
            (unsigned int)FCar_GetTargetRoom(),
            (unsigned long)FCar_GetRunningTimeMs());
    }
    else if (FSerial_Match(buf, "[f,load,1]"))
    {
        FCar_SimulateLoadComplete();
        FSerial_SendOk("load");
    }
    else if (FSerial_Match(buf, "[f,unload,1]"))
    {
        FCar_SimulateUnloadComplete();
        FSerial_SendOk("unload");
    }
}

void FCar_Serial_Init(void)
{
    Serial_Printf("[f,init]\r\n");
}

void FCar_SerialProcess(void)
{
    static char buf[32];
    static uint8_t idx = 0U;
    static uint8_t inFrame = 0U;

    while (1)
    {
        uint8_t byte;
        if (!Serial_ReadByte(&byte))
        {
            break;
        }

        if ((char)byte == '[')
        {
            idx = 0U;
            inFrame = 1U;
            buf[idx++] = '[';
        }
        else if (inFrame)
        {
            if (idx < sizeof(buf) - 1U)
            {
                buf[idx++] = (char)byte;
            }
            if ((char)byte == ']')
            {
                buf[idx] = '\0';
                inFrame = 0U;
                FSerial_ParseCommand(buf);
                idx = 0U;
            }
        }
    }
}

void FCar_SerialPlot100ms(void)
{
    Serial_Printf("[f,val,state,%u,room,%u,time,%lu]\r\n",
        (unsigned int)FCar_GetState(),
        (unsigned int)FCar_GetTargetRoom(),
        (unsigned long)FCar_GetRunningTimeMs());
}
