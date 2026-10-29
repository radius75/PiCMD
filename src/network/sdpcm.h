#ifndef SDPCM_H
#define SDPCM_H
#include "types.h"
struct BrcmChipInfo;
bool Sdpcm_WaitReady(const BrcmChipInfo& chip, u32 timeoutUs);
// Sends one SDPCM frame over BCM43430 SDIO function 2. Payload is limited to
// 496 bytes here because the raw byte-mode CMD53 transport is capped at 508.
bool Sdpcm_SendFrame(u8 channel, const u8* payload, u32 payloadLength);
bool Sdpcm_ReceiveFrame(u8& channel, u8* payload, u32 capacity, u32& payloadLength);
const char* Sdpcm_LastError();
void Sdpcm_SetDebug(bool enabled);
void Sdpcm_GetTxStats(u32& attempts,u32& ok,u32& fail,u8& nextSequence);
void Sdpcm_GetRxStats(u32& nonEmpty,u32& ch1,u32& ch2,u32& other,u32& badHeader,u32& badPayload,u32& cmd53Fail);
#endif
