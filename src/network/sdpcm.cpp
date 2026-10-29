#include "sdpcm.h"
#include "wifi_sdio.h"
#include "brcm_chip.h"
#include "rpihardware.h"
static u8 s_sequence;
static const char* s_error;
static bool s_debugEnabled;
static u32 s_txAttempts, s_txCmd53Ok, s_txCmd53Fail;
static u32 s_rxNonEmpty, s_rxCh1, s_rxCh2, s_rxOther, s_rxBadHeader, s_rxBadPayload, s_rxCmd53Fail;
void Sdpcm_SetDebug(bool enabled)
{
	s_debugEnabled=enabled;
	if(enabled)
	{
		s_txAttempts=s_txCmd53Ok=s_txCmd53Fail=0;
		s_rxNonEmpty=s_rxCh1=s_rxCh2=s_rxOther=s_rxBadHeader=s_rxBadPayload=s_rxCmd53Fail=0;
	}
}
void Sdpcm_GetTxStats(u32& attempts,u32& ok,u32& fail,u8& nextSequence)
{ attempts=s_txAttempts; ok=s_txCmd53Ok; fail=s_txCmd53Fail; nextSequence=s_sequence; }
void Sdpcm_GetRxStats(u32& nonEmpty,u32& ch1,u32& ch2,u32& other,u32& badHeader,u32& badPayload,u32& cmd53Fail)
{ nonEmpty=s_rxNonEmpty; ch1=s_rxCh1; ch2=s_rxCh2; other=s_rxOther; badHeader=s_rxBadHeader; badPayload=s_rxBadPayload; cmd53Fail=s_rxCmd53Fail; }
static void Put16(u8* p,u16 v){p[0]=(u8)v;p[1]=(u8)(v>>8);}
static void Put32(u8* p,u32 v){p[0]=(u8)v;p[1]=(u8)(v>>8);p[2]=(u8)(v>>16);p[3]=(u8)(v>>24);}
bool Sdpcm_WaitReady(const BrcmChipInfo& c,u32 timeoutUs){
	if(!BrcmChip_Write32(c.sdioCoreBase+0x24,0xF0)){s_error="cannot set host interrupt mask";return false;}u32 d=read32(ARM_SYSTIMER_CLO)+timeoutUs;
	do{u32 status,mail;if(!BrcmChip_Read32(c.sdioCoreBase+0x20,status)||!BrcmChip_Read32(c.sdioCoreBase+0x4c,mail))return false;
		if(mail){BrcmChip_Write32(c.sdioCoreBase+0x40,2);if(mail&0xA){BrcmChip_Write32(c.sdioCoreBase+0x20,status&~0x40);return true;}}delay_us(1000);
 	}while((s32)(read32(ARM_SYSTIMER_CLO)-d)<0);s_error="firmware-ready mailbox timeout";return false;}
bool Sdpcm_SendFrame(u8 channel,const u8* payload,u32 payloadLength)
{
	if(channel>15 || payloadLength>496 || (payloadLength && !payload)){s_error="invalid SDPCM frame";return false;}
	u8 frame[508]; u32 length=12+payloadLength, padded=(length+3)&~3u;
	for(u32 i=0;i<padded;++i)frame[i]=0;
	Put16(frame,(u16)length); Put16(frame+2,(u16)~length);
	Put32(frame+4,(u32)s_sequence|((u32)channel<<8)|(12u<<24));
	if(payloadLength) for(u32 i=0;i<payloadLength;++i)frame[12+i]=payload[i];
	if(s_debugEnabled) ++s_txAttempts;
	if(!WifiSdio_Cmd53(true,2,0,false,frame,padded)){if(s_debugEnabled) ++s_txCmd53Fail;s_error=WifiSdio_LastError();return false;}
	if(s_debugEnabled) ++s_txCmd53Ok;
	++s_sequence; s_error="";return true;
}
bool Sdpcm_ReceiveFrame(u8& channel,u8* payload,u32 capacity,u32& payloadLength)
{
	u8 first[64];
	payloadLength=0;
	if(!WifiSdio_Cmd53(false,2,0,false,first,64)){if(s_debugEnabled) ++s_rxCmd53Fail;s_error=WifiSdio_LastError();return false;}

	u16 length=(u16)first[0]|((u16)first[1]<<8);
	u16 inverse=(u16)first[2]|((u16)first[3]<<8);
	if(!length){s_error="SDPCM FIFO returned empty frame";return false;}
	if(s_debugEnabled) ++s_rxNonEmpty;
	if((u16)~length!=inverse || length<12){if(s_debugEnabled) ++s_rxBadHeader;s_error="invalid SDPCM header";return false;}

	u32 word=(u32)first[4]|((u32)first[5]<<8)|((u32)first[6]<<16)|((u32)first[7]<<24);
	channel=(u8)(word>>8);
	if(s_debugEnabled){if(channel==1)++s_rxCh1;else if(channel==2)++s_rxCh2;else ++s_rxOther;}
	u32 dataOffset=word>>24;
	if(length>2048 || dataOffset<12 || dataOffset>length){if(s_debugEnabled) ++s_rxBadHeader;s_error="invalid SDPCM header";return false;}

	u32 padded=(length+3)&~3u;
	u32 wanted=length-dataOffset;
	bool fits=wanted<=capacity;

	if(fits && dataOffset<64){
		u32 n=64-dataOffset;
		if(n>wanted)n=wanted;
		for(u32 i=0;i<n;++i)payload[i]=first[dataOffset+i];
	}

	u32 pos=64;
	u8 scratch[508];
	while(pos<padded){
		u32 chunk=padded-pos;
		if(chunk>508)chunk=508;
		if(!WifiSdio_Cmd53(false,2,0,false,scratch,chunk)){if(s_debugEnabled) ++s_rxCmd53Fail;s_error=WifiSdio_LastError();return false;}

		if(fits){
			u32 from=(pos<dataOffset)?dataOffset:pos;
			u32 to=pos+chunk;
			if(to>length)to=length;
			if(from<to){
				for(u32 p=from;p<to;++p)payload[p-dataOffset]=scratch[p-pos];
			}
		}
		pos+=chunk;
	}

	if(!fits){if(s_debugEnabled) ++s_rxBadPayload;s_error="SDPCM payload too large";return false;}

	payloadLength=wanted;
	s_error="";
	return true;
}
const char* Sdpcm_LastError(){return s_error?s_error:"not initialised";}
