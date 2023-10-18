/* Copyright holders: SA1988
   see COPYING for more details
*/
/*Buslogic SCSI emulation (including Adaptec 154x ISA software backward compatibility) and the Adaptec 154x itself*/

/* Emulated SCSI controllers:
	0 - Adaptec AHA-154xB ISA;
	1 - BusLogic BT-542B ISA;
	2 - BusLogic BT-958 PCI (but BT-542B ISA on non-PCI machines). */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "ibm.h"
#include "device.h"
#include "io.h"
#include "mem.h"
#include "rom.h"
#include "dma.h"
#include "pic.h"
#include "pci.h"
#include "timer.h"

#include "scsi.h"
#include "cdrom.h"

#include "buslogic.h"

#define BUSLOGIC_RESET_DURATION_NS UINT64_C(50000000)

typedef struct __attribute__((packed))
{
	uint8_t hi;
	uint8_t mid;
	uint8_t lo;
} addr24;

#define ADDR_TO_U32(x) 		(((x).hi << 16) | ((x).mid << 8) | (x).lo & 0xFF)
#define U32_TO_ADDR(a, x) 	do {(a).hi = (x) >> 16; (a).mid = (x) >> 8; (a).lo = (x) & 0xFF;} while(0)
	
// I/O Port interface
// READ  Port x+0: STATUS
// WRITE Port x+0: CONTROL
//
// READ  Port x+1: DATA
// WRITE Port x+1: COMMAND
//
// READ  Port x+2: INTERRUPT STATUS
// WRITE Port x+2: (undefined?)
//
// R/W   Port x+3: (undefined)

// READ STATUS flags
#define STAT_STST   0x80    // self-test in progress
#define STAT_DFAIL  0x40    // internal diagnostic failure
#define STAT_INIT  0x20    // mailbox initialization required
#define STAT_IDLE  0x10    // HBA is idle
#define STAT_CDFULL 0x08    // Command/Data output port is full
#define STAT_DFULL 0x04    // Data input port is full
#define STAT_INVCMD 0x01    // Invalid command

// READ INTERRUPT STATUS flags
#define INTR_ANY   0x80    // any interrupt
#define INTR_SRCD   0x08    // SCSI reset detected
#define INTR_HACC   0x04    // HA command complete
#define INTR_MBOA   0x02    // MBO empty
#define INTR_MBIF   0x01    // MBI full

// WRITE CONTROL commands
#define CTRL_HRST  0x80    // Hard reset
#define CTRL_SRST  0x40    // Soft reset
#define CTRL_IRST   0x20    // interrupt reset
#define CTRL_SCRST  0x10    // SCSI bus reset

// READ/WRITE DATA commands
#define CMD_NOP        					0x00    // No operation
#define CMD_MBINIT      				0x01    // mailbox initialization
#define CMD_START_SCSI  				0x02    // Start SCSI command
#define CMD_INQUIRY     				0x04    // Adapter inquiry
#define CMD_EMBOI       				0x05    // enable Mailbox Out Interrupt
#define CMD_SELTIMEOUT  				0x06    // Set SEL timeout
#define CMD_BUSON_TIME  				0x07    // set bus-On time
#define CMD_BUSOFF_TIME 				0x08    // set bus-off time
#define CMD_DMASPEED    				0x09    // set ISA DMA speed
#define CMD_RETDEVS   					0x0A    // return installed devices
#define CMD_RETCONF     				0x0B    // return configuration data
#define CMD_TARGET      				0x0C    // set HBA to target mode
#define CMD_RETSETUP   					0x0D    // return setup data
#define CMD_ECHO        				0x1F    // ECHO command data

#pragma pack(1)
/**
 * Auto SCSI structure which is located
 * in host adapter RAM and contains several
 * configuration parameters.
 */
typedef struct __attribute__((packed)) AutoSCSIRam
{
    uint8_t       aInternalSignature[2];
    uint8_t       cbInformation;
    uint8_t       aHostAdaptertype[6];
    uint8_t       uReserved1;
    uint8_t          fFloppyEnabled :                  1;
    uint8_t          fFloppySecondary :                1;
    uint8_t          fLevelSensitiveInterrupt :        1;
    unsigned char uReserved2 :                      2;
    unsigned char uSystemRAMAreForBIOS :            3;
    unsigned char uDMAChannel :                     7;
    uint8_t          fDMAAutoConfiguration :           1;
    unsigned char uIrqChannel :                     7;
    uint8_t          fIrqAutoConfiguration :           1;
    uint8_t       uDMATransferRate;
    uint8_t       uSCSIId;
    uint8_t          fLowByteTerminated :              1;
    uint8_t          fParityCheckingEnabled :          1;
    uint8_t          fHighByteTerminated :             1;
    uint8_t          fNoisyCablingEnvironment :        1;
    uint8_t          fFastSynchronousNeogtiation :     1;
    uint8_t          fBusResetEnabled :                1;
    uint8_t          fReserved3 :                      1;
    uint8_t          fActiveNegotiationEnabled :       1;
    uint8_t       uBusOnDelay;
    uint8_t       uBusOffDelay;
    uint8_t          fHostAdapterBIOSEnabled :         1;
    uint8_t          fBIOSRedirectionOfInt19 :         1;
    uint8_t          fExtendedTranslation :            1;
    uint8_t          fMapRemovableAsFixed :            1;
    uint8_t          fReserved4 :                      1;
    uint8_t          fBIOSSupportsMoreThan2Drives :    1;
    uint8_t          fBIOSInterruptMode :              1;
    uint8_t          fFlopticalSupport :               1;
    uint16_t      u16DeviceEnabledMask;
    uint16_t      u16WidePermittedMask;
    uint16_t      u16FastPermittedMask;
    uint16_t      u16SynchronousPermittedMask;
    uint16_t      u16DisconnectPermittedMask;
    uint16_t      u16SendStartUnitCommandMask;
    uint16_t      u16IgnoreInBIOSScanMask;
    unsigned char uPCIInterruptPin :                2;
    unsigned char uHostAdapterIoPortAddress :       2;
    uint8_t          fStrictRoundRobinMode :           1;
    uint8_t          fVesaBusSpeedGreaterThan33MHz :   1;
    uint8_t          fVesaBurstWrite :                 1;
    uint8_t          fVesaBurstRead :                  1;
    uint16_t      u16UltraPermittedMask;
    uint32_t      uReserved5;
    uint8_t       uReserved6;
    uint8_t       uAutoSCSIMaximumLUN;
    uint8_t          fReserved7 :                      1;
    uint8_t          fSCAMDominant :                   1;
    uint8_t          fSCAMenabled :                    1;
    uint8_t          fSCAMLevel2 :                     1;
    unsigned char uReserved8 :                      4;
    uint8_t          fInt13Extension :                 1;
    uint8_t          fReserved9 :                      1;
    uint8_t          fCDROMBoot :                      1;
    unsigned char uReserved10 :                     5;
    unsigned char uBootTargetId :                   4;
    unsigned char uBootChannel :                    4;
    uint8_t          fForceBusDeviceScanningOrder :    1;
    unsigned char uReserved11 :                     7;
    uint16_t      u16NonTaggedToAlternateLunPermittedMask;
    uint16_t      u16RenegotiateSyncAfterCheckConditionMask;
    uint8_t       aReserved12[10];
    uint8_t       aManufacturingDiagnostic[2];
    uint16_t      u16Checksum;
} AutoSCSIRam;
#pragma pack()

/**
 * The local Ram.
 */
typedef union HostAdapterLocalRam
{
    /** Byte view. */
    uint8_t u8View[256];
    /** Structured view. */
    struct __attribute__((packed))
    {
        /** Offset 0 - 63 is for BIOS. */
        uint8_t     u8Bios[64];
        /** Auto SCSI structure. */
        AutoSCSIRam autoSCSIData;
    } structured;
} HostAdapterLocalRam;

/** Structure for the INQUIRE_SETUP_INFORMATION reply. */
typedef struct __attribute__((packed)) ReplyInquireSetupInformationSynchronousValue
{
    uint8_t uOffset :         4;
    uint8_t uTransferPeriod : 3;
    uint8_t fSynchronous :    1;
}ReplyInquireSetupInformationSynchronousValue;

typedef struct __attribute__((packed)) ReplyInquireSetupInformation
{
    uint8_t fSynchronousInitiationEnabled : 1;
    uint8_t fParityCheckingEnabled :        1;
    uint8_t uReserved1 :           6;
    uint8_t uBusTransferRate;
    uint8_t uPreemptTimeOnBus;
    uint8_t uTimeOffBus;
    uint8_t cMailbox;
    addr24  MailboxAddress;
    ReplyInquireSetupInformationSynchronousValue SynchronousValuesId0To7[8];
    uint8_t uDisconnectPermittedId0To7;
    uint8_t uSignature;
    uint8_t uCharacterD;
    uint8_t uHostBusType;
    uint8_t uWideTransferPermittedId0To7;
    uint8_t uWideTransfersActiveId0To7;
    ReplyInquireSetupInformationSynchronousValue SynchronousValuesId8To15[8];
    uint8_t uDisconnectPermittedId8To15;
    uint8_t uReserved2;
    uint8_t uWideTransferPermittedId8To15;
    uint8_t uWideTransfersActiveId8To15;
} ReplyInquireSetupInformation;

/** Structure for the INQUIRE_EXTENDED_SETUP_INFORMATION. */
#pragma pack(1)
typedef struct __attribute__((packed)) ReplyInquireExtendedSetupInformation
{
    uint8_t       uBusType;
    uint8_t       uBiosAddress;
    uint16_t      u16ScatterGatherLimit;
    uint8_t       cMailbox;
    uint32_t      uMailboxAddressBase;
    uint8_t 		uReserved1 : 2;
    uint8_t         fFastEISA : 1;
    uint8_t 		uReserved2 : 3;
    uint8_t          fLevelSensitiveInterrupt : 1;
    uint8_t 		uReserved3 : 1;
    uint8_t 		aFirmwareRevision[3];
    uint8_t          fHostWideSCSI : 1;
    uint8_t          fHostDifferentialSCSI : 1;
    uint8_t          fHostSupportsSCAM : 1;
    uint8_t          fHostUltraSCSI : 1;
    uint8_t          fHostSmartTermination : 1;
    uint8_t 		uReserved4 : 3;
} ReplyInquireExtendedSetupInformation;
#pragma pack()

typedef struct __attribute__((packed)) MailboxInit_t
{
	uint8_t Count;
	addr24 Address;
} MailboxInit_t;

#pragma pack(1)
typedef struct __attribute__((packed)) MailboxInitExtended_t
{
	uint8_t Count;
	uint32_t Address;
} MailboxInitExtended_t;
#pragma pack()

///////////////////////////////////////////////////////////////////////////////
//
// Mailbox Definitions
//
//
///////////////////////////////////////////////////////////////////////////////

//
// Mailbox Out
//
//
// MBO Command Values
//

#define MBO_FREE                  0x00
#define MBO_START                 0x01
#define MBO_ABORT                 0x02

//
// Mailbox In
//
//
// MBI Status Values
//

#define MBI_FREE                  0x00
#define MBI_SUCCESS               0x01
#define MBI_ABORT                 0x02
#define MBI_NOT_FOUND             0x03
#define MBI_ERROR                 0x04

typedef struct __attribute__((packed)) Mailbox_t
{
	uint8_t CmdStatus;
	addr24 CCBPointer;
} Mailbox_t;

typedef struct __attribute__((packed)) Mailbox32_t
{
	uint32_t CCBPointer;
	union
	{
		struct
		{
			uint8_t Reserved[3];
			uint8_t ActionCode;
		} out;
		struct
		{
			uint8_t HostStatus;
			uint8_t TargetStatus;
			uint8_t Reserved;
			uint8_t CompletionCode;
		} in;
	} u;
} Mailbox32_t;

///////////////////////////////////////////////////////////////////////////////
//
// CCB - Buslogic SCSI Command Control Block
//
//    The CCB is a superset of the CDB (Command Descriptor Block)
//    and specifies detailed information about a SCSI command.
//
///////////////////////////////////////////////////////////////////////////////

//
//    Byte 0    Command Control Block Operation Code
//

#define SCSI_INITIATOR_COMMAND    	  0x00
#define TARGET_MODE_COMMAND       	  0x01
#define SCATTER_GATHER_COMMAND		  0x02
#define SCSI_INITIATOR_COMMAND_RES	  0x03
#define SCATTER_GATHER_COMMAND_RES    0x04
#define BUS_RESET				  	  0x81

//
//    Byte 1    Address and Direction Control
//

#define CCB_TARGET_ID_SHIFT       0x06            // CCB Op Code = 00, 02
#define CCB_INITIATOR_ID_SHIFT    0x06            // CCB Op Code = 01
#define CCB_DATA_XFER_IN		  0x01
#define CCB_DATA_XFER_OUT		  0x02
#define CCB_LUN_MASK              0x07            // Logical Unit Number

//
//    Byte 2    SCSI_Command_Length - Length of SCSI CDB
//
//    Byte 3    Request Sense Allocation Length
//

#define FOURTEEN_BYTES            0x00            // Request Sense Buffer size
#define NO_AUTO_REQUEST_SENSE     0x01            // No Request Sense Buffer

//
//    Bytes 4, 5 and 6    Data Length             // Data transfer byte count
//
//    Bytes 7, 8 and 9    Data Pointer            // SGD List or Data Buffer
//
//    Bytes 10, 11 and 12 Link Pointer            // Next CCB in Linked List
//
//    Byte 13   Command Link ID                   // TBD (I don't know yet)
//
//    Byte 14   Host Status                       // Host Adapter status
//

#define CCB_COMPLETE              0x00            // CCB completed without error
#define CCB_LINKED_COMPLETE       0x0A            // Linked command completed
#define CCB_LINKED_COMPLETE_INT   0x0B            // Linked complete with interrupt
#define CCB_SELECTION_TIMEOUT     0x11            // Set SCSI selection timed out
#define CCB_DATA_OVER_UNDER_RUN   0x12
#define CCB_UNEXPECTED_BUS_FREE   0x13            // Target dropped SCSI BSY
#define CCB_PHASE_SEQUENCE_FAIL   0x14            // Target bus phase sequence failure
#define CCB_BAD_MBO_COMMAND       0x15            // MBO command not 0, 1 or 2
#define CCB_INVALID_OP_CODE       0x16            // CCB invalid operation code
#define CCB_BAD_LINKED_LUN        0x17            // Linked CCB LUN different from first
#define CCB_INVALID_DIRECTION     0x18            // Invalid target direction
#define CCB_DUPLICATE_CCB         0x19            // Duplicate CCB
#define CCB_INVALID_CCB           0x1A            // Invalid CCB - bad parameter

//
//    Byte 15   Target Status
//
//    See scsi.h files for these statuses.
//

//
//    Bytes 16 and 17   Reserved (must be 0)
//

//
//    Bytes 18 through 18+n-1, where n=size of CDB  Command Descriptor Block
//

typedef struct __attribute__((packed)) CCB32
{
	uint8_t Opcode;
	uint8_t Reserved1:3;
	uint8_t ControlByte:2;
	uint8_t TagQueued:1;
	uint8_t QueueTag:2;
	uint8_t CdbLength;
	uint8_t RequestSenseLength;
	uint32_t DataLength;
	uint32_t DataPointer;
	uint8_t Reserved2[2];
	uint8_t HostStatus;
	uint8_t TargetStatus;
	uint8_t Id;
	uint8_t Lun:5;
	uint8_t LegacyTagEnable:1;
	uint8_t LegacyQueueTag:2;
	uint8_t Cdb[12];
	uint8_t Reserved3[6];
	uint32_t SensePointer;
} CCB32;

typedef struct __attribute__((packed)) CCB
{
	uint8_t Opcode;
	uint8_t Lun:3;
	uint8_t ControlByte:2;
	uint8_t Id:3;
	uint8_t CdbLength;
	uint8_t RequestSenseLength;
	addr24 DataLength;
	addr24 DataPointer;
	addr24 LinkPointer;
	uint8_t LinkId;
	uint8_t HostStatus;
	uint8_t TargetStatus;
	uint8_t Reserved[2];
	uint8_t Cdb[12];
} CCB;

typedef struct __attribute__((packed)) CCBC
{
	uint8_t Opcode;
	uint8_t Pad1:3;
	uint8_t ControlByte:2;
	uint8_t Pad2:3;
	uint8_t CdbLength;
	uint8_t RequestSenseLength;
	uint8_t Pad3[10];
	uint8_t HostStatus;
	uint8_t TargetStatus;
	uint8_t Pad4[2];
	uint8_t Cdb[12];
} CCBC;

typedef union __attribute__((packed)) CCBU
{
	CCB32 new;
	CCB   old;
	CCBC  common;
} CCBU;

///////////////////////////////////////////////////////////////////////////////
//
// Scatter/Gather Segment List Definitions
//
///////////////////////////////////////////////////////////////////////////////

//
// Adapter limits
//

#define MAX_SG_DESCRIPTORS (scsi_model ? 32 : 17)

typedef struct __attribute__((packed)) SGE32
{
	uint32_t Segment;
	uint32_t SegmentPointer;
} SGE32;

typedef struct __attribute__((packed)) SGE
{
	addr24 Segment;
	addr24 SegmentPointer;
} SGE;

typedef struct __attribute__((packed)) BuslogicRequests_t
{
	CCBU CmdBlock;
	uint8_t *RequestSenseBuffer;
	uint32_t CCBPointer;
	int Is24bit;
	uint8_t TargetID;
	uint8_t LUN;
	uint8_t HostStatus;
	uint8_t TargetStatus;
	uint8_t MailboxCompletionCode;
} BuslogicRequests_t;

typedef struct __attribute__((packed)) Buslogic_t
{
	rom_t bios;
	int UseLocalRam;
	int StrictRoundRobinMode;
	int ExtendedLUNCCBFormat;
	HostAdapterLocalRam LocalRam;
	BuslogicRequests_t BuslogicRequests;
	uint8_t Status;
	uint8_t Interrupt;
	uint8_t Geometry;
	uint8_t Control;
	uint8_t Command;
	uint8_t CmdBuf[53];
	uint8_t CmdParam;
	uint8_t CmdParamLeft;
	uint8_t DataBuf[64];
	uint8_t DataReply;
	uint8_t DataReplyLeft;
	uint32_t MailboxCount;
	uint32_t MailboxOutAddr;
	uint32_t MailboxOutPosCur;
	uint32_t MailboxInAddr;
	uint32_t MailboxInPosCur;
	int Base;
	int PCIBase;
	int MMIOBase;
	int Irq;
	int DmaChannel;
	int IrqEnabled;
	int Mbx24bit;
	int MailboxOutInterrupts;
	int MbiActive[256];
	int PendingInterrupt;
	int Lock;
        mem_mapping_t mmio_mapping;
} Buslogic_t;

int scsi_model = 1;

int BuslogicResetCallback = 0;
int BuslogicCallback = 0;

int BuslogicInOperation = 0;

/** Structure for the INQUIRE_PCI_HOST_ADAPTER_INFORMATION reply. */
typedef struct __attribute__((packed)) BuslogicPCIInformation_t
{
	uint8_t IsaIOPort;
	uint8_t IRQ;
	unsigned char LowByteTerminated:1;
	unsigned char HighByteTerminated:1;
	unsigned char uReserved:2;	/* Reserved. */
	unsigned char JP1:1;		/* Whatever that means. */
	unsigned char JP2:1;		/* Whatever that means. */
	unsigned char JP3:1;		/* Whatever that means. */
	/** Whether the provided info is valid. */
	unsigned char InformationIsValid:1;
	uint8_t uReserved2; /* Reserved. */
} BuslogicPCIInformation_t;

static void BuslogicStartMailbox(Buslogic_t *Buslogic);

int buslogic_do_log = 0;

void BuslogicLog(const char *format, ...)
{
#ifdef ENABLE_BUSLOGIC_LOG
   if (buslogic_do_log)
   {
		va_list ap;
		va_start(ap, format);
		vprintf(format, ap);
		va_end(ap);
		fflush(stdout);
   }
#endif
}
		
static int BuslogicIsPCI(void)
{
	if (PCI && (scsi_model == 2))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void BuslogicClearInterrupt(Buslogic_t *Buslogic)
{
	BuslogicLog("Buslogic: Lowering Interrupt 0x%02X\n", Buslogic->Interrupt);
	Buslogic->Interrupt = 0;
	BuslogicLog("Lowering IRQ %i\n", Buslogic->Irq);
	picintc(1 << Buslogic->Irq);
	if (Buslogic->PendingInterrupt)
	{
		Buslogic->Interrupt = Buslogic->PendingInterrupt;
		BuslogicLog("Buslogic: Raising Interrupt 0x%02X (Pending)\n", Buslogic->Interrupt);
		if (Buslogic->MailboxOutInterrupts || !(Buslogic->Interrupt & INTR_MBOA))
		{
			if (Buslogic->IrqEnabled)  picint(1 << Buslogic->Irq);
		}
		Buslogic->PendingInterrupt = 0;
	}
}

static void BuslogicLocalRam(Buslogic_t *Buslogic)
{
    /*
     * These values are mostly from what I think is right
     * looking at the dmesg output from a Linux guest inside
     * a VMware server VM.
     *
     * So they don't have to be right :)
     */
    memset(Buslogic->LocalRam.u8View, 0, sizeof(HostAdapterLocalRam));
    Buslogic->LocalRam.structured.autoSCSIData.fLevelSensitiveInterrupt = 1;
    Buslogic->LocalRam.structured.autoSCSIData.fParityCheckingEnabled = 1;
    Buslogic->LocalRam.structured.autoSCSIData.fExtendedTranslation = 1; /* Same as in geometry register. */
    Buslogic->LocalRam.structured.autoSCSIData.u16DeviceEnabledMask = UINT16_MAX; /* All enabled. Maybe mask out non present devices? */
    Buslogic->LocalRam.structured.autoSCSIData.u16WidePermittedMask = UINT16_MAX;
    Buslogic->LocalRam.structured.autoSCSIData.u16FastPermittedMask = UINT16_MAX;
    Buslogic->LocalRam.structured.autoSCSIData.u16SynchronousPermittedMask = UINT16_MAX;
    Buslogic->LocalRam.structured.autoSCSIData.u16DisconnectPermittedMask = UINT16_MAX;
    Buslogic->LocalRam.structured.autoSCSIData.fStrictRoundRobinMode = Buslogic->StrictRoundRobinMode;
    Buslogic->LocalRam.structured.autoSCSIData.u16UltraPermittedMask = UINT16_MAX;
    /** @todo calculate checksum? */
}

static Buslogic_t *BuslogicResetDevice;

static void BuslogicReset(Buslogic_t *Buslogic)
{
	BuslogicCallback = 0;
	BuslogicResetCallback = 0;
	Buslogic->Status = STAT_IDLE | STAT_INIT;
	Buslogic->Geometry = 0x80;
	Buslogic->Command = 0xFF;
	Buslogic->CmdParam = 0;
	Buslogic->CmdParamLeft = 0;
	Buslogic->IrqEnabled = 1;
	Buslogic->StrictRoundRobinMode = 0;
	Buslogic->ExtendedLUNCCBFormat = 0;
	Buslogic->MailboxOutPosCur = 0;
	Buslogic->MailboxInPosCur = 0;
	Buslogic->MailboxOutInterrupts = 0;
	Buslogic->PendingInterrupt = 0;
	Buslogic->Lock = 0;
	BuslogicInOperation = 0;

	BuslogicClearInterrupt(Buslogic);

	BuslogicLocalRam(Buslogic);
}

void BuslogicSoftReset(void)
{
	if (BuslogicResetDevice != NULL)
	{
		BuslogicReset(BuslogicResetDevice);
	}
}

static void BuslogicResetControl(Buslogic_t *Buslogic, uint8_t Reset)
{
	BuslogicReset(Buslogic);
	if (Reset)
	{
		Buslogic->Status |= STAT_STST;
		Buslogic->Status &= ~STAT_IDLE;
	}
	BuslogicResetCallback = BUSLOGIC_RESET_DURATION_NS * TIMER_USEC;
}

static void BuslogicCommandComplete(Buslogic_t *Buslogic)
{
	Buslogic->DataReply = 0;
	Buslogic->Status |= STAT_IDLE;
				
	if (Buslogic->Command != 0x02)
	{
		Buslogic->Status &= ~STAT_DFULL;
		Buslogic->Interrupt = (INTR_ANY | INTR_HACC);
		BuslogicLog("Raising IRQ %i\n", Buslogic->Irq);
		if (Buslogic->IrqEnabled)  picint(1 << Buslogic->Irq);
	}
	
	Buslogic->Command = 0xFF;
	Buslogic->CmdParam = 0;
}

static void BuslogicRaiseInterrupt(Buslogic_t *Buslogic, uint8_t Interrupt)
{
	if (Buslogic->Interrupt & INTR_HACC)
	{
		BuslogicLog("Pending IRQ\n");
		Buslogic->PendingInterrupt = Interrupt;
	}
	else
	{
		Buslogic->Interrupt = Interrupt;
		BuslogicLog("Raising IRQ %i\n", Buslogic->Irq);
		if (Buslogic->IrqEnabled)  picint(1 << Buslogic->Irq);
	}
}

static void BuslogicMailboxInSetup(Buslogic_t *Buslogic, uint32_t CCBPointer, CCBU *CmdBlock, 
			uint8_t HostStatus, uint8_t TargetStatus, uint8_t MailboxCompletionCode)
{
	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;

	BuslogicRequests->CCBPointer = CCBPointer;
	memcpy(&(BuslogicRequests->CmdBlock), CmdBlock, sizeof(CCB32));
	BuslogicRequests->Is24bit = Buslogic->Mbx24bit;
	BuslogicRequests->HostStatus = HostStatus;
	BuslogicRequests->TargetStatus = TargetStatus;
	BuslogicRequests->MailboxCompletionCode = MailboxCompletionCode;

	BuslogicLog("Mailbox in setup\n");

	BuslogicInOperation = 2;
}

static uint32_t BuslogicMailboxInRead(Buslogic_t *Buslogic, uint8_t *CompletionCode)
{
	Mailbox32_t TempMailbox32;
	Mailbox_t TempMailboxIn;

	uint32_t Incoming = 0;

	Incoming = Buslogic->MailboxInAddr + (Buslogic->MailboxInPosCur * (Buslogic->Mbx24bit ? sizeof(Mailbox_t) : sizeof(Mailbox32_t)));

	if (Buslogic->Mbx24bit)
	{
		DMAPageRead(Incoming, &TempMailboxIn, sizeof(Mailbox_t));
		*CompletionCode = TempMailboxIn.CmdStatus;
	}
	else
	{
		DMAPageRead(Incoming, &TempMailbox32, sizeof(Mailbox32_t));
		*CompletionCode = TempMailbox32.u.in.CompletionCode;
	}
	
	return Incoming;
}

static void BuslogicMailboxInAdvance(Buslogic_t *Buslogic)
{
	Buslogic->MailboxInPosCur = (Buslogic->MailboxInPosCur + 1) % Buslogic->MailboxCount;
}

static void BuslogicMailboxIn(Buslogic_t *Buslogic)
{	
	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;

	uint32_t CCBPointer = BuslogicRequests->CCBPointer;
	CCBU *CmdBlock = &(BuslogicRequests->CmdBlock);
	uint8_t HostStatus = BuslogicRequests->HostStatus;
	uint8_t TargetStatus = BuslogicRequests->TargetStatus;
	uint8_t MailboxCompletionCode = BuslogicRequests->MailboxCompletionCode;

	Mailbox32_t Mailbox32;
	Mailbox_t MailboxIn;

	Mailbox32_t TempMailbox32;
	Mailbox_t TempMailboxIn;

	Mailbox32.CCBPointer = CCBPointer;
	Mailbox32.u.in.HostStatus = HostStatus;
	Mailbox32.u.in.TargetStatus = TargetStatus;
	Mailbox32.u.in.CompletionCode = MailboxCompletionCode;

	uint32_t Incoming = Buslogic->MailboxInAddr + (Buslogic->MailboxInPosCur * (Buslogic->Mbx24bit ? sizeof(Mailbox_t) : sizeof(Mailbox32_t)));

	if (MailboxCompletionCode != MBI_NOT_FOUND)
	{
		CmdBlock->common.HostStatus = HostStatus;
		CmdBlock->common.TargetStatus = TargetStatus;		
		
		//Rewrite the CCB up to the CDB.
		BuslogicLog("CCB rewritten to the CDB (pointer %08X, length %i)\n", CCBPointer, offsetof(CCBC, Cdb));
		DMAPageWrite(CCBPointer, CmdBlock, offsetof(CCBC, Cdb));
	}
	else
	{
		BuslogicLog("Mailbox not found!\n");
	}

	BuslogicLog("Host Status 0x%02X, Target Status 0x%02X\n", HostStatus, TargetStatus);	
	
	if (Buslogic->Mbx24bit)
	{
		MailboxIn.CmdStatus = Mailbox32.u.in.CompletionCode;
		U32_TO_ADDR(MailboxIn.CCBPointer, Mailbox32.CCBPointer);
		BuslogicLog("Mailbox 24-bit: Status=0x%02X, CCB at 0x%04X\n", MailboxIn.CmdStatus, ADDR_TO_U32(MailboxIn.CCBPointer));
		
		DMAPageWrite(Incoming, &MailboxIn, sizeof(Mailbox_t));
		BuslogicLog("%i bytes of 24-bit mailbox written to: %08X\n", sizeof(Mailbox_t), Incoming);
	}
	else
	{
		BuslogicLog("Mailbox 32-bit: Status=0x%02X, CCB at 0x%04X\n", Mailbox32.u.in.CompletionCode, Mailbox32.CCBPointer);
		
		DMAPageWrite(Incoming, &Mailbox32, sizeof(Mailbox32_t));		
		BuslogicLog("%i bytes of 32-bit mailbox written to: %08X\n", sizeof(Mailbox32_t), Incoming);
	}
	
	Buslogic->MailboxInPosCur++;
	if (Buslogic->MailboxInPosCur >= Buslogic->MailboxCount)
		Buslogic->MailboxInPosCur = 0;

	BuslogicRaiseInterrupt(Buslogic, INTR_MBIF | INTR_ANY);

	BuslogicInOperation = 0;
}

static void BuslogicReadSGEntries(int Is24bit, uint32_t SGList, uint32_t Entries, SGE32 *SG)
{
	if (Is24bit)
	{
		uint32_t i;
		SGE SGE24[MAX_SG_DESCRIPTORS];
		
		DMAPageRead(SGList, &SGE24, Entries * sizeof(SGE));

		for (i=0;i<Entries;++i)
		{
			//Convert the 24-bit entries into 32-bit entries.
			SG[i].Segment = ADDR_TO_U32(SGE24[i].Segment);
			SG[i].SegmentPointer = ADDR_TO_U32(SGE24[i].SegmentPointer);
		}
	}
	else
	{
		DMAPageRead(SGList, SG, Entries * sizeof(SGE32));		
	}
}

void BuslogicDataBufferAllocate(BuslogicRequests_t *BuslogicRequests, int Is24bit)
{
	uint32_t sg_buffer_pos = 0;
	uint32_t DataPointer, DataLength;

	uint32_t ScatterGatherEntryLength = (Is24bit ? sizeof(SGE) : sizeof(SGE32));

	if (Is24bit)
	{
		DataPointer = ADDR_TO_U32(BuslogicRequests->CmdBlock.old.DataPointer);
		DataLength = ADDR_TO_U32(BuslogicRequests->CmdBlock.old.DataLength);
	}
	else
	{
		DataPointer = BuslogicRequests->CmdBlock.new.DataPointer;
		DataLength = BuslogicRequests->CmdBlock.new.DataLength;		
	}
	
	BuslogicLog("Data Buffer write: length %d, pointer 0x%04X\n", DataLength, DataPointer);	

	if ((BuslogicRequests->CmdBlock.common.ControlByte != 0x03) && DataLength)
	{
		if (BuslogicRequests->CmdBlock.common.Opcode == SCATTER_GATHER_COMMAND ||
			BuslogicRequests->CmdBlock.common.Opcode == SCATTER_GATHER_COMMAND_RES)
		{
			uint32_t ScatterGatherRead;
			uint32_t ScatterEntry;
			SGE32 ScatterGatherBuffer[MAX_SG_DESCRIPTORS];
			uint32_t ScatterGatherLeft = DataLength / ScatterGatherEntryLength;
			uint32_t ScatterGatherAddrCurrent = DataPointer;
			uint32_t DataToTransfer = 0;
			
			do
			{
				ScatterGatherRead = (ScatterGatherLeft < ELEMENTS(ScatterGatherBuffer))
									? ScatterGatherLeft : ELEMENTS(ScatterGatherBuffer);
								
				ScatterGatherLeft -= ScatterGatherRead;

				BuslogicReadSGEntries(Is24bit, ScatterGatherAddrCurrent, ScatterGatherRead, ScatterGatherBuffer);
				
				for (ScatterEntry = 0; ScatterEntry < ScatterGatherRead; ScatterEntry++)
				{
					uint32_t Address;

					BuslogicLog("BusLogic S/G Write: ScatterEntry=%u\n", ScatterEntry);

					Address = ScatterGatherBuffer[ScatterEntry].SegmentPointer;
					DataToTransfer += ScatterGatherBuffer[ScatterEntry].Segment;

					BuslogicLog("BusLogic S/G Write: Address=%08X DatatoTransfer=%u\n", Address, DataToTransfer);
				}
								
				ScatterGatherAddrCurrent += ScatterGatherRead * ScatterGatherEntryLength;
			} while (ScatterGatherLeft > 0);
			
			BuslogicLog("Data to transfer (S/G) %d\n", DataToTransfer);
			
			SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength = DataToTransfer;
			
			//If the control byte is 0x00, it means that the transfer direction is set up by the SCSI command without
			//checking its length, so do this procedure for both no read/write commands.
			if ((BuslogicRequests->CmdBlock.common.ControlByte == CCB_DATA_XFER_OUT) || (BuslogicRequests->CmdBlock.common.ControlByte == 0x00))
			{
				ScatterGatherLeft = DataLength / ScatterGatherEntryLength;
				ScatterGatherAddrCurrent = DataPointer;
				
				do
				{
					ScatterGatherRead = (ScatterGatherLeft < ELEMENTS(ScatterGatherBuffer))
										? ScatterGatherLeft : ELEMENTS(ScatterGatherBuffer);
									
					ScatterGatherLeft -= ScatterGatherRead;

					BuslogicReadSGEntries(Is24bit, ScatterGatherAddrCurrent, ScatterGatherRead, ScatterGatherBuffer);
									
					for (ScatterEntry = 0; ScatterEntry < ScatterGatherRead; ScatterEntry++)
					{
						uint32_t Address;

						BuslogicLog("BusLogic S/G Write: ScatterEntry=%u\n", ScatterEntry);

						Address = ScatterGatherBuffer[ScatterEntry].SegmentPointer;
						DataToTransfer = ScatterGatherBuffer[ScatterEntry].Segment;
						
						BuslogicLog("BusLogic S/G Write: Address=%08X DatatoTransfer=%u\n", Address, DataToTransfer);

						DMAPageRead(Address, SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].CmdBuffer + sg_buffer_pos, DataToTransfer);
						sg_buffer_pos += DataToTransfer;
					}
									
					ScatterGatherAddrCurrent += ScatterGatherRead * (Is24bit ? sizeof(SGE) : sizeof(SGE32));
				} while (ScatterGatherLeft > 0);				
			}
		}
		else if (BuslogicRequests->CmdBlock.common.Opcode == SCSI_INITIATOR_COMMAND ||
				BuslogicRequests->CmdBlock.common.Opcode == SCSI_INITIATOR_COMMAND_RES)
		{
			uint32_t Address = DataPointer;
			SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength = DataLength;
			
			if (DataLength > 0)
			{
				DMAPageRead(Address, SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].CmdBuffer, SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength);
			}
		}
	}
}

void BuslogicDataBufferFree(BuslogicRequests_t *BuslogicRequests)
{
	uint32_t DataPointer = 0;
	uint32_t DataLength = 0;

	uint32_t sg_buffer_pos = 0;

	uint32_t transfer_length = 0;

	if (BuslogicRequests->Is24bit)
	{
		DataPointer = ADDR_TO_U32(BuslogicRequests->CmdBlock.old.DataPointer);
		DataLength = ADDR_TO_U32(BuslogicRequests->CmdBlock.old.DataLength);
	}
	else
	{
		DataPointer = BuslogicRequests->CmdBlock.new.DataPointer;
		DataLength = BuslogicRequests->CmdBlock.new.DataLength;		
	}

	/* if (DataLength > SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength)
	{
		DataLength = SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength;
	} */

	if ((DataLength != 0) && (BuslogicRequests->CmdBlock.common.Cdb[0] == GPCMD_TEST_UNIT_READY))
	{
		BuslogicLog("Data length not 0 with TEST UNIT READY: %i (%i)\n", DataLength, SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength);
	}
	
	if (BuslogicRequests->CmdBlock.common.Cdb[0] == GPCMD_TEST_UNIT_READY)
	{
		DataLength = 0;
	}
	
	BuslogicLog("Data Buffer read: length %d, pointer 0x%04X\n", DataLength, DataPointer);

	//If the control byte is 0x00, it means that the transfer direction is set up by the SCSI command without
	//checking its length, so do this procedure for both read/write commands.
	if ((DataLength > 0) &&
	    ((BuslogicRequests->CmdBlock.common.ControlByte == CCB_DATA_XFER_IN) ||
	     (BuslogicRequests->CmdBlock.common.ControlByte == 0x00)))
	{	
		if ((BuslogicRequests->CmdBlock.common.Opcode == SCATTER_GATHER_COMMAND) ||
		    (BuslogicRequests->CmdBlock.common.Opcode == SCATTER_GATHER_COMMAND_RES))
		{
			uint32_t ScatterGatherRead;
			uint32_t ScatterEntry;
			SGE32 ScatterGatherBuffer[MAX_SG_DESCRIPTORS];
			uint32_t ScatterGatherEntrySize = (BuslogicRequests->Is24bit ? sizeof(SGE) : sizeof(SGE32));			
			uint32_t ScatterGatherLeft = DataLength / ScatterGatherEntrySize;
			// uint32_t ScatterGatherLength = (ScatterGatherLeft * ScatterGatherEntrySize);
			uint32_t ScatterGatherAddrCurrent = DataPointer;

			do
			{
				ScatterGatherRead = (ScatterGatherLeft < ELEMENTS(ScatterGatherBuffer))
									? ScatterGatherLeft : ELEMENTS(ScatterGatherBuffer);
							
				ScatterGatherLeft -= ScatterGatherRead;

				BuslogicReadSGEntries(BuslogicRequests->Is24bit, ScatterGatherAddrCurrent, ScatterGatherRead, ScatterGatherBuffer);
					
				for (ScatterEntry = 0; ScatterEntry < ScatterGatherRead; ScatterEntry++)
				{
					uint32_t Address;
					uint32_t DataToTransfer;

					BuslogicLog("BusLogic S/G: ScatterEntry=%u\n", ScatterEntry);
					
					Address = ScatterGatherBuffer[ScatterEntry].SegmentPointer;
					DataToTransfer = ScatterGatherBuffer[ScatterEntry].Segment;

					BuslogicLog("BusLogic S/G: Writing %i bytes at %08X\n", DataToTransfer, Address);

					DMAPageWrite(Address, SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].CmdBuffer + sg_buffer_pos, DataToTransfer);
					sg_buffer_pos += DataToTransfer;
				}
					
				ScatterGatherAddrCurrent += (ScatterGatherRead * ScatterGatherEntrySize);
			} while (ScatterGatherLeft > 0);
		}
		else if (BuslogicRequests->CmdBlock.common.Opcode == SCSI_INITIATOR_COMMAND ||
				BuslogicRequests->CmdBlock.common.Opcode == SCSI_INITIATOR_COMMAND_RES)
		{
			uint32_t Address = DataPointer;

			BuslogicLog("BusLogic DMA: Writing %i bytes at %08X\n", DataLength, Address);
			DMAPageWrite(Address, SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].CmdBuffer, DataLength);
		}
	}
	
	if ((BuslogicRequests->CmdBlock.common.Opcode == SCSI_INITIATOR_COMMAND_RES) || (BuslogicRequests->CmdBlock.common.Opcode == SCATTER_GATHER_COMMAND_RES))
	{
		uint32_t Residual;

		/* Should be 0 when scatter/gather? */
		if (DataLength >= SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength)
		{
			Residual = DataLength;
			Residual -= SCSIDevices[BuslogicRequests->TargetID][BuslogicRequests->LUN].InitLength;
		}
		else
		{
			Residual = 0;
		}

		if (BuslogicRequests->Is24bit)
		{
			U32_TO_ADDR(BuslogicRequests->CmdBlock.old.DataLength, Residual);
			BuslogicLog("24-bit Residual data length for reading: %d\n", ADDR_TO_U32(BuslogicRequests->CmdBlock.old.DataLength));
		}
		else
		{
			BuslogicRequests->CmdBlock.new.DataLength = Residual;
			BuslogicLog("32-bit Residual data length for reading: %d\n", BuslogicRequests->CmdBlock.new.DataLength);
		}
	}
}

uint8_t BuslogicRead(uint16_t Port, void *p)
{
	Buslogic_t *Buslogic = (Buslogic_t *)p;
	uint8_t Temp;
	
	switch (Port & 3)
	{
		case 0:
		Temp = Buslogic->Status;
#if 0
		if (Buslogic->Status & STAT_STST)
		{
			Buslogic->Status &= ~STAT_STST;
			Buslogic->Status |= STAT_IDLE;

			if (BuslogicResetCallback <= 0)
			{
				Temp = Buslogic->Status;
				BuslogicResetCallback;
			}
		}
#endif
		break;
		
		case 1:
		if (Buslogic->UseLocalRam)
			Temp = Buslogic->LocalRam.u8View[Buslogic->DataReply];
		else
			Temp = Buslogic->DataBuf[Buslogic->DataReply];
		if (Buslogic->DataReplyLeft)
		{
			Buslogic->DataReply++;
			Buslogic->DataReplyLeft--;
			if (!Buslogic->DataReplyLeft)
			{
				BuslogicCommandComplete(Buslogic);
			}
		}
		break;
		
		case 2:
		Temp = Buslogic->Interrupt;
		break;
		
		case 3:
		Temp = Buslogic->Geometry;
		break;
	}

	if (Port < 0x1000)
	{
		BuslogicLog("Buslogic: Read Port 0x%02X, Returned Value %02X\n", Port, Temp);
	}
	return Temp;	
}

uint16_t BuslogicReadW(uint16_t Port, void *p)
{
	return BuslogicRead(Port, p);
}

uint32_t BuslogicReadL(uint16_t Port, void *p)
{
	return BuslogicRead(Port, p);
}

int buslogic_scsi_drive_is_cdrom(uint8_t id, uint8_t lun)
{
	if (lun > 7)
	{
		return 0;
	}

	if (scsi_cdrom_drives[id][lun] >= CDROM_NUM)
	{
		return 0;
	}
	else
	{
		if (cdrom_drives[scsi_cdrom_drives[id][lun]].enabled && cdrom_drives[scsi_cdrom_drives[id][lun]].bus_type && (cdrom_drives[scsi_cdrom_drives[id][lun]].bus_mode & 2))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
}

void BuslogicWriteW(uint16_t Port, uint16_t Val, void *p);
void BuslogicWriteL(uint16_t Port, uint32_t Val, void *p);

void BuslogicWrite(uint16_t Port, uint8_t Val, void *p)
{
	int i = 0;

	uint8_t j = 0;

	uint8_t max_id = scsi_model ? 16 : 8;

	Buslogic_t *Buslogic = (Buslogic_t *)p;
	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;	
	BuslogicLog("Buslogic: Write Port 0x%02X, Value %02X\n", Port, Val);
	
	switch (Port & 3)
	{
		case 0:
		if ((Val & CTRL_HRST) || (Val & CTRL_SRST))
		{	
			uint8_t Reset = !!(Val & CTRL_HRST);
			BuslogicResetControl(Buslogic, Reset);
			break;
		}
		
		if (Val & CTRL_IRST)
		{
			BuslogicClearInterrupt(Buslogic);
		}
		break;
		
		case 1:
		/* Fast path for the mailbox execution command. */
		if ((Val == 0x02) && (Buslogic->Command == 0xFF))
		{
			/* If there are no mailboxes configured, don't even try to do anything. */
			if (Buslogic->MailboxCount)
			{
				if (!BuslogicCallback)
				{
					BuslogicCallback = 50 * SCSI_TIME;
				}
			}
			return;
		}

		if (Buslogic->Command == 0xFF)
		{
			Buslogic->Command = Val;
			Buslogic->CmdParam = 0;
			
			Buslogic->Status &= ~(STAT_INVCMD | STAT_IDLE);
			BuslogicLog("Buslogic: Operation Code 0x%02X\n", Val);
			switch (Buslogic->Command)
			{
				case 0x00:
				case 0x04:
				case 0x0A:
				case 0x0B:
				case 0x20:
				case 0x23:
				case 0x84:
				case 0x85:
				Buslogic->CmdParamLeft = 0;
				break;				

				case 0x05:
				case 0x07:
				case 0x08:
				case 0x09:
				case 0x0D:
				case 0x1F:
				case 0x21:
				case 0x24:
				case 0x25:
				Buslogic->CmdParamLeft = 1;
				break;	

				case 0x8B:
				case 0x8D:
				case 0x8F:
				case 0x96:
				Buslogic->CmdParamLeft = scsi_model ? 1 : 0;
				break;				

				case 0x91:
				Buslogic->CmdParamLeft = 2;
				break;
				
				case 0x1C:
				case 0x1D:
				Buslogic->CmdParamLeft = 3;
				break;
				
				case 0x06:
				Buslogic->CmdParamLeft = 4;
				break;
				
				case 0x01:
				Buslogic->CmdParamLeft = sizeof(MailboxInit_t);
				break;
				
				case 0x81:
				BuslogicLog("Command 0x81 on %s\n", scsi_model ? "BusLogic" : "Adaptec");
				Buslogic->CmdParamLeft = scsi_model ? sizeof(MailboxInitExtended_t) : 0;
				break;

				case 0x29:
				Buslogic->CmdParamLeft = scsi_model ? 0 : 2;
				break;

				case 0x86: //Valid only for PCI
				Buslogic->CmdParamLeft = 0;
				break;

				case 0x8C:
				Buslogic->CmdParamLeft = scsi_model ? 1 : 0;
				break;

				case 0x95: //Valid only for PCI
				Buslogic->CmdParamLeft = BuslogicIsPCI() ? 1 : 0;
				break;

				case 0x28:
				Buslogic->CmdParamLeft = 0;
				break;
			}
		}
		else
		{
			Buslogic->CmdBuf[Buslogic->CmdParam] = Val;
			Buslogic->CmdParam++;
			Buslogic->CmdParamLeft--;
		}
		
		if (!Buslogic->CmdParamLeft)
		{
			BuslogicLog("Running Operation Code 0x%02X\n", Buslogic->Command);
			switch (Buslogic->Command)
			{
				case 0x00:
				Buslogic->DataReplyLeft = 0;
				break;

				case 0x01:
				{
					Buslogic->Mbx24bit = 1;
							
					MailboxInit_t *MailboxInit = (MailboxInit_t *)Buslogic->CmdBuf;

					Buslogic->MailboxCount = MailboxInit->Count;
					Buslogic->MailboxOutAddr = ADDR_TO_U32(MailboxInit->Address);
					Buslogic->MailboxInAddr = Buslogic->MailboxOutAddr + (Buslogic->MailboxCount * sizeof(Mailbox_t));
						
					BuslogicLog("Buslogic Initialize Mailbox Command\n");
					BuslogicLog("Mailbox Out Address=0x%08X\n", Buslogic->MailboxOutAddr);
					BuslogicLog("Mailbox In Address=0x%08X\n", Buslogic->MailboxInAddr);
					BuslogicLog("Initialized Mailbox, %d entries at 0x%08X\n", MailboxInit->Count, ADDR_TO_U32(MailboxInit->Address));
						
					Buslogic->Status &= ~STAT_INIT;
					Buslogic->DataReplyLeft = 0;
				}
				break;

				case 0x04:
				Buslogic->DataBuf[0] = 0x41;
				Buslogic->DataBuf[1] = scsi_model ? 0x41 : 0x30;
				Buslogic->DataBuf[2] = '5';
				Buslogic->DataBuf[3] = '0';
				Buslogic->DataReplyLeft = 4;
				break;

				case 0x05:
				if (Buslogic->CmdBuf[0] <= 1)
				{
					Buslogic->MailboxOutInterrupts = Buslogic->CmdBuf[0];
					BuslogicLog("Mailbox out interrupts: %s\n", Buslogic->MailboxOutInterrupts ? "ON" : "OFF");
				}
				else
				{
					Buslogic->Status |= STAT_INVCMD;
				}
				Buslogic->DataReplyLeft = 0;
				break;

				case 0x06:
				Buslogic->DataReplyLeft = 0;
				break;
						
				case 0x07:
				Buslogic->DataReplyLeft = 0;
				Buslogic->LocalRam.structured.autoSCSIData.uBusOnDelay = Buslogic->CmdBuf[0];
				BuslogicLog("Bus-on time: %d\n", Buslogic->CmdBuf[0]);
				break;
						
				case 0x08:
				Buslogic->DataReplyLeft = 0;
				Buslogic->LocalRam.structured.autoSCSIData.uBusOffDelay = Buslogic->CmdBuf[0];
				BuslogicLog("Bus-off time: %d\n", Buslogic->CmdBuf[0]);
				break;
						
				case 0x09:
				Buslogic->DataReplyLeft = 0;
				Buslogic->LocalRam.structured.autoSCSIData.uDMATransferRate = Buslogic->CmdBuf[0];
				BuslogicLog("DMA transfer rate: %02X\n", Buslogic->CmdBuf[0]);
				break;
						
				case 0x0A:
				memset(Buslogic->DataBuf, 0, 8);
				for (i = 0; i < 7; i++)
				{
					for (j = 0; j < 8; j++)
					{
						if (buslogic_scsi_drive_is_cdrom(i, j))
							Buslogic->DataBuf[i] = 1;
					}
				}
				Buslogic->DataBuf[7] = 0;
				Buslogic->DataReplyLeft = 8;
				break;				
				
				case 0x0B:
				Buslogic->DataBuf[0] = (1 << Buslogic->DmaChannel);
				Buslogic->DataBuf[1] = (1 << (Buslogic->Irq - 9));
				Buslogic->DataBuf[2] = 7;
				Buslogic->DataReplyLeft = 3;
				break;
						
				case 0x0D:
				{
					Buslogic->DataReplyLeft = Buslogic->CmdBuf[0];

					ReplyInquireSetupInformation *Reply = (ReplyInquireSetupInformation *)Buslogic->DataBuf;
					memset(Reply, 0, sizeof(ReplyInquireSetupInformation));
					
					Reply->fSynchronousInitiationEnabled = 1;
					Reply->fParityCheckingEnabled = 1;
					Reply->cMailbox = Buslogic->MailboxCount;
					U32_TO_ADDR(Reply->MailboxAddress, Buslogic->MailboxOutAddr);
					
					if (scsi_model)
					{
						Reply->uSignature = 'B';
						/* The 'D' signature prevents Buslogic's OS/2 drivers from getting too
						* friendly with Adaptec hardware and upsetting the HBA state.
						*/
						Reply->uCharacterD = 'D';      /* BusLogic model. */
						Reply->uHostBusType = BuslogicIsPCI() ? 'F' : 'A';     /* ISA bus. */
					}
					
					BuslogicLog("Return Setup Information: %d\n", Buslogic->CmdBuf[0]);
				}
				break;

				case 0x23:
				if (scsi_model)
				{
					memset(Buslogic->DataBuf, 0, 8);
					for (i = 8; i < max_id; i++)
					{
						for (i = 0; j < 8; j++)
						{
							if (buslogic_scsi_drive_is_cdrom(i, j))
								Buslogic->DataBuf[i] = 1;
						}
					}
					Buslogic->DataReplyLeft = 8;
				}
				else
				{
					Buslogic->Status |= STAT_INVCMD;
					Buslogic->DataReplyLeft = 0;
				}
				break;

				case 0x1C:
				{
					uint32_t FIFOBuf;
					addr24 Address;
					
					Buslogic->DataReplyLeft = 0;
					Address.hi = Buslogic->CmdBuf[0];
					Address.mid = Buslogic->CmdBuf[1];
					Address.lo = Buslogic->CmdBuf[2];
					FIFOBuf = ADDR_TO_U32(Address);
					DMAPageRead(FIFOBuf, &Buslogic->LocalRam.u8View[64], 64);
				}
				break;
						
				case 0x1D:
				{
					uint32_t FIFOBuf;
					addr24 Address;
					
					Buslogic->DataReplyLeft = 0;
					Address.hi = Buslogic->CmdBuf[0];
					Address.mid = Buslogic->CmdBuf[1];
					Address.lo = Buslogic->CmdBuf[2];
					FIFOBuf = ADDR_TO_U32(Address);
					BuslogicLog("Buslogic FIFO: Writing 64 bytes at %08X\n", FIFOBuf);
					DMAPageWrite(FIFOBuf, &Buslogic->LocalRam.u8View[64], 64);
				}
				break;	
						
				case 0x1F:
				Buslogic->DataBuf[0] = Buslogic->CmdBuf[0];
				Buslogic->DataReplyLeft = 1;
				break;

				case 0x20:
				Buslogic->DataReplyLeft = 0;
				if (scsi_model)
				{
					BuslogicResetControl(Buslogic, 1);
				}
				else
				{
					Buslogic->Status |= STAT_INVCMD;
				}
				break;
		
				case 0x21:
				if (Buslogic->CmdParam == 1)
					Buslogic->CmdParamLeft = Buslogic->CmdBuf[0];
						
				Buslogic->DataReplyLeft = 0;
				break;
						
				case 0x24:
				{
					uint16_t TargetsPresentMask = 0;
							
					for (i = 0; i < max_id; i++)
					{
						for (j = 0; j < 8; j++)
						{
							if (SCSIDevices[i][j].LunType == SCSI_CDROM)
								TargetsPresentMask |= (1 << i);
						}
					}
					Buslogic->DataBuf[0] = TargetsPresentMask&0x0F;
					Buslogic->DataBuf[1] = TargetsPresentMask>>8;
					Buslogic->DataReplyLeft = 2;
				}
				break;
						
				case 0x25:
				if (Buslogic->CmdBuf[0] == 0)
					Buslogic->IrqEnabled = 0;
				else
					Buslogic->IrqEnabled = 1;
				BuslogicLog("Lowering IRQ %i\n", Buslogic->Irq);
				picintc(1 << Buslogic->Irq);
				break;

				case 0x28:
				if (!scsi_model)
				{
					Buslogic->DataBuf[0] = 0x08;
					Buslogic->DataBuf[1] = Buslogic->Lock;
					Buslogic->DataReplyLeft = 2;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;
				}
				break;

				case 0x29:
				if (!scsi_model)
				{
					if (Buslogic->CmdBuf[1] = Buslogic->Lock)
					{
						if (Buslogic->CmdBuf[0] & 1)
						{
							Buslogic->Lock = 1;
						}
						else
						{
							Buslogic->Lock = 0;
						}
					}
					Buslogic->DataReplyLeft = 0;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;
				}
				break;

				case 0x81:
				{
					if (scsi_model)
					{
						Buslogic->Mbx24bit = 0;
								
						MailboxInitExtended_t *MailboxInit = (MailboxInitExtended_t *)Buslogic->CmdBuf;

						Buslogic->MailboxCount = MailboxInit->Count;
						Buslogic->MailboxOutAddr = MailboxInit->Address;
						Buslogic->MailboxInAddr = MailboxInit->Address + (Buslogic->MailboxCount * sizeof(Mailbox32_t));
							
						BuslogicLog("Buslogic Extended Initialize Mailbox Command\n");
						BuslogicLog("Mailbox Out Address=0x%08X\n", Buslogic->MailboxOutAddr);
						BuslogicLog("Mailbox In Address=0x%08X\n", Buslogic->MailboxInAddr);
						BuslogicLog("Initialized Extended Mailbox, %d entries at 0x%08X\n", MailboxInit->Count, MailboxInit->Address);
							
						Buslogic->Status &= ~STAT_INIT;
						Buslogic->DataReplyLeft = 0;
					}
					else
					{
						Buslogic->DataReplyLeft = 0;
						Buslogic->Status |= STAT_INVCMD;
					}
				}
				break;
				
				case 0x84:
				if (scsi_model)
				{
					Buslogic->DataBuf[0] = '7';
					Buslogic->DataReplyLeft = 1;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;
				}
				break;				

				case 0x85:
				if (scsi_model)
				{
					Buslogic->DataBuf[0] = 'B';
					Buslogic->DataReplyLeft = 1;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;					
				}
				break;
		
				case 0x86:
				if (BuslogicIsPCI())
				{
					BuslogicPCIInformation_t *Reply = (BuslogicPCIInformation_t *) Buslogic->DataBuf;
					memset(Reply, 0, sizeof(BuslogicPCIInformation_t));
					Reply->InformationIsValid = 0;
					switch(Buslogic->Base)
					{
						case 0x330:
							Reply->IsaIOPort = 0;
							break;
						case 0x334:
							Reply->IsaIOPort = 1;
							break;
						case 0x230:
							Reply->IsaIOPort = 2;
							break;
						case 0x234:
							Reply->IsaIOPort = 3;
							break;
						case 0x130:
							Reply->IsaIOPort = 4;
							break;
						case 0x134:
							Reply->IsaIOPort = 5;
							break;
						default:
							Reply->IsaIOPort = 0xFF;
							break;
					}
					Reply->IRQ = Buslogic->Irq;
					Buslogic->DataReplyLeft = sizeof(BuslogicPCIInformation_t);
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;					
				}
				break;
		
				case 0x8B:
				{
					if (scsi_model)
					{
						int i;
					
						/* The reply length is set by the guest and is found in the first byte of the command buffer. */
						Buslogic->DataReplyLeft = Buslogic->CmdBuf[0];
						memset(Buslogic->DataBuf, 0, Buslogic->DataReplyLeft);
						char aModelName[] = "542B ";  /* Trailing \0 is fine, that's the filler anyway. */
						if (BuslogicIsPCI())
						{
							aModelName[0] = '9';
							aModelName[1] = '5';
							aModelName[2] = '8';
							aModelName[3] = 'D';
						}
						int cCharsToTransfer =   Buslogic->DataReplyLeft <= sizeof(aModelName)
												? Buslogic->DataReplyLeft
												: sizeof(aModelName);

						for (i = 0; i < cCharsToTransfer; i++)
							Buslogic->DataBuf[i] = aModelName[i];
					}
					else
					{
						Buslogic->DataReplyLeft = 0;
						Buslogic->Status |= STAT_INVCMD;
					}
				}
				break;
				
				case 0x8C:
				// if (BuslogicIsPCI())
				if (scsi_model)
				{
					int i = 0;
					Buslogic->DataReplyLeft = Buslogic->CmdBuf[0];
					memset(Buslogic->DataBuf, 0, Buslogic->DataReplyLeft);
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;					
				}
				break;
		
				case 0x8D:
				{
					if (scsi_model)
					{
						Buslogic->DataReplyLeft = Buslogic->CmdBuf[0];
						ReplyInquireExtendedSetupInformation *Reply = (ReplyInquireExtendedSetupInformation *)Buslogic->DataBuf;
						memset(Reply, 0, sizeof(ReplyInquireExtendedSetupInformation));

						Reply->uBusType = (BuslogicIsPCI()) ? 'E' : 'A';         /* ISA style */
						Reply->uBiosAddress = 0;
						Reply->u16ScatterGatherLimit = 8192;
						Reply->cMailbox = Buslogic->MailboxCount;
						Reply->uMailboxAddressBase = Buslogic->MailboxOutAddr;
						if (BuslogicIsPCI())
						{
							Reply->fLevelSensitiveInterrupt = 1;
							Reply->fHostWideSCSI = 1;
							Reply->fHostUltraSCSI = 1;
						}
						memcpy(Reply->aFirmwareRevision, "07B", sizeof(Reply->aFirmwareRevision));
						BuslogicLog("Return Extended Setup Information: %d\n", Buslogic->CmdBuf[0]);
					}
					else
					{
						Buslogic->DataReplyLeft = 0;
						Buslogic->Status |= STAT_INVCMD;						
					}
				}	
				break;

				/* VirtualBox has these two modes implemented in reverse.
				   According to the BusLogic datasheet:
				   0 is the strict round robin mode, which is also the one used by the AHA-154x according to the
				   Adaptec specification;
				   1 is the aggressive round robin mode, which "hunts" for an active outgoing mailbox and then
				   processes it. */
				case 0x8F:
				if (scsi_model)
				{
					if (Buslogic->CmdBuf[0] == 0)
						Buslogic->StrictRoundRobinMode = 1;
					else if (Buslogic->CmdBuf[0] == 1)
						Buslogic->StrictRoundRobinMode = 0;

					Buslogic->DataReplyLeft = 0;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;					
				}
				break;
						
				case 0x91:
				{
					uint8_t Offset = Buslogic->CmdBuf[0];
					Buslogic->DataReplyLeft = Buslogic->CmdBuf[1];
							
					Buslogic->UseLocalRam = 1;
					Buslogic->DataReply = Offset;
				}
				break;

				case 0x95:
				if (BuslogicIsPCI())
				{
					if (Buslogic->Base != 0)
					{
						io_removehandler(Buslogic->Base, 0x0004, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
					}
					switch(Buslogic->CmdBuf[0])
					{
						case 0:
							Buslogic->Base = 0x330;
							break;
						case 1:
							Buslogic->Base = 0x334;
							break;
						case 2:
							Buslogic->Base = 0x230;
							break;
						case 3:
							Buslogic->Base = 0x234;
							break;
						case 4:
							Buslogic->Base = 0x130;
							break;
						case 5:
							Buslogic->Base = 0x134;
							break;
						default:
							Buslogic->Base = 0;
							break;
					}
					if (Buslogic->Base != 0)
					{
						io_sethandler(Buslogic->Base, 0x0004, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
					}
					Buslogic->DataReplyLeft = 0;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;					
				}
				break;

				case 0x96:
				if (scsi_model)
				{
					if (Buslogic->CmdBuf[0] == 0)
						Buslogic->ExtendedLUNCCBFormat = 0;
					else if (Buslogic->CmdBuf[0] == 1)
						Buslogic->ExtendedLUNCCBFormat = 1;
					
					Buslogic->DataReplyLeft = 0;
				}
				else
				{
					Buslogic->DataReplyLeft = 0;
					Buslogic->Status |= STAT_INVCMD;					
				}
				break;
				
				default:
				case 0x22: //undocumented
				Buslogic->DataReplyLeft = 0;
				Buslogic->Status |= STAT_INVCMD;
				break;
			}
		}
		
		if (Buslogic->DataReplyLeft)
			Buslogic->Status |= STAT_DFULL;
		else if (!Buslogic->CmdParamLeft)
			BuslogicCommandComplete(Buslogic);
		break;
		
		case 2:
		if (scsi_model)
			Buslogic->Interrupt = Val; //For Buslogic
		break;
		
		case 3:
		if (scsi_model)
			Buslogic->Geometry = Val; //For Buslogic
		break;
	}
}

void BuslogicWriteW(uint16_t Port, uint16_t Val, void *p)
{
	BuslogicWrite(Port, Val & 0xFF, p);
}

void BuslogicWriteL(uint16_t Port, uint32_t Val, void *p)
{
	BuslogicWrite(Port, Val & 0xFF, p);
}

static uint8_t BuslogicConvertSenseLength(uint8_t RequestSenseLength)
{
	BuslogicLog("Unconverted Request Sense length %i\n", RequestSenseLength);

	if (RequestSenseLength == 0)
		RequestSenseLength = 14;
	else if (RequestSenseLength == 1)
		RequestSenseLength = 0;
	/* else if ((RequestSenseLength > 1) && (RequestSenseLength < 8))
	{
		if (!scsi_model)  RequestSenseLength = 0;
	} */
	
	BuslogicLog("Request Sense length %i\n", RequestSenseLength);
	
	return RequestSenseLength;
}

static void BuslogicSenseBufferFree(BuslogicRequests_t *BuslogicRequests, int Copy)
{
	uint8_t SenseLength = BuslogicConvertSenseLength(BuslogicRequests->CmdBlock.common.RequestSenseLength);
	uint8_t cdrom_id = scsi_cdrom_drives[BuslogicRequests->TargetID][BuslogicRequests->LUN];

	uint8_t temp_sense[256];

	if (SenseLength && Copy)
	{
		uint32_t SenseBufferAddress;

		cdrom_request_sense_for_scsi(cdrom_id, temp_sense, SenseLength);
		
		/*The sense address, in 32-bit mode, is located in the Sense Pointer of the CCB, but in 
		24-bit mode, it is located at the end of the Command Descriptor Block. */
		
		if (BuslogicRequests->Is24bit)
		{
			SenseBufferAddress = BuslogicRequests->CCBPointer;
			SenseBufferAddress += BuslogicRequests->CmdBlock.common.CdbLength + offsetof(CCB, Cdb);
		}
		else
		{
			SenseBufferAddress = BuslogicRequests->CmdBlock.new.SensePointer;
		}
		
		BuslogicLog("Request Sense address: %02X\n", SenseBufferAddress);

		BuslogicLog("BuslogicSenseBufferFree(): Writing %i bytes at %08X\n", SenseLength, SenseBufferAddress);
		DMAPageWrite(SenseBufferAddress, temp_sense, SenseLength);
		BuslogicLog("Sense data written to buffer: %02X %02X %02X\n", temp_sense[2], temp_sense[12], temp_sense[13]);
	}
}

static void BuslogicCDROMCommand(Buslogic_t *Buslogic)
{
	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;
	uint8_t Id, Lun;

	uint8_t cdrom_id;
	uint8_t cdrom_phase;

	uint32_t temp = 0;

	uint8_t temp_cdb[12];
	uint32_t i;
	
	Id = BuslogicRequests->TargetID;
	Lun = BuslogicRequests->LUN;

	cdrom_id = scsi_cdrom_drives[Id][Lun];

	BuslogicLog("CD-ROM command being executed on: SCSI ID %i, SCSI LUN %i, CD-ROM %i\n", Id, Lun, cdrom_id);

	BuslogicLog("SCSI Cdb[0]=0x%02X\n", BuslogicRequests->CmdBlock.common.Cdb[0]);
	for (i = 1; i < BuslogicRequests->CmdBlock.common.CdbLength; i++)
	{
		BuslogicLog("SCSI Cdb[%i]=%i\n", i, BuslogicRequests->CmdBlock.common.Cdb[i]);
	}

	memset(temp_cdb, 0, cdrom[cdrom_id].cdb_len);
	if (BuslogicRequests->CmdBlock.common.CdbLength <= cdrom[cdrom_id].cdb_len)
	{
		memcpy(temp_cdb, BuslogicRequests->CmdBlock.common.Cdb, BuslogicRequests->CmdBlock.common.CdbLength);
	}
	else
	{
		memcpy(temp_cdb, BuslogicRequests->CmdBlock.common.Cdb, cdrom[cdrom_id].cdb_len);
	}

	cdrom[cdrom_id].request_length = temp_cdb[1];	/* Since that field in the cdrom struct is never used when the bus type is SCSI, let's use it for this scope. */

	if (BuslogicRequests->CmdBlock.common.CdbLength != 12)
	{
		temp_cdb[1] &= 0x1f;				/* Make sure the LUN field of the temporary CDB is always 0, otherwise Daemon Tools drives will misehave when a command is passed through to them. */
	}

	//Finally, execute the SCSI command immediately and get the transfer length.

	SCSIPhase = SCSI_PHASE_COMMAND;
	cdrom_command(cdrom_id, temp_cdb);
	SCSIStatus = cdrom_CDROM_PHASE_to_scsi(cdrom_id);
	if (SCSIStatus == SCSI_STATUS_OK)
	{
		cdrom_phase = cdrom_atapi_phase_to_scsi(cdrom_id);
		if (cdrom_phase == 2)
		{
			/* Command completed - call the phase callback to complete the command. */
			cdrom_phase_callback(cdrom_id);
		}
		else
		{
			/* Command first phase complete - call the callback to execute the second phase. */
			cdrom_phase_callback(cdrom_id);
			SCSIStatus = cdrom_CDROM_PHASE_to_scsi(cdrom_id);
			/* Command second phase complete - call the callback to complete the command. */
			cdrom_phase_callback(cdrom_id);
		}
	}
	else
	{
		/* Error (Check Condition) - call the phase callback to complete the command. */
		cdrom_phase_callback(cdrom_id);
	}

	BuslogicLog("SCSI Status: %s, Sense: %02X, ASC: %02X, ASCQ: %02X\n", (SCSIStatus == SCSI_STATUS_OK) ? "OK" : "CHECK CONDITION", cdrom[cdrom_id].sense[2], cdrom[cdrom_id].sense[12], cdrom[cdrom_id].sense[13]);

	BuslogicDataBufferFree(BuslogicRequests);

	BuslogicSenseBufferFree(BuslogicRequests, (SCSIStatus != SCSI_STATUS_OK));

	BuslogicLog("Request complete\n");

	if (SCSIStatus == SCSI_STATUS_OK)
	{
		BuslogicMailboxInSetup(Buslogic, BuslogicRequests->CCBPointer, &BuslogicRequests->CmdBlock, CCB_COMPLETE, SCSI_STATUS_OK, MBI_SUCCESS);
	}
	else if (SCSIStatus == SCSI_STATUS_CHECK_CONDITION)
	{
		BuslogicMailboxInSetup(Buslogic, BuslogicRequests->CCBPointer, &BuslogicRequests->CmdBlock, CCB_COMPLETE, SCSI_STATUS_CHECK_CONDITION, MBI_ERROR);
	}
}

static int BuslogicSCSIRequestSetup(Buslogic_t *Buslogic, uint32_t CCBPointer, Mailbox32_t *Mailbox32)
{	
	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;
	uint8_t Id, Lun;

	uint8_t cdrom_id;
	uint8_t cdrom_phase;

	uint8_t last_id = scsi_model ? 15 : 7;

	//Fetch data from the Command Control Block.
	DMAPageRead(CCBPointer, &BuslogicRequests->CmdBlock, sizeof(CCB32));

	BuslogicRequests->Is24bit = Buslogic->Mbx24bit;
	BuslogicRequests->CCBPointer = CCBPointer;
	
	BuslogicRequests->TargetID = Buslogic->Mbx24bit ? BuslogicRequests->CmdBlock.old.Id : BuslogicRequests->CmdBlock.new.Id;
	BuslogicRequests->LUN = Buslogic->Mbx24bit ? BuslogicRequests->CmdBlock.old.Lun : BuslogicRequests->CmdBlock.new.Lun;
	
	Id = BuslogicRequests->TargetID;
	Lun = BuslogicRequests->LUN;

	if ((Id > last_id) || (Lun > 7))
	{
		BuslogicMailboxInSetup(Buslogic, CCBPointer, &BuslogicRequests->CmdBlock, CCB_INVALID_CCB, SCSI_STATUS_OK, MBI_ERROR);
		return 1;
	}
	
	BuslogicLog("Scanning SCSI Target ID %i\n", Id);		

	cdrom_id = scsi_cdrom_drives[Id][Lun];

	SCSIStatus = SCSI_STATUS_OK;

	SCSIDevices[Id][Lun].InitLength = 0;

	/* Do this here, so MODE SELECT data will does not get lost in transit. */
	memset(SCSIDevices[Id][Lun].CmdBuffer, 0, 390144);

	BuslogicDataBufferAllocate(BuslogicRequests, BuslogicRequests->Is24bit);

	if (!buslogic_scsi_drive_is_cdrom(Id, Lun))
	{
		BuslogicLog("SCSI Target ID %i and LUN %i have no device attached\n", Id, Lun);

		BuslogicDataBufferFree(BuslogicRequests);

		BuslogicSenseBufferFree(BuslogicRequests, 0);

		BuslogicMailboxInSetup(Buslogic, CCBPointer, &BuslogicRequests->CmdBlock, CCB_SELECTION_TIMEOUT, SCSI_STATUS_OK, MBI_ERROR);
	}
	else
	{
		BuslogicLog("SCSI Target ID %i and LUN %i detected and working\n", Id, Lun);

		BuslogicLog("Transfer Control %02X\n", BuslogicRequests->CmdBlock.common.ControlByte);
		BuslogicLog("CDB Length %i\n", BuslogicRequests->CmdBlock.common.CdbLength);	
		BuslogicLog("CCB Opcode %x\n", BuslogicRequests->CmdBlock.common.Opcode);		
			
		if (BuslogicRequests->CmdBlock.common.ControlByte > 0x03)
		{
			BuslogicLog("Invalid control byte: %02X\n", BuslogicRequests->CmdBlock.common.ControlByte);
		}

		BuslogicInOperation = 1;
	}

	return 1;
}

static int BuslogicSCSIRequestAbort(Buslogic_t *Buslogic, uint32_t CCBPointer)
{	
	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;
	CCBU CmdBlock;

	//Fetch data from the Command Control Block.
	DMAPageRead(CCBPointer, &CmdBlock, sizeof(CCB32));

	//Only SCSI CD-ROMs are supported at the moment, SCSI hard disk support will come soon.
	BuslogicMailboxInSetup(Buslogic, CCBPointer, &CmdBlock, 0x26, SCSI_STATUS_OK, MBI_NOT_FOUND);

	return 1;
}

static uint32_t BuslogicMailboxOut(Buslogic_t *Buslogic, Mailbox32_t *Mailbox32)
{	
	Mailbox_t MailboxOut;
	uint32_t Outgoing;
	
	if (Buslogic->Mbx24bit)
	{
		Outgoing = Buslogic->MailboxOutAddr + (Buslogic->MailboxOutPosCur * sizeof(Mailbox_t));
	
		DMAPageRead(Outgoing, &MailboxOut, sizeof(Mailbox_t));

		Mailbox32->CCBPointer = ADDR_TO_U32(MailboxOut.CCBPointer);
		Mailbox32->u.out.ActionCode = MailboxOut.CmdStatus;
	}
	else
	{
		Outgoing = Buslogic->MailboxOutAddr + (Buslogic->MailboxOutPosCur * sizeof(Mailbox32_t));

		DMAPageRead(Outgoing, Mailbox32, sizeof(Mailbox32_t));	
	}
	
	return Outgoing;
}

static void BuslogicMailboxOutAdvance(Buslogic_t *Buslogic)
{
	Buslogic->MailboxOutPosCur = (Buslogic->MailboxOutPosCur + 1) % Buslogic->MailboxCount;
}

static int BuslogicProcessMailbox(Buslogic_t *Buslogic)
{
	Mailbox32_t Mailbox32;
	Mailbox_t MailboxOut;
	uint32_t Outgoing;

	uint8_t CmdStatus = MBO_FREE;
	uint32_t CodeOffset = 0;

	int old_irq_enabled = Buslogic->IrqEnabled;

	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;

	int ret = 0;

	CodeOffset = Buslogic->Mbx24bit ? offsetof(Mailbox_t, CmdStatus) : offsetof(Mailbox32_t, u.out.ActionCode);

	if (!Buslogic->StrictRoundRobinMode)
	{
		uint8_t MailboxCur = Buslogic->MailboxOutPosCur;

		/* Search for a filled mailbox - stop if we have scanned all mailboxes. */
		do
		{
			/* Fetch mailbox from guest memory. */
			Outgoing = BuslogicMailboxOut(Buslogic, &Mailbox32);

			/* Check the next mailbox. */
			BuslogicMailboxOutAdvance(Buslogic);
		} while ((Mailbox32.u.out.ActionCode == MBO_FREE) && (MailboxCur != Buslogic->MailboxOutPosCur));
	}
	else
	{
		Outgoing = BuslogicMailboxOut(Buslogic, &Mailbox32);
	}

	if (Mailbox32.u.out.ActionCode != MBO_FREE)
	{
		/* We got the mailbox, mark it as free in the guest. */
		BuslogicLog("BuslogicStartMailbox(): Writing %i bytes at %08X\n", sizeof(CmdStatus), Outgoing + CodeOffset);
		DMAPageWrite(Outgoing + CodeOffset, &CmdStatus, sizeof(CmdStatus));
	}

	if (Buslogic->MailboxOutInterrupts)
	{
		BuslogicRaiseInterrupt(Buslogic, INTR_MBOA | INTR_ANY);
	}

	/* Check if the mailbox is actually loaded. */
	if (Mailbox32.u.out.ActionCode == MBO_FREE)
	{
		// BuslogicLog("No loaded mailbox left\n");
		return 0;
	}

	if (Mailbox32.u.out.ActionCode == MBO_START)
	{
		BuslogicLog("Start Mailbox Command\n");
		ret = BuslogicSCSIRequestSetup(Buslogic, Mailbox32.CCBPointer, &Mailbox32);
	} 
	else if (Mailbox32.u.out.ActionCode == MBO_ABORT)
	{
		BuslogicLog("Abort Mailbox Command\n");
		ret = BuslogicSCSIRequestAbort(Buslogic, Mailbox32.CCBPointer);
	}
	else
	{
		BuslogicLog("Invalid action code: %02X\n", Mailbox32.u.out.ActionCode);
		ret = 0;
	}

	/* Advance to the next mailbox. */
	if (Buslogic->StrictRoundRobinMode)
	{
		BuslogicMailboxOutAdvance(Buslogic);
	}

	return ret;
}

void BuslogicResetPoll(void *p)
{
	Buslogic_t *Buslogic = (Buslogic_t *)p;

	Buslogic->Status &= ~STAT_STST;
	Buslogic->Status |= STAT_IDLE;

	BuslogicResetCallback = 0;
}

void BuslogicCommandCallback(void *p)
{
	Buslogic_t *Buslogic = (Buslogic_t *)p;

	BuslogicRequests_t *BuslogicRequests = &Buslogic->BuslogicRequests;

	int ret = 0;
	int i = 0;

	// BuslogicLog("BusLogic Callback (%08X)!\n", BuslogicCallback);

	if (BuslogicInOperation == 0)
	{
		// BuslogicLog("BusLogic Callback: Start outgoing mailbox\n");
		if (Buslogic->MailboxCount)
		{
			ret = BuslogicProcessMailbox(Buslogic);
		}
		else
		{
			// fatal("Callback active with mailbox count 0!\n");
			BuslogicCallback += 50 * SCSI_TIME;
			return;
		}
	}
	else if (BuslogicInOperation == 1)
	{
		BuslogicLog("BusLogic Callback: Process request\n");
		BuslogicCDROMCommand(Buslogic);
	}
	else if (BuslogicInOperation == 2)
	{
		BuslogicLog("BusLogic Callback: Send incoming mailbox\n");
		BuslogicMailboxIn(Buslogic);
	}
	else
	{
		fatal("Invalid BusLogic callback phase: %i\n", BuslogicInOperation);
	}

	BuslogicCallback += 50 * SCSI_TIME;
}

uint8_t mem_read_null(uint32_t addr, void *priv)
{
        return 0;
}

uint16_t mem_read_nullw(uint32_t addr, void *priv)
{
        return 0;
}

uint32_t mem_read_nulll(uint32_t addr, void *priv)
{
        return 0;
}

typedef union
{
	uint32_t addr;
	uint8_t addr_regs[4];
} bar_t;

uint8_t buslogic_pci_regs[256];

bar_t buslogic_pci_bar[3];

uint8_t BuslogicPCIRead(int func, int addr, void *p)
{
	Buslogic_t *Buslogic = (Buslogic_t *)p;

	// BuslogicLog("BusLogic PCI read %08X\n", addr);
	switch (addr)
	{
		case 0x00:
			return 0x4b;
		case 0x01:
			return 0x10;

		case 0x02:
			return 0x40;
		case 0x03:
			return 0x10;

		case 0x2C:
			return 0x4b;
		case 0x2D:
			return 0x10;
		case 0x2E:
			return 0x40;
		case 0x2F:
			return 0x10;

		case 0x04:
			return buslogic_pci_regs[0x04];	/*Respond to IO and memory accesses*/
		case 0x05:
			return buslogic_pci_regs[0x05];

		case 0x07:
			return 2;

		case 0x08:
			return 1;						/*Revision ID*/
		case 0x09:
			return 0;						/*Programming interface*/
		case 0x0A:
			return 0;						/*Subclass*/
		case 0x0B:
			return 1;						/* Class code*/

		case 0x10:
			return (buslogic_pci_bar[0].addr_regs[0] & 0xe0) | 1;	/*I/O space*/
		case 0x11:
			return buslogic_pci_bar[0].addr_regs[1];
		case 0x12:
			return buslogic_pci_bar[0].addr_regs[2];
		case 0x13:
			return buslogic_pci_bar[0].addr_regs[3];

		case 0x14:
			return (buslogic_pci_bar[1].addr_regs[0] & 0xe0);	/*Memory space*/
		case 0x15:
			return buslogic_pci_bar[1].addr_regs[1];
		case 0x16:
			return buslogic_pci_bar[1].addr_regs[2];
		case 0x17:
			return buslogic_pci_bar[1].addr_regs[3];

		case 0x30:
			return buslogic_pci_bar[2].addr_regs[0] & 0x01;	/*BIOS ROM address*/
		case 0x31:
			return buslogic_pci_bar[2].addr_regs[1] | 0x18;
		case 0x32:
			return buslogic_pci_bar[2].addr_regs[2];
		case 0x33:
			return buslogic_pci_bar[2].addr_regs[3];

		case 0x3C:
			return Buslogic->Irq;
		case 0x3D:
			return 1;
	}
	return 0;
}

void BuslogicPCIWrite(int func, int addr, uint8_t val, void *p)
{
	Buslogic_t *Buslogic = (Buslogic_t *)p;

	switch (addr)
	{
		case 0x04:
			io_removehandler(Buslogic->PCIBase, 0x0004, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
			mem_mapping_disable(&Buslogic->mmio_mapping);
			if (val & PCI_COMMAND_IO)
			{
				if (Buslogic->PCIBase != 0)
				{
					io_sethandler(Buslogic->PCIBase, 0x0020, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
				}
			}
			if (val & PCI_COMMAND_MEM)
			{
				if (Buslogic->PCIBase != 0)
				{
		                        mem_mapping_set_addr(&Buslogic->mmio_mapping, Buslogic->MMIOBase, 0x20);
				}
			}
			buslogic_pci_regs[addr] = val;
			break;

		case 0x10:
			val &= 0xe0;
			val |= 1;
		case 0x11: case 0x12: case 0x13:
			/* I/O Base set. */
			/* First, remove the old I/O. */
			io_removehandler(Buslogic->PCIBase, 0x0020, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
			/* Then let's set the PCI regs. */
			buslogic_pci_bar[0].addr_regs[addr & 3] = val;
			/* Then let's calculate the new I/O base. */
			Buslogic->PCIBase = buslogic_pci_bar[0].addr & 0xffe0;
			/* Log the new base. */
			BuslogicLog("BusLogic PCI: New I/O base is %04X\n" , Buslogic->PCIBase);
			/* We're done, so get out of the here. */
			if (buslogic_pci_regs[4] & PCI_COMMAND_IO)
			{
				if (Buslogic->PCIBase != 0)
				{
					io_sethandler(Buslogic->PCIBase, 0x0020, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
				}
			}
			return;

		case 0x14:
			val &= 0xe0;
		case 0x15: case 0x16: case 0x17:
			/* I/O Base set. */
			/* First, remove the old I/O. */
			mem_mapping_disable(&Buslogic->mmio_mapping);
			/* Then let's set the PCI regs. */
			buslogic_pci_bar[1].addr_regs[addr & 3] = val;
			/* Then let's calculate the new I/O base. */
			Buslogic->MMIOBase = buslogic_pci_bar[1].addr & 0xffffffe0;
			/* Log the new base. */
			BuslogicLog("BusLogic PCI: New MMIO base is %04X\n" , Buslogic->MMIOBase);
			/* We're done, so get out of the here. */
			if (buslogic_pci_regs[4] & PCI_COMMAND_MEM)
			{
				if (Buslogic->PCIBase != 0)
				{
		                        mem_mapping_set_addr(&Buslogic->mmio_mapping, Buslogic->MMIOBase, 0x20);
				}
			}
			return;

		/* Commented out until an APIC controller is emulated for the PIIX3,
		   otherwise the BT-958 will not get an IRQ on boards using the PIIX3. */
#if 0
		case 0x3C:
			buslogic_pci_regs[addr] = val;
			if (val != 0xFF)
			{
				buslogic_log("BusLogic IRQ now: %i\n", val);
				Buslogic->Irq = val;
			}
			return;
#endif
	}
}

void *BuslogicInit(void)
{
	int i = 0;

	Buslogic_t *Buslogic = malloc(sizeof(Buslogic_t));
	memset(Buslogic, 0, sizeof(Buslogic_t));

	BuslogicResetDevice = Buslogic;

	scsi_model = device_get_config_int("model");
	Buslogic->Base = device_get_config_int("addr");
	Buslogic->PCIBase = 0;
	Buslogic->MMIOBase = 0;
	Buslogic->Irq = device_get_config_int("irq");
	Buslogic->DmaChannel = device_get_config_int("dma");

	if (Buslogic->Base != 0)
	{
		if (BuslogicIsPCI())
		{
			io_sethandler(Buslogic->Base, 0x0004, BuslogicRead, BuslogicReadW, BuslogicReadL, BuslogicWrite, BuslogicWriteW, BuslogicWriteL, Buslogic);
		}
		else
		{
			io_sethandler(Buslogic->Base, 0x0004, BuslogicRead, BuslogicReadW, NULL, BuslogicWrite, BuslogicWriteW, NULL, Buslogic);
		}
	}

	BuslogicLog("Building CD-ROM map...\n");
	build_scsi_cdrom_map();

	for (i = 0; i < CDROM_NUM; i++)
	{
		if (buslogic_scsi_drive_is_cdrom(cdrom_drives[i].scsi_device_id, cdrom_drives[i].scsi_device_lun))
		{
			SCSIDevices[cdrom_drives[i].scsi_device_id][cdrom_drives[i].scsi_device_lun].LunType == SCSI_CDROM;
		}
	}

	timer_add(BuslogicResetPoll, &BuslogicResetCallback, &BuslogicResetCallback, Buslogic);
	timer_add(BuslogicCommandCallback, &BuslogicCallback, &BuslogicCallback, Buslogic);

	if (BuslogicIsPCI())
	{
		pci_add(BuslogicPCIRead, BuslogicPCIWrite, Buslogic);

		buslogic_pci_bar[0].addr_regs[0] = 1;
		buslogic_pci_bar[1].addr_regs[0] = 0;

        	buslogic_pci_regs[0x04] = 1;
	        buslogic_pci_regs[0x05] = 0;

	        buslogic_pci_regs[0x07] = 2;

		buslogic_pci_bar[2].addr = 0;

	        mem_mapping_add(&Buslogic->mmio_mapping, 0xfffd0000, 0x20, mem_read_null, mem_read_nullw, mem_read_nulll, mem_write_null, mem_write_nullw, mem_write_nulll, NULL, MEM_MAPPING_EXTERNAL, Buslogic);
		mem_mapping_disable(&Buslogic->mmio_mapping);
	}
	
	BuslogicLog("Buslogic on port 0x%04X\n", Buslogic->Base);
	
	BuslogicResetControl(Buslogic, CTRL_HRST);
	
	return Buslogic;
}

void BuslogicClose(void *p)
{
	Buslogic_t *Buslogic = (Buslogic_t *)p;
	free(Buslogic);
	BuslogicResetDevice = NULL;
}

static device_config_t BuslogicConfig[] =
{
        {
                .name = "model",
                .description = "Model",
                .type = CONFIG_BINARY,
                .type = CONFIG_SELECTION,
                .selection =
                {
                        {
                                .description = "Adaptec AHA-154XB ISA",
                                .value = 0
                        },
                        {
                                .description = "BusLogic BT-542B ISA",
                                .value = 1
                        },
                        {
                                .description = "BusLogic BT-958 PCI",
                                .value = 2
                        },
                        {
                                .description = ""
                        }
                },
                .default_int = 0
        },
        {
                .name = "addr",
                .description = "Address",
                .type = CONFIG_BINARY,
                .type = CONFIG_SELECTION,
                .selection =
                {
                        {
                                .description = "None",
                                .value = 0
                        },
                        {
                                .description = "0x330",
                                .value = 0x330
                        },
                        {
                                .description = "0x334",
                                .value = 0x334
                        },
                        {
                                .description = "0x230",
                                .value = 0x230
                        },
                        {
                                .description = "0x234",
                                .value = 0x234
                        },
                        {
                                .description = "0x130",
                                .value = 0x130
                        },
                        {
                                .description = "0x134",
                                .value = 0x134
                        },
                        {
                                .description = ""
                        }
                },
                .default_int = 0x334
        },
        {
                .name = "irq",
                .description = "IRQ",
                .type = CONFIG_SELECTION,
                .selection =
                {
                        {
                                .description = "IRQ 9",
                                .value = 9
                        },
                        {
                                .description = "IRQ 10",
                                .value = 10
                        },
                        {
                                .description = "IRQ 11",
                                .value = 11
                        },
                        {
                                .description = "IRQ 12",
                                .value = 12
                        },
                        {
                                .description = "IRQ 14",
                                .value = 14
                        },
                        {
                                .description = "IRQ 15",
                                .value = 15
                        },
                        {
                                .description = ""
                        }
                },
                .default_int = 9
        },
        {
                .name = "dma",
                .description = "DMA channel",
                .type = CONFIG_SELECTION,
                .selection =
                {
                        {
                                .description = "DMA 5",
                                .value = 5
                        },
                        {
                                .description = "DMA 6",
                                .value = 6
                        },
                        {
                                .description = "DMA 7",
                                .value = 7
                        },
                        {
                                .description = ""
                        }
                },
                .default_int = 6
        },
        {
                .type = -1
        }
};

device_t BuslogicDevice =
{
	"Adaptec/Buslogic",
	0,
	BuslogicInit,
	BuslogicClose,
	NULL,
	NULL,
	NULL,
	NULL,
	BuslogicConfig
};
