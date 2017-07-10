/*************************************************************************
Title:    Yet another (small) modbus implementation for the avr.
Author:   Max Brueggemann
Hardware: any AVR with hardware UART, tested on Atmega 88/168 at 20Mhz
License:  GNU General Public License 
          
DESCRIPTION:
    Refer to the header file yaMBSiavr.h.
    
USAGE:
    Refer to the header file yaMBSiavr.h.
                    
LICENSE:
    Copyright (C) 2016 Max Brueggemann

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
                        
*************************************************************************/


#include "defines.h"
#include "modbus_RTU.h"
#include "drv_uart1.h"

volatile unsigned char BusState = 0;
volatile uint16_t modbusTimer = 0;
volatile unsigned char rxbuffer[MaxFrameIndex+1];
volatile uint16_t DataPos = 0;
volatile unsigned char PacketTopIndex = 7;
volatile unsigned char modBusStaMaStates = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*UART Driver Queue Status

  Summary
    Defines the object required for the status of the queue.
*/

typedef union
{
    struct
    {
            uint8_t full:1;
            uint8_t empty:1;
            uint8_t reserved:6;
    }s;
    uint8_t status;
}DRV_UART_BYTEQ_STATUS;
// *****************************************************************************

/* UART Driver Hardware Instance Object

  Summary:
    Defines the object required for the maintenance of the hardware instance.

*/
typedef struct
{
    /* RX Byte Q */
    uint8_t                                      *rxTail ;

    uint8_t                                      *rxHead ;

    /* TX Byte Q */
    uint8_t                                      *txTail ;

    uint8_t                                      *txHead ;

    DRV_UART_BYTEQ_STATUS                        rxStatus ;

    DRV_UART_BYTEQ_STATUS                        txStatus ;

} DRV_UART_OBJECT ;

static DRV_UART_OBJECT drv_uart1_obj ;

// *****************************************************************************
/* UART Driver Queue Length

  Summary:
    Defines the length of the Transmit and Receive Buffers

*/

#ifndef DRV_UART1_CONFIG_TX_BYTEQ_LENGTH
        #define DRV_UART1_CONFIG_TX_BYTEQ_LENGTH 4
#endif
#ifndef DRV_UART1_CONFIG_RX_BYTEQ_LENGTH
        #define DRV_UART1_CONFIG_RX_BYTEQ_LENGTH 4
#endif

#define DRV_UART1_CONFIG_8N1
// *****************************************************************************
/* Default values of the static overrides

  Summary:
   Checks for the definitions, if definitions found for stop bits, data bits,
   parity and baud rate those definitions are used, otherwise default values are
   used.

*/

#if defined(DRV_UART1_CONFIG_8N1)
        #define DRV_UART1_PDS 0
#elif defined(DRV_UART1_CONFIG_8N2)
        #define DRV_UART1_PDS 1
#elif defined(DRV_UART1_CONFIG_8E1)
        #define DRV_UART1_PDS 2
#elif defined(DRV_UART1_CONFIG_8E2)
        #define DRV_UART1_PDS 3
#elif defined(DRV_UART1_CONFIG_8O1)
        #define DRV_UART1_PDS 4
#elif defined(DRV_UART1_CONFIG_8O2)
        #define DRV_UART1_PDS 5
#elif defined(DRV_UART1_CONFIG_9N1)
        #define DRV_UART1_PDS 6
#elif defined(DRV_UART1_CONFIG_9N2)
        #define DRV_UART1_PDS 7
#else
        #define DRV_UART1_PDS 0
#endif

#if !defined(DRV_UART1_CONFIG_BAUD_RATE)
        #define DRV_UART1_CONFIG_BAUD_RATE 9600
#endif



#define BRG_TEMP (unsigned long)(((FCY / DRV_UART1_CONFIG_BAUD_RATE) / 16) - 1)
 


// *****************************************************************************

/* UART Driver Queue

  Summary:
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart1_txByteQ[DRV_UART1_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart1_rxByteQ[DRV_UART1_CONFIG_RX_BYTEQ_LENGTH] ;

// *****************************************************************************
/* UART Hardware FIFO Buffer Length

  Summary:
    Defines the length of the Transmit and Receive FIFOs
 
*/

#define DRV_UART1_TX_FIFO_LENGTH 4
#define DRV_UART1_RX_FIFO_LENGTH 4 

// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function: void DRV_UART1_InitializerDefault(void)

  Summary:
    Initializes the UART instance : 1

*/

extern unsigned char Segment_Digit[3];
extern unsigned char UART_last_Byte_Sent_Flag;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void DRV_UART1_InitializerDefault (void)
{
   U1MODE = (0x8008 & 0xFFFC)| DRV_UART1_PDS;
   U1MODEbits.ALTIO = 1;
   // UTXEN disabled; UTXINV disabled; URXISEL RX_ONE_CHAR; ADDEN disabled; UTXISEL0 TX_ONE_CHAR; UTXBRK COMPLETED; OERR NO_ERROR_cleared; 
   U1STA = 0x0000;
   U1STAbits.UTXISEL =1;
   U1BRG = BRG_TEMP;
   
   IPC2bits.U1RXIP = 5;
   IEC0bits.U1RXIE = 1;
   
   U1STAbits.UTXEN = 1;
   

   drv_uart1_obj.txHead = uart1_txByteQ;
   drv_uart1_obj.txTail = uart1_txByteQ;
   drv_uart1_obj.rxHead = uart1_rxByteQ;
   drv_uart1_obj.rxTail = uart1_rxByteQ;
   drv_uart1_obj.rxStatus.s.empty = true;
   drv_uart1_obj.txStatus.s.empty = true;
   drv_uart1_obj.txStatus.s.full = false;
   drv_uart1_obj.rxStatus.s.full = false;
}

// *****************************************************************************
/* Function:
    void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )

  Summary:
    Maintains the driver's transmitter state machine and implements its ISR

*/

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{ 
   /* if(drv_uart1_obj.txStatus.s.empty)
    {
        IEC0bits.U1TXIE = false;
        
        return;
    }

    IFS0bits.U1TXIF = false;

    int count = 0;
    while((count < DRV_UART1_TX_FIFO_LENGTH)&& !(U1STAbits.UTXBF == 1))
    {
        count++;

        U1TXREG = *drv_uart1_obj.txHead;

        drv_uart1_obj.txHead++;

        if(drv_uart1_obj.txHead == (uart1_txByteQ + DRV_UART1_CONFIG_TX_BYTEQ_LENGTH))
        {
            drv_uart1_obj.txHead = uart1_txByteQ;
        }

        drv_uart1_obj.txStatus.s.full = false;

        if(drv_uart1_obj.txHead == drv_uart1_obj.txTail)
        {
            drv_uart1_obj.txStatus.s.empty = true;
            break;
        }
    }*/
    IFS0bits.U1TXIF = false;
    BusState&=~(1<<TransmitRequested);
	BusState|=(1<<Transmitting);
    //while (U1STAbits.UTXBF==1);
	U1TXREG=rxbuffer[DataPos];
	DataPos++;
	if (DataPos==(PacketTopIndex+1)) {
        //if (DataPos==8) {
		//UART_CONTROL&=~(1<<UART_UDRIE);
        IEC0bits.U1TXIE = false;  
        UART_last_Byte_Sent_Flag = 1;
        
	}
    
}
// *****************************************************************************
/* Function:
    void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt ( void )

  Summary:
    Maintains the driver's transmitter state machine and implements its ISR

*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt ( void )
{
    /*int count = 0;
    Segment_Digit[2] = 1 - Segment_Digit[2];
    while((count < DRV_UART1_RX_FIFO_LENGTH) && (U1STAbits.URXDA == 1))
    {
        count++;
        
        Segment_Digit[1] = U1RXREG;    
        *drv_uart1_obj.rxTail = U1RXREG;

        drv_uart1_obj.rxTail++;

        if(drv_uart1_obj.rxTail == (uart1_rxByteQ + DRV_UART1_CONFIG_RX_BYTEQ_LENGTH))
        {
            drv_uart1_obj.rxTail = uart1_rxByteQ;
        }

        drv_uart1_obj.rxStatus.s.empty = false;
        
        if(drv_uart1_obj.rxTail == drv_uart1_obj.rxHead)
        {
            //Sets the flag RX full
            drv_uart1_obj.rxStatus.s.full = true;
            break;
        }
        
    }*/
    unsigned char data;
	data = U1RXREG;
    
	modbusTimer=0; //reset timer
	if (!(BusState & (1<<ReceiveCompleted)) && !(BusState & (1<<TransmitRequested)) && !(BusState & (1<<Transmitting)) && (BusState & (1<<Receiving)) && !(BusState & (1<<BusTimedOut))) { // // 
		if (DataPos>MaxFrameIndex) modbusReset();
	    else {
			rxbuffer[DataPos]=data;            
			DataPos++; //TODO: maybe prevent this from exceeding 255?
            
		}	    
    } else if (!(BusState & (1<<ReceiveCompleted)) && !(BusState & (1<<TransmitRequested)) && !(BusState & (1<<Transmitting)) && !(BusState & (1<<Receiving)) && (BusState & (1<<BusTimedOut))) { //) {
			 rxbuffer[0]=data;
			 BusState=((1<<Receiving)|(1<<TimerActive));
			 DataPos=1;
    }

    IFS0bits.U1RXIF = false;
   
}
// *****************************************************************************
/* Function:
    void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void );

  Summary:
    Maintains the driver's error-handling state machine and implements its ISR

*/
//void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void )
//{
//    if ((U1STAbits.OERR == 1))
//    {
//        U1STAbits.OERR = 0;
//    }
//
//    IFS4bits.U1ERIF = false;
//}

// *****************************************************************************
// *****************************************************************************
// Section: UART Driver Client Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    uint8_t DRV_UART1_ReadByte( void)

  Summary:
    Reads a byte of data from the UART1

*/

uint8_t DRV_UART1_ReadByte( void)
{
    uint8_t data = 0;

    data = *drv_uart1_obj.rxHead;

    drv_uart1_obj.rxHead++;

    if (drv_uart1_obj.rxHead == (uart1_rxByteQ + DRV_UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
        drv_uart1_obj.rxHead = uart1_rxByteQ;
    }

    if (drv_uart1_obj.rxHead == drv_uart1_obj.rxTail)
    {
        drv_uart1_obj.rxStatus.s.empty = true;
    }

    drv_uart1_obj.rxStatus.s.full = false;

    return data;
}

//*******************************************************************************
/*  Function:
    int DRV_UART1_Read(uint8_t *buffer, const unsigned int numbytes )

  Summary:
    Returns the number of bytes read by the UART1 peripheral

*/

unsigned int DRV_UART1_Read( uint8_t *buffer ,  const unsigned int numbytes)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( numbytes ))
    {
        if( drv_uart1_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = DRV_UART1_ReadByte () ;
        }
    }

    return numBytesRead ;
}

// *****************************************************************************
/* Function:
    void DRV_UART1_WriteByte( const uint8_t byte)

  Summary:
    Writes a byte of data to the UART1

*/

void DRV_UART1_WriteByte( const uint8_t byte)
{
    *drv_uart1_obj.txTail = byte;
    
    drv_uart1_obj.txTail++;
    
    if (drv_uart1_obj.txTail == (uart1_txByteQ + DRV_UART1_CONFIG_TX_BYTEQ_LENGTH))
    {
        drv_uart1_obj.txTail = uart1_txByteQ;
    }

    drv_uart1_obj.txStatus.s.empty = false;

    if (drv_uart1_obj.txHead == drv_uart1_obj.txTail)
    {
        drv_uart1_obj.txStatus.s.full = true;
    }

    if (IEC0bits.U1TXIE  == false)
    {
        IEC0bits.U1TXIE = true ;
    }
	
}

//*******************************************************************************
/*  Function:
    unsigned int DRV_UART1_Write( uint8_t *buffer, const unsigned int numbytes )

  Summary:
    Returns the number of bytes written into the internal buffer

*/

unsigned int DRV_UART1_Write( const uint8_t *buffer , const unsigned int numbytes )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( numbytes ))
    {
        if((drv_uart1_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            DRV_UART1_WriteByte (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

} 

// *****************************************************************************
/* Function:
    DRV_UART1_TRANSFER_STATUS DRV_UART1_TransferStatus (void)

  Summary:
    Returns the transmitter and receiver transfer status

*/
DRV_UART1_TRANSFER_STATUS DRV_UART1_TransferStatus (void )
{
    DRV_UART1_TRANSFER_STATUS status = 0;

    if(drv_uart1_obj.txStatus.s.full)
    {
        status |= DRV_UART1_TRANSFER_STATUS_TX_FULL;
    }

    if(drv_uart1_obj.txStatus.s.empty)
    {
        status |= DRV_UART1_TRANSFER_STATUS_TX_EMPTY;
    }

    if(drv_uart1_obj.rxStatus.s.full)
    {
        status |= DRV_UART1_TRANSFER_STATUS_RX_FULL;
    }

    if(drv_uart1_obj.rxStatus.s.empty)
    {
        status |= DRV_UART1_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= DRV_UART1_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}

// *****************************************************************************
/* Function:
  uint8_t DRV_UART1_Peek(uint16_t offset)

  Summary:
    Returns the character in the read sequence at the offset provided, without
    extracting it

*/
uint8_t DRV_UART1_Peek(uint16_t offset)
{
    if( (drv_uart1_obj.rxHead + offset) > (uart1_rxByteQ + DRV_UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart1_rxByteQ[offset - (uart1_rxByteQ + DRV_UART1_CONFIG_RX_BYTEQ_LENGTH - drv_uart1_obj.rxHead)];
    }
    else
    {
      return *(drv_uart1_obj.rxHead + offset);
    }
}

// *****************************************************************************
/* Function:
  unsigned int DRV_UART1_RXBufferSizeGet (void)

  Summary:
    Returns the size of the receive buffer

*/
unsigned int DRV_UART1_RXBufferSizeGet(void)
{
    if(drv_uart1_obj.rxHead > drv_uart1_obj.rxTail)
    {
        return(DRV_UART1_CONFIG_RX_BYTEQ_LENGTH - (int)(drv_uart1_obj.rxHead) - (int)(drv_uart1_obj.rxTail));
    }
    else
    {
        return(drv_uart1_obj.rxTail - drv_uart1_obj.rxHead);
    }
}

// *****************************************************************************
/* Function:
  unsigned int DRV_UART1_TXBufferSizeGet (void)

  Summary:
    Returns the size of the transmit buffer

*/
unsigned int DRV_UART1_TXBufferSizeGet(void)
{
     if(drv_uart1_obj.txHead > drv_uart1_obj.txTail)
    {
        return(DRV_UART1_CONFIG_TX_BYTEQ_LENGTH - (int)drv_uart1_obj.txHead - (int)drv_uart1_obj.txTail);
    }
    else
    {
        return(drv_uart1_obj.txTail - drv_uart1_obj.txHead);
    }
}

// *****************************************************************************
/* Function:
  bool DRV_UART1_RXBufferIsEmpty (void)

  Summary:
    Returns the status of the receive buffer

*/
bool DRV_UART1_RXBufferIsEmpty (void)
{
    return(drv_uart1_obj.rxStatus.s.empty);
}

// *****************************************************************************
/* Function:
    bool DRV_UART1_TXBufferIsFull (void)

  Summary:
    Returns the status of the transmit buffer

*/
bool DRV_UART1_TXBufferIsFull (void)
{
    return(drv_uart1_obj.txStatus.s.full);
}


// *****************************************************************************
/* Function:
    DRV_UART1_STATUS DRV_UART1_Status (void)

  Summary:
    Returns the status of the transmit and receive

*/
DRV_UART1_STATUS DRV_UART1_Status (void)
{
    return U1STA;
}

//*******************************************************************************
/*
  End of File
*/


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint8_t modbusGetBusState(void) {
	return BusState;
}

#if ADDRESS_MODE == SINGLE_ADR
volatile unsigned char Address = 0x00;
uint8_t modbusGetAddress(void) {
	return Address;
}

void modbusSetAddress(unsigned char newadr) {
	Address = newadr;
}
#endif

#if PHYSICAL_TYPE == 485
 void transceiver_txen(void) {
	//TRANSCEIVER_ENABLE_PORT|=(1<<TRANSCEIVER_ENABLE_PIN);
     //READY = 1;
 }

 void transceiver_rxen(void) {
	//TRANSCEIVER_ENABLE_PORT&=~(1<<TRANSCEIVER_ENABLE_PIN);
     //READY = 0;
 }
#endif

/* A fairly simple Modbus compliant 16 Bit CRC algorithm.
*  Returns 1 if the crc check is positive, returns 0 if it fails.
*  Appends
*/
uint16_t crc16(volatile uint8_t *ptrToArray,uint8_t inputSize) //A standard CRC algorithm
{
	uint16_t out=0xffff;
	uint16_t carry;
	unsigned char n;
    int l=0;
	inputSize++;
	for (l=0; l<inputSize; l++) {
		out ^= ptrToArray[l];
		for (n = 0; n < 8; n++) {
			carry = out & 1;
			out >>= 1;
			if (carry) out ^= 0xA001;
		}
	}
	//out=0x1234;
	if ((ptrToArray[inputSize]==out%256) && (ptrToArray[inputSize+1]==out/256)) //check
	{
		return 1;
	} else { 
		ptrToArray[inputSize]=out%256; //append Lo
		ptrToArray[inputSize+1]=out/256; //append Hi
		return 0;	
	}
}

/* @brief: copies a single or multiple words from one array of bytes to another array of bytes
*          amount must not be bigger than 255...
*
*/
void listRegisterCopy(volatile uint8_t *source, volatile uint8_t *target, uint8_t amount) {
	uint8_t c=0;
    for ( c=0; c<amount; c++)
	{
		*(target+c)=*(source+c);
	}
}

/* @brief: copies a single bit from one char to another char (or arrays thereof)
*
*
*/
void listBitCopy(volatile uint8_t *source, uint16_t sourceNr,volatile uint8_t *target, uint16_t targetNr) {
	if(*(source+(sourceNr/8))&(1<<(sourceNr-((sourceNr/8)*8))))
	{
		*(target+(targetNr/8))|=(1<<(targetNr-((targetNr/8)*8)));
	} else *(target+(targetNr/8))&=~(1<<(targetNr-((targetNr/8)*8)));
}

void modbusReset(void) {
	BusState=(1<<TimerActive); //stop receiving (error)
	modbusTimer=0;
    DataPos=0;
    //READY = 0;
    
}

void modbusTickTimer(void) {
	if (BusState&(1<<TimerActive)) {
		modbusTimer++;
		if (BusState&(1<<Receiving)) //we are in receiving mode
		{
			if ((modbusTimer==modbusInterCharTimeout)) {
				BusState|=(1<<GapDetected);
			} else if ((modbusTimer==modbusInterFrameDelayReceiveEnd)) { //end of message
				#if ADDRESS_MODE == MULTIPLE_ADR
                if (crc16(rxbuffer,DataPos-3)) { //perform crc check only. This is for multiple/all address mode.
	                BusState=(1<<ReceiveCompleted);
                } else modbusReset();
				#endif
				#if ADDRESS_MODE == SINGLE_ADR
				if (rxbuffer[0]==Address && crc16(rxbuffer,DataPos-3)) { //is the message for us? => perform crc check
					BusState=(1<<ReceiveCompleted);
				} else modbusReset();
				#endif
				
			}	
		} else if (modbusTimer==modbusInterFrameDelayReceiveStart) BusState|=(1<<BusTimedOut);
	}
}

/*ISR(UART_RECEIVE_INTERRUPT) {
	unsigned char data;
	data = UART_DATA;
	modbusTimer=0; //reset timer
	if (!(BusState & (1<<ReceiveCompleted)) && !(BusState & (1<<TransmitRequested)) && !(BusState & (1<<Transmitting)) && (BusState & (1<<Receiving)) && !(BusState & (1<<BusTimedOut))) { // // 
		if (DataPos>MaxFrameIndex) modbusReset();
	    else {
			rxbuffer[DataPos]=data;
			DataPos++; //TODO: maybe prevent this from exceeding 255?
		}	    
    } else if (!(BusState & (1<<ReceiveCompleted)) && !(BusState & (1<<TransmitRequested)) && !(BusState & (1<<Transmitting)) && !(BusState & (1<<Receiving)) && (BusState & (1<<BusTimedOut))) { //) {
			 rxbuffer[0]=data;
			 BusState=((1<<Receiving)|(1<<TimerActive));
			 DataPos=1;
    }
}*/


/*ISR(UART_TRANSMIT_INTERRUPT) {
	BusState&=~(1<<TransmitRequested);
	BusState|=(1<<Transmitting);
	UART_DATA=rxbuffer[DataPos];
	DataPos++;
	if (DataPos==(PacketTopIndex+1)) {
		UART_CONTROL&=~(1<<UART_UDRIE);
	}
}*/

/*ISR(UART_TRANSMIT_COMPLETE_INTERRUPT) { //Sobald Stopbit übertragen wurde RS485-Transceiver wieder auf empfangen umstellen
	#if PHYSICAL_TYPE == 485
	transceiver_rxen();
	#endif
	modbusReset();
}*/

void modbusInit(void)
{
	
	#if PHYSICAL_TYPE == 485
	//TRANSCEIVER_ENABLE_PORT_DDR|=(1<<TRANSCEIVER_ENABLE_PIN);
	transceiver_rxen();
	#endif
	BusState=(1<<TimerActive);
}

/* @brief: Sends a response.
*
*         Arguments: - packtop: Position of the last byte containing data.
*                               modbusSendException is a good usage example.
*/
void modbusSendMessage(unsigned char packtop) {
	PacketTopIndex=packtop+2; //diff: +2
	crc16(rxbuffer,packtop); //diff: alt: -1, neu +1
	BusState|=(1<<TransmitRequested);
	DataPos=0;
	#if PHYSICAL_TYPE == 485
	transceiver_txen();
	#endif
	//UART_CONTROL|=(1<<UART_UDRIE);
    IEC0bits.U1TXIE = true ;
    //IFS0bits.U1TXIF = false;
    BusState&=~(1<<ReceiveCompleted);
    
//    BusState&=~(1<<TransmitRequested);
//	BusState|=(1<<Transmitting);
//    
//	
//    for(DataPos = 0; DataPos < /*8*/(PacketTopIndex+1); DataPos++)
//    {
//        ClrWdt();
//        while (U1STAbits.TRMT==0);
//        DRV_UART1_WriteByte(rxbuffer[DataPos]);
//    }
//	if (DataPos==(PacketTopIndex+1)) {
//        //if (DataPos==8) {
//		//UART_CONTROL&=~(1<<UART_UDRIE);
//        //IEC0bits.U1TXIE = false;
//        while (U1STAbits.TRMT==0);
//        READY = 0;
//        modbusReset();
//	}
//	BusState&=~(1<<ReceiveCompleted);
}

void modbusSendException(unsigned char exceptionCode) {
	rxbuffer[1]|=(1<<7); //setting MSB of the function code (the exception flag)
	rxbuffer[2]=exceptionCode; //Exceptioncode. Also the last byte containing data
	modbusSendMessage(2);
}

uint16_t modbusRequestedAmount(void) {
	return (rxbuffer[5]|(rxbuffer[4]<<8));
}

uint16_t modbusRequestedAddress(void) {
	return (rxbuffer[3]|(rxbuffer[2]<<8));
}

/* @brief: copies a single or multiple bytes from one array of bytes to an array of 16-bit-words
*          
*
*/
void intToModbusRegister(volatile uint16_t *inreg, volatile uint8_t *outreg, uint8_t amount) {
	uint8_t c=0;
    for (c=0; c<amount; c++)
	{	
			*(outreg+c*2) = (uint8_t)(*(inreg+c) >> 8);
			*(outreg+1+c*2) = (uint8_t)(*(inreg+c));
	}
}

/* @brief: copies a single or multiple 16-bit-words from one array of integers to an array of bytes
*
*
*/
void modbusRegisterToInt(volatile uint8_t *inreg, volatile uint16_t *outreg, uint8_t amount) {
	uint8_t c=0;
    for (c=0; c<amount; c++)
	{
		*(outreg+c) = (*(inreg+c*2) << 8) + *(inreg+1+c*2);
	}
}

/* @brief: Handles single/multiple register reading and single/multiple register writing.
*
*         Arguments: - ptrToInArray: pointer to the user's data array containing registers
*                    - startAddress: address of the first register in the supplied array
*                    - size: input array size in the requested format (16bit-registers)
*
*/
uint8_t modbusExchangeRegisters(volatile uint16_t *ptrToInArray, uint16_t startAddress, uint16_t size) {
	uint16_t requestedAmount = modbusRequestedAmount();
	uint16_t requestedAdr = modbusRequestedAddress();
	if (rxbuffer[1]==fcPresetSingleRegister) requestedAmount=1;
	if ((requestedAdr>=startAddress) && ((startAddress+size)>=(requestedAmount+requestedAdr))) {
		
		if ((rxbuffer[1]==fcReadHoldingRegisters) || (rxbuffer[1]==fcReadInputRegisters) )
		{
			if ((requestedAmount*2)<=(MaxFrameIndex-4)) //message buffer big enough?
			{
				rxbuffer[2]=(unsigned char)(requestedAmount*2);
				intToModbusRegister(ptrToInArray+(unsigned char)(requestedAdr-startAddress),rxbuffer+3,rxbuffer[2]);
				modbusSendMessage(2+rxbuffer[2]);
				return 1;
			} else modbusSendException(ecIllegalDataValue);
		}
		else if (rxbuffer[1]==fcPresetMultipleRegisters)
		{
			if (((rxbuffer[6]*2)>=requestedAmount) && ((DataPos-9)==rxbuffer[6])) //enough data received?
			{
				modbusRegisterToInt(rxbuffer+7,ptrToInArray+(unsigned char)(requestedAdr-startAddress),(unsigned char)(requestedAmount));
				modbusSendMessage(5);
				return 1;
			} else modbusSendException(ecIllegalDataValue);//too few data bytes received
		}
		else if (rxbuffer[1]==fcPresetSingleRegister)
		{
			modbusRegisterToInt(rxbuffer+4,ptrToInArray+(unsigned char)(requestedAdr-startAddress),1);
			modbusSendMessage(5);
			return 1;
		} 
		//modbusSendException(ecSlaveDeviceFailure); //inanpropriate call of modbusExchangeRegisters
		return 0;
		} else {
		modbusSendException(ecIllegalDataValue);
		return 0;
	}
}

/* @brief: Handles single/multiple input/coil reading and single/multiple coil writing.
*
*         Arguments: - ptrToInArray: pointer to the user's data array containing bits
*                    - startAddress: address of the first bit in the supplied array
*                    - size: input array size in the requested format (bits)
*
*/
uint8_t modbusExchangeBits(volatile uint8_t *ptrToInArray, uint16_t startAddress, uint16_t size) {
	uint16_t c = 0;
    uint16_t requestedAmount = modbusRequestedAmount();
	uint16_t requestedAdr = modbusRequestedAddress();
	if (rxbuffer[1]==fcForceSingleCoil) requestedAmount=1;
	if ((requestedAdr>=startAddress) && ((startAddress+size)>=(requestedAmount+requestedAdr))) {
		if ((rxbuffer[1]==fcReadInputStatus) || (rxbuffer[1]==fcReadCoilStatus))
		{
			if (requestedAmount<=((MaxFrameIndex-4)*8)) //message buffer big enough?
			{
				rxbuffer[2]=(requestedAmount/8);
				if (requestedAmount%8>0)
				{
					rxbuffer[(uint8_t)(requestedAmount/8)+3]=0x00; //fill last data byte with zeros
					rxbuffer[2]++;
				}
				for (c = 0; c<requestedAmount; c++)
				{
					listBitCopy(ptrToInArray,requestedAdr-startAddress+c,rxbuffer+3,c);
				}

				modbusSendMessage(rxbuffer[2]+2);
				return 1;
			} else modbusSendException(ecIllegalDataValue); //too many bits requested within single request
		}
		else if (rxbuffer[1]==fcForceMultipleCoils)
		{
			if (((rxbuffer[6]*8)>=requestedAmount) && ((DataPos-9)==rxbuffer[6])) //enough data received?
			{
				for (c = 0; c<requestedAmount; c++)
				{
					listBitCopy(rxbuffer+7,c,ptrToInArray,requestedAdr-startAddress+c);
				}
				modbusSendMessage(5);
				return 1;
			} else modbusSendException(ecIllegalDataValue);//exception too few data bytes received
		}
		else if (rxbuffer[1]==fcForceSingleCoil) {
			listBitCopy(rxbuffer+4,0,ptrToInArray,requestedAdr-startAddress);
			modbusSendMessage(5); 
			return 1;
		}
		//modbusSendException(ecSlaveDeviceFailure); //inanpropriate call of modbusExchangeBits
		return 0;
	} else {
		modbusSendException(ecIllegalDataValue);
		return 0;
	}
}
