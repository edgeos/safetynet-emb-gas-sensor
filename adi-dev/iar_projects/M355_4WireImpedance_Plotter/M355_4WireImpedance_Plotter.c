/**
 *****************************************************************************
   @addtogroup
   @{
   @file     M355_4WireImpedance.c
   @brief    4 wire Impedance measurement
   @par Revision History:
   @version  V0.1
   @author   ADI
   @date     May 10th 2018
   @par Revision History:
   - V0.1, May 10th 2018: initial version.

   Decription:
     -- UART baud rate 57600, 8 data bit, 1 stop bit
      4 wire and 2 wire Impedance measurement, determined by macro _4_WIRE 
      200 Ohm RCAL, Rx is 200 Ohm resistor combined with 1nF cap in parallel
      Below is DPNT switch matrix connection for the test
             4-Wire                             2-Wire
          ___________ D-AIN3               ___________ D&P-AIN1                                                               
         |                                |                                                   
        _|_______  P-AIN2                _|                               
       |  |                             |  |                                
       |Rx|                             |Rx|                                      
       |__|______ N-AIN1                |__|                                       
         |                                |                     
         |__________ T-AIN0               |__________ N&T-AIN2      

     Make sure C13, JP21 and JP22 are removed if you are using S1 version Evaluation board.
     Because capcitor to GND connected in the loop can influence measurement result.


All files for ADuCM355 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.
**/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "M355_4WireImpedance_Plotter.h"
#include "FreqArray.h"  //specified frequency information
#include "Uart_Plotter_Control.h"


#define MAX_UART_TRY 3

void ClockInit(void);
void UartInit(void);

volatile unsigned char ucCOMSTA0 = 0;          // Variable used to store COMSTA0, UART status register
volatile unsigned char ucCOMIID0 = 0;          // Variable used to store COMIID0, UART Interrupt status register
volatile unsigned char ucComRx = 0;            // Variable used to read UART Rx buffer contents into
unsigned char ucTxBufferEmpty  = 0;	       // Used to indicate that the UART Tx buffer is empty
unsigned char szTemp[256] = {0};	       // Used to build string before printing to UART
unsigned char szInSring[256] = {0};	       // Used to acquire incoming string from the UART
volatile unsigned char szbuffer[16];           // Debug buffer to test Rx FIFO
unsigned char ucButtonPress = 0;               // used to detect IRQ0 interrupt
unsigned int uiPressNum = 0;                   // Count number of PB0 button presses
unsigned char ucPacketReceived = 0;            // Flag to indicate UART byte received
unsigned char ucInCnt = 0;                     // Used to count incoming bytes over the UART
unsigned char ucNumRxIrqs = 0;                 // test variable to count number of Rx interrupts
unsigned char ucNumToIrqs = 0;                 // test variable to count number of Time-out interrupts
int iNumBytesInFifo = 0;                       // Used to determine the number of bytes in the UART FIFO
        
volatile uint32_t dftRdy = 0;
volatile uint32_t ucButtonPressed = 0;
volatile uint32_t ucUartDataRcvd = 0;
volatile uint32_t ackRcvd = 0;
volatile uint32_t measureOn = 0;

static uint32_t numFreqInds = 20;
static uart_packet rx_packet = {0};
static uint8_t uart_tx_buffer[256] = {0};
static uint8_t uart_tx_length = 0;
static bool send_ack = false;

ImpResult_t ImpResult[100] = {0};

/*
      1 - use 4 wire impedance measurement
      0 - use 2 wire impedance measurement
*/
#define _4_WIRE 0

void main(void)
{
   AfeWdtGo(false);
   ClockInit();
   UartInit();
   
   /*S2 button configuration*/
   DioCfgPin(pADI_GPIO1,PIN0,0); //confige as gpio
   DioIenPin(pADI_GPIO1,PIN0,1); //enable input
   DioPulPin(pADI_GPIO1,PIN0,1);  //enable pull-up
   DioIntPolPin(pADI_GPIO1,PIN0,1);           // Set polarity of P1.0 interrupt to low-high transition
   DioIntPin(pADI_GPIO1,PIN0,INTA,1);         // Enable External interrupt A on P1.0
   NVIC_EnableIRQ(SYS_GPIO_INTA_IRQn);         // Enable GPIO_INTA interrupt source in NVIC

   ImpResult[0].freq = 10000;   
   for(uint32_t i=1; i<50; i++)
   {
      ImpResult[i].freq = ImpResult[i-1].freq + 3500;
   } 
   
   measureOn = 0;
   while(1)
   {
      if (send_ack)
      {
         // found a valid packet, construct and send an ACK before we act on the packet
         build_ack_packet(&uart_tx_buffer[0], &uart_tx_length, &rx_packet);
         TxUartBuffer(uart_tx_length, false);
         send_ack = false;
      }
      else if (measureOn)
      {
         StartMeasurement();
         SendMeasurement();
      }
   }
}

void UartRxParser()
{
   uint8_t mid_ind = sizeof(szInSring)/2;
   if (look_for_packet(&szInSring[0],(uint8_t)ucInCnt,&rx_packet))
   {
      // move rx buffer pointer back to front
      ucInCnt = 0;
    
      // do different actions depending on packet type
      switch (rx_packet.cmd)
      {
         case CMD_PING:
            send_ack = true;
            break;
         case CMD_START_MEASURE:
            ConfigImpMeasurement();
            measureOn = 1;
            send_ack = true;
            break;
         case CMD_STOP_MEASURE:
            measureOn = 0;
            send_ack = true;
            break;
         case CMD_ACK:
            ackRcvd = 1;
            break;
         case CMD_SEND_DATA: // shouldn't receive here
         default:
            break;
      }
  }
  else if ((uint8_t)ucInCnt == sizeof(szInSring)-1-PAYLOAD_LENGTH)
  {
     // shift buffer data to prevent overflow, should not happen
     memcpy(&szInSring[0], &szInSring[mid_ind], ucInCnt-mid_ind); 
     ucInCnt = ucInCnt-mid_ind;
  }
}

void SendMeasurement()
{
   uint8_t i = 0;
   uint8_t num_try;
   for (i = 0; i < numFreqInds; i++)
   {
      build_data_packet(&uart_tx_buffer[0], &uart_tx_length, &ImpResult[i], i+1, numFreqInds);
      
      // try to send maximum X times.
      //num_try = 0;
      //while(!TxUartBuffer(uart_tx_length, true) & num_try < MAX_UART_TRY) { num_try++; }
      
      // don't wait for ACK, but put a Tx delay between packets
      uint16_t timeout = 0;
      TxUartBuffer(uart_tx_length, true);
      do
      {
         delay_10us(1);
         timeout++;
      } while (timeout < 20);
   }
}

void ConfigImpMeasurement()
{
  measure_payload *settings = (measure_payload *)&rx_packet.payload;
  numFreqInds = (uint32_t)((settings->freq_stop - settings->freq_start)/settings->freq_step + 1);
  for(uint32_t i=0; i<numFreqInds; i++)
  {
     ImpResult[i].freq = settings->freq_start + i*settings->freq_step;
  }   
}

void StartMeasurement()
{
   ImpInit();
   
   ImpMeasurement();
   ImpCalculation();
   
   /*power off high power exitation loop if required*/
   AfeAdcIntCfg(NOINT); //disable all ADC interrupts
   NVIC_DisableIRQ(AFE_ADC_IRQn);
   pADI_AFE->AFECON &= ~(BITM_AFE_AFECON_ADCEN|BITM_AFE_AFECON_WAVEGENEN| \
    BITM_AFE_AFECON_TIAEN|BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_SINC2EN|  \
      BITM_AFE_AFECON_EXBUFEN|BITM_AFE_AFECON_DACEN|BITM_AFE_AFECON_DACREFEN);
   
   //restore clock
   ClkDivCfg(1,1);                       // digital die to 26MHz 
   AfeHFOsc32M(0);                       //AFE oscillator change to 16MHz
   AfeSysClkDiv(AFE_SYSCLKDIV_1);        //AFE system clock   
   
   /*print Impedance result*/
   /*printf("Impedance Result:\r\n");
   printf("Frequency,    RCAL_REAL,   RCAL_IMG,   RX_REAL,   RX_IMG,   MAG,   PHASE\r\n");
   for(uint32_t i=0;i<sizeof(ImpResult)/sizeof(ImpResult_t);i++)
   {
     printf("%.4f, %8d, %8d, %8d, %8d, %.4f, %.4f \r\n",ImpResult[i].freq, ImpResult[i].DFT_result[0],  \
       ImpResult[i].DFT_result[1],  \
         ImpResult[i].DFT_result[2],  \
           ImpResult[i].DFT_result[3],  \
             ImpResult[i].Mag,            \
               ImpResult[i].Phase);
   }*/
}

/**
   @brief void ImpInit()
          Initialization for Impedance measurement. setup HPTIA/HPDAC/ADC/DFT block
   @return 1.
*/
void ImpInit()
{
   uint32_t ctia;
   /*DFT interrupt enable*/
   AfeAdcIntCfg(BITM_AFE_ADCINTIEN_DFTRDYIEN);
   NVIC_EnableIRQ(AFE_ADC_IRQn);
   /******setup exitation loop and TIA********/
   AfeHpTiaCon(HPTIABIAS_1V1); /*Normal power mode, 1.1V biased HP TIA*/
   ctia = BITM_HPTIA_CTIA_2PF;//BITM_HPTIA_CTIA_2PF;
   AfeHpTiaSeCfg(HPTIASE_RTIA_10K,ctia,0);   /*rtia,ctia,no diosel*/ //HPTIASE_RTIA_10K
   /*switch to RCAL, loop exitation before power up*/
   AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T9);
   /*********Initialize ADC and DFT********/
   AfeAdcPgaCfg(GNPGA_1,0);
   AfeAdcChan(MUXSELP_HPTIA_P,MUXSELN_HPTIA_N);
   pADI_AFE->AFECON |= BITM_AFE_AFECON_DACEN|BITM_AFE_AFECON_DACREFEN;
}

/**
   @brief uint8_t ImpSigChainCfg(uint32_t freq)
         ======== configuration of AC signal chain depends on required excitation frequency.
   @param freq :{}
            - excitation AC signal frequency
   @return 1.
   @note settings including DAC update rate, ADC update rate and DFT samples can be adjusted for
   different excitation frequencies to get better performance. As general guidelines,
       - DAC update rate: make sure at least 4 points per sinewave period. Higher rate comsumes more power.
       - ADC update rate:  at least follow Nyquist sampling rule.
       - DFT samples should cover more than 1 sine wave period. more DFT sample reduce variation but take longer time.
          the configuration can be optimised depending on user's applicationn
*/
uint8_t ImpSigChainCfg(float freq)
{
   uint32_t WgFreqReg;
   uint16_t ampReg;

   ampReg = (uint16_t)(300.0/HPDAC_LSB+0.5);  //300mV amplitude sinewave from DAC
   WgFreqReg = (uint32_t)(freq*67.108864+0.5);  //ATE version 0x14
   if(freq<450)   /*frequency lower than 450 Hz*/
   {
      ClkDivCfg(1,1);                       // digital die to 26MHz 
      AfeHFOsc32M(0);                       //AFE oscillator change to 16MHz
      AfeSysClkDiv(AFE_SYSCLKDIV_1);        //AFE system clock 
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);         //set High speed DAC and ADC in Low Power mode, filter cut-off at 50kHz
		AfeHpTiaCon(HPTIABIAS_1V1); 
      /*set low DAC update rate,16MHz/255=~62.7KHz, output amplitude = 600mV,*/
      AfeHPDacCfg(HPDAC_ATTEN_DIV5,255,HPDAC_INAMPGAIN_DIV4);
      AfeAdcFiltCfg(SINC3OSR_4,SINC2OSR_178,LFPBYPEN_BYP,ADCSAMPLERATE_800K); //1.1KHz
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,DFTNUM_8192,DFTIN_SINC2);
      AfeAdcChopEn(1);   //Enable ADC input buffer chop for LP mode (up to 80kHz)
   }
   else if(freq<80000)  /*450Hz < frequency < 80KHz*/
   {
      ClkDivCfg(1,1); 
      AfeHFOsc32M(0);
      AfeSysClkDiv(AFE_SYSCLKDIV_1);
      AfeSysCfg(ENUM_AFE_PMBW_LP,ENUM_AFE_PMBW_BW250);
		AfeHpTiaCon(HPTIABIAS_1V1); 
      /*set middle DAC update rate,16MHz/18=~889KHz update rate*/
      AfeHPDacCfg(HPDAC_ATTEN_DIV5,18,HPDAC_INAMPGAIN_DIV4);
      AfeAdcChopEn(1);   //Enable ADC input buffer chop for LP mode (up to 80kHz)
      AfeAdcFiltCfg(SINC3OSR_4,SINC2OSR_178,LFPBYPEN_BYP,ADCSAMPLERATE_800K); //bypass LPF, 200KHz ADC update rate
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,DFTNUM_16384,DFTIN_SINC3); //DFT source: Sinc3 result
   }
   else if(freq<=200000) /*80KHz < frequency < 200KHz*/
   {
      /*****boost ADC sample rate to 1.6MHz****/
      ClkDivCfg(2,2);   //reduce digtal clock to reduce AFE clock
      AfeSysClkDiv(AFE_SYSCLKDIV_2);   //afe system clock reduce to 8MHz
      AfeHFOsc32M(BITM_AFE_HPOSCCON_CLK32MHZEN);   //AFE oscillator change to 32MHz, afe system clock = 16Mhz, ADC clock=32MHz
      ClkDivCfg(1,1);   //back to 26MHz
      AfeAdcChopEn(0);  //Disable ADC input buffer chop for HP mode (>80kHz)
      AfeSysCfg(ENUM_AFE_PMBW_HP,ENUM_AFE_PMBW_BW250);  //set High speed DAC and ADC in high power mode
		AfeHpTiaCon(HPTIABIAS_1V1); 
      /*set High DAC update rate,16MHz/7=~2.2MHz update rate,output amplitude = 600mV*/
      AfeHPDacCfg(HPDAC_ATTEN_DIV5,7,HPDAC_INAMPGAIN_DIV4);
      AfeAdcFiltCfg(SINC3OSR_4,SINC2OSR_178,LFPBYPEN_BYP,ADCSAMPLERATE_1600K); //bypass LPF, 400KHz ADC update rate
      AfeAdcDFTCfg(BITM_AFE_DFTCON_HANNINGEN,DFTNUM_16384,DFTIN_SINC3); //DFT source: Sinc3 result
   }
   else  /*frequency higher than 200KHz,exceeded specification*/
   {
      return 0;
   }
   AfeHPDacSineCfg(WgFreqReg,0,0,ampReg);  //set new frequency and amplitude
   AfeHPDacWgType(HPDAC_WGTYPE_SINE);
   return 1;
}

/**
   @brief void ImpMeasurement()
          Do Impedance measurement
   @return None.
*/
void ImpMeasurement()
{
   uint32_t testNum;
   testNum = numFreqInds;//sizeof(ImpResult)/sizeof(ImpResult_t);
   for(uint32_t i=0;i<testNum;i++)
   {
      ImpSigChainCfg(ImpResult[i].freq);
      /************RCAL AC measurement***************/
      /*switch to RCAL, loop exitation before power up*/
      pADI_AFE->AFECON &= ~(BITM_AFE_AFECON_WAVEGENEN|BITM_AFE_AFECON_EXBUFEN|   \
                           BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN); //disable loop before switching
      AfeSwitchDPNT(SWID_DR0_RCAL0,SWID_PR0_RCAL0,SWID_NR1_RCAL1,SWID_TR1_RCAL1|SWID_T9);
      pADI_AFE->AFECON |= BITM_AFE_AFECON_ADCEN|BITM_AFE_AFECON_SINC2EN|BITM_AFE_AFECON_WAVEGENEN| \
                           BITM_AFE_AFECON_EXBUFEN|BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN;
      delay_10us(30);   //300us for power up and switch settling 
      /*start ADC conversion and DFT*/
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
         //PwrCfg(ENUM_PMG_PWRMOD_FLEXI,BITM_PMG_PWRMOD_MONVBATN,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;

      ImpResult[i].DFT_result[0] = convertDftToInt(AfeAdcRd(DFT_REAL));
      ImpResult[i].DFT_result[1] = convertDftToInt(AfeAdcRd(DFT_IMAG));

      /********* Rx measurement*************/
      /*switch to Rx*/
      pADI_AFE->AFECON &= ~(BITM_AFE_AFECON_WAVEGENEN|BITM_AFE_AFECON_EXBUFEN|   \
                           BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN); //disable loop before switching
#if _4_WIRE
      AfeSwitchDPNT(SWID_D4_AIN3,SWID_P3_AIN2,SWID_N2_AIN1,SWID_T1_AIN0|SWID_T9);
#else
      AfeSwitchDPNT(SWID_D2_AIN1,SWID_P2_AIN1,SWID_N3_AIN2,SWID_T3_AIN2|SWID_T9);
#endif
      pADI_AFE->AFECON |= (BITM_AFE_AFECON_WAVEGENEN|BITM_AFE_AFECON_EXBUFEN|   \
                           BITM_AFE_AFECON_INAMPEN|BITM_AFE_AFECON_TIAEN);
      delay_10us(30);   //300us for switch settling
      /*start ADC conversion and DFT*/
      pADI_AFE->AFECON |= BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN;
      while(!dftRdy)
      {
         //PwrCfg(ENUM_PMG_PWRMOD_FLEXI,BITM_PMG_PWRMOD_MONVBATN,BITM_PMG_SRAMRET_BNK2EN);
      }
      dftRdy = 0;
      ImpResult[i].DFT_result[2] = convertDftToInt(AfeAdcRd(DFT_REAL));
      ImpResult[i].DFT_result[3] = convertDftToInt(AfeAdcRd(DFT_IMAG));
      AfeWaveGenGo(false);
   }
}

/**
   @brief void ImpCalculation()
          calculate magnitude and phase of sensor
   @note application scope of RPhase :(-180, 180]
*/
void ImpCalculation()
{

   float Src[4];
   float Mag[2];
   float Phase[2];

   uint32_t testNum;
   testNum = sizeof(ImpResult)/sizeof(ImpResult_t);
   for(uint32_t i=0;i<testNum;i++)
   {
      for (uint8_t ix=0;ix<4;ix++)
      {
         Src[ix] = (float)(ImpResult[i].DFT_result[ix]);
      }
      for (uint8_t ix=0;ix<2;ix++)
      {
         Mag[ix] = Src[ix*2]*Src[ix*2]+Src[ix*2+1]*Src[ix*2+1];
         Phase[ix] = atan2(Src[ix*2+1], Src[ix*2]);
         Mag[ix] = sqrt(Mag[ix]);
      }
      ImpResult[i].Mag = Mag[0]*AFE_RCAL/Mag[1];
      ImpResult[i].Phase = -(Phase[0]- Phase[1]);
      ImpResult[i].Phase = ImpResult[i].Phase*180/PI;
      /*shift back to range (-180,180]*/
      if(ImpResult[i].Phase > 180)
      {
         do
         {
            ImpResult[i].Phase -= 360;
         }
         while(ImpResult[i].Phase > 180);
      }
      else if(ImpResult[i].Phase < -180)
      {
         do
         {
            ImpResult[i].Phase += 360;
         }
         while(ImpResult[i].Phase < -180);
      }
   }
}


//rewrite putchar to support printf in IAR
int putchar(int c)
{
   UrtTx(pADI_UART0,c);
   while(!(pADI_UART0->COMLSR&BITM_UART_COMLSR_TEMT));
   return c;
}

uint8_t TxUartBuffer(unsigned char length, bool waitForAck)
{
   uint16_t timeout = 0;
   unsigned char i = 0;
   for ( i = 0 ; i < length ; i++ )	          // loop to send String to UART
   {
      ucTxBufferEmpty = 0;	                  // Clear flag
      UrtTx(pADI_UART0,uart_tx_buffer[i]);        // Load UART Tx register.
      while (ucTxBufferEmpty == 0)                // Wait for UART Tx interrupt
      {
      }
   }
   if(waitForAck)
   {
      ackRcvd = 0;
      while (ackRcvd == 0 & timeout < 1000) 
      {
         delay_10us(1);
         timeout++;
      }
      return (ackRcvd == 1) ? 1 : 0;
   }
   return 1;
}


void ClockInit(void)
{
   DigClkSel(DIGCLK_SOURCE_HFOSC);
   ClkDivCfg(1,1);
   AfeClkSel(AFECLK_SOURCE_HFOSC);
   AfeSysClkDiv(AFE_SYSCLKDIV_1);
}

void UartInit(void)
{
   DioCfgPin(pADI_GPIO0,PIN10,1);               // Setup P0.10 as UART pin
   DioCfgPin(pADI_GPIO0,PIN11,1);               // Setup P0.11 as UART pin
   pADI_UART0->COMLCR2 = 0x3;                  // Set PCLk oversampling rate 32. (PCLK to UART baudrate generator is /32)
   UrtCfg(pADI_UART0,B115200,
          (BITM_UART_COMLCR_WLS|3),0);         // Configure UART for 115200 baud rate
   UrtFifoCfg(pADI_UART0, RX_FIFO_14BYTE,      // Configure the UART FIFOs for 14 bytes deep
              BITM_UART_COMFCR_FIFOEN);
   UrtFifoClr(pADI_UART0, BITM_UART_COMFCR_RFCLR// Clear the Rx/TX FIFOs
              |BITM_UART_COMFCR_TFCLR);
   UrtIntCfg(pADI_UART0,BITM_UART_COMIEN_ERBFI |
             BITM_UART_COMIEN_ETBEI |
                BITM_UART_COMIEN_ELSI);                  // Enable Rx, Tx and Rx buffer full Interrupts

   NVIC_EnableIRQ(UART_EVT_IRQn);              // Enable UART interrupt source in NVIC
}

void UART_Int_Handler()
{
   int i = 0;

   ucCOMSTA0 = UrtLinSta(pADI_UART0);
   ucCOMIID0 = UrtIntSta(pADI_UART0);
   if ((ucCOMIID0 & 0xE) == 0x2)	          // Transmit buffer empty
   {
      ucTxBufferEmpty = 1;
   }
   if ((ucCOMIID0 & 0xE) == 0x4)	          // Receive byte
   {
      ucNumRxIrqs++;
      iNumBytesInFifo = pADI_UART0->COMRFC;    // read the Num of bytes in FIFO
      for (i=0; i<iNumBytesInFifo;i++)
      {
         ucComRx = UrtRx(pADI_UART0);
         szInSring[ucInCnt++]= ucComRx;
      }
      ucUartDataRcvd = 1;
   }
   if ((ucCOMIID0 & 0xE) == 0xC)	          // UART Time-out condition
   {
      ucNumToIrqs++;
      iNumBytesInFifo = pADI_UART0->COMRFC;    // read the Num of bytes in FIFO
      for (i=0; i<iNumBytesInFifo;i++)
      {
         ucComRx = UrtRx(pADI_UART0);
         szInSring[ucInCnt++]= ucComRx;
      }
      ucUartDataRcvd = 1;
   }
   // handle received UART bytes
   if (ucUartDataRcvd)
   {
      ucUartDataRcvd = 0;
      UartRxParser();
   }
}

void AfeAdc_Int_Handler()
{
	uint32_t sta;
	sta = pADI_AFE->ADCINTSTA;
	if(sta&BITM_AFE_ADCINTSTA_DFTRDY)
	{
      pADI_AFE->ADCINTSTA = BITM_AFE_ADCINTSTA_DFTRDY;	//clear interrupt
      dftRdy = 1;
      pADI_AFE->AFECON &= (~(BITM_AFE_AFECON_DFTEN|BITM_AFE_AFECON_ADCCONVEN));  //stop conversion
	}
}

void GPIO_A_Int_Handler()
{
   unsigned int uiIntSta = 0;

   uiIntSta = DioIntSta(pADI_GPIO1);
   if ((uiIntSta & 0x0001) ==0x0001)
   {
      DioIntClrPin(pADI_GPIO1,PIN0);
      ucButtonPressed = 1;
   }
}




/**@}*/
