/*
 * tiva_i2c.c
 *
 *  Created on: 31/gen/2015
 *  robocupjr 15
 *  tiva c
 *  I2C bus
 *  Author: massimo
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"



//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================

//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg){


    //specify that we are writing (a register address) to the
    //slave device
  /// intanto prepara il master a scrivere nello slave (il registro da leggere)
  HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | false;
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //specify register to be read
    /// specifica l'indirizzo da leggere
    HWREG(I2C0_BASE + I2C_O_MDR) = reg;
    //I2CMasterDataPut(I2C0_BASE, reg);

    //send control byte and register address byte to slave device
    /// invia lo START (BIT1) e il RUN (BIT0)
    /// da idle a transmit mode, page 1023 U.G.
    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_START;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //specify that we are going to read from slave device
    HWREG(I2C0_BASE + I2C_O_MSA) = (slave_addr << 1) | true;
    //I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    /*
     * #define I2C_MASTER_CMD_BURST_RECEIVE_START                                    \
                                0x0000000b
#define I2C_MASTER_CMD_BURST_RECEIVE_CONT                                     \
                                0x00000009
#define I2C_MASTER_CMD_BURST_RECEIVE_FINISH                                   \
                0x00000005
     * */
    /// Repeated START condition followed by RECEIVE and STOP condition (master goes to Idle state). pag. 1024
    /// repeated start
    HWREG(I2C0_BASE + I2C_O_MCS) = I2C_MASTER_CMD_SINGLE_RECEIVE;
    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //return data pulled from the specified register
    // return(HWREG(I2C0_BASE + I2C_O_MDR));
    return I2CMasterDataGet(I2C0_BASE);
}
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================

//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    //enable I2C module 0
  /// abilita il bit relativo ad I2C  (BIT0) nel registro RCGCI2C 0x400FE620
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    /// dapprima resetta il modula e poi lo riporta nella condizione inziale che in questo caso e'
    /// ON
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    /// imposta il funzionamento da master e la velocita' di comunicazione.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    /// qui non ho trovato il registro I2C_FIFOCTL
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void InitI2C1(void)
{
    //enable I2C module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    //enable GPIO peripheral that contains I2C 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for I2C1 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}


void InitI2C2(void)
{
    //enable I2C module 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    //enable GPIO peripheral that contains I2C 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}



//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================


//sends an I2C command to the specified slave
void I2CSend(uint8_t port, uint8_t slave_addr, uint8_t num_of_args, ...)
{
    uint32_t PortI2c;

    if (port == 0)
        PortI2c = I2C0_BASE;

    else if (port == 1)
        PortI2c = I2C1_BASE;

    else if (port == 2)
        PortI2c = I2C2_BASE;


    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(PortI2c, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(PortI2c, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
      /// p. 1023 U.G.: START condition followed by RECEIVE and STOP condition (master remains in Idle state).
        I2CMasterControl(PortI2c, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(PortI2c));

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
      uint8_t i;
        //Initiate send of data from the MCU
      /// START condition followed by TRANSMIT (master goes  to the Master Transmit state).
      HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_START;
        //I2CMasterControl(PortI2c, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(PortI2c));

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(PortI2c, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            /// TRANSIT opertion (master remains in Master Transmit state).
            HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_CONT;
            //I2CMasterControl(PortI2c, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(PortI2c));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(PortI2c, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        /// TRANSMIT followed by STOP condition (master goes to Idle state).
        HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_FINISH;
        ///I2CMasterControl(PortI2c, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(HWREG(PortI2c + I2C_O_MCS) & I2C_MCS_BUSY);
        /*while(I2CMasterBusy(PortI2c));
        if(HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSY)
            {
                return(true);
            }
            else
            {
                return(false);
            }*/
        //"close" variable args list
        va_end(vargs);
    }
}


//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================
//======================================================================================================================================

/// rivece n bytes da I2c
void  I2CReceiveN(uint8_t port, uint8_t slave_addr, uint8_t reg, uint8_t numElem, uint8_t buff[]){
  uint32_t i;
  uint32_t PortI2c;

    if (port == 0)
        PortI2c = I2C0_BASE;

    else if (port == 1)
        PortI2c = I2C1_BASE;

    else if (port == 2)
        PortI2c = I2C2_BASE;


  /// Impsta l'indirizzo dello slave e la modalità di scrittura
  HWREG(PortI2c + I2C_O_MSA) = (slave_addr << 1) | false;
  //I2CMasterSlaveAddrSet(PortI2c, slave_addr, false);

  /// specifica l'indirizzo da leggere
  HWREG(PortI2c + I2C_O_MDR) = reg;

  /// invia lo START (BIT1) e il RUN (BIT0)
  /// da idle a transmit mode, page 1023 U.G.
  HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_SEND_START;
  //I2CMasterControl(PortI2c, I2C_MASTER_CMD_BURST_SEND_START);

  //wait for MCU to finish transaction
  while(I2CMasterBusy(PortI2c));

    //specify that we are going to read from slave device
    HWREG(PortI2c + I2C_O_MSA) = (slave_addr << 1) | true;
    //I2CMasterSlaveAddrSet(PortI2c, slave_addr, true);

    /// Repeated START condition followed by RECEIVE    (master remains in Master Receive state). pag. 1024
  /// repeated start
  HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_START;
  //I2CMasterControl(PortI2c, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //wait for MCU to finish transaction
  while(I2CMasterBusy(PortI2c));

  /// il primo byte e' pronto
  //buff[0] = (HWREG(PortI2c + I2C_O_MDR));
  buff[0] = I2CMasterDataGet(PortI2c);

  for (i = 1; i < numElem - 1; i++){
    /// ReceivE operation (master remains in Master Receive state).
    HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
    //I2CMasterControl(PortI2c, I2C_MASTER_CMD_SINGLE_RECEIVE);I2C_MASTER_CMD_BURST_RECEIVE_CONT
    //wait for MCU to finish transaction
    while(I2CMasterBusy(PortI2c));
    //// legge il dato
    buff[i] = I2CMasterDataGet(PortI2c);
  }

  /// ultimo elemento
  ///RECEIVE followed by STOP condition (master goes to Idle state).
  HWREG(PortI2c + I2C_O_MCS) = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
  //I2CMasterControl(PortI2c, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  //wait for MCU to finish transaction
  while(I2CMasterBusy(PortI2c));
  buff[numElem - 1] = I2CMasterDataGet(PortI2c);


}
