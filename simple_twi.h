//
//
//  Oldi2c
//
//  Created by ZeroWolf on 011/03/2016.
//




#define F_CPU 4000000UL
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#ifndef simple_twi_h
#define simple_twi_h

#define TWI_BUS_PORT    PORTC
#define TWI_BUS_DDR     DDRC
#define SDA             PC4
#define SCL             PC5

#define MAX_RESTARTS    20
#define I2C_FREQ        200000UL

#define TW_START					0x08
#define TW_REP_START				0x10
// Master Transmitter
#define TW_MT_SLA_ACK				0x18
#define TW_MT_SLA_NACK				0x20
#define TW_MT_DATA_ACK				0x28
#define TW_MT_DATA_NACK				0x30
#define TW_MT_ARB_LOST				0x38
// Master Receiver
#define TW_MR_ARB_LOST				0x38
#define TW_MR_SLA_ACK				0x40
#define TW_MR_SLA_NACK				0x48
#define TW_MR_DATA_ACK				0x50
#define TW_MR_DATA_NACK				0x58
#define TW_BUS_ERROR				0x00
#define TWSR_STATUS_MASK            0xF8
#define TW_STATUS                   TWSR & TWSR_STATUS_MASK
//read or write modes
#define TW_READ                     1
#define TW_WRITE                    0


void twi_INIT(void){
    
    TWI_BUS_DDR  &= ~(1 << SDA) & ~(1 << SCL);   //make sda and scl inputs
    TWI_BUS_PORT &= ~(1 << SDA) & ~(1 << SCL);   //disable internal pullups

    TWSR = 0;                                    //clear the prescaler bits
    
#if F_CPU < 1600000UL                            //if F_CPU is less than 0.16MHz
    TWBR = 0;                                    //whats the point of scalling down the TW Clock?
#else 
    TWBR = ((F_CPU / I2C_FREQ) - 16)/2;          //should be > 10 to work properly
#endif
    TWCR = _BV(TWEN);                            //set TWEn to enable twi


}/*end of INIT*/



unsigned char twi_START(void){
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);  //SEND A start condition.this takes a few clock cycles
    
    //since it is slow a while loop is used to check when TWINT is high
    while ((TWCR & _BV(TWINT)) == 0) ;
    //now proceeds
    
    switch (TW_STATUS) {
        
        case TW_START:
        case TW_REP_START:
            return 0;                       //start condition was accepted, no error
        default:
            return 1;                       //A bus collision occured, error, two masters comm at same time
    }
    
}/*end of START*/

void twi_STOP(void){
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);     //send stop condition on bus
    while (TWCR & _BV(TWSTO));                      //wait for stop to be applied
}/*end of stop*/





unsigned char twi_writeChar(unsigned char data){
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    
    while ((TWCR & _BV(TWINT)) == 0) ;          //any non 0 return indicates an error
    
    switch (TW_STATUS) {
        case TW_MT_SLA_ACK:     //if any of these statuses are returned, succesful write
        case TW_MT_DATA_ACK:
        case TW_MR_SLA_ACK:
        case TW_MR_DATA_ACK:
            return 0;
        
        case TW_BUS_ERROR:      //if any of these statuses are returned, failure to write
        case TW_MT_SLA_NACK:
        case TW_MT_DATA_NACK:
        case TW_MR_SLA_NACK:
        case TW_MR_DATA_NACK:
        case TW_MT_ARB_LOST:
            return 1;
            
        default:
            return 2;           //complete failure
            

    }
    
}/*end of writeChar*/



unsigned char twi_readChar_ACK(void){
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN);  //setting TWEA sends an ack
    
    while ((TWCR & _BV(TWINT)) == 0);
    
    return TWDR;
}/* end of readChar_ack */




unsigned char twi_readChar_NACK(void){
    TWCR = _BV(TWINT) | _BV(TWEN);  //not setting TWEA sends an nack
    
    while ((TWCR & _BV(TWINT)) == 0);
    
    return TWDR;
}/* end of readChar_nack */



/*----------*----------combining all the funtions, create two advanced ones----------*----------*/


unsigned char twi_read (unsigned char slvadrs,   // device address to read from
                        unsigned char numberofbytes,       // number of bytes to read
                        unsigned char adrs_to_read_Within_device,      // device register to start reading from
                        unsigned char *buf){     // RAM address to put read data in
    
    unsigned char restarts = MAX_RESTARTS;
    
    // dont do anything if theres no data to be read
    if (numberofbytes == 0) {
        return 0;
    }
    
 ReadRRestart:   //loop back here until start condition accpeted
    if(twi_START()){
        if (--restarts == 0) {
            return 1;
        }
        goto ReadRRestart;
   }

    /*
     Write the control Byte to the bus. If the slave is busy
     a NACK will be returned.loog here issuing start and control device until
     slave device accepts the byte(ACK returned). Count the number of retries
     and if a maximum is exceeded, exit so processer won't hang(it's been a problem)
    */
    

    if (twi_writeChar(slvadrs << 1|TW_WRITE)) {             //send the slave address
        if (--restarts == 0) {
            twi_STOP();
            return 2;               //return error code 2
        }
        goto ReadRRestart;
    }
    
    //write the device address to read from
    if (twi_writeChar(adrs_to_read_Within_device)) {
        return 3;
    }
    
    
    //issue a repeated start to the slave with read
    if (twi_START()) {//repeated start
        return 4;
    }
    if (twi_writeChar(slvadrs<<1|TW_READ)) {
        return 5;
    }
    
    /*recieve the bytes from the slave and put them in a buffer*/
    for (; numberofbytes > 1; numberofbytes--) {
        *(buf++) = twi_readChar_ACK();//send back ACK for all bytes but the last one
    }
    
    *(buf++) = twi_readChar_NACK(); //send nack upon recieving last byte
    //issue a STOP condition on the bus
    twi_STOP();
    return 0;





}/*end of twi_read*/

/*
 STEPS to do for write
 was very lazy so wrote the steps last night and finished 
 it in the morning.
 
 1.Issue a start condition on the bus
 2.put the slave address with bit 0 as 0(indication a write)
 3.Put the byte address that u want to write to on the bus
 4.put a data byte onto the bus
 5.repeat step 4 for all bytes
 6.issue a stop condition
*/


unsigned char twi_write(unsigned char slvadrs,                      // dslave address of the device to write to
                        unsigned char numberofbytes,                // number of bytes to write
                        unsigned char adrs_to_write_Within_device,  // device register to start writing to
                        unsigned char *buf){                        // RAM address to get data to put in

    unsigned char restarts = MAX_RESTARTS;
                              
    // dont do anything if theres no data to be read
          if (numberofbytes == 0) {
             return 0;
            }
                              
          ReadRRestart:   //loop back here until start condition accpeted
               
        if(twi_START()){
            if (restarts-- == 0) {
                    return 1;
            }
            goto ReadRRestart;
        }

    /*
     Write the control Byte to the bus. If the slave is busy
     a NACK will be returned.loog here issuing start and control device until
     slave device accepts the byte(ACK returned). Count the number of retries
     and if a maximum is exceeded, exit so processer won't hang(it's been a problem)
    */
    
    
    if (twi_writeChar(slvadrs << 1|TW_WRITE)) {             //send the slave address
        if (--restarts == 0) {
            twi_STOP();
            return 2;               //return error code 2
        }
        goto ReadRRestart;
    }
    
    //write the device address to write to
    if (twi_writeChar(adrs_to_write_Within_device)) {
        return 3;
    }
    
    //write data from the buffer to the slave address
    for (; numberofbytes > 0; numberofbytes--) {
        if (twi_writeChar(*buf++)) {
            return 4;
        }
    }
    
    //send stop condition
    twi_STOP();
    return 0;
    
}/*end of twi_write*/


#endif /* simple_twi_h */
