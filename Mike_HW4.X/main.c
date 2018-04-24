#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h>

// DEVCFG0
#pragma config DEBUG = 0b00 // no debugging
#pragma config JTAGEN = 0b0 // no jtag
#pragma config ICESEL = 0b11 // use PGED1 and PGEC1
#pragma config PWP = 0b111111111 // no write protect
#pragma config BWP = 0b1 // no boot write protect
#pragma config CP = 0b1 // no code protect

// DEVCFG1
#pragma config FNOSC = 0b011 // use primary oscillator with pll
#pragma config FSOSCEN = 0b0 // turn off secondary oscillator
#pragma config IESO = 0b0 // no switching clocks
#pragma config POSCMOD = 0b10 // high speed crystal mode
#pragma config OSCIOFNC = 0b1 // disable secondary osc
#pragma config FPBDIV = 0b00 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = 0b10 // do not enable clock switch
#pragma config WDTPS = 0b00000 // use slowest wdt
#pragma config WINDIS = 0b1 // wdt no window mode
#pragma config FWDTEN = 0b0 // wdt disabled
#pragma config FWDTWINSZ = 0b11 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = 0b001 //(2x: 8->4) divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = 0b111 // (24x: 4->96) multiply clock after FPLLIDIV
#pragma config FPLLODIV = 0b001 // (2: 96->48) divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = 0b001 // (2: 8->4, 4*12->48) divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = 0b0 // USB clock on

// DEVCFG3
#pragma config USERID = 0b1010101010101010 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = 0b0 // allow multiple reconfigurations
#pragma config IOL1WAY = 0b0 // allow multiple reconfigurations
#pragma config FUSBIDIO = 0b1 // USB pins controlled by USB module
#pragma config FVBUSONIO = 0b1 // USB BUSON controlled by USB module

#define CS LATBbits.LATB3 // Use RB3 (Pin 7) as the chip select pin

// send a byte over SPI1 and return the response
unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;
    LATAbits.LATA4 = 0;
    while (!SPI1STATbits.SPIRBF) {
        ;  // Do nothing while waiting to receive the response byte
    }
    LATAbits.LATA4 = 1;
    return SPI1BUF;
}



// initialize SPI1 and the ram module
void spi_init() {
    // "set up the chip select pin as an output
    //  the chip select pin is used by the sram to indicate
    //  when a command is beginning (clear CS to low) and when it
    //  is ending (set CS high)"
    TRISBbits.TRISB3 = 0;
    CS = 1;
    
    // Master - SPI1, pins are: SDI1(B8), SDO1(B2), SCK1(B15)
    // we manually control SS1 as a digital output (B3)
    // since the pic is just starting, we know that the spi is off.
    // We rely on defaults here
    
    // Setup SPI1
    SPI1CON = 0;                // turn off the spi module and reset it
    SPI1BUF;                    // clear the rx buffer by reading from it
    SPI1BRG = 1;              // baud rate to 20 MHz [SPI1BRG= (80mil/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;    // clear the overflow bit
    SPI1CONbits.CKE = 1;        // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;      // master operation
    SPI1CONbits.ON = 1;         // turn on SPI1
}

void setVoltage(char a, int v) {
    unsigned short t = 0; // 16 bit message
    t = a << 15;
    t = t | 0b0111000000000000;
    t = t | ((v&0b1111111111) <<2); // making sure v is a 10 bit number and shit by two because we have a 10 bit DAC
    
    CS = 0; // Chip select enabled
    spi_io(t>>8); // first 8 bit transfer
    spi_io(t&0xFF); // second 8 bit transfer
    CS = 1; // Chip select disabled
}

int main(){
    
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    
    // Pin function select
    RPB2Rbits.RPB2R = 0b0011; // B2(6) = SDO1 -> SDI(5)
    SDI1Rbits.SDI1R = 0b0100; // B8(17) = SDI1 <- X
    // !! B3 I/O pin is used as chip select (SS1, but we're doing it manually)
    
    spi_init(); // Initialize SPI1

    __builtin_enable_interrupts();
    int i = 0, j = 0, k = 0;
    
//    setVoltage(0,1023);
//    setVoltage(1,512);
  
    while(1) {
    _CP0_SET_COUNT(0);
//    setVoltage(0,1023);
//    setVoltage(1,512);
    
    // sin wave output at VoutA
    float f = 512 + 512*sin(i*0.0628); // (+1 to -1), possible int math bug!
    i++;
    setVoltage(0,(int)f);
    
    // sawtooth output at VoutB
    if (j >= 100) {
        k = 0; // descent
    }else if (j == 0) {
        k = 1; // ascent
    }
    if (k) {
        j++;
    } else {
        j--;
    }
    float s = 1023*((float)j/100);
    setVoltage(1,(int)s);
  
   
    
    while(_CP0_GET_COUNT() < 24000000/1000){;} // update at 1000Hz
   } 
}