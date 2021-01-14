#include "spi1.h"

void InitSPI1(void){
    /*
     SPI1STAT
        bit 15 SPIEN: SPIx Enable bit(1)
            1 = Enables module and configures SCKx, SDOx, SDIx and SSx as serial port pins
            0 = Disables module
        bit 14 Unimplemented: Read as ?0?
        bit 13 SPISIDL: Stop in Idle Mode bit
            1 = Discontinue module operation when device enters Idle mode
            0 = Continue module operation in Idle mode
        bit 12-11 Unimplemented: Read as ?0?
        bit 10-8 SPIBEC<2:0>: SPIx Buffer Element Count bits (valid in Enhanced Buffer mode)
            Master mode:
            Number of SPI transfers pending.
            Slave mode:
            Number of SPI transfers unread.
        bit 7 SRMPT: Shift Register (SPIxSR) Empty bit (valid in Enhanced Buffer mode)
            1 = SPIx Shift register is empty and ready to send or receive
            0 = SPIx Shift register is not empty
        bit 6 SPIROV: Receive Overflow Flag bit
            1 = A new byte/word is completely received and discarded
            (The user software has not read the previous data in the SPIxBUF register.)
            0 = No overflow has occurred
        bit 5 SRXMPT: Receive FIFO Empty bit (valid in Enhanced Buffer mode)
            1 = Receive FIFO is empty
            0 = Receive FIFO is not empty
        bit 4-2 SISEL<2:0>: SPIx Buffer Interrupt Mode bits (valid in Enhanced Buffer mode)
            111 = Interrupt when the SPIx transmit buffer is full (SPITBF bit is set)
            110 = Interrupt when the last bit is shifted into SPIxSR; as a result, the TX FIFO is empty
            101 = Interrupt when the last bit is shifted out of SPIxSR; now the transmit is complete
            100 = Interrupt when one data is shifted into the SPIxSR; as a result, the TX FIFO has one open spot
            011 = Interrupt when the SPIx receive buffer is full (SPIRBF bit set)
            010 = Interrupt when the SPIx receive buffer is 3/4 or more full
            001 = Interrupt when data is available in the receive buffer (SRMPT bit is set)
            000 = Interrupt when the last data in the receive buffer is read; as a result, the buffer is empty (SRXMPT
            bit set)        
        bit 1 SPITBF: SPIx Transmit Buffer Full Status bit
            1 = Transmit not yet started, SPIxTXB is full
            0 = Transmit started, SPIxTXB is empty
            In Standard Buffer mode:
                Automatically set in hardware when the CPU writes to the SPIxBUF location, loading SPIxTXB.
                Automatically cleared in hardware when the SPIx module transfers data from SPIxTXB to SPIxSR.
            In Enhanced Buffer mode:
                Automatically set in hardware when the CPU writes to the SPIxBUF location, loading the last available
                buffer location.
                Automatically cleared in hardware when a buffer location is available for a CPU write.
        bit 0 SPIRBF: SPIx Receive Buffer Full Status bit
            1 = Receive complete, SPIxRXB is full
            0 = Receive is not complete, SPIxRXB is empty
            In Standard Buffer mode:
                Automatically set in hardware when SPIx transfers data from SPIxSR to SPIxRXB.
                Automatically cleared in hardware when the core reads the SPIxBUF location, reading SPIxRXB.
            In Enhanced Buffer mode:
                Automatically set in hardware when SPIx transfers data from the SPIxSR to the buffer, filling the last
                unread buffer location.
                Automatically cleared in hardware when a buffer location is available for a transfer from SPIxSR
    */
    SPI1STATbits.SPIEN = 0; // Desabilitar o módulo SPI para configurar.
    
    /*
     SPI1STAT
    */
    SPI1BUF = 0;            // Limpar o buffer do SPI1
    
    /*
     IFS0
    */
    IFS0bits.SPI1IF = 0;	// Limpar flag de interrupção
    IFS0bits.SPF1IF = 0;    // Limpar flag de falha de interrupção
    
    /*
     IEC0
    */
    IEC0bits.SPI1IE = 0;	// Desabilitar interrupção no SPI1
    IEC0bits.SPF1IE = 0;    // Desabilitar interrupção de falha no SPI1
    
    /*
    SPI1CON1
        bit 15-13 Unimplemented: Read as ?0?
        bit 12 DISSCK: Disable SCKx Pin bit (SPI Master modes only)(1)
            1 = Internal SPI clock is disabled; pin functions as I/O
            0 = Internal SPI clock is enabled
        bit 11 DISSDO: Disable SDOx Pin bit(2)
            1 = SDOx pin is not used by the module; pin functions as I/O
            0 = SDOx pin is controlled by the module
        bit 10 MODE16: Word/Byte Communication Select bit
            1 = Communication is word-wide (16 bits)
            0 = Communication is byte-wide (8 bits)
        bit 9 SMP: SPIx Data Input Sample Phase bit
            Master mode:
                1 = Input data sampled at the end of data output time
                0 = Input data sampled at the middle of data output time
            Slave mode:
                SMP must be cleared when SPIx is used in Slave mode.
        bit 8 CKE: SPIx Clock Edge Select bit(3)
            1 = Serial output data changes on transition from active clock state to Idle clock state (see bit 6)
            0 = Serial output data changes on transition from Idle clock state to active clock state (see bit 6)
        bit 7 SSEN: Slave Select Enable (Slave mode) bit(4)
            1 = SSx pin is used for Slave mode
            0 = SSx pin is not used by the module; pin is controlled by the port function
        bit 6 CKP: Clock Polarity Select bit
            1 = Idle state for the clock is a high level; active state is a low level
            0 = Idle state for the clock is a low level; active state is a high level
        bit 5 MSTEN: Master Mode Enable bit
            1 = Master mode
            0 = Slave mode
        bit 4-2 SPRE<2:0>: Secondary Prescale bits (Master mode)
            111 = Secondary prescale 1:1
            110 = Secondary prescale 2:1
            .
            000 = Secondary prescale 8:1
        bit 1-0 PPRE<1:0>: Primary Prescale bits (Master mode)
            11 = Primary prescale 1:1
            10 = Primary prescale 4:1
            01 = Primary prescale 16:1
            00 = Primary prescale 64:1
    */    
    SPI1CON1bits.DISSCK = 0; // Habilitar clock interno do SPI
    SPI1CON1bits.DISSDO = 0; // Habilita o pino SDO e o coloca sob controle do módulo SPI1
    SPI1CON1bits.MODE16 = 0; // Define a largura da comunicação em 8 bits (1 byte)
    SPI1CON1bits.SMP    = 0; // Recepção de dados é realizada no meio do tempo de transmissão. Ou seja, pega sempre no meio de cada pulso.
    SPI1CON1bits.CKE    = 0; // Transição do clock ocorre do modo ocioso para o modo ativo.
    SPI1CON1bits.SSEN   = 0; // Pino de slave select não é controlado pelo módulo SPI, é controlado pelo próprio PORT
    SPI1CON1bits.CKP    = 0; // Estado ocioso significa nível lógico baixo (zero)
    SPI1CON1bits.MSTEN  = 1; // Habilitar o modo Master do SPI1
    SPI1CON1bits.SPRE   = 4; // Seleciona prescaler secundario em 4:1
    SPI1CON1bits.PPRE   = 3; // Selecionar prescaler primario em 1:1
    
    //SPI1CON1 = 0x0033;
    
    /*
    SPI1CON2     
        bit 15 FRMEN: Framed SPIx Support bit
            1 = Framed SPIx support is enabled
            0 = Framed SPIx support is disabled
        bit 14 SPIFSD: Frame Sync Pulse Direction Control on SSx Pin bit
            1 = Frame sync pulse input (slave)
            0 = Frame sync pulse output (master)
        bit 13 SPIFPOL: Frame Sync Pulse Polarity bit (Frame mode only)
            1 = Frame sync pulse is active-high
            0 = Frame sync pulse is active-low
        bit 12-2 Unimplemented: Read as ?0?
        bit 1 SPIFE: Frame Sync Pulse Edge Select bit
            1 = Frame sync pulse coincides with the first bit clock
            0 = Frame sync pulse precedes the first bit clock
        bit 0 SPIBEN: Enhanced Buffer Enable bit
            1 = Enhanced buffer is enabled
            0 = Enhanced buffer is disabled (Legacy mode) 
    */
    SPI1CON2bits.FRMEN   = 0;   // Desabilitar suporte ao SPI em quadros
    SPI1CON2bits.SPIFSD  = 0;   // Desabilitar pulso de sincronismo de quadros do SPI
    SPI1CON2bits.SPIFPOL = 0;   // Pulso de sincronismo fica em nível lógico baixo quando ativo
    SPI1CON2bits.SPIFE   = 0;   // Pulso de sincronismo de quadros precede o primeiro bit de clock
    SPI1CON2bits.SPIBEN  = 1;   // Habilitar modo de buffer aprimorado.
    
    //SPI1CON2 = 0x0001;	// non-framed mode
    
    /*
    SPI1STAT
        bit 15 SPIEN: SPIx Enable bit(1)
            1 = Enables module and configures SCKx, SDOx, SDIx and SSx as serial port pins
            0 = Disables module
        bit 14 Unimplemented: Read as ?0?
        bit 13 SPISIDL: Stop in Idle Mode bit
            1 = Discontinue module operation when device enters Idle mode
            0 = Continue module operation in Idle mode
        bit 12-11 Unimplemented: Read as ?0?
        bit 10-8 SPIBEC<2:0>: SPIx Buffer Element Count bits (valid in Enhanced Buffer mode)
            Master mode:
                Number of SPI transfers pending.
            Slave mode:
                Number of SPI transfers unread.
        bit 7 SRMPT: Shift Register (SPIxSR) Empty bit (valid in Enhanced Buffer mode)
            1 = SPIx Shift register is empty and ready to send or receive
            0 = SPIx Shift register is not empty
        bit 6 SPIROV: Receive Overflow Flag bit
            1 = A new byte/word is completely received and discarded
            (The user software has not read the previous data in the SPIxBUF register.)
            0 = No overflow has occurred
        bit 5 SRXMPT: Receive FIFO Empty bit (valid in Enhanced Buffer mode)
            1 = Receive FIFO is empty
            0 = Receive FIFO is not empty
        bit 4-2 SISEL<2:0>: SPIx Buffer Interrupt Mode bits (valid in Enhanced Buffer mode)
            111 = Interrupt when the SPIx transmit buffer is full (SPITBF bit is set)
            110 = Interrupt when the last bit is shifted into SPIxSR; as a result, the TX FIFO is empty
            101 = Interrupt when the last bit is shifted out of SPIxSR; now the transmit is complete
            100 = Interrupt when one data is shifted into the SPIxSR; as a result, the TX FIFO has one open spot
            011 = Interrupt when the SPIx receive buffer is full (SPIRBF bit set)
            010 = Interrupt when the SPIx receive buffer is 3/4 or more full
            001 = Interrupt when data is available in the receive buffer (SRMPT bit is set)
            000 = Interrupt when the last data in the receive buffer is read; as a result, the buffer is empty (SRXMPT
        bit 1 SPITBF: SPIx Transmit Buffer Full Status bit
            1 = Transmit not yet started, SPIxTXB is full
            0 = Transmit started, SPIxTXB is empty
        In Standard Buffer mode:
            Automatically set in hardware when the CPU writes to the SPIxBUF location, loading SPIxTXB.
            Automatically cleared in hardware when the SPIx module transfers data from SPIxTXB to SPIxSR.
            In Enhanced Buffer mode:
            Automatically set in hardware when the CPU writes to the SPIxBUF location, loading the last available
            buffer location.
            Automatically cleared in hardware when a buffer location is available for a CPU write.
        bit 0 SPIRBF: SPIx Receive Buffer Full Status bit
            1 = Receive complete, SPIxRXB is full
            0 = Receive is not complete, SPIxRXB is empty
        In Standard Buffer mode:
            Automatically set in hardware when SPIx transfers data from SPIxSR to SPIxRXB.
            Automatically cleared in hardware when the core reads the SPIxBUF location, reading SPIxRXB.
        In Enhanced Buffer mode:
            Automatically set in hardware when SPIx transfers data from the SPIxSR to the buffer, filling the last
            unread buffer location.
            Automatically cleared in hardware when a buffer location is available for a transfer from SPIxSR.

    */
    // 0b 1 0   000 1010 0000
    SPI1STATbits.SPIEN   = 1; // Habilitar módulo SPI
    SPI1STATbits.SPISIDL = 0; // Continuar funcionando mesmo em modo ocioso
    SPI1STATbits.SPIBEC  = 0; // Limpar buffer com a contagem de quantas transmissões estão pendentes
    SPI1STATbits.SRMPT   = 1; // Shift register está vazio e pronto para enviar
    SPI1STATbits.SPIROV  = 0; // Não ocorreu overflow
    SPI1STATbits.SRXMPT  = 1; // Buffer de recebimento da FIFO está vazio.
    SPI1STATbits.SISEL   = 0; // Interrupção ocorre quando o último bit for lido. Como resultado, o buffer está vazio SRXMPT = 1
    SPI1STATbits.SPITBF  = 0; // Transmissão iniciada SPIxTXB está vazio 
    SPI1STATbits.SPIRBF  = 0; // Recebimento não foi completado, SPIxRXB está vazio
    
    //SPI1STAT = 0x80A0; 	// enable SPI port, clear status    
}

int WriteReadSPI1(const unsigned char *txdata, unsigned char *rxdata, unsigned int lenght){
    
    int timeout = 0;
    int posicaoTx = 0;
    int posicaoRx = 0;
    
    if(lenght == 0) return 0;
   
    while(posicaoTx < lenght && posicaoRx < lenght){         
        if(txdata != 0){  // Se um ponteiro foi passado, entra
            SPI1BUF = txdata[posicaoTx++]; // Envia o dado e incrementa para enviar o próximo
        }
        else{
            SPI1BUF = 0;
        }    
        
        timeout = 0;
        
        while(SPI1STATbits.SRXMPT || !timeout > 10){ // Aguarda o envio ou no máximo 10 micros segundos
            __delay_us(1);
            timeout++;
        }         
        
        if(timeout > 10) break; // Se demorou mais que 10 micro segundos, retorna

        if(rxdata != 0){ // Se um ponteiro foi passado, entra
            rxdata[posicaoRx++] = SPI1BUF; // Lê o dado e incrementa para ler o próximo         
        }
        else{
            SPI1BUF;
        }             
    }
    return 0;
}
