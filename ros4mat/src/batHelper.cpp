/*
 *    This file is part of ros4mat.
 *
 *    ros4mat is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of
 *    the License, or (at your option) any later version.
 *
 *    ros4mat is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ros4mat. If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sstream>
#include <sys/ioctl.h>

#define DEBUG false
#define COMMANDWAIT 10000
#define ADCWAIT 75000
#define TIMEOUT 500000

int BQ_pCon;
unsigned char BQ_sbuf[20] = {0};
unsigned char BQ_srbuf[20] = {0};
unsigned char BQ_error = 0x00;
double BQ_voltage_calibration[9] = {0}; // Vdout(0), Vref_m, Voutref_m, Vdout(2.5v), Vout(0.975 ou 1.2v), Vout(2.5V), KDact, Kact, Vout(0)


bool BQ_initialised = false;

struct BQ_State {
    bool charge;
    bool discharge;
    bool vprog;
    bool overtemp;
    bool undervoltage;
    bool overvoltage;
    bool overcurrent;
    bool shortcircuit;
};

int SendToI2C(int in_size_out=1, int in_size_in=1) {
    struct timeval Timeout;
    Timeout.tv_usec = TIMEOUT;
    Timeout.tv_sec = 0;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(BQ_pCon, &rfds);

    // Flushing buffer
	struct termios port_settings;
	tcgetattr(BQ_pCon, &port_settings);
	tcsetattr(BQ_pCon, TCSAFLUSH, &port_settings);

    // Reinitialise receive buffer
    for (unsigned int i = 0; i <= in_size_in; i++)
        BQ_srbuf[i] = 0x00;

    // Send query
	write(BQ_pCon, BQ_sbuf, in_size_out);

    // Wait for data to be propagated to I2C
	usleep(COMMANDWAIT);

    // Timeout
    int selectresult = select(BQ_pCon + 1, &rfds, NULL, NULL, &Timeout);
    if (selectresult == -1) {
        printf("ERREUR LORS DE LA LECTURE DU PORT I2C\n");
        return -1;
    } else if (!selectresult) {
        printf("Timeout lors de l'attente pour i2c.\n");
        return -2;
    }

    int BytesReady;
    ioctl(BQ_pCon, FIONREAD, &BytesReady);

    char* BufferRead = new char[BytesReady];

    // Read answer
	read(BQ_pCon, BufferRead, BytesReady);
	usleep(COMMANDWAIT);

    memcpy(BQ_srbuf, BufferRead, (in_size_in > BytesReady) ? BytesReady : in_size_in);
    delete[] BufferRead;

	if (DEBUG) {
        printf("Received: %u, reading: %u: ", (in_size_in > BytesReady) ? BytesReady : in_size_in, in_size_in);
        for (unsigned int i = 0; i < ((in_size_in > BytesReady) ? BytesReady : in_size_in); i++) {
    		printf("%x ", BQ_srbuf[i]);
        }
        printf("\n");
    }
    // By default, first byte will be returned. The whole data will be in BQ_srbuf array.
	return BQ_srbuf[0];
}

int SetBQModeHost(bool in_mode) {
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x02; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = (in_mode) ? 0xC3 : 0xC0;
	if (SendToI2C(5) != 1)
		return -4;
	return 0;
}

int AttachI2C(void) {
    // Verify that we don't already have the socket open
    if (BQ_initialised == true)
        return 0;
    // Get an handle to the Devantech USB-ISS device
	BQ_pCon = open("/dev/serial/by-id/usb-Devantech_Ltd._USB-ISS._00001333-if00", O_RDWR | O_NOCTTY | O_NDELAY);
	if(BQ_pCon == -1)
		return -1;
	fcntl(BQ_pCon, F_SETFL, 0);

	// Configure Serial link
	struct termios port_settings;
	tcgetattr(BQ_pCon, &port_settings);

    // Setting Baud rates
	cfsetispeed(&port_settings, B19200);
	cfsetospeed(&port_settings, B19200);

    // No Party, 2 stop bits and 8bit data
	port_settings.c_cflag &= ~PARENB;
  	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= (CSTOPB | CS8);
	port_settings.c_oflag &= ~OPOST;

    // Set raw input (no software / hardware flow control or buffering
	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
    // Apply configuration
	tcsetattr(BQ_pCon, TCSANOW, &port_settings);

	// Verify that it's the good device
	BQ_sbuf[0] = 0x5A; BQ_sbuf[1] = 0x01;
    SendToI2C(2, 2);
	
	if (BQ_srbuf[0] != 0x07 || BQ_srbuf[1] != 0x02) {
        if (DEBUG)
            printf("Wrong device! Received: %x %x\n", BQ_srbuf[0], BQ_srbuf[1]);
		return -2;
    }

	// Envoyer la configuration du Devantech en I2C + I/O (0x5A 0x02 0x60 0x08)
	BQ_sbuf[0] = 0x5A; BQ_sbuf[1] = 0x02; BQ_sbuf[2] = 0x30; BQ_sbuf[3] = 0x08;
    SendToI2C(4, 2);

	if (BQ_srbuf[0] != 0xFF || BQ_srbuf[1] != 0x00)
		return -3;

	// Configuration du BQ77PL900 pour activer les FETs de charge et decharge avant de setter le mode host.
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x06;
	if (SendToI2C(5) != 1)
		return -4;

	// Configuration du BQ77PL900 en mode standalone avec un gain en tension de 0.2 et 50 en courant
	if (SetBQModeHost(false) != 0)
		return -5;
    
    if (DEBUG)
    	printf("Configuration OK.\n");
    BQ_initialised = true;
	return 0;
}

int DetachI2C(void){
    if (BQ_initialised == false)
        return -1;
	SetBQModeHost(false);
	close(BQ_pCon);
    BQ_initialised = false;
	return 0;
}

BQ_State GetState() {
	// Check status
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x21; BQ_sbuf[2] = 0x00; BQ_sbuf[3] = 0x01;
	SendToI2C(4);
    if (DEBUG)
    	printf("BQ State: 0x%X\n", BQ_srbuf[0]);
    // Reinterpret entry to struct...
    BQ_State lRet;
    // Hack incroyable pour mis-enligner le struct sur des pointeurs de bool
    unsigned long lInt = (unsigned long)(&lRet);
    bool *lBoolPtr = (bool*)lInt;
    for (unsigned int i=0; i < sizeof(bool) * 8; i++) {
        (*(lBoolPtr + sizeof(bool) * ( i ))) = (BQ_srbuf[0] & (0x01 << ( 7 - i ))) > 0;
    }
    return lRet;
}

int GetADCValue(unsigned char in_adc_config = 0x08) {
    // Default: 16 bits single-shot mode
    // Verifier si l'utilisateur a specifie d'utiliser l'ADC de facon continue
    bool lADC_Continue = false;
    if ((in_adc_config & 0x10) != 0)
        lADC_Continue = true;
	
    if (lADC_Continue == false) {
        BQ_sbuf[0] = 0x53; BQ_sbuf[1] = 0xD0; BQ_sbuf[2] = (in_adc_config & 0x7F);
        if (SendToI2C(3) != 1)
            return -1;
    }

	BQ_sbuf[0] = 0x53; BQ_sbuf[1] = 0xD0; BQ_sbuf[2] = (in_adc_config | 0x80);
	if (SendToI2C(3) != 1)
		return -1;
	
    usleep(ADCWAIT);
    if (lADC_Continue == false) {
        BQ_sbuf[0] = 0x54; BQ_sbuf[1] = 0xD1; BQ_sbuf[2] = 0x04;
        SendToI2C(3, 4);
    }

    return 0;
}

float GetCurrent() {
	SetBQModeHost(true);
	
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x03; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x06;
	if (SendToI2C(5) != 1)
		return -1;

    GetADCValue(0x28);

	float voffset = ((float)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits

	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x03; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x02;
	if (SendToI2C(5) != 1)
		return -1;

    GetADCValue(0x28);

	float vout = ((float)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits
    //  Vcurr = 1.2 + (Ipack x Rsense) x IGAIN      BQ77PL900 datasheet p.33
    vout = (vout - voffset) / ( 50.0 * 0.01 );
	
	SetBQModeHost(false);
	
    return vout;
}

int CalibrateVoltage() {
    if (DEBUG)
        printf("Calibration de la batterie...\n");
		
	SetBQModeHost(true);
	
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x03; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x01;
	if (SendToI2C(5) != 1)
		return -1;

	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x10;
	if (SendToI2C(5) != 1)
		return -1;

    GetADCValue(0x08);
    BQ_voltage_calibration[0] = ((double)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits
    
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x30;
	if (SendToI2C(5) != 1)
		return -1;
    
    GetADCValue(0x08);
    BQ_voltage_calibration[1] = ((double)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits

	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x20;
	if (SendToI2C(5) != 1)
		return -1;
    
    GetADCValue(0x08);
    BQ_voltage_calibration[2] = ((double)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits
    
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x40;
	if (SendToI2C(5) != 1)
		return -1;
    
    GetADCValue(0x08);
    BQ_voltage_calibration[3] = ((double)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits

	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x50;
	if (SendToI2C(5) != 1)
		return -1;
    
    GetADCValue(0x08);
    BQ_voltage_calibration[4] = ((double)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits

	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x60;
	if (SendToI2C(5) != 1)
		return -1;
    
    GetADCValue(0x08);
    BQ_voltage_calibration[5] = ((double)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits

    // Kdact
    BQ_voltage_calibration[6] = (BQ_voltage_calibration[0] - BQ_voltage_calibration[2]) / BQ_voltage_calibration[1];

    // Kact
    BQ_voltage_calibration[7] = -(BQ_voltage_calibration[5] - BQ_voltage_calibration[4]) / (((BQ_voltage_calibration[0] - BQ_voltage_calibration[3]) / BQ_voltage_calibration[6]) - BQ_voltage_calibration[1]);

    // Vout(0v)
    BQ_voltage_calibration[8] = BQ_voltage_calibration[5] + BQ_voltage_calibration[7] * ((BQ_voltage_calibration[0] - BQ_voltage_calibration[3]) / BQ_voltage_calibration[6]);

	SetBQModeHost(false);
    if (DEBUG)
        printf("Calibration constants: %f - %f\n", BQ_voltage_calibration[8], BQ_voltage_calibration[7]);
    return 0;
}

float GetCellVoltage(unsigned short in_cell){
    if (BQ_voltage_calibration[0] == 0) {
        CalibrateVoltage();
    }
	SetBQModeHost(true);
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x05; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = in_cell;
	if (SendToI2C(5) != 1)
		return -1;
	
	BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x03; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = 0x01;
	if (SendToI2C(5) != 1)
		return -1;
	
    GetADCValue(0x08);
	//float vout = ((float)(((BQ_srbuf[0] & 0x03) << 16) + (BQ_srbuf[1] << 8) + BQ_srbuf[2]) * 0.000015625); // 18 bits
	//printf("Cell %d: %X %X %X - %f (config: %X)\n", in_cell, BQ_srbuf[0], BQ_srbuf[1], BQ_srbuf[2], vout, BQ_srbuf[3]);
	float vout = ((float)((BQ_srbuf[0] << 8) + BQ_srbuf[1]) * 0.0000625); // 16 bits
	float voutreal = (BQ_voltage_calibration[8] - vout) / BQ_voltage_calibration[7];
    if(DEBUG)
        printf("Cell %d: %X %X - %f = %f (config: %X)\n", in_cell, BQ_srbuf[0], BQ_srbuf[1], vout, voutreal, BQ_srbuf[2]);
	
	SetBQModeHost(false);
	return voutreal;
}

int CheckAllCells() {
	if (!BQ_initialised) {
		return -1;
	}
	SetBQModeHost(true);
	
	// Config ADC
	BQ_sbuf[0] = 0x53; BQ_sbuf[1] = 0xD0; BQ_sbuf[2] = 0x08;
	if (SendToI2C(3) != 1)
		return -1;
	GetCellVoltage(0);
	GetCellVoltage(1);
	GetCellVoltage(2);
	GetCellVoltage(3);
	GetCellVoltage(4);
	GetCellVoltage(5);
	
	SetBQModeHost(false);
}

int ResetProtectionState() {
	SetBQModeHost(false);
	SetBQModeHost(true);
	return 0;
	// La procedure suivante ne fonctionne pas
	// To reset the BQ77PL900, il faut ecrire 0, puis 1 puis 0 dans le registre LTCLR, soit bit 0 du registre 0x01 pour setter CHG / DSG et ensuite getter l'etat. Voir datasheet p.40
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x21; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01;
	unsigned int original_reg = SendToI2C(4);
        return -1;
    // Reset LTCLR
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = original_reg & 0xFE;
	if (SendToI2C(5) != 1)
        return -1;
	BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = original_reg & 0xFE | 0x01;
	if (SendToI2C(5) != 1)
        return -1;
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = original_reg & 0xFE;
	if (SendToI2C(5) != 1)
        return -1;
    // Set CHG DSG
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = original_reg & 0xF8 | 0x06;
	if (SendToI2C(5) != 1)
        return -1;
    // Get state
    GetState();
    return BQ_srbuf[0];
}

int SetChargeState(bool in_charge) {
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x21; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01;
	unsigned int original_reg = SendToI2C(4);
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = original_reg & 0xFB | (in_charge << 2);
	if (SendToI2C(5) != 1)
        return -1;
    return 0;
}

int SetDischargeState(bool in_discharge) {
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x21; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01;
	unsigned int original_reg = SendToI2C(4);
    BQ_sbuf[0] = 0x55; BQ_sbuf[1] = 0x20; BQ_sbuf[2] = 0x01; BQ_sbuf[3] = 0x01; BQ_sbuf[4] = original_reg & 0xFD | (in_discharge << 1);
	if (SendToI2C(5) != 1)
        return -1;
    return 0;
}

int ProgramEEPROM() {
    return 0;
}
