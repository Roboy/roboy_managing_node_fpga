#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"
#include "interface.hpp"
#include <limits.h>

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//#define ADC_MEASUREMENT

int main(int argc, char *argv[]) {

	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction = 0;
	int led_mask = 0x01;
	void *h2p_lw_led_addr, *h2p_lw_spi_addr, *h2p_lw_adc_addr;
	vector<int32_t*> h2p_lw_myo_addr;
	vector<int32_t*> h2p_lw_i2c_addr;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_LED_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_adc_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
	h2p_lw_myo_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + MYOCONTROL_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
	h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_0_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));
	h2p_lw_i2c_addr.push_back((int32_t*)(virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_1_BASE ) & ( unsigned long)( HW_REGS_MASK )) ));

	Interface interface(h2p_lw_myo_addr);
	interface.myoControl->adc_base = (uint32_t*)h2p_lw_adc_addr;
	interface.timeout_ms = 10;
//
////	vector<float> x(5), y(5);
////	x[0] = 0;
////	x[1] = 1;
////	x[2] = 2;
////	x[3] = 3;
////	x[4] = 4;
////	y[0] = 0;
////	y[1] = 1;
////	y[2] = 2;
////	y[3] = 3;
////	y[4] = 4;
////	vector<float> coeffs;
////	interface.myoControl->polynomialRegression(1,x,y,coeffs);
//
	char cmd;
	  noecho();
	  do {
	    timeout(interface.timeout_ms);
	    cmd = mvgetch(5, 0);
	    switch (cmd) {
	    case '0':
	    	interface.positionControl();
	      break;
	    case '1':
	    	interface.velocityControl();
	      break;
	    case '2':
	    	interface.displacementControl();
	      break;
	    case '3':
	    	interface.switchMotor();
	      break;
	    case '4':
	    	interface.zeroWeight();
	      break;
	    case '5':
	    	interface.setAllTo();
	      break;
	    case '6':
			interface.estimateSpringParameters();
		  break;
	    case '7':
			interface.toggleSPI();
		  break;
	    case '8':
	    	interface.reset();
		  break;
	    case ' ':
			interface.recordTrajectories();
		  break;
	    case '\n':
			interface.playTrajectories();
		  break;
	    case '/':
			interface.setGains();
		  break;
	    }
	    interface.querySensoryData();
	    *(uint32_t *)h2p_lw_led_addr = ~led_mask;
		// update led mask
		if (led_direction == 0){
			led_mask <<= 1;
			if (led_mask == (0x01 << (PIO_LED_DATA_WIDTH-1)))
				 led_direction = 1;
		}else{
			led_mask >>= 1;
			if (led_mask == 0x01){
				led_direction = 0;
				loop_count++;
			}
		}
	  } while (cmd != '9');

//	int iter = 0;
//	time_t rawtime;
//	time ( &rawtime );
//	struct tm * timeinfo = localtime ( &rawtime );
//	char filename[200];
//	sprintf(filename,"force_measurement_adc_%s.txt" ,asctime (timeinfo));
//	ofstream outfile;
//	outfile.open (filename);
//
//	int counter = 0;
//
//	while( true ) {
//		myoControl.update();
//
//		// Toggling the LEDs to show off
//		if((iter++)%100==0){
//			// control led
//			*(uint32_t *)h2p_lw_led_addr = ~led_mask;
//
//			// update led mask
//			if (led_direction == 0){
//				led_mask <<= 1;
//				if (led_mask == (0x01 << (PIO_LED_DATA_WIDTH-1)))
//					 led_direction = 1;
//			}else{
//				led_mask >>= 1;
//				if (led_mask == 0x01){
//					led_direction = 0;
//					loop_count++;
//				}
//			}
//			*adc = 0;
//			uint32_t adc_data = *adc;
//			printf("adc:       %d\n", adc_data);
//
//			if(counter++ > 10){
//				char k;
//				cin >> k;
//				if(k=='.'){
//					cout << "what was the weight? [kg]" << endl;
//					float weight;
//					cin >> weight;
//					outfile << adc_data << ", " << weight << endl;
//				}else if(k=='/'){
//					cout << "setPoint?" << endl;
//					float pos;
//					cin >> pos;
//					myoControl.setPosition(0,pos);
//				}else if(k=='s'){
//					cout << "saving and quitting" << endl;
//					outfile.close();
//					break;
//				}else{
//					cout << "skipping" << endl;
//					counter = 0;
//				}
//			}
//			usleep(1000*100);
//		}
//
//	}

	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
