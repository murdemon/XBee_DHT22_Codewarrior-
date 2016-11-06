/***** XBEE APPLICATION PROJECT *****
 * 
 * Auto-generated header with information about the 
 * relation between the XBee module pins and the 
 * project components.
 * 
 ************ XBEE LAYOUT ***********
 * 
 * This layout represents the XBee-PRO 900HP DigiMesh (S3B) module 
 * selected for the project with its pin distribution:
 *               _________________
 *              /     ________    \ 
 *             /     |   __   |    \ 
 *            /      | //  \\ |     \ 
 *   XPIN1  -|       | \\__// |      |- XPIN20
 *   XPIN2  -|       |________|      |- XPIN19
 *   XPIN3  -|                       |- XPIN18
 *   XPIN4  -| ===================== |- XPIN17
 *   XPIN5  -| #   # ####  #### #### |- XPIN16
 *   XPIN6  -|  # #  #   # #    #    |- XPIN15
 *   XPIN7  -|   #   ####  ###  ###  |- XPIN14
 *   XPIN8  -|  # #  #   # #    #    |- XPIN13
 *   XPIN9  -| #   # ####  #### #### |- XPIN12
 *   XPIN10 -| ===================== |- XPIN11
 *           |_______________________|
 * 
 ************ PINS LEGEND ***********
 * 
 * The following list displays all the XBee Module pins 
 * with the project component which is using each one:
 * 
 *   XPIN1 = VCC
 *   XPIN2 = uart0 [TX Pin]
 *   XPIN3 = uart0 [RX Pin]
 *   XPIN4 = <<UNUSED>>
 *   XPIN5 = special0 [Reset Pin]
 *   XPIN6 = special0 [RSSI PWM Pin]
 *   XPIN7 = <<UNUSED>>
 *   XPIN8 = special0 [BKGD Pin]
 *   XPIN9 = <<UNUSED>>
 *   XPIN10 = GND
 *   XPIN11 = <<UNUSED>>
 *   XPIN12 = <<UNUSED>>
 *   XPIN13 = power_management0 [On Sleep Pin]
 *   XPIN14 = VCC REF
 *   XPIN15 = special0 [Association Pin]
 *   XPIN16 = <<UNUSED>>
 *   XPIN17 = <<UNUSED>>
 *   XPIN18 = gpio0 [GPIO Pin]
 *   XPIN19 = adc1 [ADC Pin]
 *   XPIN20 = adc0 [ADC Pin]
 *
 ************************************/


#include <xbee_config.h>
#include <types.h>

#include <stdio.h>

#include <xbee/discovery.h>
#include <xbee/transparent_serial.h>
#include <xbee/byteorder.h>
#include <xbee/atcmd.h>

#define MONITOR_NI 	"MASTER"

/* Circular buffer to collect samples taken while radio is slept
 * Calculation done based on radio sleep time and sample rate 
 */
/*#define QUEUE_SIZE (UINT32_C(XBEE_ATCMD_PARAM_SP * 10) / 	\
				   (RTC_CFG_PERIODIC_TASK_PERIOD * 4) + 1)*/

#define QUEUE_SIZE 1

#define HIGH 1
#define LOW 0 
#define bitSet(var,bitno) ((var) |=1UL<<(bitno))

typedef struct  {
	float SoilTemp;
	float SoilMoist;
	float AirTemp;
	float AirHumid;
} Telegram;

Telegram send_queue[QUEUE_SIZE];

/* Circular buffer head and tail pointers */
uint8_t send_queue_head = 0;
uint8_t send_queue_tail = 0;

/* Timer signaled a new sample has to be taken */
bool_t app_request_send = FALSE;

/* Pending samples to send */
uint8_t send_queue_cnt = 0;

wpan_envelope_t env;

/* Buffer used to transmit samples over the air
 * dimensioned for a 16bit sample plus the terminator
 */
char txbuf[sizeof(send_queue[0]) + 1];

/* Signal when radio is awaked */
bool_t radio_ready = TRUE;

/* Signal when monitor discover is in progress */
bool_t nd_in_progress = FALSE;

/* Monitor ieeeaddr resolved by monitor_discovery() */
addr64 monitor_ieeeaddr;




int readDHT22(int *AirTemp, int *AirHumid)
{
	
	uint8_t bits[5];
	uint8_t i;
	uint8_t j;
	int mon[50];
	int old_pin;
	
	i = 0;
	j = 0;
	old_pin = 0;
	
	
	memset(bits, 0, sizeof(bits));

	gpio_config(gpio0,GPIO_CFG_OUTPUT | GPIO_CFG_PULL_UP_EN |  GPIO_CFG_SLEW_RATE_DIS | GPIO_CFG_DRV_STR_HIGH);
	gpio_set(gpio0, LOW); 
	
	delay_ticks(1);	
	
	gpio_set(gpio0, HIGH); 
	gpio_config(gpio0,GPIO_CFG_INPUT  | GPIO_CFG_PULL_UP_EN |  GPIO_CFG_SLEW_RATE_DIS | GPIO_CFG_DRV_STR_HIGH ); 
	

	
	for (i = 0; i < 5; i++)	{mon[i]=gpio_get(gpio0);} //40 microsec

		//check start condition 1
		if(gpio_get(gpio0)==HIGH) {
			printf("DHT22 check start condition 1 fail\n"); return -1;
		}
		
	for (i = 0; i < 10; i++)	{mon[i]=gpio_get(gpio0);} //80 microsec
		
		//check start condition 2
		if(gpio_get(gpio0)==LOW) {
			printf("DHT22 check start condition 2 fail\n");return -2;
		}
		 
		while (gpio_get(gpio0)) {};
		
		
	        //Sensor init ok, now read 5 bytes ...
	        for (j=0; j<5; j++)
	        {
	        	for(i=0; i< 8; i++)
	        	  	  {
	        		   while (!gpio_get(gpio0)) {};
	        		         	
	        		   while (gpio_get(gpio0)) {old_pin ++;};
	        		   if (old_pin > 5 ) {bitSet(bits[j], 7-i);}
	        		   old_pin = 0;
	        		  }
	        }
	    
	        
		//reset port
	    gpio_config(gpio0,GPIO_CFG_OUTPUT | GPIO_CFG_PULL_UP_EN |  GPIO_CFG_SLEW_RATE_DIS | GPIO_CFG_DRV_STR_HIGH);
	    gpio_set(gpio0, HIGH);    
		
	    printf("DHT22 done %x %x %x %x %x\n",bits[0],bits[1],bits[2],bits[3],bits[4] ); //return 0;
	    
		if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) == bits[4]) {
			//return temperature and humidity
			printf("DHT22 done\n");
	    			  *AirHumid 	= bits[0]<<8 | bits[1];
	                 *AirTemp 		= bits[2]<<8 | bits[3];
	      
	       return 0;
		}
		printf("DHT22 checksum fail\n");
		return -5;
	
}

/* Takes a sample and put it in the send_queue circular
 * buffer so it's delivered when radio is ready.
 * The sample taken here is a fake sequential number that
 * can be easily followed in the monitor. Feel free to change 
 * it to sample an ADC or any other peripheral.
 */
void get_sample(void)
{
	static uint16_t fake_sample = 0; 
	uint16_t adc0_value = adc_ch_read(adc0);
	uint16_t adc1_value = adc_ch_read(adc1);
	
	float SoilMoist;
	float SoilTemp;
	float Volt_tmp;
	float AirTemp;
	float AirHumid;
	int AirTemp_i;
	int AirHumid_i;
	int res;
	
	AirTemp 	= 0;
	AirHumid 	= 0;
	
	//res = readDHT22(&AirTemp, &AirHumid);
			AirTemp_i 	= 0;
			AirHumid_i 	= 0;
			
			delay_ticks(500);
			res = readDHT22(&AirTemp_i, &AirHumid_i);
			AirTemp = AirTemp_i;
			AirHumid = AirHumid_i;
			AirTemp = AirTemp/10;
			AirHumid = AirHumid/10;
			printf("%.1f %.1f \n",AirTemp,AirHumid);
			
	SoilTemp = 3.3 / 4095 * adc0_value *75.006 - 40;
	Volt_tmp = 3.3 / 4095 * adc1_value;
	
		 if (Volt_tmp > 0 && Volt_tmp <= 1.1)		{  SoilMoist = 10*Volt_tmp-1; 		}
		 if (Volt_tmp > 1.1 && Volt_tmp <= 1.3)		{  SoilMoist = 25*Volt_tmp-17.5; 	}
		 if (Volt_tmp > 1.3 && Volt_tmp <= 1.82)	{  SoilMoist = 48.08*Volt_tmp-47.5; }
		 if (Volt_tmp > 1.82)						{  SoilMoist = 26.32*Volt_tmp-7.89; }
		 
	printf("Soil Temp RAW %x, Soil Temp Real %.1f\n",adc0_value,SoilTemp);	 
	printf("Soil Moist RAW %x, Soil Moist Real %.1f\n",adc1_value,SoilMoist);
	send_queue[send_queue_head].SoilTemp = SoilTemp;
	send_queue[send_queue_head].SoilMoist = SoilMoist;
	send_queue[send_queue_head].AirTemp = AirTemp/10;
	send_queue[send_queue_head].AirHumid = AirHumid/10;
	send_queue_head++;
	if (send_queue_head == QUEUE_SIZE)
		send_queue_head = 0;
	
	if (send_queue_head == send_queue_tail)
		printf("WARNING!!! Circular buffer overflow\n");
	
	send_queue_cnt++;
}

/* Send a Node Discovery request for the MONITOR */
void monitor_discovery(void)
{
	xbee_disc_discover_nodes(&xdev, MONITOR_NI);
}

/* Keeps MONITOR information and configure send envelop accordingly */
void set_monitor(const xbee_node_id_t *node_id)
{
	char addr_buffer[ADDR64_STRING_LENGTH];

	memcpy(&monitor_ieeeaddr, &node_id->ieee_addr_be, sizeof(monitor_ieeeaddr));
	
	printf("MASTER is %s\n", addr64_format(addr_buffer, &monitor_ieeeaddr));
	
	wpan_envelope_create(&env, &xdev.wpan_dev, &monitor_ieeeaddr,
											 WPAN_NET_ADDR_UNDEFINED);
	env.payload = txbuf;
	env.length = sizeof(send_queue[0]);
	
	/* Simulate Commissioning Button press so a Node Identification broadcast 
	 * transmission is queued to be sent at the beginning of the next network 
	 * wake cycle. This way the MONITOR will know our NI.
	 */
	(void)xbee_cmd_simple(&xdev, "CB", 1);
}

#ifdef ENABLE_XBEE_HANDLE_ND_RESPONSE_FRAMES
void node_discovery_callback(xbee_dev_t *xbee, const xbee_node_id_t *node_id)
{
	/* This function is called every time a node is discovered, either by
	 * receiving a NodeID message or because a node search was started with
	 * function xbee_disc_discover_nodes() */

	if (node_id == NULL) {
		printf("Node Discovery timed out\n");
		nd_in_progress = FALSE;

		return;
	}
	
	if (strncmp(node_id->node_info, MONITOR_NI, strlen(MONITOR_NI)) == 0) {
		set_monitor(node_id);
		nd_in_progress = FALSE;
	}

	return;
}
#endif

#ifdef ENABLE_XBEE_HANDLE_TX_STATUS_FRAMES
int xbee_transmit_status_handler(xbee_dev_t *xbee, const void FAR *payload,
                                 uint16_t length, void FAR *context)
{
	const xbee_frame_transmit_status_t FAR *frame = payload;

	/* it may be necessary to push information up to user code so they know when
	 * a packet has been received or if it didn't make it out
	 */
	printf("TransmitStatus: id 0x%02x retries = %d del = 0x%02x disc = 0x%02x\n",
	      frame->frame_id,
	      frame->retries, frame->delivery,
	      frame->discovery);

	return 0;
}
#endif

#if defined(RTC_ENABLE_PERIODIC_TASK)
void rtc_periodic_task(void)
{
    /* Function call every RTC_CFG_PERIODIC_TASK_PERIOD * 8 ms.
     * This function is called from the timer ISR, please be brief
     * and exit, or just set a flag and do your home work in the 
     * main loop
     */
	app_request_send = TRUE;
}
#endif

#ifdef ENABLE_XBEE_HANDLE_MODEM_STATUS_FRAMES
int xbee_modem_status_handler(xbee_dev_t *xbee, const void FAR *payload,
                              uint16_t length, void FAR *context)
{
	const xbee_frame_modem_status_t FAR *frame = payload;
	
	/* First modem status reporting WOKE UP indicates that radio
	 * has just synchronized; a good event to do monitor discovery
	 */ 
	if (!nd_in_progress && addr64_is_zero(&monitor_ieeeaddr) 
				&& frame->status == XBEE_MODEM_STATUS_WOKE_UP) {
		monitor_discovery();
		nd_in_progress = TRUE;
		xbee_frame_dump_modem_status(xbee, payload, length, context);
		printf("MONITOR not yet discovered. Sending DiscoverNode\n");
	}
	
	return 0;
}
#endif


#ifdef ENABLE_XBEE_HANDLE_RX
int xbee_transparent_rx(const wpan_envelope_t FAR *envelope, void FAR *context)
{
    /* Add your code here... */
	
	return 0;
}
#endif



void main(void)
{	
	
	
	
	
	sys_hw_init();
	sys_xbee_init();

	sys_app_banner();
	
				
#ifdef XBEE_ATCMD_PARAM_NI
	printf("Starting using NI=%s\n", XBEE_ATCMD_PARAM_NI);
#else
	printf("NI parameter not configured. You should configure it.\n");
#endif
	
	memset(txbuf, 0, sizeof(txbuf));
		
		
		
	for (;;) {
		/* Queue packets when radio is slept */
		if (app_request_send) {
			app_request_send = FALSE;
			if (addr64_is_zero(&monitor_ieeeaddr)) 
				printf("MONITOR not yet discovered. Sample lost\n");
			else if (!radio_ready) {
				printf("Sample taken. Radio slept: queuing deliver\n");
				delay_ticks(HZ / 2);
				get_sample();
			} else {
				printf("Sample taken.\n");
				get_sample();
			}
		}
				
		/* Send packets once radio is ready */
		if (radio_ready && send_queue_cnt) {
			send_queue_cnt--;
			
			printf("sending sample: 0x%04x\n", send_queue[send_queue_tail]);
			*(Telegram *)txbuf = send_queue[send_queue_tail];
			xbee_transparent_serial(&env);
			
			send_queue_tail++;
			if (send_queue_tail == QUEUE_SIZE)
				send_queue_tail = 0;
		}
	
		/* Enter CPU in low power while radio is in low power */
		if (pm_get_radio_mode() == PM_MODE_STOP) {
			radio_ready = FALSE;
				
			printf("Going to sleep...\n");
			delay_ticks(2); /* This is for avoiding writing garbage on the UART */

			pm_set_cpu_mode(PM_MODE_STOP, WAIT_INFINITE); /* Start sleeping */
			/* When CPU wakes-up, it continues executing from here */
			
			/* Possible wake sources in this example: Radio and RTC */
									
			delay_ticks(2); /* This is for avoiding writing garbage on the UART */
			printf("CPU Awaked\n");
		}
		
		/* Signal when radio is ready */
		if (!radio_ready && (pm_get_radio_mode() == PM_MODE_RUN)) {
			radio_ready = TRUE;
			printf("Radio Awaked\n");

			
			
			/* First modem status reporting WOKE UP indicates that radio
			 * has just synchronized; a good event to do monitor discovery
			 */ 
			if (!nd_in_progress && addr64_is_zero(&monitor_ieeeaddr))
			{

				monitor_discovery();
				nd_in_progress = TRUE;
	
				printf("MONITOR not yet discovered. Sending DiscoverNode\n");
			}
		}
		
		sys_watchdog_reset();
		sys_xbee_tick();
	}
}
