#include "app_util_platform.h"
//#include "nrf_gpio.h"
#include "max30102.h"
#include "nrf_drv_twi.h"
#include "boards.h"
#include "nrf_delay.h"


//����I2C����
#define TWI_SCL_M      25    //SCL����
#define TWI_SDA_M      27     //SDA����



//TWI��������ʵ��ID
#define TWI_INSTANCE_ID     1
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

//TWI��������ʵ����IDΪ1��ӦTWI1
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWI�¼�������
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
				
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
					  nrf_gpio_pin_clear(LED_2);
					  break;
				case NRF_DRV_TWI_EVT_DATA_NACK:
					  nrf_gpio_pin_clear(LED_1);		
						
        default:
            break;
    }
}

void twi_master_init(void)
{
    ret_code_t err_code;
    //���岢��ʼ��TWI���ýṹ��
    const nrf_drv_twi_config_t twi_max30102_config = {
       .scl                = TWI_SCL_M,  //����TWI SCL����
       .sda                = TWI_SDA_M,  //����TWI SDA����
       .frequency          = NRF_DRV_TWI_FREQ_100K, //TWI����
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI���ȼ�
       .clear_bus_init     = false//��ʼ���ڼ䲻����9��SCLʱ��
    };
    //��ʼ��TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_max30102_config, twi_handler, NULL);
	  //��鷵�صĴ������
    APP_ERROR_CHECK(err_code);
    //ʹ��TWI
    nrf_drv_twi_enable(&m_twi);
}

//дMAX30102�Ĵ���
void max30102_write_reg(uint8_t reg,uint8_t val)
{
	uint8_t tx_packet[2];
	
	tx_packet[0] = reg;
	tx_packet[1] = val;
	
	m_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, tx_packet, sizeof(tx_packet),false);
  APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false);
}
//��MAX30102�Ĵ���
uint8_t max30102_read_reg(uint8_t reg,uint8_t len)
{
	  ret_code_t err_code;
	  uint8_t tx_packet[1];
	  uint8_t rx_packet;
	  tx_packet[0] = reg;
	
	  m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, tx_packet, sizeof(tx_packet),false);
    APP_ERROR_CHECK(err_code);
	  while (m_xfer_done == false){};

		m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDRESS, &rx_packet, 1);
    APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false){};
			
		return rx_packet;
}
//��MAX30102 FIFO
uint8_t max30102_read_fifo(uint32_t *p_red_led, uint32_t *p_ir_led)
{
	ret_code_t err_code;
	uint32_t un_temp;
	//  unsigned char uch_temp;
	uint8_t tx_packet[1];
	*p_red_led=0;
	*p_ir_led=0;
	//char ach_i2c_data[6];
	char ach_i2c_data[6];

	//�������״̬�Ĵ���
	(void)max30102_read_reg(REG_INTR_STATUS_1, 1);
	(void)max30102_read_reg(REG_INTR_STATUS_2, 1);

	//д
	tx_packet[0] = REG_FIFO_DATA;
	m_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, MAX30102_ADDRESS, tx_packet, sizeof(tx_packet),false);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false){};
	//��
	m_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, MAX30102_ADDRESS, ach_i2c_data, 6);
	APP_ERROR_CHECK(err_code);
	while (m_xfer_done == false){};
	
	un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *p_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *p_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *p_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *p_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *p_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *p_ir_led+=un_temp;
  *p_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *p_ir_led&=0x03FFFF;  //Mask MSB [23:18]
			
		
		return true;
}
void max30102_reset(void)
{
   max30102_write_reg(REG_MODE_CONFIG,0x40);
}


void MAX30102_Init(void)
{   

	  //ʹ��A_FULL_EN �� PPG_RDY_EN �ж�
		max30102_write_reg(REG_INTR_ENABLE_1,0xc0); 

		max30102_write_reg(REG_INTR_ENABLE_2,0x00);

		max30102_write_reg(REG_FIFO_WR_PTR,0x00);  //FIFO_WR_PTR[4:0]

		max30102_write_reg(REG_OVF_COUNTER,0x00);  //OVF_COUNTER[4:0]

		max30102_write_reg(REG_FIFO_RD_PTR,0x00); //FIFO_RD_PTR[4:0]

		max30102_write_reg(REG_FIFO_CONFIG,0x0f);  //sample avg = 1, fifo rollover=false, fifo almost full = 17

		max30102_write_reg(REG_MODE_CONFIG,0x03);  //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED

		max30102_write_reg(REG_SPO2_CONFIG,0x27);  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)


		max30102_write_reg(REG_LED1_PA,0x24);   //Choose value for ~ 7mA for LED1

		max30102_write_reg(REG_LED2_PA,0x24);  // Choose value for ~ 7mA for LED2

		max30102_write_reg(REG_PILOT_PA,0x7f);   // Choose value for ~ 25mA for Pilot LED

}





