#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "app_uart.h"
//Log��Ҫ���õ�ͷ�ļ�
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_drv_twi.h"
#include "max30102.h"


#define UART_TX_BUF_SIZE 256       //���ڷ��ͻ����С���ֽ�����
#define UART_RX_BUF_SIZE 256       //���ڽ��ջ����С���ֽ�����

//�����¼��ص��������ú������ж��¼����Ͳ����д���
void uart_error_handle(app_uart_evt_t * p_event)
{
    //ͨѶ�����¼�
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    //FIFO�����¼�
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
//��������
void uart_config(void)
{
	uint32_t err_code;
	
	//���崮��ͨѶ�������ýṹ�岢��ʼ��
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//����uart��������
    TX_PIN_NUMBER,//����uart��������
    RTS_PIN_NUMBER,//����uart RTS���ţ����عرպ���Ȼ������RTS��CTS���ţ����������������ԣ������������������ţ����������Կ���ΪIOʹ��
    CTS_PIN_NUMBER,//����uart CTS����
    APP_UART_FLOW_CONTROL_DISABLED,//�ر�uartӲ������
    false,//��ֹ��ż����
    NRF_UART_BAUDRATE_115200//uart����������Ϊ115200bps
  };
  //��ʼ�����ڣ�ע�ᴮ���¼��ص�����
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    //��ʼ��log����ģ��
	  ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //����log����նˣ�����sdk_config.h�е�������������ն�ΪUART����RTT��
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
/***************************************************************************
* ��  �� : main���� 
* ��  �� : �� 
* ����ֵ : int ����
**************************************************************************/
int main(void)
{
	uint32_t reddat,irdat;
		
	//��ʼ��log����ģ�飬������ʹ��RTT��Ϊ����ն˴�ӡ��Ϣ
	log_init();
	
	//������������LEDָʾ��D1 D2 D3 D4�����Žţ�������P0.13~P0.16Ϊ���������LED�ĳ�ʼ״̬����ΪϨ�� 
  //�������ڼ��4��������IOλ���룬��������������
	bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); 
  //��ʼ������
  uart_config();
	
	nrf_gpio_cfg_input(MAX30102_INTPIN,NRF_GPIO_PIN_PULLUP);
	
	//��ʼ��I2C����
	twi_master_init();
	
	//��λMAX30102
  max30102_reset(); 
	
	//read and clear status register
  (void)max30102_read_reg(0,1);
	
  //��ʼ��MAX30102
  MAX30102_Init();
  nrf_delay_ms(1000);
	
  //LOG��ӡ������Ϣ
  NRF_LOG_INFO("max30102 example started");	
	NRF_LOG_FLUSH();
	
	while (true)
  {
	   while(nrf_gpio_pin_read(MAX30102_INTPIN) == 1){}   //wait until the interrupt pin asserts
		
		 max30102_read_fifo(&reddat, &irdat);  //read from MAX30102 FIFO
		 printf("red=  %d  ir= %d\r\n",reddat,irdat);
		 nrf_delay_ms(50);
		 nrf_gpio_pin_toggle(LED_1); //ָʾ��D1״̬��ת��ָʾ��������
  }
}
