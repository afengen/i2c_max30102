#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include "app_uart.h"
//Log需要引用的头文件
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


#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）

//串口事件回调函数，该函数中判断事件类型并进行处理
void uart_error_handle(app_uart_evt_t * p_event)
{
    //通讯错误事件
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    //FIFO错误事件
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
//串口配置
void uart_config(void)
{
	uint32_t err_code;
	
	//定义串口通讯参数配置结构体并初始化
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//定义uart接收引脚
    TX_PIN_NUMBER,//定义uart发送引脚
    RTS_PIN_NUMBER,//定义uart RTS引脚，流控关闭后虽然定义了RTS和CTS引脚，但是驱动程序会忽略，不会配置这两个引脚，两个引脚仍可作为IO使用
    CTS_PIN_NUMBER,//定义uart CTS引脚
    APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
    false,//禁止奇偶检验
    NRF_UART_BAUDRATE_115200//uart波特率设置为115200bps
  };
  //初始化串口，注册串口事件回调函数
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
    //初始化log程序模块
	  ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //设置log输出终端（根据sdk_config.h中的配置设置输出终端为UART或者RTT）
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
/***************************************************************************
* 描  述 : main函数 
* 入  参 : 无 
* 返回值 : int 类型
**************************************************************************/
int main(void)
{
	uint32_t reddat,irdat;
		
	//初始化log程序模块，本例中使用RTT作为输出终端打印信息
	log_init();
	
	//配置用于驱动LED指示灯D1 D2 D3 D4的引脚脚，即配置P0.13~P0.16为输出，并将LED的初始状态设置为熄灭 
  //配置用于检测4个按键的IO位输入，并开启上拉电阻
	bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); 
  //初始化串口
  uart_config();
	
	nrf_gpio_cfg_input(MAX30102_INTPIN,NRF_GPIO_PIN_PULLUP);
	
	//初始化I2C总线
	twi_master_init();
	
	//复位MAX30102
  max30102_reset(); 
	
	//read and clear status register
  (void)max30102_read_reg(0,1);
	
  //初始化MAX30102
  MAX30102_Init();
  nrf_delay_ms(1000);
	
  //LOG打印启动信息
  NRF_LOG_INFO("max30102 example started");	
	NRF_LOG_FLUSH();
	
	while (true)
  {
	   while(nrf_gpio_pin_read(MAX30102_INTPIN) == 1){}   //wait until the interrupt pin asserts
		
		 max30102_read_fifo(&reddat, &irdat);  //read from MAX30102 FIFO
		 printf("red=  %d  ir= %d\r\n",reddat,irdat);
		 nrf_delay_ms(50);
		 nrf_gpio_pin_toggle(LED_1); //指示灯D1状态翻转，指示程序运行
  }
}
