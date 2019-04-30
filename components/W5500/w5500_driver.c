/*******************************************************************************
  * @file       Uart1 Application Task  
  * @author 
  * @version
  * @date 
  * @brief
  ******************************************************************************
  * @attention
  *
  *
*******************************************************************************/

/*-------------------------------- Includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"

#include "wizchip_conf.h"
#include "w5500_driver.h"

#include "socket.h"

#include "w5500_spi.h"
#include "dhcp.h"
#include "dns.h"

// extern OsiSyncObj_t Spi_Mutex;   //Used for SPI Lock
// extern OsiSyncObj_t cJSON_Mutex; //Used for cJSON Lock
// extern OsiSyncObj_t xMutex5;     //Used for Post_Data_Buffer Lock

// extern OsiSyncObj_t xBinary2; //For Temp&Humi Sensor Task
// extern OsiSyncObj_t xBinary3; //For Light Sensor Task
// extern OsiSyncObj_t xBinary6; //For external Temprature Measure
// extern OsiSyncObj_t xBinary7; //For Power Measure Task

#define RJ45_DEBUG 1
#define FAILURE 0
#define SUCCESS 1
#define RJ45_STATUS_TIMEOUT 3

uint32_t socker_port = 3000;
uint8_t LAN_WEB_SERVER[16];
uint8_t ethernet_buf[ETHERNET_DATA_BUF_SIZE];
uint8_t dns_host_ip[4];
uint8_t server_port = 80;
uint8_t standby_dns[4] = {8, 8, 8, 8};
wiz_NetInfo gWIZNETINFO;
wiz_NetInfo gWIZNETINFO_READ;
extern char WEB_SERVER[64];
extern char POST_REQUEST_URI[255];
extern char Post_Data_Buffer[4096];
extern volatile bool NET_DATA_POST;
extern volatile unsigned long POST_NUM;
extern volatile unsigned long DELETE_ADDR, POST_ADDR, WRITE_ADDR;

extern short Prase_Resp_Data(uint8_t mode, char *recv_buf, uint16_t buf_len); //Prase GPRS/ETHERNET Post Resp Data
extern void PostAddrChage(unsigned long data_num, unsigned long end_addr);    //deleted the post data

uint8_t Ethernet_Timeout = 0; //ethernet http application time out
uint8_t Ethernet_netSet_val = 0;
uint8_t ethernet_http_mode = ACTIVE_DEVICE_MODE;
static bool lan_EndFlag = 0;
static uint16_t lan_read_data_num = 0;
static uint16_t lan_post_data_sum = 0;
static uint16_t lan_post_data_num = 0;
static uint16_t lan_delete_data_num = 0;
static unsigned long lan_post_data_len = 0;
static unsigned long lan_read_data_end_addr = 0;
static unsigned long lan_MemoryAddr = 0;
static char lan_mac_buf[18] = {0};
uint8_t Http_Buffer;

struct HTTP_STA
{
        char GET[10];
        char POST[10];
        char HEART_BEAT[64];

        char POST_URL1[64];
        char POST_URL_METADATA[16];
        char POST_URL_FIRMWARE[16];
        char POST_URL_SSID[16];
        char POST_URL_COMMAND_ID[16];

        char WEB_URL1[50];
        char WEB_URL2[20];
        char WEB_URL3[20];

        char HTTP_VERSION10[20];
        char HTTP_VERSION11[20];

        char HOST[30];
        char USER_AHENT[40];
        char CONTENT_LENGTH[30];
        char ENTER[10];
} http = {"GET ",
          "POST ",
          "http://api.ubibot.cn/heartbeat?api_key=",

          "http://api.ubibot.cn/update.json?api_key=",
          "&metadata=true",
          "&firmware=",
          "&ssid=",
          "&command_id=",

          "http://api.ubibot.cn/products/",
          "/devices/",
          "/activate",

          " HTTP/1.0\r\n",
          " HTTP/1.1\r\n",

          "Host: api.ubibot.cn\r\n",
          "User-Agent: dalian urban ILS1\r\n",
          "Content-Length:",
          "\r\n\r\n"};

/*******************************************************************************
// Reset w5500 chip with w5500 RST pin                                
*******************************************************************************/
void w5500_reset(void)
{
        gpio_set_level(PIN_NUM_W5500_REST, 0);
        vTaskDelay(10 / portTICK_RATE_MS);
        gpio_set_level(PIN_NUM_W5500_REST, 1);
        vTaskDelay(2000 / portTICK_RATE_MS);
}

/*******************************************************************************
// callback function for critical section enter.                            
*******************************************************************************/
void spi_cris_en(void)
{
        //osi_SyncObjWait(&Spi_Mutex, OSI_WAIT_FOREVER); //SPI Semaphore Take
}

/*******************************************************************************
// callback function for critical section exit.                              
*******************************************************************************/
void spi_cris_ex(void)
{
        //////osi_SyncObjSignal(&Spi_Mutex); //SPI Semaphore Give
}

/*******************************************************************************
// callback function for WIZCHIP select                             
*******************************************************************************/
void spi_cs_select(void)
{
        // gpio_set_level(PIN_NUM_CS, 0);
        spi_select_w5500();
        // printf("SPI CS SELECT! \n");
        // return;
}

/*******************************************************************************
// callback fucntion for WIZCHIP deselect                             
*******************************************************************************/
void spi_cs_deselect(void)
{
        spi_deselect_w5500();
        // printf("SPI CS DESELECT! \n");
        // return;
        // gpio_set_level(PIN_NUM_CS, 1);
}

/*******************************************************************************
// init w5500 driver lib                               
*******************************************************************************/
void w5500_lib_init(void)
{
        // /* Critical section callback */
        reg_wizchip_cris_cbfunc(spi_cris_en, spi_cris_ex);

        /* Chip selection call back  */
        reg_wizchip_cs_cbfunc(spi_cs_select, spi_cs_deselect);

        /* SPI Read & Write callback function */
        reg_wizchip_spi_cbfunc(spi_readbyte, spi_writebyte);

        /* Registers call back function for SPI interface */
        reg_wizchip_spiburst_cbfunc(spi_readburst, spi_writeburst);
        printf("lib_init success\n");
}

/*******************************************************************************
// w5500 enter sleep mode                              
*******************************************************************************/
// void w5500_enter_sleep(void)
// {
//         uint8_t write_val = 0x7f;
//         spi_write_data(0x002e, FDM1 | RWB_WRITE | COMMON_R, &write_val, 1);

//         write_val = 0xf7;
//         spi_write_data(0x002e, FDM1 | RWB_WRITE | COMMON_R, &write_val, 1);
// }

/*******************************************************************************
// check RJ45 connected                              
*******************************************************************************/
uint8_t check_rj45_status(void)
{
        uint8_t i;

        for (i = 0; i < RJ45_STATUS_TIMEOUT; i++)
        {
                if (IINCHIP_READ(PHYCFGR) & 0x01)
                {
                        printf("RJ45 OK\n");
                        return ESP_OK;
                }
                printf("RJ45 FAIL\n ");
                vTaskDelay(500 / portTICK_RATE_MS);
        }

        printf("RJ45 FAIL\n ");
        return ESP_FAIL;
}

/****************DHCP IP更新回调函数*****************/
void my_ip_assign(void)
{
        getIPfromDHCP(gWIZNETINFO.ip);

        getGWfromDHCP(gWIZNETINFO.gw);

        getSNfromDHCP(gWIZNETINFO.sn);

        getDNSfromDHCP(gWIZNETINFO.dns);

        gWIZNETINFO.dhcp = NETINFO_DHCP; //NETINFO_STATIC; //< 1 - Static, 2 - DHCP

        ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);

#ifdef RJ45_DEBUG
        ctlnetwork(CN_GET_NETINFO, (void *)&gWIZNETINFO);
        printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
        printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
        printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
        printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
        printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
#endif
}

/****************DHCP IP冲突函数*****************/
void my_ip_conflict(void)
{
        printf("DHCP IP 冲突\n");
}

/****************解析DNS*****************/
uint8_t lan_dns_resolve(uint8_t *dns_buf)
{
        //   osi_at24c08_ReadData(HOST_ADDR,(uint8_t*)WEB_SERVER,sizeof(WEB_SERVER),1);  //read the host name

        DNS_init(SOCK_DHCP, dns_buf);

        if (DNS_run(gWIZNETINFO.dns, (uint8_t *)HOST_NAME, dns_host_ip) > 0)
        {
#ifdef RJ45_DEBUG
                printf("host ip: %d.%d.%d.%d\r\n", dns_host_ip[0], dns_host_ip[1], dns_host_ip[2], dns_host_ip[3]);
#endif

                return SUCCESS;
        }
        else if (DNS_run(standby_dns, (uint8_t *)HOST_NAME, dns_host_ip) > 0)
        {
#ifdef RJ45_DEBUG
                printf("s_host ip: %d.%d.%d.%d\r\n", dns_host_ip[0], dns_host_ip[1], dns_host_ip[2], dns_host_ip[3]);
#endif

                return SUCCESS;
        }

#ifdef RJ45_DEBUG
        printf("n_host ip: %d.%d.%d.%d\r\n", dns_host_ip[0], dns_host_ip[1], dns_host_ip[2], dns_host_ip[3]);
#endif

        return FAILURE;
}

/*******************************************************************************
// w5500 init network                              
*******************************************************************************/
void W5500_Network_Init(void)
{
        uint8_t mac[6] = {0x06, 0x08, 0xdc, 0x00, 0xab, 0xcd}; //< Source Mac Address
        uint8_t dhcp_mode = 1;                                 //0:static ;1:dhcp
        uint8_t ip[4] = {192, 168, 1, 123};                    //< Source IP Address
        uint8_t sn[4] = {255, 255, 255, 0};                    //< Subnet Mask
        uint8_t gw[4] = {192, 168, 1, 1};                      //< Gateway IP Address
        uint8_t dns[4] = {114, 114, 114, 114};                 //< DNS server IP Address

        uint8_t txsize[MAX_SOCK_NUM] = {4, 2, 2, 2, 2, 2, 2, 0}; //socket 0,16K
        uint8_t rxsize[MAX_SOCK_NUM] = {4, 2, 2, 2, 2, 2, 2, 0}; //socket 0,16K

        esp_read_mac(&mac, 3); //      获取芯片内部默认出厂MAC，

        wizchip_init(txsize, rxsize);

        memcpy(gWIZNETINFO.mac, mac, 6);
        memcpy(gWIZNETINFO.ip, ip, 4);
        memcpy(gWIZNETINFO.sn, sn, 4);
        memcpy(gWIZNETINFO.gw, gw, 4);
        memcpy(gWIZNETINFO.dns, dns, 4);

        if (dhcp_mode)
        {
                gWIZNETINFO.dhcp = NETINFO_DHCP; //< 1 - Static, 2 - DHCP
        }
        else
        {
                gWIZNETINFO.dhcp = NETINFO_STATIC; //< 1 - Static, 2 - DHCP
        }

        ctlnetwork(CN_SET_NETINFO, (void *)&gWIZNETINFO);

#ifdef RJ45_DEBUG
        ctlnetwork(CN_GET_NETINFO, (void *)&gWIZNETINFO_READ);
        printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO_READ.mac[0], gWIZNETINFO_READ.mac[1], gWIZNETINFO_READ.mac[2], gWIZNETINFO_READ.mac[3], gWIZNETINFO_READ.mac[4], gWIZNETINFO_READ.mac[5]);
        printf("SIP: %d.%d.%d.%d\r\n", gWIZNETINFO_READ.ip[0], gWIZNETINFO_READ.ip[1], gWIZNETINFO_READ.ip[2], gWIZNETINFO_READ.ip[3]);
        printf("GAR: %d.%d.%d.%d\r\n", gWIZNETINFO_READ.gw[0], gWIZNETINFO_READ.gw[1], gWIZNETINFO_READ.gw[2], gWIZNETINFO_READ.gw[3]);
        printf("SUB: %d.%d.%d.%d\r\n", gWIZNETINFO_READ.sn[0], gWIZNETINFO_READ.sn[1], gWIZNETINFO_READ.sn[2], gWIZNETINFO_READ.sn[3]);
        printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO_READ.dns[0], gWIZNETINFO_READ.dns[1], gWIZNETINFO_READ.dns[2], gWIZNETINFO_READ.dns[3]);
#endif

        wiz_NetTimeout E_NetTimeout;
        E_NetTimeout.retry_cnt = 50;    //< retry count
        E_NetTimeout.time_100us = 1000; //< time unit 100us
        wizchip_settimeout(&E_NetTimeout);

        if (gWIZNETINFO.dhcp == NETINFO_DHCP)
        {
                Ethernet_Timeout = 0;
                uint8_t dhcp_retry = 0;

                reg_dhcp_cbfunc(my_ip_assign, my_ip_assign, my_ip_conflict);

                DHCP_init(SOCK_DHCP, ethernet_buf);

                while (DHCP_run() != DHCP_IP_LEASED)
                {
                        switch (DHCP_run())
                        {
                        case DHCP_IP_ASSIGN:
#ifdef RJ45_DEBUG
                                printf("DHCP_IP_ASSIGN.\r\n");
#endif
                        case DHCP_IP_CHANGED: /* If this block empty, act with default_ip_assign & default_ip_update */
                                              //
                                              // Add to ...
                                              //
#ifdef RJ45_DEBUG
                                printf("DHCP_IP_CHANGED.\r\n");
#endif
                                break;

                        case DHCP_IP_LEASED:
                                //
                                // TO DO YOUR NETWORK APPs.
                                //
#ifdef RJ45_DEBUG
                                printf("DHCP_IP_LEASED.\r\n");

                                printf("DHCP LEASED TIME : %d Sec\r\n", getDHCPLeasetime());
#endif
                                break;

                        case DHCP_FAILED:
#ifdef RJ45_DEBUG
                                printf("DHCP_FAILED.\r\n");
#endif

                                if (dhcp_retry++ > RETRY_TIME_OUT)
                                {
                                        DHCP_stop(); // if restart, recall DHCP_init()
                                }
                                break;

                        default:
                                break;
                        }

                        // if (Ethernet_Timeout >= ETHERNET_HTTP_TIMEOUT)
                        // {
                        //         break;
                        // }
                }
        }
        printf("Network_init success!!!\n");
}

/*****************http发送****************/
void http_send(void)
{

        char build_http[256];
        char recv_buf[1024];
        sprintf(build_http, "%s%s%s%s%s%s%s", http.GET, http.WEB_URL1, "dc6719fd1120443a9e13916aaef07ef5", http.WEB_URL2, "SP10001", http.WEB_URL3, http.ENTER);
        //http.HTTP_VERSION10, http.HOST, http.USER_AHENT, http.ENTER);

        printf("build_http=%s\n", build_http);

        while (1)
        {
                switch (getSn_SR(SOCK_DHCP))
                {
                case SOCK_INIT:
                        printf("SOCK_INIT!!!\n");
                        int8_t ret = connect(SOCK_DHCP, dns_host_ip, server_port);
                        printf("%d\n", ret);
                        break;
                case SOCK_ESTABLISHED:
                        if (getSn_IR(SOCK_DHCP) & Sn_IR_CON)
                        {
                                setSn_IR(SOCK_DHCP, Sn_IR_CON);
                        }
                        // Http_Buffer = "http://api.ubibot.cn/products/产品号/devices/序列号/activate";
                        printf("SOCK_ESTABLISHED!!!\n");
                        send(SOCK_DHCP, (const uint8_t *)build_http, sizeof(build_http));
                        vTaskDelay(500 / portTICK_RATE_MS);
                        close(SOCK_DHCP);
                        break;

                case SOCK_CLOSE_WAIT:
                        printf("SOCK_CLOSE_WAIT!!!\n");
                        break;

                case SOCK_CLOSED:
                        printf("Closed\r\n");
                        if ((socket(SOCK_DHCP, Sn_MR_TCP, socker_port, 0x00)) != SOCK_DHCP)
                        {
                                printf("OK\r\n");
                                int32_t r = 0;
                                r = recv(SOCK_DHCP, &recv_buf, sizeof(recv_buf));
                                if (r > 0)
                                {
                                        printf("接收到%d字节   recvdate = %s\n ", r, recv_buf);
                                }
                                else
                                {
                                        printf("fail!!! code:%d\n", r);
                                }
                        }
                        break;
                default:
                        break;
                }

                vTaskDelay(500 / portTICK_RATE_MS);
        }
}

/*******************有线网初始化*******************/
void w5500_user_int(void)
{
        gpio_config_t io_conf;

        //disable interrupt
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO16
        io_conf.pin_bit_mask = (1 << PIN_NUM_W5500_REST);
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 1;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        io_conf.pin_bit_mask = (1 << PIN_NUM_W5500_INT);
        gpio_config(&io_conf);

        w5500_reset();
        w5500_lib_init();

        // printf(" VERSIONR_ID: %02x\n", IINCHIP_READ(VERSIONR));
        esp_err_t ret;
        ret = check_rj45_status();
        if (ret == ESP_OK)
        {
                W5500_Network_Init();
                ret = lan_dns_resolve(ethernet_buf);
                if (ret == SUCCESS)
                {
                        printf("DNS_SUCCESS!!!\n");
                        http_send();
                        // xTaskCreate(http_send, "http_send", 8192, NULL, 2, NULL);
                }
        }
}

/*******************************************************************************
                                      END         
*******************************************************************************/
