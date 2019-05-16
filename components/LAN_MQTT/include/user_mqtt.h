#ifndef __USER_MQTT_H
#define __USER_MQTT_H

#define MQTT_SOCKET 2
#define MQTT_PORT 1883
#define MQTT_CLIEND_ID "d8034f7509c44389b30194d4c373f09c"    //ID
#define MQTT_DOMAIN "api.ubibot.cn"                          //MQTT服务器域名
#define MQTT_USER "c_id=4777"                                //登陆用户名
#define MQTT_PASS "api_key=f2c931e60aec1a39338b4523c417eb54" //登陆密码
#define MQTT_Topic "/product/28343913545840b3b9b42c568e78e243/channel/4777/control"

#define MQTT_BUFFLEN 200
typedef void (*mqtt_connect_callback)(void);

void user_mqtt_task(void *pvParameter);
void user_mqtt_init(void);

#endif /*__USER_MQTT_H*/
