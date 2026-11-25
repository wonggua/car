#include "car_radio.h"
#include "car_config.h"
#include "car_protocol.h"
#include "car_board.h"
#include <string.h>
#include <stdio.h>

/* Active-receive ESP-AT, one controlling TCP connection. */
static const char *const setup[] = {
    "AT\r\n", "ATE0\r\n", "AT+CWMODE=2\r\n",
    "AT+CIPAP=\"192.168.4.1\"\r\n",
    "AT+CWSAP=\"STM32-Car\",\"Car8266wifi\",6,3\r\n",
    "AT+CIPMODE=0\r\n", "AT+CIPMUX=1\r\n", "AT+CIPDINFO=0\r\n",
    "AT+CIPSERVER=1,8080\r\n"
};
enum { DELAY, WAIT_OK, ONLINE, PROMPT, SEND_OK };
static unsigned int phase, stage, used, remaining, deliver, discard;
static int client = -1;
static int wifi_enabled = 1;
int Wifi_Enabled(void) { return wifi_enabled; }
static uint32_t since;
static char line[128], pending[512], sending[512];

/* Wired-console provisioning. A full STM32 reset restores the default AP. */
static char next_ssid[33], join_command[216], station_ip[16];
enum { MODE_STEP = 2, JOIN_STEP, ADDRESS_STEP };
static const char *setup_command(void)
{
    if (join_command[0]) {
        if (stage == MODE_STEP) return "AT+CWMODE=1\r\n";
        if (stage == JOIN_STEP) return join_command;
        if (stage == ADDRESS_STEP) return "AT+CIFSR\r\n";
    }
    return setup[stage];
}
static char *quoted(char *out, const char *s)
{
    *out++ = '"';
    while (*s) {
        if (*s == '"' || *s == ',' || *s == '\\') *out++ = '\\';
        *out++ = *s++;
    }
    *out++ = '"';
    return out;
}
void Wifi_Config(const char *line_text, uint32_t now)
{
    const char *value = line_text;
    char *out;
    if (!strcmp(value,"BT") || !strcmp(value,"WIFI")) {
        int enable = !strcmp(value,"WIFI");
        if (enable != wifi_enabled) {
            Control_Run(0,now);
            Board_RadioBaud(enable ? 115200U : CAR_BLUETOOTH_BAUD);
            Wifi_Reset(now);
            wifi_enabled = enable;
        }
        Board_Print(0, enable ? "radio=WIFI\r\n" : "radio=BT\r\n");
        return;
    }
    if (!wifi_enabled) { Board_Print(0,"radio=BT; send $WIFI first\r\n"); return; }
    if (!strncmp(value, "SSID=", 5)) {
        value += 5;
        next_ssid[0] = 0;
        if (*value && strlen(value) <= 32) {
            strcpy(next_ssid, value);
            Board_Print(0,"wifi: SSID set; send $PASS=...\r\n");
            return;
        }
        Board_Print(0,"wifi: invalid SSID\r\n"); return;
    } else if (!strncmp(value, "PASS=", 5) && next_ssid[0] && strlen(value+5) <= 64) {
        strcpy(join_command,"AT+CWJAP=");
        out = quoted(join_command+strlen(join_command), next_ssid);
        *out++ = ',';
        out = quoted(out,value+5);
        strcpy(out,"\r\n");
    } else if (!strcmp(value,"AP")) {
        memset(join_command,0,sizeof(join_command));
        next_ssid[0] = 0;
    } else {
        Board_Print(0,"wifi: invalid settings\r\n"); return;
    }
    Board_WifiRaw("AT+RST\r\n");
    Wifi_Reset(now);
    Board_Print(0,"wifi: configuring network\r\n");
}
void Wifi_Reset(uint32_t now)
{
    phase = DELAY; stage = used = remaining = discard = 0;
    station_ip[0] = 0; client = -1; pending[0] = sending[0] = 0; since = now;
    Protocol_ResetPort(1);
}
void Wifi_Print(const char *s)
{
    if (client >= 0 && strlen(s) + strlen(pending) < sizeof(pending))
        strcat(pending, s);
}
void Wifi_Poll(uint32_t now)
{
    char command[40];
    if (phase == DELAY && now - since >= 1500U) {
        Board_WifiRaw(setup_command()); phase = WAIT_OK; since = now;
    } else if (phase == ONLINE && client >= 0 && pending[0]) {
        strcpy(sending, pending); pending[0] = 0;
        sprintf(command, "AT+CIPSEND=%d,%u\r\n", client, (unsigned)strlen(sending));
        Board_WifiRaw(command); phase = PROMPT; since = now;
    } else if (phase != ONLINE && phase != DELAY && now - since > ((phase == WAIT_OK && join_command[0] && stage == JOIN_STEP) ? 30000U : 5000U)) {
        Board_Print(0, "wifi: AT timeout; retry\r\n");
        Board_WifiRaw("AT+RST\r\n"); Wifi_Reset(now);
    }
}
static void response(uint32_t now)
{
    if (!strcmp(line,"ready")) { Wifi_Reset(now); return; }
    if (join_command[0] && phase >= ONLINE && !strcmp(line,"WIFI DISCONNECT")) {
        Board_WifiRaw("AT+RST\r\n"); Wifi_Reset(now); return;
    }
    if (phase == WAIT_OK) {
        if (join_command[0] && stage == ADDRESS_STEP && !strncmp(line,"+CIFSR:STAIP,\"",14)) {
            const char *end = strchr(line+14,'"');
            if (end && end > line+14 && end-(line+14) < (int)sizeof(station_ip)) {
                memcpy(station_ip,line+14,end-(line+14)); station_ip[end-(line+14)] = 0;
            }
        }
        if (join_command[0] && stage == JOIN_STEP && !strncmp(line,"+CWJAP:",7)) {
            if (line[7] >= '1' && line[7] <= '4' && !line[8]) {
                Board_Print(0,line); Board_Print(0,"\r\n");
            }
        }
        if (!strcmp(line,"OK")) {
            if (++stage == sizeof(setup)/sizeof(setup[0])) {
                phase = ONLINE;
                Board_Print(0,join_command[0] ? "wifi: phone hotspot TCP " : "wifi: STM32-Car TCP ");
                Board_Print(0,join_command[0] ? (station_ip[0] ? station_ip : "(see hotspot clients)") : "192.168.4.1");
                Board_Print(0,":8080 ready\r\n");
            } else { Board_WifiRaw(setup_command()); since = now; }
        } else if (!strcmp(line,"ERROR") || !strcmp(line,"FAIL")) {
            Board_Print(0,"wifi: failed command "); Board_Print(0,(join_command[0] && stage == JOIN_STEP) ? "AT+CWJAP\r\n" : setup_command());
            Board_WifiRaw("AT+RST\r\n"); Wifi_Reset(now);
        }
        return;
    }
    if (phase == DELAY) return;
    if (line[0] >= '0' && line[0] <= '4' && line[1] == ',') {
        int id = line[0] - '0';
        if (!strcmp(line+2,"CONNECT") && client < 0) {
            client = id; Protocol_ResetPort(1);
        } else if (!strcmp(line+2,"CLOSED") && client == id) {
            client = -1; pending[0] = 0; Protocol_ResetPort(1);
            phase = ONLINE;
        }
    } else if ((phase == SEND_OK || phase == PROMPT) &&
               (!strcmp(line,"SEND OK") || !strcmp(line,"SEND FAIL") || !strcmp(line,"ERROR"))) {
        phase = ONLINE;
    }
}
void Wifi_Byte(unsigned int b, uint32_t now)
{
    unsigned int n;
    if (remaining) {
        --remaining;
        if (deliver) Protocol_Byte(1,b,now);
        return;
    }
    if (phase == PROMPT && !used && b == '>') {
        Board_WifiRaw(sending); phase = SEND_OK; since = now; return;
    }
    if (b == '\n') {
        line[used] = 0;
        if (!discard) response(now);
        used = discard = 0; return;
    }
    if (b == '\r' || discard || (!used && b == ' ')) return;
    if (b == ':' && used >= 8 && !memcmp(line,"+IPD,",5) &&
        line[5] >= '0' && line[5] <= '4' && line[6] == ',') {
        unsigned int length = 0;
        for (n = 7; n < used; ++n) {
            if (line[n] < '0' || line[n] > '9' || length > 6553U) break;
            length = length * 10U + (unsigned)(line[n]-'0');
        }
        if (n == used && length <= 65535U) {
            remaining = length;
            deliver = phase >= ONLINE && client == line[5]-'0';
            used = 0; return;
        }
    }
    if (used < sizeof(line)-1) line[used++] = (char)b;
    else discard = 1;
}
