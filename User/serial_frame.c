#include "serial_frame.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Serial2.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

static void fail_both(const char *msg)
{
    Serial_SendString((char *)msg);
    Serial2_SendString((char *)msg);
}

static int lower_prefix(const char *p, const char *lit)
{
    while (*lit) {
        char c = *p++;
        if (c >= 'A' && c <= 'Z') {
            c = (char)(c - 'A' + 'a');
        }
        if (c != *lit++) {
            return 0;
        }
    }
    return 1;
}

int SerialFrame_ParseData(char *line, int16_t raw6[6])
{
    char *p = line;
    char *star;
    unsigned int chk_expect = 0;
    unsigned int chk_got;
    int n;
    int i;

    while (*p == ' ' || *p == '\t') {
        p++;
    }

    if (!lower_prefix(p, "data:")) {
        fail_both("FAIL no header\r\n");
        return 0;
    }

    star = strchr(p, '*');
    if (star == 0) {
        fail_both("FAIL no checksum\r\n");
        return 0;
    }

    for (i = 0; p + i < star; i++) {
        chk_expect ^= (unsigned char)p[i];
    }

    if (sscanf(star + 1, "%2x", &chk_got) != 1) {
        fail_both("FAIL bad chk\r\n");
        return 0;
    }

    if (((unsigned int)chk_got & 0xFFu) != (chk_expect & 0xFFu)) {
        char buf[72];
        snprintf(buf, sizeof(buf),
                 "FAIL chk err got %02X expect %02X (XOR byte0..before*)\r\n",
                 (unsigned int)(chk_got & 0xFFu),
                 (unsigned int)(chk_expect & 0xFFu));
        fail_both(buf);
        return 0;
    }

    *star = '\0';

    {
        char *colon = strchr(p, ':');
        if (colon == 0) {
            fail_both("FAIL parse\r\n");
            return 0;
        }
        n = sscanf(colon + 1, "%hd,%hd,%hd,%hd,%hd,%hd",
                   &raw6[0], &raw6[1], &raw6[2], &raw6[3], &raw6[4], &raw6[5]);
    }
    if (n != 6) {
        fail_both("FAIL parse\r\n");
        return 0;
    }

    return 1;
}
