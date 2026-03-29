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
            *star = '*';
            fail_both("FAIL parse\r\n");
            return 0;
        }
        n = sscanf(colon + 1, "%hd,%hd,%hd,%hd,%hd,%hd",
                   &raw6[0], &raw6[1], &raw6[2], &raw6[3], &raw6[4], &raw6[5]);
    }
    if (n != 6) {
        *star = '*';
        fail_both("FAIL parse\r\n");
        return 0;
    }

    return 1;
}

int SerialFrame_IsTauLine(const char *line)
{
    const char *p = line;

    if (line == 0) {
        return 0;
    }
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    return lower_prefix(p, "tau:") ? 1 : 0;
}

int SerialFrame_ParseTau(char *line, float tau4[4])
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

    if (!lower_prefix(p, "tau:")) {
        return 0;
    }

    star = strchr(p, '*');
    if (star == 0) {
        fail_both("FAIL TAU no checksum\r\n");
        return 0;
    }

    for (i = 0; p + i < star; i++) {
        chk_expect ^= (unsigned char)p[i];
    }

    if (sscanf(star + 1, "%2x", &chk_got) != 1) {
        fail_both("FAIL TAU bad chk\r\n");
        return 0;
    }

    if (((unsigned int)chk_got & 0xFFu) != (chk_expect & 0xFFu)) {
        char buf[72];
        snprintf(buf, sizeof(buf),
                 "FAIL TAU chk err got %02X expect %02X\r\n",
                 (unsigned int)(chk_got & 0xFFu),
                 (unsigned int)(chk_expect & 0xFFu));
        fail_both(buf);
        return 0;
    }

    *star = '\0';

    {
        char *colon = strchr(p, ':');
        if (colon == 0) {
            *star = '*';
            fail_both("FAIL TAU parse\r\n");
            return 0;
        }
        n = sscanf(colon + 1, "%f,%f,%f,%f",
                   &tau4[0], &tau4[1], &tau4[2], &tau4[3]);
    }
    if (n != 4) {
        *star = '*';
        fail_both("FAIL TAU parse floats\r\n");
        return 0;
    }

    return 1;
}
