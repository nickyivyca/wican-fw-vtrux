/*
 * extract_probe -- expose the core's hand-rolled signal extractors so
 * test_signals.py can cross-check them against cantools.
 *
 * WHY THIS EXISTS. AGENTS.md's standing rule is to decode with cantools and
 * never hand-roll bit extraction. The core hand-rolls it, because it must run
 * on an ESP32 with no DBC. Purity makes those extractors testable; it does
 * NOT make them right -- they are a hand transcription of an external
 * artifact, and a transcription error would be invisible to every other test
 * in this suite, all of which would happily agree with the wrong value.
 *
 * This project already has one live instance of exactly that failure: the
 * truck-validated Python reference decodes GENE_RotSpd with a generic
 * -32768 helper where the DBC says -32767. That is why this cross-check is
 * not optional.
 *
 * Reads lines of "<kind> <hexbytes>" on stdin, prints the extracted values.
 */
#include "gen_inhibit_core.h"

#include <stdio.h>
#include <string.h>

static uint8_t hexnib(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0;
}

int main(void)
{
    char kind[32], hex[64];

    while (scanf("%31s %63s", kind, hex) == 2)
    {
        uint8_t d[8] = { 0 };
        size_t n = strlen(hex) / 2;
        if (n > 8) n = 8;
        for (size_t i = 0; i < n; i++)
        {
            d[i] = (uint8_t)((hexnib(hex[i * 2]) << 4) | hexnib(hex[i * 2 + 1]));
        }

        if (!strcmp(kind, "cmd"))
        {
            /* 0x051: gen_torque_cmd B1-B2, gen_rpm_ref B3-B4, counter B5. */
            printf("%d %d %u\n",
                   gi_le16c(&d[1], GI_CMD_ZERO),
                   gi_le16c(&d[3], GI_CMD_ZERO),
                   (unsigned)gi_ctr(d));
        }
        else if (!strcmp(kind, "rpm"))
        {
            /* 0x054: GENE_RotSpd B0-B1, zero at 32767 per the DBC. */
            printf("%d\n", gi_le16c(&d[0], GI_RPM_ZERO));
        }
        else if (!strcmp(kind, "soc"))
        {
            /* 0x411: BMS_SoC_HiRes, 14-bit BE at bit 7, scale 0.01 %. */
            printf("%u\n", gi_soc_raw(d));
        }
        else if (!strcmp(kind, "shift"))
        {
            /* 0x639: shift_lever_pos, B6 bits 6:4. */
            printf("%u\n", (unsigned)gi_shift_pos(d));
        }
        else
        {
            fprintf(stderr, "unknown kind %s\n", kind);
            return 2;
        }
    }
    return 0;
}
