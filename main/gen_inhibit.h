/*
 * gen_inhibit -- Vtrux generator-inhibit transmitter, first increment.
 *
 * This increment exists to MEASURE, not yet to inhibit. It answers the one
 * question the whole design turns on and that no capture so far could answer:
 * when a VCM 0x051 frame arrives, how long until a frame of ours is on the
 * wire, and how tightly is that bounded?
 *
 * Why existing data cannot answer it: BUSMASTER timestamps host-side in
 * batches (up to 96 frames sharing one 100 us tick), so recorded inter-frame
 * times are the logger's, not the bus's. projects/vtrux/tools/gen_inhibit/
 * therefore sweeps jitter as a parameter rather than assuming a value.
 *
 * Why this path and not wican-fw's: main.c's can_rx_task drains with a
 * NON-BLOCKING can_receive(..., 0) and then vTaskDelay(pdMS_TO_TICKS(1)).
 * That alone injects up to 1 ms of latency -- most of the guard band we are
 * trying to fit inside. This component takes the bus with a dedicated
 * high-priority task doing a blocking twai_receive(), which the driver ISR
 * wakes directly.
 *
 * SAFETY -- read before enabling RESPOND on a vehicle:
 *
 *   RESPOND mode transmits GEN_INHIBIT_PROBE_ID (0x7F0), never 0x051. It is a
 *   timing probe, not a command: nothing on the truck consumes that ID, so a
 *   probe frame emitted at the wrong instant cannot be mistaken for a torque
 *   command. Emitting real 0x051 is the NEXT increment and is deliberately
 *   not implemented here.
 *
 *   OBSERVE mode never transmits. Note it cannot use transceiver standby to
 *   guarantee that: on MCP2561/2 the STBY pin disables the receiver as well,
 *   so standby is fully silent rather than listen-only. Set the device's
 *   can_mode to "silent" (TWAI_MODE_LISTEN_ONLY) for a hardware-backed
 *   guarantee.
 */
#ifndef GEN_INHIBIT_H
#define GEN_INHIBIT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The VCM's generator torque command. Observed only; never transmitted here. */
#define GEN_INHIBIT_VCM_ID      0x051

/* Timing probe. Deliberately an ID nothing on the truck consumes. */
#define GEN_INHIBIT_PROBE_ID    0x7F0

typedef enum
{
    GEN_INHIBIT_OFF = 0,    /* component idle; wican-fw owns the bus as usual */
    GEN_INHIBIT_OBSERVE,    /* timestamp 0x051 arrivals; transmit nothing */
    GEN_INHIBIT_RESPOND,    /* also emit a probe frame (0x7F0) at a fixed offset */
    GEN_INHIBIT_INHIBIT,    /* THE REAL THING: on each 0x051 RX, transmit a
                             * zero-torque 0x051 with the stolen next counter.
                             * Transmits the real command ID -- bench only. */
} gen_inhibit_mode_t;

/* Start the worker. Call once, after can_init(). Starts in OFF. */
void gen_inhibit_init(void);

/* Mode change. `offset_us` applies to RESPOND: delay from RX to probe TX. */
esp_err_t gen_inhibit_set_mode(gen_inhibit_mode_t mode, uint32_t offset_us);

/* True while this component owns the TWAI receive path, so wican-fw's
 * can_rx_task can stand down rather than race us for frames. */
bool gen_inhibit_owns_bus(void);

/*
 * Force the worker out of the TWAI driver and keep it out until re-armed.
 *
 * can_disable() uninstalls the driver; doing that while the worker is blocked
 * inside twai_receive() frees the driver under it. Any task that is about to
 * tear the bus down -- the sleep monitor, an SLCAN close, a protocol switch --
 * MUST call this first. It blocks until the worker is provably outside the
 * driver (bounded by the receive timeout). Safe to call from the worker itself
 * (returns immediately) and safe when the worker never started.
 */
void gen_inhibit_quiesce(void);

/* JSON stats into `buf`. Returns bytes written. */
int gen_inhibit_get_stats_json(char *buf, int buflen);

/* Discard all accumulated samples. */
void gen_inhibit_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* GEN_INHIBIT_H */
