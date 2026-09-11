/*
 * ota_health.h -- gate the OTA "image is good" decision on the recovery
 * channel actually working.
 *
 * Stock wican-fw calls esp_ota_mark_app_valid_cancel_rollback() in app_main
 * immediately after creating three FreeRTOS queues -- before config_server_start(),
 * before can_init(), and before wifi_network_init(). An image therefore certifies
 * itself as good roughly 100 lines before the channel we would use to recover it
 * exists, so a build that fails to bring up WiFi or the web server is marked
 * valid and unreachable. That is the exact failure rollback is meant to catch.
 *
 * Here the mark-valid call happens only once every subsystem needed to push a
 * replacement image has reported in. If they do not report within
 * OTA_HEALTH_TIMEOUT_MS, and this image is still PENDING_VERIFY, we roll back
 * to the previous slot instead.
 */
#ifndef OTA_HEALTH_H
#define OTA_HEALTH_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Subsystems that together constitute a *recoverable* device -- i.e. one we
 * can push a replacement image to. That is WiFi + the HTTP server, and only
 * those. CAN is deliberately NOT gated: a build that brings up the recovery
 * channel but mishandles the bus is still recoverable (reflash it), and gating
 * on CAN gave false confidence anyway -- the report fired after can_init(),
 * which only records the bitrate and does not start the driver. Whether CAN
 * works is judged after boot, by the thing that uses it.
 */
#define OTA_HEALTH_WIFI_AP   BIT0   /* SoftAP started -- we are reachable */
#define OTA_HEALTH_HTTPD     BIT1   /* config server listening -- OTA endpoint live */
#define OTA_HEALTH_CAN       BIT2   /* reported for logging only; not in _ALL */

#define OTA_HEALTH_ALL  (OTA_HEALTH_WIFI_AP | OTA_HEALTH_HTTPD)

#ifndef OTA_HEALTH_TIMEOUT_MS
#define OTA_HEALTH_TIMEOUT_MS  60000
#endif

/* Start the gate task. Call early in app_main, after dev_status_init(). */
void ota_health_init(void);

/* Report a subsystem up. Safe to call before ota_health_init(). */
void ota_health_report(EventBits_t bits);

#ifdef __cplusplus
}
#endif

#endif /* OTA_HEALTH_H */
