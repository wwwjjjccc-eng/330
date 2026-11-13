#ifndef ANTAGONISTIC_TENDON_H
#define ANTAGONISTIC_TENDON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* STS3215 position units in the configured position mode. Verify the mode and
 * limits on your servo before enabling real hardware. */
#define AT_SERVO_POSITION_MAX 4095U
#define AT_DOF_COUNT 10U
#define AT_SERVO_COUNT (AT_DOF_COUNT * 2U)

typedef struct {
    uint8_t id;
    uint16_t open_position;   /* calibrated target at q = 0 */
    uint16_t closed_position; /* calibrated target at q = 1 */
    uint16_t current_position;/* seeded from servo readback before motion */
} AT_TendonServo;

typedef struct {
    AT_TendonServo flexor;
    AT_TendonServo extensor;
} AT_Axis;

typedef struct {
    AT_Axis axes[AT_DOF_COUNT];
    uint16_t max_step_per_update;
    bool initialized;
} AT_Controller;

/* The transport adapter should issue one synchronized/batched position write
 * for all IDs. Return true only if the command was accepted by the bus. */
typedef bool (*AT_SyncWritePositions)(void *context,
                                      const uint8_t *ids,
                                      const uint16_t *positions,
                                      size_t count);

/* Initialize using measured present positions. IDs must be unique across all
 * 20 servos. max_step_per_update limits each servo's change per call. */
bool at_init(AT_Controller *controller,
             const AT_Axis axes[AT_DOF_COUNT],
             uint16_t max_step_per_update);

/* q[i] is the desired flexion for axis i: 0.0 = open, 1.0 = closed.
 * Both tendons are commanded from their individually calibrated endpoints. */
bool at_update(AT_Controller *controller,
               const float q[AT_DOF_COUNT],
               AT_SyncWritePositions sync_write,
               void *context);

#ifdef __cplusplus
}
#endif

#endif
