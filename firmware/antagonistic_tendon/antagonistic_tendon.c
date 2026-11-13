#include "antagonistic_tendon.h"

#include <math.h>
#include <string.h>

static bool position_valid(uint16_t position) {
    return position <= AT_SERVO_POSITION_MAX;
}

static bool servo_valid(const AT_TendonServo *servo) {
    return servo != NULL && servo->id != 0U &&
           position_valid(servo->open_position) &&
           position_valid(servo->closed_position) &&
           position_valid(servo->current_position);
}

static uint16_t interpolate(uint16_t open_position,
                            uint16_t closed_position,
                            float q) {
    const float value = (float)open_position +
                        ((float)closed_position - (float)open_position) * q;
    return (uint16_t)(value + 0.5f);
}

static uint16_t slew_toward(uint16_t current,
                            uint16_t target,
                            uint16_t max_step) {
    if (target > current) {
        const uint16_t difference = (uint16_t)(target - current);
        return difference > max_step ? (uint16_t)(current + max_step) : target;
    }
    if (current > target) {
        const uint16_t difference = (uint16_t)(current - target);
        return difference > max_step ? (uint16_t)(current - max_step) : target;
    }
    return current;
}

bool at_init(AT_Controller *controller,
             const AT_Axis axes[AT_DOF_COUNT],
             uint16_t max_step_per_update) {
    if (controller == NULL || axes == NULL || max_step_per_update == 0U) {
        return false;
    }

    for (size_t i = 0; i < AT_DOF_COUNT; ++i) {
        if (!servo_valid(&axes[i].flexor) || !servo_valid(&axes[i].extensor) ||
            axes[i].flexor.id == axes[i].extensor.id) {
            return false;
        }
        for (size_t j = 0; j < i; ++j) {
            if (axes[i].flexor.id == axes[j].flexor.id ||
                axes[i].flexor.id == axes[j].extensor.id ||
                axes[i].extensor.id == axes[j].flexor.id ||
                axes[i].extensor.id == axes[j].extensor.id) {
                return false;
            }
        }
    }

    memcpy(controller->axes, axes, sizeof(controller->axes));
    controller->max_step_per_update = max_step_per_update;
    controller->initialized = true;
    return true;
}

bool at_update(AT_Controller *controller,
               const float q[AT_DOF_COUNT],
               AT_SyncWritePositions sync_write,
               void *context) {
    if (controller == NULL || q == NULL || sync_write == NULL ||
        !controller->initialized || controller->max_step_per_update == 0U) {
        return false;
    }

    uint8_t ids[AT_SERVO_COUNT];
    uint16_t next_positions[AT_SERVO_COUNT];
    size_t index = 0U;

    /* Validate the complete command before sending anything to the bus. */
    for (size_t i = 0; i < AT_DOF_COUNT; ++i) {
        if (!isfinite(q[i]) || q[i] < 0.0f || q[i] > 1.0f) {
            return false;
        }

        const AT_TendonServo *servos[2] = {
            &controller->axes[i].flexor,
            &controller->axes[i].extensor,
        };
        for (size_t side = 0; side < 2U; ++side) {
            const AT_TendonServo *servo = servos[side];
            const uint16_t target = interpolate(servo->open_position,
                                                servo->closed_position,
                                                q[i]);
            ids[index] = servo->id;
            next_positions[index] = slew_toward(
                servo->current_position, target,
                controller->max_step_per_update);
            ++index;
        }
    }

    if (!sync_write(context, ids, next_positions, AT_SERVO_COUNT)) {
        return false;
    }

    index = 0U;
    for (size_t i = 0; i < AT_DOF_COUNT; ++i) {
        controller->axes[i].flexor.current_position = next_positions[index++];
        controller->axes[i].extensor.current_position = next_positions[index++];
    }
    return true;
}
