#pragma once

#include <Arduino.h>

namespace CoreMailbox {

    struct RawSensorData {
        float values[9];
        uint32_t timestamp_us;
    };

    struct FilteredData {
        float state[12];
        float dt;
    };

    void setCore1Sleeping(bool sleeping);
    bool core1Sleeping();

    void publishRaw(const float (&values)[9], uint32_t timestamp_us);
    bool consumeRaw(uint32_t& last_sequence, RawSensorData& data);

    void publishFiltered(const float (&state)[12], float dt);
    bool consumeFiltered(uint32_t& last_sequence, FilteredData& data);

} // namespace CoreMailbox