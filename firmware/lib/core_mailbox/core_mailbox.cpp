#include "core_mailbox.h"

namespace CoreMailbox {
    namespace {

        volatile uint32_t g_raw_sequence = 0;
        volatile uint32_t g_filtered_sequence = 0;
        volatile uint32_t g_core1_sleeping = 0;
        volatile RawSensorData g_raw_data = {};
        volatile FilteredData g_filtered_data = {};

        template<size_t Size>
        void copyToVolatile(volatile float (&destination)[Size], const float (&source)[Size])
        {
            for (size_t index = 0; index < Size; ++index) {
                destination[index] = source[index];
            }
        }

        template<size_t Size>
        void copyFromVolatile(float (&destination)[Size], const volatile float (&source)[Size])
        {
            for (size_t index = 0; index < Size; ++index) {
                destination[index] = source[index];
            }
        }

    } // namespace

    void setCore1Sleeping(bool sleeping)
    {
        g_core1_sleeping = sleeping ? 1u : 0u;
        __dmb();
    }

    bool core1Sleeping()
    {
        return g_core1_sleeping != 0u;
    }

    void publishRaw(const float (&values)[9], uint32_t timestamp_us)
    {
        const uint32_t sequence = g_raw_sequence;
        g_raw_sequence = sequence + 1u;
        __dmb();

        copyToVolatile(g_raw_data.values, values);
        g_raw_data.timestamp_us = timestamp_us;

        __dmb();
        g_raw_sequence = sequence + 2u;
    }

    bool consumeRaw(uint32_t& last_sequence, RawSensorData& data)
    {
        uint32_t first_sequence = 0;
        uint32_t second_sequence = 0;

        do {
            first_sequence = g_raw_sequence;
            if (first_sequence & 1u) {
                continue;
            }

            __dmb();
            copyFromVolatile(data.values, g_raw_data.values);
            data.timestamp_us = g_raw_data.timestamp_us;
            __dmb();

            second_sequence = g_raw_sequence;
        } while (first_sequence != second_sequence || (second_sequence & 1u));

        if (second_sequence == 0 || second_sequence == last_sequence) {
            return false;
        }

        last_sequence = second_sequence;
        return true;
    }

    void publishFiltered(const float (&state)[12], float dt)
    {
        const uint32_t sequence = g_filtered_sequence;
        g_filtered_sequence = sequence + 1u;
        __dmb();

        copyToVolatile(g_filtered_data.state, state);
        g_filtered_data.dt = dt;

        __dmb();
        g_filtered_sequence = sequence + 2u;
    }

    bool consumeFiltered(uint32_t& last_sequence, FilteredData& data)
    {
        uint32_t first_sequence = 0;
        uint32_t second_sequence = 0;

        do {
            first_sequence = g_filtered_sequence;
            if (first_sequence & 1u) {
                continue;
            }

            __dmb();
            copyFromVolatile(data.state, g_filtered_data.state);
            data.dt = g_filtered_data.dt;
            __dmb();

            second_sequence = g_filtered_sequence;
        } while (first_sequence != second_sequence || (second_sequence & 1u));

        if (second_sequence == 0 || second_sequence == last_sequence) {
            return false;
        }

        last_sequence = second_sequence;
        return true;
    }

} // namespace CoreMailbox