#pragma once

#include <Arduino.h>

#include <Adafruit_NeoPixel.h>

class LEDController {
public:
    enum class Mode {
        SOLID,
        BLINKING,
        PULSE,
        SPINNER,
        FULL_SPINNER,
    };

    enum class SubMode {
        NONE,
        SPINNER_SINGLE,
        SPINNER_TAIL,
        SPINNER_FULL,
    };

    // Struct for all permanent animations and their parameters
    // so we can have short animations and then return to the previous permanent animation.
    // Hint: As long as struct does not contain pointers, it can be copied and assigned easily.
    //       With pointers, we would need deepcopy.
    struct PermanentState {
        // Permanent specific parameters
        bool on; // Permanent power state (on/off)
        // Shared parameters
        Mode mode;
        uint32_t color;
        uint32_t step1_ms;   // blink on; spinner speed
        uint32_t step2_ms;   // blink off
        uint8_t mode_state;  // Blink current on/off state; spinner current position (0 to num_leds-1)
        bool mode_clockwise; // Spinner direction (true = clockwise, false = counter-clockwise)
        SubMode submode;     // Spinner mode (single, short tail, long tail, full)
    };

    // Struct for all temporary animations and their parameters
    // TODO: Nx blinking, Nx spinner
    struct AnimationState {
        // Shared parameters
        Mode mode;
        uint32_t color;
        uint32_t step1_ms;   // blink on; spinner speed
        uint32_t step2_ms;   // blink off
        uint8_t mode_state;  // Blink current on/off state; spinner current position (0 to num_leds-1)
        bool mode_clockwise; // Spinner direction (true = clockwise, false = counter-clockwise)
        SubMode submode;     // Spinner mode (single, short tail, long tail, full)
        // Transient specific parameters
        uint32_t n_steps; // Blink: 2xN (on/off); Spinner: N_LEDs * N (full rotation)
    };

    LEDController(uint8_t pin_ls, uint8_t pin_data, uint8_t num_leds);

    void begin();
    void on();
    void off();
    void fade_on(uint32_t hex_color, uint32_t duration_ms);
    void fade_off(uint32_t duration_ms);
    bool is_fading() const;
    // Inputs are normalized knob translation/tilt values in the range [-1, 1].
    void update(float input_x = 0.0f, float input_y = 0.0f, float input_rx = 0.0f, float input_ry = 0.0f);

    /*
        PERMANENT ANIMATION FUNCTIONS
    */

    void set_solid(uint32_t hex_color);
    void set_blinking(uint32_t hex_color, uint32_t on_duration_ms, uint32_t off_duration_ms);
    void set_pulse(uint32_t hex_color, uint32_t pulse_duration_ms);
    void set_spinner(uint32_t hex_color, uint32_t spin_speed_ms, bool clockwise = true, SubMode spinner_mode = SubMode::SPINNER_SINGLE);

    /*
        TEMPORARY ANIMATION FUNCTIONS
    */
    bool push_animation(AnimationState animation);
    bool pop_animation(AnimationState& animation);
    uint8_t get_queue_size();
    bool is_animation_running();
    bool is_queue_full();

    bool queue_solid_animation(uint32_t hex_color, uint32_t step1_ms);
    bool queue_blinking_animation(uint32_t hex_color, uint32_t on_duration_ms, uint32_t off_duration_ms, uint8_t n_repeats);
    bool queue_pulse_animation(uint32_t hex_color, uint32_t pulse_duration_ms, uint8_t n_repeats);
    bool queue_spinner_animation(uint32_t hex_color, uint32_t spin_speed_ms, uint8_t n_repeats, bool clockwise = true, SubMode spinner_mode = SubMode::SPINNER_SINGLE);

private:
    uint8_t m_pinLS;
    uint8_t m_pinData;
    uint8_t m_numLEDs;

    Adafruit_NeoPixel m_leds;

    // Track the last update time for any animation
    uint32_t m_last_update_time_ms{0};

    // Non-blocking power transition state. Pixel values are stored at their
    // rendered brightness so each channel can be interpolated independently.
    static constexpr uint8_t MAX_FADE_LEDS = 8;
    bool m_fade_active{false};
    bool m_fade_turn_off_when_complete{false};
    uint32_t m_fade_start_time_ms{0};
    uint32_t m_fade_duration_ms{0};
    uint8_t m_fade_start_rgb[MAX_FADE_LEDS][3]{};
    uint8_t m_fade_target_rgb[MAX_FADE_LEDS][3]{};

    // General helper functions
    bool is_on();
    void hex_to_rgb(uint32_t hex_color, uint8_t& r, uint8_t& g, uint8_t& b);
    void start_fade(uint32_t target_color, uint8_t target_brightness, uint32_t duration_ms, bool turn_off_when_complete);
    void update_fade();

    // Permanent animation state variables and functions
    PermanentState m_current_state;  // Store the current permanent state
    PermanentState m_previous_state; // Store the previous permanent state for restoration
    void save_current_state();       // Save the current state to m_previous_state
    void restore_previous_state();   // Restore the previous state from m_previous_state

    // Animation queue for temporary animations
    static constexpr uint8_t MAX_ANIMATION_QUEUE_SIZE = 10;
    AnimationState m_animation_queue[MAX_ANIMATION_QUEUE_SIZE];
    uint8_t m_queue_head{0};
    uint8_t m_queue_tail{0};
    uint8_t m_queue_size{0};

    // Animation state variables and functions
    bool m_animation_running{false};
    uint8_t m_remaining_steps{0}; // Track remaining steps for the current animation
    void enter_animation(AnimationState animation);
    void exit_animation();

    // Individual mode functions
    void update_solid();
    void update_input(float direction_x, float direction_y);
    void update_blinking();
    void update_pulse();
    uint8_t first_spinner_LED(bool clockwise);
    uint8_t next_spinner_LED(uint8_t current_LED, bool clockwise);
    void update_spinner();
};