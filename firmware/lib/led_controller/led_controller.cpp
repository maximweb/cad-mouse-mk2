#include "config.h"

#include "led_controller.h"

#include <math.h>

namespace {
    // D1 is at 11 o'clock and the LED indices advance counter-clockwise.
    // These unit vectors point from the center of the ring toward D1-D8.
    constexpr float LED_DIRECTION_X[8] = {
      -0.38268343f,
      -0.92387953f,
      -0.92387953f,
      -0.38268343f,
      0.38268343f,
      0.92387953f,
      0.92387953f,
      0.38268343f,
    };
    constexpr float LED_DIRECTION_Y[8] = {
      0.92387953f,
      0.38268343f,
      -0.38268343f,
      -0.92387953f,
      -0.92387953f,
      -0.38268343f,
      0.38268343f,
      0.92387953f,
    };

    float clamp_unit(float value)
    {
        if (value < 0.0f) {
            return 0.0f;
        }
        if (value > 1.0f) {
            return 1.0f;
        }
        return value;
    }

    uint8_t lerp_channel(uint8_t from, uint8_t to, float amount)
    {
        return static_cast<uint8_t>(from + (to - from) * amount + 0.5f);
    }

    uint8_t apply_brightness(uint8_t channel, uint8_t brightness)
    {
        // Match Adafruit_NeoPixel::setBrightness(), including its 255 = full
        // brightness behavior, while allowing a different value per LED.
        return static_cast<uint16_t>(channel) * (brightness + 1u) >> 8;
    }
}

/*
    GENERAL FUNCTIONS
*/

LEDController::LEDController(uint8_t pin_ls, uint8_t pin_data, uint8_t num_leds)
: m_pinLS(pin_ls)
, m_pinData(pin_data)
, m_numLEDs(num_leds)
, m_leds(num_leds, pin_data, NEO_GRB + NEO_KHZ800)
{
}

void LEDController::begin()
{
    digitalWrite(m_pinLS, LOW);
    pinMode(m_pinLS, OUTPUT);
    digitalWrite(m_pinData, LOW);

    m_leds.begin();
    m_leds.setBrightness(LED_BRIGHTNESS);
    m_leds.show();

    m_current_state = PermanentState{
      .on = false,
      .mode = Mode::SOLID,
      .color = 0x000000,
      .step1_ms = 0,
      .step2_ms = 0,
      .mode_state = 0,
      .mode_clockwise = true,
      .submode = SubMode::NONE,
    };
}

void LEDController::on()
{
    m_fade_active = false;
    digitalWrite(m_pinLS, HIGH);
    m_last_update_time_ms = 0; // Reset so that when turned back on, the animation starts fresh
    m_current_state.on = true; // Update the permanent state to reflect that the LEDs are on
    delay(10);                 // Wait for the latch signal to stabilize
}

void LEDController::off()
{
    m_fade_active = false;
    digitalWrite(m_pinLS, LOW);
    m_last_update_time_ms = 0;  // Reset so that when turned back on, the animation starts fresh
    m_current_state.on = false; // Update the permanent state to reflect that the LEDs are off
    delay(10);                  // Wait for the latch signal to stabilize
}

void LEDController::fade_on(uint32_t hex_color, uint32_t duration_ms)
{
    m_current_state.mode = Mode::SOLID;
    m_current_state.color = hex_color;
    m_current_state.step1_ms = 0;
    m_current_state.step2_ms = 0;
    m_current_state.mode_state = 0;
    m_current_state.mode_clockwise = true;
    m_current_state.submode = SubMode::NONE;

    // The completed fade-out leaves zeroes in the pixel buffer, so powering
    // the ring back up cannot flash the previous running color.
    if (!is_on()) {
        on();
    }
    start_fade(hex_color, LED_BRIGHTNESS, duration_ms, false);
}

void LEDController::fade_off(uint32_t duration_ms)
{
    if (!is_on()) {
        return;
    }
    start_fade(0x000000, 0, duration_ms, true);
}

bool LEDController::is_fading() const
{
    return m_fade_active;
}

void LEDController::start_fade(uint32_t target_color, uint8_t target_brightness, uint32_t duration_ms, bool turn_off_when_complete)
{
    uint8_t target_r, target_g, target_b;
    hex_to_rgb(target_color, target_r, target_g, target_b);
    target_r = apply_brightness(target_r, target_brightness);
    target_g = apply_brightness(target_g, target_brightness);
    target_b = apply_brightness(target_b, target_brightness);

    // Capture the bytes currently being sent to the LEDs before changing the
    // strip's global brightness. This also allows a fade to reverse smoothly.
    for (uint8_t i = 0; i < m_numLEDs && i < MAX_FADE_LEDS; ++i) {
        const uint32_t color = m_leds.getPixelColor(i);
        m_fade_start_rgb[i][0] = (color >> 16) & 0xFF;
        m_fade_start_rgb[i][1] = (color >> 8) & 0xFF;
        m_fade_start_rgb[i][2] = color & 0xFF;
        m_fade_target_rgb[i][0] = target_r;
        m_fade_target_rgb[i][1] = target_g;
        m_fade_target_rgb[i][2] = target_b;
    }

    m_leds.setBrightness(255);
    m_fade_start_time_ms = millis();
    m_fade_duration_ms = duration_ms;
    m_fade_turn_off_when_complete = turn_off_when_complete;
    m_fade_active = true;
}

void LEDController::update_fade()
{
    const uint32_t elapsed_ms = millis() - m_fade_start_time_ms;
    const float progress = m_fade_duration_ms == 0
                             ? 1.0f
                             : clamp_unit(static_cast<float>(elapsed_ms) / m_fade_duration_ms);

    for (uint8_t i = 0; i < m_numLEDs && i < MAX_FADE_LEDS; ++i) {
        m_leds.setPixelColor(i, m_leds.Color(
                                  lerp_channel(m_fade_start_rgb[i][0], m_fade_target_rgb[i][0], progress),
                                  lerp_channel(m_fade_start_rgb[i][1], m_fade_target_rgb[i][1], progress),
                                  lerp_channel(m_fade_start_rgb[i][2], m_fade_target_rgb[i][2], progress)));
    }
    m_leds.show();

    if (progress >= 1.0f) {
        const bool turn_off = m_fade_turn_off_when_complete;
        m_fade_active = false;
        if (turn_off) {
            off();
        }
    }
}

/*
    SOLID MODE FUNCTIONS
*/

void LEDController::set_solid(uint32_t hex_color)
{
    if (!m_animation_running) {
        m_current_state.mode = Mode::SOLID;
        m_current_state.color = hex_color;
        m_current_state.step1_ms = 0;            // Not used for solid
        m_current_state.step2_ms = 0;            // Not used for solid
        m_current_state.mode_clockwise = true;   // Not used for solid
        m_current_state.mode_state = 0;          // Not used for solid
        m_current_state.submode = SubMode::NONE; // Not used for solid
        // No need to reset m_last_update_time_ms here,
        // as permanent solid mode doesn't rely on timing for updates.

        update_solid();
    }
    else {
        // If an animation is running, we do not interrupt animation,
        // but amend the stored permanent state.
        m_previous_state.mode = Mode::SOLID;
        m_previous_state.color = hex_color;
        m_previous_state.step1_ms = 0;            // Not used for solid
        m_previous_state.step2_ms = 0;            // Not used for solid
        m_previous_state.mode_clockwise = true;   // Not used for solid
        m_previous_state.mode_state = 0;          // Not used for solid
        m_previous_state.submode = SubMode::NONE; // Not used for solid
    }
}

bool LEDController::queue_solid_animation(uint32_t hex_color, uint32_t step1_ms)
{
    AnimationState animation;
    animation.mode = Mode::SOLID;
    animation.color = hex_color;
    animation.step1_ms = step1_ms;
    animation.step2_ms = 0;            // Not used for solid
    animation.mode_clockwise = true;   // Not used for solid
    animation.mode_state = 0;          // Not used for solid
    animation.submode = SubMode::NONE; // Not used for solid
    animation.n_steps = 2;

    return push_animation(animation);
}

void LEDController::update_solid()
{
    m_leds.setBrightness(LED_BRIGHTNESS);

    if (m_animation_running) {
        const uint32_t now = millis();
        if (now - m_last_update_time_ms > m_current_state.step1_ms) {
            if (m_remaining_steps > 0) {
                m_remaining_steps--;
            }

            if (m_remaining_steps == 0) {
                // m_last_update_time_ms = now;
                return; // Animation completed, nothing to do
            }
        }
    }

    uint8_t r, g, b;
    hex_to_rgb(m_current_state.color, r, g, b);

    // Fill all LEDs with the specified color
    for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
        m_leds.setPixelColor(i, m_leds.Color(r, g, b));
    }

    m_leds.show();
}

void LEDController::update_input(float input_x, float input_y)
{
    const float magnitude = clamp_unit(sqrtf(input_x * input_x + input_y * input_y));

    uint8_t running_r, running_g, running_b;
    uint8_t input_r, input_g, input_b;
    hex_to_rgb(LED_RUNNING_COLOR, running_r, running_g, running_b);
    hex_to_rgb(LED_INPUT_GLOW_COLOR, input_r, input_g, input_b);

    // Per-pixel brightness is encoded into the RGB channels so LEDs can have
    // different brightness levels. Global brightness must therefore be 255.
    m_leds.setBrightness(255);

    float direction_x = 0.0f;
    float direction_y = 0.0f;
    float strongest_projection = 1.0f;
    if (magnitude > 0.0f) {
        direction_x = input_x / magnitude;
        direction_y = input_y / magnitude;

        // Normalize against the best-aligned LED. A full right input therefore
        // drives both right-side LEDs (D6 and D7) all the way to the target.
        strongest_projection = 0.0f;
        for (uint8_t i = 0; i < m_numLEDs && i < 8; ++i) {
            const float projection = direction_x * LED_DIRECTION_X[i] + direction_y * LED_DIRECTION_Y[i];
            if (projection > strongest_projection) {
                strongest_projection = projection;
            }
        }
    }

    for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
        float influence = 0.0f;
        if (magnitude > 0.0f && i < 8) {
            const float projection = direction_x * LED_DIRECTION_X[i] + direction_y * LED_DIRECTION_Y[i];
            influence = clamp_unit(magnitude * projection / strongest_projection);
        }

        const uint8_t brightness = lerp_channel(LED_BRIGHTNESS, LED_INPUT_GLOW_MAX_BRIGHTNESS, influence);
        const uint8_t r = lerp_channel(running_r, input_r, influence);
        const uint8_t g = lerp_channel(running_g, input_g, influence);
        const uint8_t b = lerp_channel(running_b, input_b, influence);

        m_leds.setPixelColor(i, m_leds.Color(
                                  apply_brightness(r, brightness),
                                  apply_brightness(g, brightness),
                                  apply_brightness(b, brightness)));
    }

    m_leds.show();
}

/*
    PERMANENT BLINKING MODE FUNCTIONS
*/

void LEDController::set_blinking(uint32_t hex_color, uint32_t on_duration_ms, uint32_t off_duration_ms)
{
    if (!m_animation_running) {
        m_current_state.mode = Mode::BLINKING;
        m_current_state.mode_state = 1; // Start with LEDs ON
        m_current_state.color = hex_color;
        m_current_state.step1_ms = on_duration_ms;
        m_current_state.step2_ms = off_duration_ms;
        m_last_update_time_ms = 0; // Reset the last update time to ensure immediate effect

        update_blinking(); // Immediately update to set the initial state
    }
    else {
        // If an animation is running, we do not interrupt animation,
        // but amend the stored permanent state.
        m_previous_state.mode = Mode::BLINKING;
        m_previous_state.mode_state = 1; // Start with LEDs ON
        m_previous_state.color = hex_color;
        m_previous_state.step1_ms = on_duration_ms;
        m_previous_state.step2_ms = off_duration_ms;
    }
}

void LEDController::update_blinking()
{
    m_leds.setBrightness(LED_BRIGHTNESS);

    uint32_t now = millis();
    if (now - m_last_update_time_ms >= (m_current_state.mode_state ? m_current_state.step1_ms : m_current_state.step2_ms)) {
        // Handle decrementing remaining steps for temporary animations
        if (m_animation_running && m_remaining_steps > 0) {
            m_remaining_steps--;

            // Skip actions if the animation has completed
            if (m_remaining_steps == 0) {
                return;
            }
        }

        // Blink logic
        uint8_t r, g, b;
        if (m_current_state.mode_state) {
            // Turn on LEDs with the specified color
            hex_to_rgb(m_current_state.color, r, g, b);
            for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
                m_leds.setPixelColor(i, m_leds.Color(r, g, b));
            }
        }
        else {
            // Blinking OFF state
            // "Off" depends on whether the animation is running or not.
            if (!m_animation_running) {
                r = g = b = 0; // Turn off
            }
            else {
                hex_to_rgb(m_previous_state.color, r, g, b); // Retained color
            }
            for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
                m_leds.setPixelColor(i, m_leds.Color(r, g, b));
            }
        }

        m_current_state.mode_state = m_current_state.mode_state == 1 ? 0 : 1; // Toggle state
        m_last_update_time_ms = now;

        m_leds.show();
    }
}

/*
    PULSE MODE FUNCTIONS
*/

void LEDController::set_pulse(uint32_t hex_color, uint32_t pulse_duration_ms)
{
    if (!m_animation_running) {
        m_current_state.mode = Mode::PULSE;
        m_current_state.color = hex_color;
        m_current_state.mode_state = 0;                         // Start at the beginning of the pulse
        m_current_state.mode_clockwise = true;                  // Not used for pulse
        m_current_state.submode = SubMode::NONE;                // Not used for pulse
        m_current_state.step1_ms = pulse_duration_ms / 20 * 20; // Total duration, ensure multiple of 10ms for smoothness
        m_current_state.step2_ms = 20;                          // Step duration for each brightness level
        m_last_update_time_ms = 0;                              // Reset the last update time to ensure immediate effect

        update_pulse();
    }
    else {
        // If an animation is running, we do not interrupt animation,
        // but amend the stored permanent state.
        m_previous_state.mode = Mode::PULSE;
        m_previous_state.color = hex_color;
        m_previous_state.mode_state = 0;                         // Start at the beginning of the pulse
        m_previous_state.mode_clockwise = true;                  // Not used for pulse
        m_previous_state.submode = SubMode::NONE;                // Not used for pulse
        m_previous_state.step1_ms = pulse_duration_ms / 20 * 20; // Total duration, ensure multiple of 10ms for smoothness
        m_previous_state.step2_ms = 20;                          // Step duration for each brightness level
    }
}

bool LEDController::queue_pulse_animation(uint32_t hex_color, uint32_t pulse_duration_ms, uint8_t n_repeats)
{
    AnimationState animation;
    animation.mode = Mode::PULSE;
    animation.color = hex_color;
    animation.step1_ms = pulse_duration_ms / 20 * 20;         // Total duration, ensure multiple of 20ms for smoothness
    animation.step2_ms = 20;                                  // Step duration for each brightness level
    animation.mode_clockwise = true;                          // Not used for pulse
    animation.mode_state = 0;                                 // Start at the beginning of the pulse
    animation.submode = SubMode::NONE;                        // Not used for pulse
    animation.n_steps = n_repeats * (pulse_duration_ms / 20); // Total steps for the specified number of repeats

    return push_animation(animation);
}

void LEDController::update_pulse()
{
    m_leds.setBrightness(LED_BRIGHTNESS);

    uint32_t now = millis();
    if (now - m_last_update_time_ms >= m_current_state.step2_ms) {
        // Handle decrementing remaining steps for temporary animations
        if (m_animation_running && m_remaining_steps > 0) {
            m_remaining_steps--;

            if (m_remaining_steps == 0) {
                // Animation completed, nothing to do
                return;
            }
        }

        // Calculate brightness
        // We start at 10% brightness, increase to standard brightness (100%),
        // then decrease back to 10%.
        uint8_t steps = m_current_state.step1_ms / m_current_state.step2_ms; // Total steps for a full pulse cycle
        float brightness_factor;
        if (m_current_state.mode_state < steps / 2) {
            // Increasing brightness
            brightness_factor = 0.1f + 0.9f * (static_cast<float>(m_current_state.mode_state) / (steps / 2));
        }
        else {
            // Decreasing brightness
            brightness_factor = 1.0f - 0.9f * (static_cast<float>(m_current_state.mode_state - steps / 2) / (steps / 2));
        }

        // Set the color with adjusted brightness
        uint8_t r, g, b;
        hex_to_rgb(m_current_state.color, r, g, b);
        r = static_cast<uint8_t>(r * brightness_factor);
        g = static_cast<uint8_t>(g * brightness_factor);
        b = static_cast<uint8_t>(b * brightness_factor);
        for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
            m_leds.setPixelColor(i, m_leds.Color(r, g, b));
        }
        // Update the mode state for the next call
        m_current_state.mode_state = (m_current_state.mode_state + 1) % steps; // Cycle through 0-(steps-1) for a full pulse cycle

        m_leds.show();
        m_last_update_time_ms = now;
    }
}

/*
    SPINNER MODE FUNCTIONS
*/

uint8_t LEDController::first_spinner_LED(bool clockwise)
{
    return clockwise ? m_numLEDs - 1 : 0;
}

uint8_t LEDController::next_spinner_LED(uint8_t current_LED, bool clockwise)
{
    // LEDs indices are 0-based counter clockwise.
    if (clockwise) {
        return (current_LED + m_numLEDs - 1) % m_numLEDs;
    }
    else {
        return (current_LED + 1) % m_numLEDs;
    }
}

void LEDController::set_spinner(uint32_t hex_color, uint32_t spin_speed_ms, bool clockwise, SubMode submode)
{
    if (!m_animation_running) {
        m_current_state.mode = Mode::SPINNER;
        m_current_state.color = hex_color;
        m_current_state.step1_ms = spin_speed_ms;
        m_current_state.step2_ms = 0; // Not used for spinner
        m_current_state.mode_clockwise = clockwise;
        m_current_state.mode_state = first_spinner_LED(clockwise);
        m_current_state.submode = submode;
        m_last_update_time_ms = 0;

        update_spinner();
    }
    else {
        // If an animation is running, we do not interrupt animation,
        // but amend the stored permanent state.
        m_previous_state.mode = Mode::SPINNER;
        m_previous_state.color = hex_color;
        m_previous_state.step1_ms = spin_speed_ms;
        m_previous_state.step2_ms = 0; // Not used for spinner
        m_previous_state.mode_clockwise = clockwise;
        m_previous_state.mode_state = first_spinner_LED(clockwise);
        m_previous_state.submode = submode;
    }
}

bool LEDController::queue_spinner_animation(uint32_t hex_color, uint32_t spin_speed_ms, uint8_t n_repeats, bool clockwise, SubMode submode)
{
    AnimationState animation;
    animation.mode = Mode::SPINNER;
    animation.color = hex_color;
    animation.step1_ms = spin_speed_ms;
    animation.step2_ms = 0; // Not used for spinner
    animation.mode_clockwise = clockwise;
    animation.mode_state = first_spinner_LED(clockwise);
    animation.submode = submode;
    animation.n_steps = n_repeats * m_numLEDs + 1;

    return push_animation(animation);
}

void LEDController::update_spinner()
{
    m_leds.setBrightness(LED_BRIGHTNESS);

    uint32_t now = millis();
    if (now - m_last_update_time_ms >= m_current_state.step1_ms) {
        // Handle decrementing remaining steps for temporary animations
        if (m_animation_running && m_remaining_steps > 0) {
            m_remaining_steps--;

            if (m_remaining_steps == 0) {
                // Animation completed, nothing to do
                return;
            }
        }

        // Clear all LEDs
        for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
            m_leds.setPixelColor(i, 0); // Turn off
        }

        // Set the spinner LED to the specified color
        uint8_t r, g, b;
        hex_to_rgb(m_current_state.color, r, g, b);
        m_leds.setPixelColor(m_current_state.mode_state, m_leds.Color(r, g, b));

        // Submode determines other LEDs
        switch (m_current_state.submode) {
            case SubMode::SPINNER_SINGLE: {
                // Only the spinner LED is lit, nothing else to do
                break;
            }

            case SubMode::SPINNER_TAIL: {
                // Light up 2 LEDs behind the spinner with decreasing brightness
                uint8_t tail1 = next_spinner_LED(m_current_state.mode_state, !m_current_state.mode_clockwise);
                uint8_t tail2 = next_spinner_LED(tail1, !m_current_state.mode_clockwise);
                m_leds.setPixelColor(tail1, m_leds.Color(r / 10, g / 10, b / 10));
                m_leds.setPixelColor(tail2, m_leds.Color(r / 10, g / 10, b / 10));
                break;
            }
            case SubMode::SPINNER_FULL: {
                // Light up all other LEDs with half brightness
                for (uint8_t i = 0; i < m_numLEDs; ++i) {
                    if (i != m_current_state.mode_state) {
                        m_leds.setPixelColor(i, m_leds.Color(r / 4, g / 4, b / 4)); // 25% brightness
                    }
                }
                break;
            }

            default:
                // Unknown submode, do nothing
                break;
        }

        // Update spinner position for the next call
        m_current_state.mode_state = next_spinner_LED(m_current_state.mode_state, m_current_state.mode_clockwise);

        m_leds.show();
        m_last_update_time_ms = now;
    }
}

/*
    ANIMATION QUEUE FUNCTIONS
*/

bool LEDController::push_animation(AnimationState animation)
{
    if (m_queue_size >= MAX_ANIMATION_QUEUE_SIZE) {
        return false; // Queue is full
    }

    m_animation_queue[m_queue_tail] = animation;
    m_queue_tail = (m_queue_tail + 1) % MAX_ANIMATION_QUEUE_SIZE;
    m_queue_size++;
    return true;
}

bool LEDController::pop_animation(AnimationState& animation)
{
    if (m_queue_size == 0) {
        return false; // Queue is empty
    }

    animation = m_animation_queue[m_queue_head];
    m_queue_head = (m_queue_head + 1) % MAX_ANIMATION_QUEUE_SIZE;
    m_queue_size--;
    return true;
}

uint8_t LEDController::get_queue_size()
{
    return m_queue_size;
}

bool LEDController::is_animation_running()
{
    return m_animation_running;
}

bool LEDController::is_queue_full()
{
    return m_queue_size >= MAX_ANIMATION_QUEUE_SIZE;
}

/*
    ANIMATION FUNCTIONS
*/

void LEDController::enter_animation(AnimationState animation)
{
    // Save permanent state if not already running an animation
    if (!m_animation_running) {
        save_current_state();
        m_animation_running = true;
    }

    // Set the current state to the animation parameters
    m_current_state.mode = animation.mode;
    m_current_state.color = animation.color;
    m_current_state.step1_ms = animation.step1_ms;
    m_current_state.step2_ms = animation.step2_ms;
    m_current_state.mode_clockwise = animation.mode_clockwise;
    m_current_state.mode_state = animation.mode_state;
    m_current_state.submode = animation.submode;

    // Set the remaining repeats for the animation
    m_remaining_steps = animation.n_steps;

    // Reset the last update time to ensure immediate effect
    m_last_update_time_ms = 0;
    // As solid as only a single step, we must set the last update time to now to avoid immediate exit of the animation
    if (m_current_state.mode == Mode::SOLID) {
        m_last_update_time_ms = millis();
    }
}

void LEDController::exit_animation()
{
    // Restore the previous permanent state
    restore_previous_state();
    m_animation_running = false;

    // Reset the last update time to ensure immediate effect
    // m_last_update_time_ms = 0;

    // Set power, rest is handled by the update function
    if (m_current_state.on) {
        on();
    }
    else {
        off();
    }
}

bool LEDController::queue_blinking_animation(uint32_t hex_color, uint32_t on_duration_ms, uint32_t off_duration_ms, uint8_t n_repeats)
{
    AnimationState animation;
    animation.mode = Mode::BLINKING;
    animation.color = hex_color;
    animation.step1_ms = on_duration_ms;
    animation.step2_ms = off_duration_ms;
    animation.mode_clockwise = true;       // Not used for blinking
    animation.mode_state = 1;              // Start with LEDs on
    animation.submode = SubMode::NONE;     // Not used for blinking
    animation.n_steps = n_repeats * 2 + 1; // N * on/off

    return push_animation(animation);
}

/*
    UPDATE FUNCTION
*/

void LEDController::update(float input_x, float input_y, float input_rx, float input_ry)
{
    if (m_fade_active) {
        update_fade();
        return;
    }

    // Save some cycles if off
    if (!is_on()) {
        return;
    }

    // Handle animation queue start
    if (!m_animation_running && m_queue_size > 0) {
        // Pop the next animation from the queue and enter it
        AnimationState next_animation;
        if (pop_animation(next_animation)) {
            enter_animation(next_animation);
        }
    }

    if (LED_USE_INPUT_GLOW_EFFECT && !m_animation_running && m_current_state.mode == Mode::SOLID && m_current_state.color == LED_RUNNING_COLOR) {
        // A rightward translation (+X) and a rightward tilt (+RY) point to the
        // same LEDs. Likewise, a forward/up-ring tilt is represented by -RX.
        update_input(input_x + input_ry, input_y - input_rx);
    }
    else if (m_current_state.mode == Mode::SOLID) {
        update_solid();
    }
    else if (m_current_state.mode == Mode::BLINKING) {
        update_blinking();
    }
    else if (m_current_state.mode == Mode::PULSE) {
        update_pulse();
    }
    else if (m_current_state.mode == Mode::SPINNER) {
        update_spinner();
    }

    // Handle animation completion and restoration of previous state
    if (m_animation_running && m_remaining_steps == 0) {
        // Animation completed, check if there are more animations in the queue
        AnimationState next_animation;
        if (pop_animation(next_animation)) {
            // Start the next animation
            enter_animation(next_animation);
        }
        else {
            // No more animations, exit the current animation and restore the previous state
            exit_animation();

            // Recheck restored power state and return if off
            if (!is_on()) {
                return;
            }
        }
    }
}

/*
    HELPER FUNCTION
*/

void LEDController::save_current_state()
{
    m_previous_state = m_current_state;
}

void LEDController::restore_previous_state()
{
    m_current_state = m_previous_state;
}

bool LEDController::is_on()
{
    return m_current_state.on;
}

void LEDController::hex_to_rgb(uint32_t hex_color, uint8_t& r, uint8_t& g, uint8_t& b)
{
    // Clamp the hex_color to ensure it's within the valid range
    if (hex_color > 0xFFFFFF) {
        hex_color = 0xFFFFFF;
    }

    r = (hex_color >> 16) & 0xFF;
    g = (hex_color >> 8) & 0xFF;
    b = hex_color & 0xFF;
}