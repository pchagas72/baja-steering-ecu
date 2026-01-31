#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ssd1309_interface.h"
#include "can_management.h"

// Hardware configurations
// Check the can_management.h and ssd1309_interface.h for CAN and I2C
#define PIN_BUTTON      GPIO_NUM_0
#define TAG             "DASH_MAIN"

// Settings
#define FILTER_ALPHA    0.1f  // 0.1 = Smooth/Slow, 1.0 = Instant/Jittery
#define TIMEOUT_MS      1500  // Time before "NO DATA" error in ms

// Different screen modes
typedef enum {
    MODE_PILOT = 0,
    MODE_ENGINEER,
    MODE_ADVENTURE,
    MODE_NIGHT,
    MODE_COUNT
} dash_mode_t;

static dash_mode_t current_mode = MODE_PILOT;
static uint8_t s_buffer[SSD1309_BUFFER_SIZE];

// Filtering (keep these as floats)
typedef struct {
    float rpm;
    float fuel;
} filter_t;

filter_t signal_filter = {0};

// Graphics

// Draw Arc (Bresenham-ish approximation)
void draw_arc(uint8_t *fb, int cx, int cy, int r, int start_angle, int end_angle) {
    float step = 10.0f; 
    float prev_x = -1, prev_y = -1;
    for (float a = start_angle; a >= end_angle; a -= step) {
        float rad = a * (3.14159f / 180.0f);
        int x = cx + (int)(r * cos(rad));
        int y = cy - (int)(r * sin(rad));
        if (prev_x != -1) ssd1309_draw_line(fb, (int)prev_x, (int)prev_y, x, y, 1);
        prev_x = x; prev_y = y;
    }
}

// Analog Gauge, useful for the night mode
void draw_analog_gauge(uint8_t *fb, int cx, int cy, int r, float val, float max_val, const char* label) {
    int start_deg = 220;
    int end_deg = -40;
    int total_sweep = start_deg - end_deg;

    // 1. Arc
    draw_arc(fb, cx, cy, r, start_deg, end_deg);

    // 2. Ticks
    int num_ticks = 5;
    for (int i = 0; i <= num_ticks; i++) {
        float angle = start_deg - (i * (total_sweep / (float)num_ticks));
        float rad = angle * (3.14159f / 180.0f);
        int x0 = cx + (int)(r * cos(rad));
        int y0 = cy - (int)(r * sin(rad));
        int len = (i==0 || i==num_ticks) ? 6 : 3;
        int x1 = cx + (int)((r - len) * cos(rad));
        int y1 = cy - (int)((r - len) * sin(rad));
        ssd1309_draw_line(fb, x0, y0, x1, y1, 1);
    }

    // 3. Needle
    float angle = start_deg - ((val / max_val) * total_sweep);
    if (angle > start_deg) angle = start_deg;
    if (angle < end_deg) angle = end_deg;
    float rad = angle * (3.14159f / 180.0f);
    int tip_x = cx + (int)((r - 2) * cos(rad));
    int tip_y = cy - (int)((r - 2) * sin(rad));
    ssd1309_draw_line(fb, cx, cy, tip_x, tip_y, 1);
    
    // 4. Hub & Text
    ssd1309_draw_rect(fb, cx-2, cy-2, 5, 5, 1, 1);
    int txt_x = (val < 10) ? cx-3 : (val < 100) ? cx-6 : cx-9;
    ssd1309_draw_string(fb, txt_x, cy+6, "%.0f", val);
    ssd1309_draw_string(fb, cx-8, cy-8, label);
}

// Drawing Functions

// Pilot feedback
void draw_pilot(uint8_t *fb, car_state_t *car) {
    ssd1309_clear_buffer(fb);
    // Big Digital Speed
    ssd1309_draw_string_large(fb, 45, 10, 4, "%d", car->speed);
    ssd1309_draw_string(fb, 95, 40, "km/h");
    
    // Simple RPM Bar
    ssd1309_draw_rect(fb, 0, 0, 128, 8, 1, 0); 
    int bar_w = (car->rpm * 126) / 3800;
    if(bar_w > 126) bar_w = 126;
    for(int i=2; i<bar_w; i+=2) ssd1309_draw_rect(fb, i, 2, 1, 4, 1, 1);
    
    // Warnings
    if(car->cvt_temp > 85) ssd1309_draw_string(fb, 45, 54, "CVT HOT!");
    else ssd1309_draw_string(fb, 5, 54, "CVT:%d", car->cvt_temp);
}

// Heavy data mode
void draw_engineer(uint8_t *fb, car_state_t *car) {
    ssd1309_clear_buffer(fb);
    ssd1309_draw_string(fb, 0, 0, "SYSTEM: ONLINE");
    ssd1309_draw_line(fb, 0, 10, 128, 10, 1);
    
    ssd1309_draw_string(fb, 0,  15, "RPM:  %d", car->rpm);
    ssd1309_draw_string(fb, 64, 15, "SPD:  %d", car->speed);
    ssd1309_draw_string(fb, 0,  28, "ENG:  %d C", car->eng_temp);
    ssd1309_draw_string(fb, 64, 28, "CVT:  %d C", car->cvt_temp);
    ssd1309_draw_string(fb, 0,  41, "BAT:  %.1fV", car->voltage);
    ssd1309_draw_string(fb, 64, 41, "FUEL: %d%%", car->fuel);
    ssd1309_draw_string(fb, 0, 54, "R:%d P:%d", car->roll, car->pitch);
}

// Adventure mode, add more data here
void draw_adventure(uint8_t *fb, car_state_t *car) {
    ssd1309_clear_buffer(fb);
    int cx = 64, cy = 32;
    // Scale: 100 = 10.0 degrees
    float roll_rad = (car->roll / 10.0f) * (3.14159f / 180.0f); 
    int pitch_offset = (car->pitch / 10.0f); 
    
    // Calculate Horizon Line
    float cos_a = cos(roll_rad);
    float sin_a = sin(roll_rad);
    int len = 80; 
    int x0 = cx - (int)(len * cos_a);
    int y0 = (cy + pitch_offset) + (int)(len * sin_a);
    int x1 = cx + (int)(len * cos_a);
    int y1 = (cy + pitch_offset) - (int)(len * sin_a);
    
    ssd1309_draw_line(fb, x0, y0, x1, y1, 1);
    ssd1309_draw_line(fb, 60, 0, 68, 0, 1); // Sky Ref
    ssd1309_draw_line(fb, 44, 32, 84, 32, 1); // Wings
    
    ssd1309_draw_string(fb, 0, 56, "P:%d", car->pitch/10);
    ssd1309_draw_string(fb, 90, 56, "R:%d", car->roll/10);
}

// My mode, saab inspired
// Still needs much tweaking
void draw_night_mode(uint8_t *fb, car_state_t *car) {
    ssd1309_clear_buffer(fb);
    bool show_rpm = (car->rpm > 2800); 
    bool show_fuel = (car->fuel < 20);

    // Speedometer (Centered Left)
    draw_analog_gauge(fb, 32, 32, 28, car->speed, 60.0f, "KPH");

    // Tachometer (Centered Right - Ghost)
    if (show_rpm) {
        draw_analog_gauge(fb, 96, 32, 28, car->rpm, 3800.0f, "RPM");
        if(car->rpm > 3600) ssd1309_draw_rect(fb, 120, 0, 8, 64, 1, 1); // Shift Light
    }

    // Low Fuel Warning
    if (show_fuel) {
        ssd1309_draw_rect(fb, 54, 50, 20, 10, 1, 1);
        ssd1309_draw_string(fb, 56, 38, "FUEL");
    }
}

// Main function, no FreeRTOS needed here
void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ssd1309_init();
    can_init(); // Pin 5 (TX) - Pin 18 (RX)

    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON, GPIO_PULLUP_ONLY);

    car_state_t car = {0};
    int64_t last_pkt_time = 0;
    int64_t last_btn_time = 0;

    ESP_LOGI(TAG, "Dashboard Initialized.");

    while(1) {
        int64_t now = esp_timer_get_time() / 1000;

        // Updates data if available
        if (can_update_state(&car)) {
            last_pkt_time = now;
            car.link_active = true;
            
            // Apply Low-Pass Filter (EMA) to smooth needles
            signal_filter.rpm = (FILTER_ALPHA * car.rpm) + ((1.0 - FILTER_ALPHA) * signal_filter.rpm);
            signal_filter.fuel = (FILTER_ALPHA * car.fuel) + ((1.0 - FILTER_ALPHA) * signal_filter.fuel);
            
            // Write smoothed values back for display
            car.rpm = (uint16_t)signal_filter.rpm;
            car.fuel = (uint16_t)signal_filter.fuel;
        }

        // Dead link warning
        if (now - last_pkt_time > TIMEOUT_MS) {
            car.link_active = false;
            car.rpm = 0; car.speed = 0; // Kill gauges
        }

        // Check for button input
        if (gpio_get_level(PIN_BUTTON) == 0) {
            if (now - last_btn_time > 300) { 
                current_mode++;
                if (current_mode >= MODE_COUNT) current_mode = 0;
                last_btn_time = now;
            }
        }

        // Render screen
        if (!car.link_active) {
            ssd1309_clear_buffer(s_buffer);
            ssd1309_draw_rect(s_buffer, 0, 0, 128, 64, 1, 0); // Warning border
            ssd1309_draw_string_large(s_buffer, 15, 20, 2, "NO LINK");
            ssd1309_draw_string(s_buffer, 35, 45, "CHECK ECU");
        } else {
            switch(current_mode) {
                case MODE_PILOT:     draw_pilot(s_buffer, &car); break;
                case MODE_ENGINEER:  draw_engineer(s_buffer, &car); break;
                case MODE_ADVENTURE: draw_adventure(s_buffer, &car); break;
                case MODE_NIGHT:     draw_night_mode(s_buffer, &car); break;
                case MODE_COUNT: break;
            }
        }

        ssd1309_display_buffer(s_buffer);
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 FPS target (system permitting)
    }
}
