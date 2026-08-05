#include "yahboom_overlay.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct
{
    uint8_t y;
    uint8_t cb;
    uint8_t cr;
} yuv422_color_t;

typedef enum
{
    OVERLAY_STATUS_NONE,
    OVERLAY_STATUS_WAIT,
    OVERLAY_STATUS_START,
} overlay_status_t;

static const yuv422_color_t kOverlayBackgroundColor = {16, 128, 128};
static const yuv422_color_t kOverlayTextColor = {235, 128, 128};
static const char kFpsCharacters[] = "FPS:0123456789";
static const uint16_t kFpsGlyphs[] = {
    0x79A4, 0x6BA4, 0x388E, 0x0410,
    0x7B6F, 0x2C97, 0x73E7, 0x73CF, 0x5BC9,
    0x79CF, 0x79EF, 0x7249, 0x7BEF, 0x7BCF,
};
static const uint8_t kStartFont[5][7] = {
    {0x0f, 0x10, 0x0e, 0x01, 0x1e, 0x10, 0x0f},
    {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11},
    {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11},
    {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
};
static const uint8_t kWaitFont[4][7] = {
    {0x11, 0x11, 0x11, 0x11, 0x15, 0x15, 0x0a},
    {0x0e, 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11},
    {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f},
    {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
};

static overlay_status_t status = OVERLAY_STATUS_NONE;
static TickType_t status_start_tick = 0;
static TickType_t status_duration_ticks = 0;

static inline void draw_yuv422_pixel(camera_fb_t *frame, int x, int y, const yuv422_color_t *color)
{
    if (x < 0 || x >= frame->width || y < 0 || y >= frame->height)
        return;

    size_t pixel_offset = (y * frame->width + x) * 2;
    size_t pair_offset = (y * frame->width + (x & ~1)) * 2;
    frame->buf[pixel_offset] = color->y;
    frame->buf[pair_offset + 1] = color->cb;
    frame->buf[pair_offset + 3] = color->cr;
}

static void show_status(overlay_status_t new_status, uint32_t duration_ms)
{
    status = new_status;
    status_start_tick = xTaskGetTickCount();
    status_duration_ticks = pdMS_TO_TICKS(duration_ms);
}

void yahboom_overlay_show_wait(uint32_t duration_ms)
{
    show_status(OVERLAY_STATUS_WAIT, duration_ms);
}

void yahboom_overlay_show_start(uint32_t duration_ms)
{
    show_status(OVERLAY_STATUS_START, duration_ms);
}

void yahboom_overlay_draw_status(camera_fb_t *frame)
{
    const uint8_t (*font)[7] = NULL;
    int character_count = 0;

    if (status == OVERLAY_STATUS_NONE ||
        xTaskGetTickCount() - status_start_tick >= status_duration_ticks)
    {
        status = OVERLAY_STATUS_NONE;
        return;
    }

    if (status == OVERLAY_STATUS_WAIT)
    {
        font = kWaitFont;
        character_count = 4;
    }
    else
    {
        font = kStartFont;
        character_count = 5;
    }

    const int scale = frame->width >= 200 ? 3 : 2;
    const int origin_x = 8;
    const int origin_y = 8;
    const int glyph_width = 5 * scale;
    const int glyph_height = 7 * scale;
    const int text_width = glyph_width * character_count + scale * (character_count - 1);
    const int banner_width = text_width + scale * 4;
    const int banner_height = glyph_height + scale * 4;

    for (int y = origin_y; y < origin_y + banner_height; y++)
    {
        for (int x = origin_x; x < origin_x + banner_width; x++)
            draw_yuv422_pixel(frame, x, y, &kOverlayBackgroundColor);
    }

    for (int character = 0; character < character_count; character++)
    {
        int glyph_x = origin_x + scale * 2 + character * (glyph_width + scale);
        int glyph_y = origin_y + scale * 2;
        for (int row = 0; row < 7; row++)
        {
            for (int column = 0; column < 5; column++)
            {
                if ((font[character][row] & (1 << (4 - column))) == 0)
                    continue;

                for (int offset_y = 0; offset_y < scale; offset_y++)
                {
                    for (int offset_x = 0; offset_x < scale; offset_x++)
                        draw_yuv422_pixel(frame, glyph_x + column * scale + offset_x,
                                          glyph_y + row * scale + offset_y, &kOverlayTextColor);
                }
            }
        }
    }
}

void yahboom_overlay_draw_fps(camera_fb_t *frame)
{
    static TickType_t fps_start_tick = 0;
    static uint16_t fps_frame_count = 0;
    static uint16_t displayed_fps = 0;
    TickType_t now = xTaskGetTickCount();

    if (fps_start_tick == 0)
        fps_start_tick = now;
    fps_frame_count++;

    TickType_t elapsed_ticks = now - fps_start_tick;
    if (elapsed_ticks >= pdMS_TO_TICKS(1000))
    {
        uint32_t elapsed_ms = pdTICKS_TO_MS(elapsed_ticks);
        displayed_fps = elapsed_ms == 0 ? 0 : fps_frame_count * 1000 / elapsed_ms;
        fps_frame_count = 0;
        fps_start_tick = now;
    }
    if (displayed_fps > 999)
        displayed_fps = 999;

    char text[10];
    snprintf(text, sizeof(text), "FPS:%u", (unsigned)displayed_fps);
    const int scale = frame->width >= 200 ? 2 : 1;
    const int glyph_width = 3 * scale;
    const int glyph_height = 5 * scale;
    const int character_count = strlen(text);
    const int text_width = character_count * glyph_width + (character_count - 1) * scale;
    const int origin_x = (frame->width - text_width) / 2;
    const int origin_y = scale * 3;

    for (int y = origin_y - scale; y < origin_y + glyph_height + scale; y++)
    {
        for (int x = origin_x - scale; x < origin_x + text_width + scale; x++)
            draw_yuv422_pixel(frame, x, y, &kOverlayBackgroundColor);
    }

    for (int character = 0; character < character_count; character++)
    {
        const char *glyph_character = strchr(kFpsCharacters, text[character]);
        if (glyph_character == NULL)
            continue;

        uint16_t glyph = kFpsGlyphs[glyph_character - kFpsCharacters];
        int glyph_x = origin_x + character * (glyph_width + scale);
        for (int row = 0; row < 5; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                if ((glyph & (1U << (14 - row * 3 - column))) == 0)
                    continue;

                for (int offset_y = 0; offset_y < scale; offset_y++)
                {
                    for (int offset_x = 0; offset_x < scale; offset_x++)
                        draw_yuv422_pixel(frame, glyph_x + column * scale + offset_x,
                                          origin_y + row * scale + offset_y, &kOverlayTextColor);
                }
            }
        }
    }
}
