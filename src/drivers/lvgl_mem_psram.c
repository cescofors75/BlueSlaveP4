// =============================================================================
// lvgl_mem_psram.c — LVGL 9 custom memory core (LV_STDLIB_CUSTOM)
//
// LVGL 8 used LV_MEM_CUSTOM to route lv_mem_alloc/free straight to
// heap_caps_*(MALLOC_CAP_SPIRAM). LVGL 9 removed LV_MEM_CUSTOM; the equivalent
// is LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM, which disables LVGL's built-in
// allocators and expects these *_core() symbols to be provided externally.
//
// Routing every LVGL allocation to the PSRAM heap (no fixed pool, no cap) keeps
// the exact behaviour of the old config: LVGL shares the 32MB OPI PSRAM heap
// dynamically with the MIDI loader / audio buffers.
// =============================================================================
#include <lvgl.h>

#if LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM

#include <esp_heap_caps.h>

void lv_mem_init(void)
{
    /* PSRAM heap is brought up by the ESP-IDF startup — nothing to do. */
}

void lv_mem_deinit(void)
{
    /* Nothing to deinit. */
}

lv_mem_pool_t lv_mem_add_pool(void * mem, size_t bytes)
{
    /* Not supported: we allocate from the global PSRAM heap, not a pool. */
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    LV_UNUSED(pool);
}

void * lv_malloc_core(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
}

void * lv_realloc_core(void * p, size_t new_size)
{
    return heap_caps_realloc(p, new_size, MALLOC_CAP_SPIRAM);
}

void lv_free_core(void * p)
{
    heap_caps_free(p);
}

void lv_mem_monitor_core(lv_mem_monitor_t * mon_p)
{
    /* Heap is shared with the rest of the firmware; LVGL-only stats are N/A. */
    LV_UNUSED(mon_p);
}

lv_result_t lv_mem_test_core(void)
{
    return LV_RESULT_OK;
}

#endif /* LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM */
