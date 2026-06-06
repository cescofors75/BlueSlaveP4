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
    /* LVGL shares the global PSRAM heap, so report the whole PSRAM pool — that's
     * the number that actually matters for OOM here. */
    if (mon_p == NULL) return;
    size_t total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t freeb = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t big   = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t used  = (total > freeb) ? (total - freeb) : 0;
    mon_p->total_size        = total;
    mon_p->free_cnt          = 0;
    mon_p->free_size         = freeb;
    mon_p->free_biggest_size = big;
    mon_p->used_cnt          = 0;
    mon_p->max_used          = used;
    mon_p->used_pct          = total ? (uint8_t)((used * 100) / total) : 0;
    mon_p->frag_pct          = freeb ? (uint8_t)(100 - (big * 100) / freeb) : 0;
}

lv_result_t lv_mem_test_core(void)
{
    return LV_RESULT_OK;
}

#endif /* LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM */
