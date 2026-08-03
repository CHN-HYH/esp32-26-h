#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>

    void app_mymdns_main();
    void app_mymdns_update_framesize(int size);
    const char *app_mymdns_query(size_t *out_len);

#ifdef __cplusplus
}
#endif
