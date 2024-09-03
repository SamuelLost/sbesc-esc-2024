#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Check if a pointer is valid */
#define IS_VALID(ptr) ((ptr) != NULL)

/** Unused parameter macro */
#define UNUSED(x) (void)(x)

/** Convert a macro to a string */
#define STRINGIFY(x) #x

/** G-force constant */
#define G_FORCE 9.81

/** Time to restart the ESP32 */
#define TIME_TO_RESTART 7

/** Restart the ESP32 */
#define RESTART(tag, delay)                                     \
    do {                                                        \
        ESP_LOGE(tag, "Restarting in %d seconds...", delay);    \
        vTaskDelay((delay) * 1000 / portTICK_PERIOD_MS);        \
        esp_restart();                                          \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif // UTILS_H