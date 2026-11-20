// ESP32-S31 internal-flash / NVS guard for Metalio E-Ink4-Plus (Item 6).
//
// Serializes all NVS operations with EPDiy's high-voltage frame updates so an
// internal SPI-flash erase/program stall never coincides with an active LCD/DMA
// scan or TPS65185 charge-pump cycle. Lock order matches EpdiyDriver::drawFront
// (EpdFlashGuard -> I2cBusLock) using recursive FreeRTOS mutexes.
#include <sdkconfig.h>

#if CONFIG_IDF_TARGET_ESP32S31 && FREEINK_DEVICE_METALIO_EINK4_PLUS

#include <EpdFlashGuard.h>
#include <I2cBusLock.h>
#include <nvs.h>
#include <nvs_flash.h>

namespace {
struct NvsGuard {
  freeink::EpdFlashGuard epdGuard;
  freeink::I2cBusLock i2cGuard;
};
}  // namespace

#define EPD_NVS_WRAP_RET(ret_type, name, params, args) \
  ret_type __real_##name params;                      \
  ret_type __wrap_##name params {                      \
    const NvsGuard guard;                              \
    return __real_##name args;                         \
  }

#define EPD_NVS_WRAP_VOID(name, params, args) \
  void __real_##name params;                  \
  void __wrap_##name params {                  \
    const NvsGuard guard;                     \
    __real_##name args;                       \
  }

extern "C" {

EPD_NVS_WRAP_RET(esp_err_t, nvs_open,
                 (const char* namespace_name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle),
                 (namespace_name, open_mode, out_handle))
EPD_NVS_WRAP_RET(esp_err_t, nvs_open_from_partition,
                 (const char* part_name, const char* namespace_name, nvs_open_mode_t open_mode,
                  nvs_handle_t* out_handle),
                 (part_name, namespace_name, open_mode, out_handle))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_i8, (nvs_handle_t handle, const char* key, int8_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_u8, (nvs_handle_t handle, const char* key, uint8_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_i16, (nvs_handle_t handle, const char* key, int16_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_u16, (nvs_handle_t handle, const char* key, uint16_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_i32, (nvs_handle_t handle, const char* key, int32_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_u32, (nvs_handle_t handle, const char* key, uint32_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_i64, (nvs_handle_t handle, const char* key, int64_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_u64, (nvs_handle_t handle, const char* key, uint64_t value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_float, (nvs_handle_t handle, const char* key, float value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_double, (nvs_handle_t handle, const char* key, double value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_str, (nvs_handle_t handle, const char* key, const char* value), (handle, key, value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_set_blob,
                 (nvs_handle_t handle, const char* key, const void* value, size_t length),
                 (handle, key, value, length))

EPD_NVS_WRAP_RET(esp_err_t, nvs_get_i8, (nvs_handle_t handle, const char* key, int8_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_u8, (nvs_handle_t handle, const char* key, uint8_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_i16, (nvs_handle_t handle, const char* key, int16_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_u16, (nvs_handle_t handle, const char* key, uint16_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_i32, (nvs_handle_t handle, const char* key, int32_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_u32, (nvs_handle_t handle, const char* key, uint32_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_i64, (nvs_handle_t handle, const char* key, int64_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_u64, (nvs_handle_t handle, const char* key, uint64_t* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_float, (nvs_handle_t handle, const char* key, float* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_double, (nvs_handle_t handle, const char* key, double* out_value),
                 (handle, key, out_value))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_str,
                 (nvs_handle_t handle, const char* key, char* out_value, size_t* length),
                 (handle, key, out_value, length))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_blob,
                 (nvs_handle_t handle, const char* key, void* out_value, size_t* length),
                 (handle, key, out_value, length))

EPD_NVS_WRAP_RET(esp_err_t, nvs_find_key, (nvs_handle_t handle, const char* key, nvs_type_t* out_type),
                 (handle, key, out_type))
EPD_NVS_WRAP_RET(esp_err_t, nvs_erase_key, (nvs_handle_t handle, const char* key), (handle, key))
EPD_NVS_WRAP_RET(esp_err_t, nvs_erase_all, (nvs_handle_t handle), (handle))
EPD_NVS_WRAP_RET(esp_err_t, nvs_purge_all, (nvs_handle_t handle), (handle))
EPD_NVS_WRAP_RET(esp_err_t, nvs_commit, (nvs_handle_t handle), (handle))
EPD_NVS_WRAP_VOID(nvs_close, (nvs_handle_t handle), (handle))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_stats, (const char* part_name, nvs_stats_t* nvs_stats), (part_name, nvs_stats))
EPD_NVS_WRAP_RET(esp_err_t, nvs_get_used_entry_count, (nvs_handle_t handle, size_t* used_entries),
                 (handle, used_entries))
EPD_NVS_WRAP_RET(esp_err_t, nvs_entry_find,
                 (const char* part_name, const char* namespace_name, nvs_type_t type, nvs_iterator_t* output_iterator),
                 (part_name, namespace_name, type, output_iterator))
EPD_NVS_WRAP_RET(esp_err_t, nvs_entry_find_in_handle,
                 (nvs_handle_t handle, nvs_type_t type, nvs_iterator_t* output_iterator),
                 (handle, type, output_iterator))
EPD_NVS_WRAP_RET(esp_err_t, nvs_entry_next, (nvs_iterator_t* iterator), (iterator))
EPD_NVS_WRAP_RET(esp_err_t, nvs_entry_info, (const nvs_iterator_t iterator, nvs_entry_info_t* out_info),
                 (iterator, out_info))
EPD_NVS_WRAP_VOID(nvs_release_iterator, (nvs_iterator_t iterator), (iterator))

EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_init, (void), ())
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_init_partition, (const char* partition_label), (partition_label))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_init_partition_ptr, (const esp_partition_t* partition), (partition))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_deinit, (void), ())
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_deinit_partition, (const char* partition_label), (partition_label))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_erase, (void), ())
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_erase_partition, (const char* part_name), (part_name))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_erase_partition_ptr, (const esp_partition_t* partition), (partition))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_secure_init, (nvs_sec_cfg_t* cfg), (cfg))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_secure_init_partition, (const char* partition_label, nvs_sec_cfg_t* cfg),
                 (partition_label, cfg))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_generate_keys, (const esp_partition_t* partition, nvs_sec_cfg_t* cfg),
                 (partition, cfg))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_read_security_cfg, (const esp_partition_t* partition, nvs_sec_cfg_t* cfg),
                 (partition, cfg))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_register_security_scheme, (nvs_sec_scheme_t* scheme_cfg), (scheme_cfg))
EPD_NVS_WRAP_VOID(nvs_flash_deregister_security_scheme, (void), ())
EPD_NVS_WRAP_RET(nvs_sec_scheme_t*, nvs_flash_get_default_security_scheme, (void), ())
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_generate_keys_v2, (nvs_sec_scheme_t* scheme_cfg, nvs_sec_cfg_t* cfg),
                 (scheme_cfg, cfg))
EPD_NVS_WRAP_RET(esp_err_t, nvs_flash_read_security_cfg_v2, (nvs_sec_scheme_t* scheme_cfg, nvs_sec_cfg_t* cfg),
                 (scheme_cfg, cfg))

}  // extern "C"

#endif
