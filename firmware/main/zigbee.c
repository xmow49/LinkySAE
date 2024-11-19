/**
 * @file zigbee.cpp
 * @author Dorian Benech
 * @brief
 * @version 1.0
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023 GammaTroniques
 *
 */

/*==============================================================================
 Local Include
===============================================================================*/

#include "esp_zigbee_core.h"
#include "esp_app_desc.h"
#include <stdio.h>
#include <inttypes.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include <string.h>
#include "esp_check.h"
#include "esp_pm.h"
#include "esp_timer.h"

#include "ota_zlib.h"
#include "zigbee.h"
#include "gpio.h"
#include "led.h"
#include "config.h"
#include "main.h"
/*==============================================================================
 Local Define
===============================================================================*/

#define TAG "ZIGBEE"

/*==============================================================================
 Local Macro
===============================================================================*/
#define DEFINE_PSTRING(var, str)   \
    const struct                   \
    {                              \
        unsigned char len;         \
        char content[sizeof(str)]; \
    }(var) = {sizeof(str) - 1, (str)}

/*==============================================================================
 Local Type
===============================================================================*/

typedef enum
{
    ZIGBEE_NOT_CONNECTED,
    ZIGBEE_CONNECTING,
    ZIGBEE_CONNECTED,
    ZIGBEE_COMMISIONING_ERROR,
} zigbee_state_t;

typedef enum
{
    ZB_CMD_REBOOT = 0,
    ZB_CMD_RESET = 1,
} zb_cmd_t;

/*==============================================================================
 Local Function Declaration
===============================================================================*/
static void zigbee_bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static esp_err_t zigbee_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static void zigbee_task(void *pvParameters);
static esp_err_t zigbee_report_attribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length);
static void zigbee_print_value(char *out_buffer, void *data, linky_label_type_t type);
static esp_err_t zigbee_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t messsage);
static uint32_t zigbee_get_hex_version(const char *version);
static uint16_t zigbee_get_hex16_version(const char *version);

/*==============================================================================
Public Variable
===============================================================================*/

/*==============================================================================
 Local Variable
===============================================================================*/

DEFINE_PSTRING(zigbee_device_name, "TICMeter");
DEFINE_PSTRING(zigbee_device_manufacturer, "GammaTroniques");
// DEFINE_PSTRING(zigbee_date_code, BUILD_TIME);

zigbee_state_t zigbee_state = ZIGBEE_NOT_CONNECTED;
// static esp_pm_lock_handle_t zigbee_pm_apb_lock;
// static esp_pm_lock_handle_t zigbee_pm_cpu_lock;

uint8_t zigbee_ota_running = 0;
uint8_t zigbee_sending = 0;

const TickType_t zigbee_lock_timeout = pdMS_TO_TICKS(5000);

/*==============================================================================
Function Implementation
===============================================================================*/
void zigbee_start_pairing()
{
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

void zigbee_config_report()
{
    // uint32_t record_count = 0;

    for (int i = 0; i < linky_label_list_size; i++)
    {
        if (linky_label_list[i].zb_access == 0)
        {
            continue;
        }

        if (linky_label_list[i].mode != linky_mode && linky_label_list[i].mode != ANY)
        {
            continue;
        }

        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = LINKY_TIC_ENDPOINT,
            .cluster_id = linky_label_list[i].clusterID,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .u.send_info = {
                .min_interval = 15,
                .max_interval = 300,
                .def_min_interval = 15,
                .def_max_interval = 300,
                // .delta =
            },
            .attr_id = linky_label_list[i].attributeID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,

        };

        if (linky_label_list[i].data == NULL)
        {
            continue;
        }

        switch (linky_label_list[i].type)
        {
        case UINT8:
        {
            if (*(uint8_t *)linky_label_list[i].data == UINT8_MAX)
            {
                continue;
            }
            reporting_info.u.send_info.delta.u8 = 1;
            break;
        }
        case UINT16:
        {
            if (*(uint16_t *)linky_label_list[i].data == UINT16_MAX)
            {
                continue;
            }
            reporting_info.u.send_info.delta.u16 = 1;
            break;
        }
        case UINT32:
        {
            if (*(uint32_t *)linky_label_list[i].data == UINT32_MAX)
            {
                continue;
            }
            if (linky_label_list[i].device_class == ENERGY && *(uint32_t *)linky_label_list[i].data == 0)
            {
                continue;
            }
            reporting_info.u.send_info.delta.u32 = 1;
            break;
        }
        case UINT64:
        {
            if (*(uint64_t *)linky_label_list[i].data == UINT64_MAX)
            {
                continue;
            }
            if (linky_label_list[i].device_class == ENERGY && *(uint64_t *)linky_label_list[i].data == 0)
            {
                continue;
            }
            reporting_info.u.send_info.delta.u48.high = 0;
            reporting_info.u.send_info.delta.u48.low = 1;
            break;
        }
        case STRING:
        {
            if (strnlen((char *)linky_label_list[i].data, linky_label_list[i].size) == 0)
            {
                continue;
            }
            continue;
            //  reporting_info.u.send_info.delta.u8 = 1;
            break;
        }
        case UINT32_TIME:
        {
            if (((time_label_t *)linky_label_list[i].data)->value == UINT32_MAX)
            {
                continue;
            }
            reporting_info.u.send_info.delta.u32 = 1;
            break;
        }
        default:
            break;
        };

        bool sucess = esp_zb_lock_acquire(zigbee_lock_timeout);
        if (!sucess)
        {
            ESP_LOGE(TAG, "Failed to acquire lock");
            return;
        }
        esp_zb_zcl_update_reporting_info(&reporting_info);
        esp_zb_lock_release();
        ESP_LOGI(TAG, "Send configure reporting for: %s", linky_label_list[i].label);
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    esp_zb_app_signal_type_t *p_sg_p = (esp_zb_app_signal_type_t *)signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        // ESP_LOGW(TAG, "Signal: %x", sig_type);
        zigbee_state = ZIGBEE_CONNECTING;
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                led_start_pattern(LED_PAIRING);
                suspend_task(main_task_handle);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Commissioning failed (status: %d)", err_status);
            zigbee_state = ZIGBEE_COMMISIONING_ERROR;
            led_start_pattern(LED_CONNECTING_FAILED);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zigbee_config_report();

            if (config_values.zigbee.state != ZIGBEE_PAIRED)
            {
                ESP_LOGI(TAG, "Zigbee paired");
                config_values.zigbee.state = ZIGBEE_PAIRED;
                config_write();
                led_stop_pattern(LED_PAIRING);
                resume_task(main_task_handle);
            }
            zigbee_state = ZIGBEE_CONNECTED;
            // xTaskCreate(zigbee_send_first_datas, "zigbee_send_first_datas", 4 * 1024, NULL, PRIORITY_ZIGBEE, NULL);
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)zigbee_bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            zigbee_state = ZIGBEE_CONNECTING;
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        // esp_rom_printf("can sleep\n");
        esp_zb_sleep_now();
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Leave signal");
        if (config_values.zigbee.state != ZIGBEE_WANT_PAIRING)
        {
            config_values.zigbee.state = ZIGBEE_NOT_CONFIGURED;
            config_write();
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        zigbee_config_report();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status), err_status);

        break;
    }
}

static void zigbee_bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t zigbee_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(0x%x), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);

    uint8_t *data = message->attribute.data.value;
    char buffer[101];
    for (int i = 0; i < linky_label_list_size; i++)
    {
        if (linky_label_list[i].clusterID == message->info.cluster && linky_label_list[i].attributeID == message->attribute.id)
        {
            switch (linky_label_list[i].type)
            {
            case UINT8:
                *(uint8_t *)linky_label_list[i].data = data[0];
                break;
            case UINT16:
                *(uint16_t *)linky_label_list[i].data = data[1] << 8 | data[0];
                break;
            case UINT32:
                *(uint32_t *)linky_label_list[i].data = (uint32_t)data[3] << 24 | (uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[0];
                break;
            case UINT64:
                *(uint64_t *)linky_label_list[i].data = (uint64_t)data[7] << 56 | (uint64_t)data[6] << 48 | (uint64_t)data[5] << 40 | (uint64_t)data[4] << 32 | (uint64_t)data[3] << 24 | (uint64_t)data[2] << 16 | (uint64_t)data[1] << 8 | (uint64_t)data[0];
                break;
            case STRING:
                memcpy(linky_label_list[i].data, data, MIN(linky_label_list[i].size, message->attribute.data.size));
                ((char *)linky_label_list[i].data)[linky_label_list[i].size] = '\0';
                break;
            case UINT32_TIME:
                ((time_label_t *)linky_label_list[i].data)->value = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
                break;
            default:
                return ESP_ERR_INVALID_ARG;
                break;
            }
            zigbee_print_value(buffer, linky_label_list[i].data, linky_label_list[i].type);
            ESP_LOGI(TAG, "Attribute %s updated to %s", linky_label_list[i].label, buffer);
            config_write();
            break;
        }
    }

    return ret;
}

static esp_err_t zigbee_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zigbee_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zigbee_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        esp_zb_zcl_cmd_default_resp_message_t *resp = (esp_zb_zcl_cmd_default_resp_message_t *)(message);
        ESP_LOGD(TAG, "Received default response cluster: %x", resp->info.cluster);
        break;
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
        esp_zb_zcl_custom_cluster_command_message_t *cmd = (esp_zb_zcl_custom_cluster_command_message_t *)(message);
        // uint8_t *data = cmd->data.value;
        switch (cmd->info.command.id)
        {
        case ZB_CMD_REBOOT:
        {
            ESP_LOGI(TAG, "Reboot command received");
            hard_restart();
            break;
        }
        case ZB_CMD_RESET:
        {
            ESP_LOGI(TAG, "Reset command received");
            gpio_start_pariring();

            break;
        }
        default:
            ESP_LOGI(TAG, "Received custom cluster: cluster: %x, command: %x, size %d", cmd->info.cluster, cmd->info.command.id, cmd->data.size);
            ESP_LOG_BUFFER_HEXDUMP(TAG, cmd->data.value, cmd->data.size, ESP_LOG_INFO);
            break;
        }

        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback, message: %s", callback_id, (char *)message);
        break;
    }
    return ret;
}

void zigbee_init_stack()
{
    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_flash_init failed: 0x%x", ret);
    }

    ESP_LOGI(TAG, "Initializing Zigbee stack");
    esp_zb_platform_config_t config = {
        .radio_config.radio_mode = ZB_RADIO_MODE_NATIVE,
        .host_config.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
    };
    ret = esp_zb_platform_config(&config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_zb_platform_config failed: 0x%x", ret);
    }

    xTaskCreate(zigbee_task, "Zigbee_main", 8 * 1024, NULL, PRIORITY_ZIGBEE, NULL);
}

static void zigbee_task(void *pvParameters)
{
    // clang-format on
    // ESP_LOGI(TAG, "Resetting Zigbee stack");
    // esp_zb_factory_reset();
    // ESP_LOGI(TAG, "Starting Zigbee stack");
    /* initialize Zigbee stack with Zigbee end-device config */
    // esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_WARN, ESP_ZB_TRACE_SUBSYSTEM_NWK | ESP_ZB_TRACE_SUBSYSTEM_APP);
    // esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_INFO, ESP_ZB_TRACE_SUBSYSTEM_COMMON);
    esp_zb_cfg_t zigbee_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
        .nwk_cfg.zed_cfg.keep_alive = 4000, // in seconds
    };

    // if (!gpio_vusb_connected())
    //{
    ESP_LOGW(TAG, "Enable sleep");
    esp_zb_sleep_enable(true);
    // }
    // else
    // {
    //     ESP_LOGW(TAG, "Disable sleep: VUSB connected");
    // }

    esp_zb_aps_src_binding_table_size_set(64);
    esp_zb_init(&zigbee_cfg);

    ESP_LOGI(TAG, "Zigbee stack initialized");
    //------------------ Basic cluster ------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);

    uint8_t ApplicationVersion = 1;
    uint8_t StackVersion = 1;
    uint8_t HWVersion = 1;

    const esp_app_desc_t *app_desc = esp_app_get_description();
    uint8_t SWBuildID[35];
    snprintf((char *)SWBuildID, sizeof(SWBuildID), "%c%s", strlen(app_desc->version), app_desc->version);

    char zigbee_date_code[50];
    snprintf(zigbee_date_code + 1, sizeof(zigbee_date_code), "%s", BUILD_TIME);
    zigbee_date_code[0] = strlen(BUILD_TIME);

    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)&zigbee_device_manufacturer);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)&zigbee_device_name);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *)&zigbee_date_code);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, SWBuildID);

    // ---------------------- Identify cluster ----------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {};
    identify_cluster_cfg.identify_time = 0;
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // ---------------------- Meter identification cluster ----------------------
    // esp_zb_attribute_list_t *esp_zb_meter_identification_cluster = esp_zb_cluster_create

    // ---------------------- Metering cluster ----------------------
    uint8_t formating = (1 << 7) | (15 << 3) | (3 << 0);
    uint32_t divisor = 1000;
    esp_zb_metering_cluster_cfg_t metering_cluster_cfg = {
        .uint_of_measure = ESP_ZB_ZCL_METERING_UNIT_KW_KWH_BINARY,
        .metering_device_type = ESP_ZB_ZCL_METERING_ELECTRIC_METERING,
        //
        .summation_formatting = formating,
    };
    esp_zb_attribute_list_t *esp_zb_metering_cluster = esp_zb_metering_cluster_create(&metering_cluster_cfg);

    for (esp_zb_attribute_list_t *temp = esp_zb_metering_cluster; temp != NULL; temp = temp->next)
    {
        // ESP_LOGI(TAG, "Metering attr created: att: %x %x", temp->attribute.id, temp->attribute.access);
        if (temp->attribute.id == 0)
        {
            temp->attribute.access = ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
        }
    }

    esp_zb_cluster_add_attr(esp_zb_metering_cluster, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_ATTR_METERING_DEMAND_FORMATTING_ID, ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &formating);
    esp_zb_cluster_add_attr(esp_zb_metering_cluster, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_ATTR_METERING_HISTORICAL_CONSUMPTION_FORMATTING_ID, ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &formating);
    esp_zb_cluster_add_attr(esp_zb_metering_cluster, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_ATTR_METERING_UNIT_OF_MEASURE_ID, ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &metering_cluster_cfg.uint_of_measure);
    esp_zb_cluster_add_attr(esp_zb_metering_cluster, ESP_ZB_ZCL_CLUSTER_ID_METERING, ESP_ZB_ZCL_ATTR_METERING_DIVISOR_ID, ESP_ZB_ZCL_ATTR_TYPE_U24, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &divisor);

    //------------------ Electrical Measurement cluster ------------------
    esp_zb_electrical_meas_cluster_cfg_t electrical_meas_cluster_cfg = {
        .measured_type = ESP_ZB_ZCL_ELECTRICAL_MEASUREMENT_APPARENT_MEASUREMENT,
    };
    esp_zb_attribute_list_t *esp_zb_electrical_meas_cluster = esp_zb_electrical_meas_cluster_create(&electrical_meas_cluster_cfg);
    uint16_t ac_power_divisor = 1;
    esp_zb_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_DIVISOR_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &ac_power_divisor);
    esp_zb_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACPOWER_MULTIPLIER_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &ac_power_divisor);
    esp_zb_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_MULTIPLIER_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &ac_power_divisor);
    esp_zb_cluster_add_attr(esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT, ESP_ZB_ZCL_ATTR_ELECTRICAL_MEASUREMENT_ACCURRENT_DIVISOR_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &ac_power_divisor);

    // ------------------ TICMeter cluster ------------------
    esp_zb_attribute_list_t *esp_zb_ticmeter_cluster = esp_zb_zcl_attr_list_create(TICMETER_CLUSTER_ID);
    esp_zb_cluster_add_attr(esp_zb_ticmeter_cluster, TICMETER_CLUSTER_ID, 0x0018, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &ApplicationVersion);

    // ------------------ OTA cluster ------------------

    /** Create ota client cluster with attributes.
     *  Manufacturer code, image type and file version should match with configured values for server.
     *  If the client values do not match with configured values then it shall discard the command and
     *  no further processing shall continue.
     */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_manufacturer = 0xFFFF,
        .ota_upgrade_image_type = 0x00c8,
    };
    ota_cluster_cfg.ota_upgrade_file_version = zigbee_get_hex_version(app_desc->version);
    ota_cluster_cfg.ota_upgrade_downloaded_file_ver = zigbee_get_hex_version(app_desc->version);

    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    /** add client parameters to ota client cluster */
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .max_data_size = 64,
    };

    variable_config.hw_version = zigbee_get_hex16_version(efuse_values.hw_version);
    variable_config.hw_version = 0x0342;

    ESP_LOGI(TAG, "HW version: %x", variable_config.hw_version);
    ESP_LOGI(TAG, "File version: %lx", ota_cluster_cfg.ota_upgrade_file_version);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config);

    // ------------------ Add commands ------------------

    // ------------------ Add attributes ------------------
    uint32_t attributes_count = 0;

    for (int i = 0; i < linky_label_list_size; i++)
    {
        if (linky_label_list[i].zb_access == 0)
        {
            continue;
        }

        if (linky_label_list[i].mode != linky_mode && linky_label_list[i].mode != ANY)
        {
            continue;
        }
        if (linky_label_list[i].data == NULL)
        {
            continue;
        }

        switch (linky_label_list[i].type)
        {
        case UINT8:
        {
            if (*(uint8_t *)linky_label_list[i].data == UINT8_MAX)
            {
                continue;
            }
            break;
        }
        case UINT16:
        {
            if (*(uint16_t *)linky_label_list[i].data == UINT16_MAX)
            {
                continue;
            }
            break;
        }
        case UINT32:
        {
            if (*(uint32_t *)linky_label_list[i].data == UINT32_MAX)
            {
                continue;
            }
            if (linky_label_list[i].device_class == ENERGY && *(uint32_t *)linky_label_list[i].data == 0)
            {
                continue;
            }
            break;
        }
        case UINT64:
        {
            if (*(uint64_t *)linky_label_list[i].data == UINT64_MAX)
            {
                continue;
            }
            if (linky_label_list[i].device_class == ENERGY && *(uint64_t *)linky_label_list[i].data == 0)
            {
                continue;
            }
            break;
        }
        case STRING:
        {
            if (strnlen((char *)linky_label_list[i].data, linky_label_list[i].size) == 0)
            {
                continue;
            }
            break;
        }
        case UINT32_TIME:
        {
            if (((time_label_t *)linky_label_list[i].data)->value == UINT32_MAX)
            {
                continue;
            }
            break;
        }
        default:
            break;
        };

        char str_value[100];
        void *ptr_value = linky_label_list[i].data;

        if (linky_label_list[i].type == UINT32_TIME)
        {
            ptr_value = &((time_label_t *)linky_label_list[i].data)->value;
        }

        switch (linky_label_list[i].type)
        {
        case UINT8:
            sprintf(str_value, "%u", *(uint8_t *)ptr_value);
            break;
        case UINT16:
            sprintf(str_value, "%u", *(uint16_t *)ptr_value);
            break;
        case UINT32:
            sprintf(str_value, "%lu", *(uint32_t *)ptr_value);
            break;
        case UINT64:
            sprintf(str_value, "%llu", *(uint64_t *)ptr_value);
            break;
        case UINT32_TIME:
            sprintf(str_value, "%lu", *(uint32_t *)ptr_value);
            break;
        case STRING:
            sprintf(str_value, "%s", (char *)ptr_value);
            char *str = (char *)ptr_value;
            char temp[100];
            memcpy(temp + 1, str, linky_label_list[i].size + 1);
            temp[0] = strlen(temp + 1);
            if (temp[0] > linky_label_list[i].size)
            {
                temp[0] = linky_label_list[i].size;
            }
            temp[linky_label_list[i].size + 2] = '\0';
            memcpy(linky_label_list[i].data, temp, linky_label_list[i].size + 1);
            // ESP_LOG_BUFFER_HEXDUMP(TAG, linky_label_list[i].data, linky_label_list[i].size + 2, ESP_LOG_INFO);
            break;
        default:
            ESP_LOGE(TAG, "%s : Unknown type", linky_label_list[i].label);
            continue;
            break;
        }

        ESP_LOGI(TAG, "Adding %s : Cluster: 0x%04x, attribute: 0x%04x, zbtype: %x, value: %s", linky_label_list[i].label, linky_label_list[i].clusterID, linky_label_list[i].attributeID, linky_label_list[i].zb_type, str_value);

        switch (linky_label_list[i].clusterID)
        {
        case ESP_ZB_ZCL_CLUSTER_ID_ELECTRICAL_MEASUREMENT:
        {
            esp_zb_electrical_meas_cluster_add_attr(esp_zb_electrical_meas_cluster, linky_label_list[i].attributeID, ptr_value);
            break;
        }
        case ESP_ZB_ZCL_CLUSTER_ID_METERING:
        {
            esp_zb_cluster_add_attr(esp_zb_metering_cluster, ESP_ZB_ZCL_CLUSTER_ID_METERING, linky_label_list[i].attributeID, linky_label_list[i].zb_type, linky_label_list[i].zb_access, ptr_value);
            break;
        }
        case TICMETER_CLUSTER_ID:
        {
            esp_zb_custom_cluster_add_custom_attr(esp_zb_ticmeter_cluster, linky_label_list[i].attributeID, linky_label_list[i].zb_type, linky_label_list[i].zb_access, ptr_value);
            break;
        }
        default:
            break;
        }
        attributes_count++;
    }

    ESP_LOGI(TAG, "Attributes count: %ld", attributes_count);
    if (attributes_count != config_values.zigbee.last_attribute_count)
    {
        ESP_LOGI(TAG, "Attributes count changed: %ld -> %ld", config_values.zigbee.last_attribute_count, attributes_count);
        config_values.zigbee.last_attribute_count = attributes_count;
        config_write();
    }
    //------------------ Cluster list ------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_electrical_meas_cluster(esp_zb_cluster_list, esp_zb_electrical_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_metering_cluster(esp_zb_cluster_list, esp_zb_metering_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_ticmeter_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    // esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_meter_custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //------------------ Endpoint list ------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = LINKY_TIC_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_METER_INTERFACE_DEVICE_ID,
        .app_device_version = zigbee_get_hex_version(app_desc->version),
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);

    //------------------ Callbacks ------------------
    esp_zb_core_action_handler_register(zigbee_action_handler);

    esp_zb_set_primary_network_channel_set(ZIGBEE_CHANNEL_MASK);
    esp_zb_set_secondary_network_channel_set(0x07FFF800); // all channels
    ESP_ERROR_CHECK(esp_zb_start(false));
    ESP_LOGI(TAG, "Zigbee stack started");
    esp_zb_stack_main_loop();
}

static esp_err_t zigbee_report_attribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    esp_err_t ret = ESP_OK;
    // ESP_LOGW(TAG, "zigbee_report_attribute: 0x%x, attribute: 0x%x, value: %llu, length: %d", clusterID, attributeID, *(uint64_t *)value, value_length);
    esp_zb_zcl_report_attr_cmd_t cmd = {
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_GROUP_ENDP_NOT_PRESENT,
        .clusterID = clusterID,
        .attributeID = attributeID,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .zcl_basic_cmd.src_endpoint = endpoint,
    };
    bool success = esp_zb_lock_acquire(zigbee_lock_timeout);
    if (!success)
    {
        ESP_LOGE(TAG, "Failed to acquire lock");
        return ESP_ERR_TIMEOUT;
    }

    esp_zb_zcl_attr_t *value_r = esp_zb_zcl_get_attribute(endpoint, clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attributeID);
    if (value_r == NULL)
    {
        ESP_LOGE(TAG, "Attribute not found: 0x%x, attribute: 0x%x", clusterID, attributeID);
        ret = ESP_ERR_NOT_FOUND;
    }
    if (value == NULL)
    {
        ESP_LOGE(TAG, "Report value is NULL");
        ret = ESP_ERR_INVALID_ARG;
    }
    if (ret != ESP_OK)
    {
        esp_zb_lock_release();
        return ret;
    }

    memcpy(value_r->data_p, value, value_length);
    ret = esp_zb_zcl_report_attr_cmd_req(&cmd);
    esp_zb_lock_release();

    return ret;
}

esp_err_t zigbee_send(linky_data_t *data)
{
    esp_err_t ret = ESP_OK;

    if (config_values.zigbee.state != ZIGBEE_PAIRED)
    {
        ESP_LOGE(TAG, "Zigbee not paired");
        return ESP_ERR_INVALID_STATE;
    }

    if (zigbee_state == ZIGBEE_COMMISIONING_ERROR)
    {
        ESP_LOGE(TAG, "Zigbee commisioning error: retrying for the next time");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        return ESP_ERR_INVALID_STATE;
    }

    if (zigbee_ota_running)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    zigbee_sending = true;

    static uint32_t seq = 0;
    data->hist.PAPP = seq++;

    for (int i = 0; i < linky_label_list_size; i++)
    {
        char str_value[102];
        void *ptr_value = linky_label_list[i].data;
        ESP_LOGD(TAG, "check %s %d", linky_label_list[i].label, i);
        if (linky_label_list[i].mode != linky_mode && linky_label_list[i].mode != ANY)
        {
            continue;
        }
        if (linky_label_list[i].data == NULL)
        {
            continue;
        }
        if (linky_label_list[i].clusterID == 0 || linky_label_list[i].zb_access == 0)
        {
            continue;
        }

        switch (linky_label_list[i].type)
        {
        case UINT8:
        {
            if (*(uint8_t *)linky_label_list[i].data == UINT8_MAX)
            {
                continue;
            }
            break;
        }
        case UINT16:
        {
            if (*(uint16_t *)linky_label_list[i].data == UINT16_MAX)
            {
                continue;
            }
            break;
        }
        case UINT32:
        {
            if (*(uint32_t *)linky_label_list[i].data == UINT32_MAX)
            {
                continue;
            }
            if (linky_label_list[i].device_class == ENERGY && *(uint32_t *)linky_label_list[i].data == 0)
            {
                continue;
            }
            break;
        }
        case UINT64:
        {
            if (*(uint64_t *)linky_label_list[i].data == UINT64_MAX)
            {
                continue;
            }
            if (linky_label_list[i].device_class == ENERGY && *(uint64_t *)linky_label_list[i].data == 0)
            {
                continue;
            }
            break;
        }
        case STRING:
        {
            if (strnlen((char *)linky_label_list[i].data, linky_label_list[i].size) == 0)
            {
                continue;
            }
            char *str = (char *)linky_label_list[i].data;
            char temp[100];
            memcpy(temp + 1, str, linky_label_list[i].size + 1);
            temp[0] = strlen(temp + 1);
            if (temp[0] > linky_label_list[i].size)
            {
                temp[0] = linky_label_list[i].size;
            }
            temp[linky_label_list[i].size + 2] = '\0';
            memcpy(linky_label_list[i].data, temp, linky_label_list[i].size + 1);
            // ESP_LOG_BUFFER_HEXDUMP(TAG, linky_label_list[i].data, linky_label_list[i].size + 2, ESP_LOG_INFO);
            break;
        }
        case UINT32_TIME:
        {
            if (((time_label_t *)linky_label_list[i].data)->value == UINT32_MAX)
            {
                continue;
            }
            ptr_value = &((time_label_t *)linky_label_list[i].data)->value;
            if (linky_label_list[i].zb_type == ESP_ZB_ZCL_ATTR_TYPE_U64)
            {
                // pass only timestamp as uint64_t
                ptr_value = &((time_label_t *)linky_label_list[i].data)->time;
            }
            break;
        }
        default:
            break;
        };

        ESP_LOGD(TAG, "Send %s", linky_label_list[i].label);

        esp_zb_zcl_status_t status = ESP_ZB_ZCL_STATUS_SUCCESS;

        if (linky_label_list[i].zb_access == ESP_ZB_ZCL_ATTR_ACCESS_REPORTING)
        {
            zigbee_print_value(str_value, linky_label_list[i].data, linky_label_list[i].type);
            ESP_LOGI(TAG, "Repporting cluster: 0x%x, attribute: 0x%x, name: %s, value: %s", linky_label_list[i].clusterID, linky_label_list[i].attributeID, linky_label_list[i].label, str_value);
            uint8_t size;
            switch (linky_label_list[i].zb_type)
            {
            case ESP_ZB_ZCL_ATTR_TYPE_U8:
            case ESP_ZB_ZCL_ATTR_TYPE_S8:
                size = sizeof(uint8_t);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_U16:
            case ESP_ZB_ZCL_ATTR_TYPE_S16:
                size = sizeof(uint16_t);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_U24:
            case ESP_ZB_ZCL_ATTR_TYPE_S24:
                size = 24 / 8;
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_U32:
            case ESP_ZB_ZCL_ATTR_TYPE_S32:
                size = sizeof(uint32_t);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_U48:
            case ESP_ZB_ZCL_ATTR_TYPE_S48:
                size = 48 / 8;
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_U64:
            case ESP_ZB_ZCL_ATTR_TYPE_S64:
                size = sizeof(uint64_t);
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING:
            case ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING:
                size = linky_label_list[i].size + 1;
                break;
            case ESP_ZB_ZCL_ATTR_TYPE_BOOL:
                size = sizeof(uint8_t);
                break;
            default:
                ESP_LOGE(TAG, "Zigbee send: %s Unknown type. skipping...", linky_label_list[i].label);
                continue;
                break;
            }
            ret = zigbee_report_attribute(LINKY_TIC_ENDPOINT, linky_label_list[i].clusterID, linky_label_list[i].attributeID, ptr_value, size);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Report attribute failed: 0x%x", ret);
            }
        }
        else
        {
            ESP_LOGI(TAG, "Set attribute cluster: Status: 0x%X 0x%x, attribute: 0x%x, name: %s, value: %lu", status, linky_label_list[i].clusterID, linky_label_list[i].attributeID, linky_label_list[i].label, *(uint32_t *)ptr_value);
            bool success = esp_zb_lock_acquire(zigbee_lock_timeout);
            if (!success)
            {
                ESP_LOGE(TAG, "Failed to acquire lock");
                ret = ESP_ERR_TIMEOUT;
                break;
            }
            status = esp_zb_zcl_set_attribute_val(LINKY_TIC_ENDPOINT, linky_label_list[i].clusterID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, linky_label_list[i].attributeID, ptr_value, false);
            if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Set attribute failed: 0x%x", status);
                ret = ESP_FAIL;
            }
            esp_zb_lock_release();
        }
        if (zigbee_ota_running)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    return ret;
}

uint8_t zigbee_factory_reset()
{
    ESP_LOGI(TAG, "Clearing zigbee storage partition...");
    const char *partition_label = "zb_storage";

    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, partition_label);

    if (partition == NULL)
    {
        ESP_LOGE(TAG, "Can't find partition %s", partition_label);
        return 1;
    }

    esp_err_t erase_result = esp_partition_erase_range(partition, 0, partition->size);

    if (erase_result == ESP_OK)
    {
        ESP_LOGI(TAG, "Partition %s erased", partition_label);
    }
    else
    {
        ESP_LOGE(TAG, "Can't erase partition %s, error %d", partition_label, erase_result);
        return 1;
    }

    config_values.zigbee.state = ZIGBEE_NOT_CONFIGURED;
    config_write();
    ESP_LOGI(TAG, "Zigbee factory reset done");
    return 0;
}

static void zigbee_print_value(char *out_buffer, void *data, linky_label_type_t type)
{
    switch (type)
    {
    case UINT8:
        sprintf(out_buffer, "%u", *(uint8_t *)data);
        break;
    case UINT16:
        sprintf(out_buffer, "%u", *(uint16_t *)data);
        break;
    case UINT32:
        sprintf(out_buffer, "%lu", *(uint32_t *)data);
        break;
    case UINT64:
        sprintf(out_buffer, "%llu", *(uint64_t *)data);
        break;
    case STRING:
        sprintf(out_buffer, "%s", (char *)data);
        break;
    default:
        break;
    };
}

static esp_err_t zigbee_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t messsage)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;

    static esp_err_t ret = ESP_OK;

    if (messsage.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        switch (messsage.upgrade_status)
        {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start, OTA type: 0x%x, OTA file version: 0x%lx, OTA image size: %ld bytes", messsage.ota_header.image_type, messsage.ota_header.file_version, messsage.ota_header.image_size);

            ret = ota_zlib_init();
            if (ret != ESP_OK)
            {
                zigbee_ota_running = false;
                return ret;
            }
            start_time = esp_timer_get_time();
            zigbee_ota_running = true;
            // led_start_pattern(LED_OTA_IN_PROGRESS);

            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:

            // next_subelement:
            size_t payload_size = messsage.payload_size;
            const uint8_t *payload = messsage.payload;

            // ESP_LOG_BUFFER_HEXDUMP(TAG, payload, payload_size, ESP_LOG_INFO);

            total_size = messsage.ota_header.image_size;
            offset += payload_size;

            // ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
            /* Read and process the first sub-element, ignoring everything else */
            // ESP_LOGW(TAG, "OTA header len %zu, received %zu, ota_data_len_ %zu", ota_header_len_, payload_size, ota_data_len_);
            // ESP_LOG_BUFFER_HEXDUMP(TAG, messsage.payload, messsage.payload_size, ESP_LOG_WARN);

            ret = ota_zlib_write(payload, payload_size);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "OTA write failed: 0x%x", ret);
                zigbee_ota_running = false;
                // led_stop_pattern(LED_OTA_IN_PROGRESS);
                return ret;
            }

            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ret = ota_zlib_end();
            if (ret != ESP_OK)
            {
                zigbee_ota_running = false;
                // led_stop_pattern(LED_OTA_IN_PROGRESS);
                return ret;
            }
            zigbee_ota_running = false;
            // led_stop_pattern(LED_OTA_IN_PROGRESS);

            ESP_LOGI(TAG,
                     "-- OTA Information: version: 0x%lx, manufactor code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
                     messsage.ota_header.file_version, messsage.ota_header.manufacturer_code, messsage.ota_header.image_type,
                     messsage.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);

            break;
        default:
            ESP_LOGI(TAG, "OTA status: %d", messsage.upgrade_status);
            break;
        }
    }
    return ret;
}

static uint32_t zigbee_get_hex_version(const char *version)
{
    if (!version)
    {
        ESP_LOGE(TAG, "Invalid version: NULL");
        return UINT32_MAX;
    }

    if (version[0] == 'v' || version[0] == 'V')
    {
        version++;
    }
    char version_buf[10];
    strncpy(version_buf, version, sizeof(version_buf));
    for (int i = 0; i < sizeof(version_buf); i++) // remove the -xxx part of the version
    {
        if (version_buf[i] == '-')
        {
            version_buf[i] = '\0';
        }
    }

    // check if the version is in the format "x.y.z", if yes we convert 3.2.1 to 0x030201
    uint32_t hex_version = 0;
    uint8_t major = 0, minor = 0, revision = 0;
    if (sscanf(version, "%hhu.%hhu.%hhu", &major, &minor, &revision) == 3)
    {
        hex_version = (major << 16) | (minor << 8) | revision;
    }
    else if (sscanf(version, "%hhu.%hhu", &major, &minor) == 2)
    {
        hex_version = (major << 8) | minor;
    }
    else
    {
        ESP_LOGE(TAG, "Invalid version format: %s", version);
        return UINT32_MAX;
    }

    return hex_version;
}

static uint16_t zigbee_get_hex16_version(const char *version)
{
    if (!version)
    {
        ESP_LOGE(TAG, "Invalid version: NULL");
        return UINT16_MAX;
    }

    return (version[0] & 0x0F) << 8 | (version[1] & 0x0F) << 4 | (version[2] & 0x0F);
}
