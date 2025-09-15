#ifndef _VARIANT_ESP32C3_SUPER_MINI_
#define _VARIANT_ESP32C3_SUPER_MINI_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// I2C (Wire) & OLED
#define WIRE_INTERFACES_COUNT (1)
#define I2C_SDA (1)
#define I2C_SCL (0)

#define USE_SSD1306

// GPS
#undef GPS_RX_PIN
#undef GPS_TX_PIN
#define GPS_RX_PIN (20)
#define GPS_TX_PIN (21)

// Button
#define BUTTON_PIN (9) // BOOT button

// BLE
#define USE_BLE_RF

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
