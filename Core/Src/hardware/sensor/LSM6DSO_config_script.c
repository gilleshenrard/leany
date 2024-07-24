/**
 * @file LSM6DSO_config_script.c
 * @brief Declare an array with all the registers and values to be sent sequentially to configure the LSM6DSO
 * @author Gilles Henrard
 * @date 24/07/2024
 *
 * @note Additional information can be found in :
 *   - Datasheet : https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 *   - AN5192 (always-on 3-axis accelerometer and 3-axis gyroscope) : https://www.st.com/resource/en/application_note/an5192-lsm6dso-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf
 *   - AN5226 (Finite State Machine) : https://www.st.com/resource/en/application_note/an5226-lsm6dso-finite-state-machine-stmicroelectronics.pdf
 *   - DT0058 (Design tip) : https://www.st.com/resource/en/design_tip/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf
 */
#include "LSM6DSO_config_script.h"
#include "LSM6DSO_registers.h"

const registerValue_t initialisationArray[NB_INIT_REG] = {
    {   CTRL3_C, LSM6_REBOOT_MEMORY | LSM6_SOFTWARE_RESET | LSM6_INT_ACTIVE_LOW}, //reboot MEMS memory and reset software
    { INT1_CTRL,                                              INT1_AXL_DATA_RDY}, //enable the accelerometer DATA READY interrupt on INT1
    {FIFO_CTRL4,                                               FIFO_MODE_BYPASS}, //disable the FIFO (bypass mode)
    {  CTRL1_XL,                                                 LSM6_ODR_416HZ}, //set the accelerometer in high-performance mode
    {   CTRL2_G,                                                 LSM6_ODR_416HZ}, //set the gyroscope in high-performance mode
};
