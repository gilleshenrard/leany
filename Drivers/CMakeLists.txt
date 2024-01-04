#############################################################################################################################
# file:  CMakeLists.txt
# date:  04/01/2023
# brief: STM32CubeMX-generated HAL drivers CMakeLists file
#############################################################################################################################
#create an object library with the CubeMX-generated files
#	note : object library required to avoid weak functions declarations issues
add_library(CubeMXgenerated OBJECT
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
)
target_compile_definitions (CubeMXgenerated PUBLIC ${PROJECT_DEFINES})
target_include_directories(CubeMXgenerated PUBLIC ${PROJECT_INCLUDES})
target_compile_options(CubeMXgenerated PUBLIC ${CUSTOM_COMPILE_OPTIONS})
target_link_options(CubeMXgenerated PUBLIC ${CUSTOM_LINK_OPTIONS})
