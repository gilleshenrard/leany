#############################################################################################################################
# file:  CMakeLists.txt
# date:  09/01/2024
# brief: STM32CubeMX-generated LL drivers CMakeLists file
#############################################################################################################################
#create an object library with the CubeMX-generated files
#	note : object library required to avoid weak functions declarations issues
add_library(CubeMXgenerated OBJECT
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_spi.c
	STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c
)
target_compile_definitions (CubeMXgenerated PUBLIC ${PROJECT_DEFINES})
target_include_directories(CubeMXgenerated PUBLIC ${PROJECT_INCLUDES})
target_compile_options(CubeMXgenerated PUBLIC ${CUSTOM_COMPILE_OPTIONS})
target_link_options(CubeMXgenerated PUBLIC ${CUSTOM_LINK_OPTIONS})
