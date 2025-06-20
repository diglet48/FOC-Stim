#include "bsp/bootloader.h"

#include "stm32g4xx_hal.h"


#define BOOTLOADER_MAGIC 0x10203040

#define BOOT_ADDR	0x1FFF0000  // my MCU boot code base address
#define MCU_IRQS	101u        // no. of NVIC IRQ inputs

struct boot_vectable_ {
    uint32_t Initial_SP;
    void (*Reset_Handler)(void);
};

#define BOOTVTAB	((struct boot_vectable_ *)BOOT_ADDR)


void BSP_ResetAndJumpToBootloader()
{
    // write magic value to backup registers, will be checked on boot.
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = BOOTLOADER_MAGIC;
    NVIC_SystemReset();
}

void BSP_CheckJumpToBootloader()
{
    HAL_PWR_EnableBkUpAccess();
    if (TAMP->BKP0R == BOOTLOADER_MAGIC) {
        TAMP->BKP0R = 0;

        /* Disable all interrupts */
        __disable_irq();

        /* Disable Systick timer */
        SysTick->CTRL = 0;

        /* Set the clock to the default state */
        HAL_RCC_DeInit();
        TIM1->CR1 &= ~TIM_CR1_CEN;
        HAL_DeInit();

        /* Clear Interrupt Enable Register & Interrupt Pending Register */
        for (uint8_t i = 0; i < (MCU_IRQS + 31u) / 32; i++)
        {
            NVIC->ICER[i]=0xFFFFFFFF;
            NVIC->ICPR[i]=0xFFFFFFFF;
        }

        /* Re-enable all interrupts */
        __enable_irq();

        __HAL_RCC_SYSCFG_CLK_ENABLE();           //make sure syscfg clocked
        __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();  //remap system memory to address 0x000000
        SCB->VTOR = 0; //set vector table offset to 0
        __DSB(); // Ensure the VTOR and SP operations are complete
        __ISB(); // Flush the pipeline because of SP change

        uint32_t const *const SYS_MEM = (uint32_t *)0x1FFF0000;
        __set_MSP(SYS_MEM[0]);                  // Set up the bootloader's stackpointer.
        void (*startBootLoader)(void) = (void (*)(void))SYS_MEM[1];
        startBootLoader();                      // This call does not return.
        while (1) {}
    }
}