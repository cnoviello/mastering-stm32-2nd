typedef unsigned long uint32_t;

/* memory and peripheral start addresses */
#define FLASH_BASE      0x08000000
#define SRAM_BASE       0x20000000
#define PERIPH_BASE     0x40000000
#define AHB1PERIPH_BASE	(PERIPH_BASE + 0x00020000)

/* Work out end of RAM address as initial stack pointer */
#define SRAM_SIZE       128*1024     // STM32G474RE has 128 KB of RAM
#define SRAM_END        (SRAM_BASE + SRAM_SIZE)

/* RCC peripheral addresses applicable to GPIOA */
#define RCC_BASE        (AHB1PERIPH_BASE + 0x1000)
#define RCC_AHB2ENR     ((uint32_t*)(RCC_BASE + (0x4C)))

/* GPIOA peripheral addresses */
#define GPIOA_BASE      (PERIPH_BASE + 0x8000000)
#define GPIOA_MODER     ((uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_ODR       ((uint32_t*)(GPIOA_BASE + 0x14))

/* User functions */
int main(void);
void delay(uint32_t count);

/* Minimal vector table */
uint32_t *vector_table[] __attribute__((section(".isr_vector"))) = {
		(uint32_t *)SRAM_END,   // initial stack pointer
		(uint32_t *)main        // main as Reset_Handler
};

int main() {
	/* enable clock on GPIOA peripheral */
	*RCC_AHB2ENR |= 0x1;// << 17;
	*GPIOA_MODER ^= 0x1 << 11; // Sets MODER[11:10] = 0x1

	while(1) {
		*GPIOA_ODR = 0x20;
		delay(400000);
		*GPIOA_ODR = 0x0;
		delay(400000);
	}
}

void delay(uint32_t count) {
	while(count--);
}
