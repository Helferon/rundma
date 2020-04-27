#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>

#include "hw-addresses.h" // for DMA addresses, etc.

#define PAGE_SIZE 4096

// Relative offsets for DMA registers
#define DMACH(n) (0x100 * (n))

#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1 << 31)
#define DMA_CS_ACTIVE (1 << 0)

#define DMA_DEBUG_READ_ERROR (1 << 2)
#define DMA_DEBUG_FIFO_ERROR (1 << 1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1 << 0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC (1 << 4)
#define DMA_CB_TI_SRC_INC (1 << 8)

// Enable the channel
void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value)
{
    uint32_t cur = *dest;
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //added safety for when crossing memory barriers.
}

struct DmaChannelHeader
{
    uint32_t CS;        // Status
    uint32_t CONBLK_AD; //Control Block Address
    uint32_t TI;        //transfer information; see DmaControlBlock.TI for description
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD;   //Destination address
    uint32_t TXFR_LEN;  //transfer length.
    uint32_t STRIDE;    //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t DEBUG;     //controls debug settings
};

struct DmaControlBlock
{
    uint32_t TI;        //transfer information
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD;   //Destination address
    uint32_t TXFR_LEN;  //transfer length.
    uint32_t STRIDE;    //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
};

// Allocate a page & simultaneously determine its physical address.
void makeVirtPhysPage(void **virtAddr, void **physAddr)
{
    *virtAddr = valloc(PAGE_SIZE); //allocate one page of RAM

    //force page into RAM and then lock it there
    ((int *)*virtAddr)[0] = 1;
    mlock(*virtAddr, PAGE_SIZE);
    memset(*virtAddr, 0, PAGE_SIZE); //zero-fill the page for convenience

    //Magic to determine the physical address for this page:
    uint64_t pageInfo;
    int file = open("/proc/self/pagemap", 'r');
    lseek(file, ((size_t)*virtAddr) / PAGE_SIZE * 8, SEEK_SET);
    read(file, &pageInfo, 8);

    *physAddr = (void *)(size_t)(pageInfo * PAGE_SIZE);
    printf("makeVirtPhysPage virtual to phys: %p -> %p\n", *virtAddr, *physAddr);
}

//call with virtual address to deallocate a page allocated with makeVirtPhysPage
void freeVirtPhysPage(void *virtAddr)
{
    munlock(virtAddr, PAGE_SIZE);
    free(virtAddr);
}

// Map a physical address into our virtual address space. memfd is the file descriptor for /dev/mem
volatile uint32_t *mapPeripheral(int memfd, int addr)
{
    // /dev/mem behaves as a file. We need to map that file into memory:
    void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, addr);
    if (mapped == MAP_FAILED)
    {
        printf("failed to map memory (did you remember to run as root?)\n");
        exit(1);
    }
    else
    {
        printf("mapped: %p\n", mapped);
    }
    return (volatile uint32_t *)mapped;
}

int main()
{
    //cat /sys/module/dma/parameters/dmachans gives a bitmask of DMA channels that are not used by GPU. Results: ch 1, 3, 6, 7 are reserved.
    //dmesg | grep "DMA"; results: Ch 2 is used by SDHC host
    //ch 0 is known to be used for graphics acceleration
    //Thus, applications can use ch 4, 5, or the LITE channels @ 8 and beyond.
    int dmaChNum = 5;
    //First, open the linux device, /dev/mem
    //dev/mem provides access to the physical memory of the entire processor+ram
    //This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0)
    {
        printf("Failed to open /dev/mem (did you remember to run as root?)\n");
        exit(1);
    }
    //now map /dev/mem into memory, but only map specific peripheral sections:
    volatile uint32_t *dmaBaseMem = mapPeripheral(memfd, DMA_BASE);

    //configure DMA:
    //allocate 1 page for the source and 1 page for the destination:
    void *virtSrcPage, *physSrcPage;
    makeVirtPhysPage(&virtSrcPage, &physSrcPage);
    void *virtDestPage, *physDestPage;
    makeVirtPhysPage(&virtDestPage, &physDestPage);

    //write a few bytes to the source page:
    char *srcArray = (char *)virtSrcPage;
    srcArray[0] = 'h';
    srcArray[1] = 'e';
    srcArray[2] = 'l';
    srcArray[3] = 'l';
    srcArray[4] = 'o';
    srcArray[5] = ' ';
    srcArray[6] = 'w';
    srcArray[7] = 'o';
    srcArray[8] = 'r';
    srcArray[9] = 'l';
    srcArray[10] = 'd';
    srcArray[11] = 0; //null terminator used for printf call.

    //allocate 1 page for the control blocks
    void *virtCbPage, *physCbPage;
    makeVirtPhysPage(&virtCbPage, &physCbPage);

    //dedicate the first 8 words of this page to holding the cb.
    struct DmaControlBlock *cb1 = (struct DmaControlBlock *)virtCbPage;

    //fill the control block:
    cb1->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
    cb1->SOURCE_AD = (uint32_t)physSrcPage;           //set source and destination DMA address
    cb1->DEST_AD = (uint32_t)physDestPage;
    cb1->TXFR_LEN = 12; //transfer 12 bytes
    cb1->STRIDE = 0;    //no 2D stride
    cb1->NEXTCONBK = 0; //no next control block

    printf("destination was initially: '%s'\n", (char *)virtDestPage);

    //enable DMA channel (it's probably already enabled, but we want to be sure):
    writeBitmasked(dmaBaseMem + DMAENABLE / 4, 1 << dmaChNum, 1 << dmaChNum);

    //configure the DMA header to point to our control block:
    volatile struct DmaChannelHeader *dmaHeader = (volatile struct DmaChannelHeader *)(dmaBaseMem + (DMACH(dmaChNum)) / 4); //dmaBaseMem is a uint32_t ptr, so divide by 4 before adding byte offset
    dmaHeader->CS = DMA_CS_RESET;                                                                                           //make sure to disable dma first.
    sleep(1);                                                                                                               //give time for the reset command to be handled.
    dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR;                     // clear debug error flags
    dmaHeader->CONBLK_AD = (uint32_t)physCbPage;                                                                            //we have to point it to the PHYSICAL address of the control block (cb1)
    dmaHeader->CS = DMA_CS_ACTIVE;                                                                                          //set active bit, but everything else is 0.

    sleep(1); //give time for copy to happen

    printf("destination reads: '%s'\n", (char *)virtDestPage);

    //cleanup
    freeVirtPhysPage(virtCbPage);
    freeVirtPhysPage(virtDestPage);
    freeVirtPhysPage(virtSrcPage);
    return 0;
}
