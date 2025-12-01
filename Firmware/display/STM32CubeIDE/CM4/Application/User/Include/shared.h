#ifndef APPLICATION_USER_INCLUDE_SHARED_H_
#define APPLICATION_USER_INCLUDE_SHARED_H_

typedef struct {
	uint32_t time;
    float t1;
    float t2;
    float h1;
    float h2;
    float pres;
    float lux;
    int bl;
    int rssi;
    int soil1;
    int soil2;
    int soil3;
    int soil4;
    uint32_t status;
} SharedData_t;

// Shared region in SRAM3 (0x30040000..)
#define SHARED_RAM_BASE   (0x30040000UL)
#define SHARED_DATA_PTR   ((SharedData_t *)SHARED_RAM_BASE)

#define HSEM_ID_1 (1U) // For indicating new data

#endif
