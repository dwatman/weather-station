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
    int soil[6];
    uint32_t status;
} SharedData_t;

// Shared region in SRAM3 (0x30040000..)
#define SHARED_RAM_BASE   (0x30040000UL)
#define SHARED_DATA_PTR   ((SharedData_t *)SHARED_RAM_BASE)

#define HSEM_ID_1 (1U) // For indicating new data

#define STATUS_CONNECTED (1 << 0)
#define STATUS_GARAGE    (1 << 1)

#endif
