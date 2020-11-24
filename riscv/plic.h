#ifndef _RISCV_PLIC_H
#define _RISCV_PLIC_H

typedef struct SiFivePLICState SiFivePLICState;

typedef enum PLICMode {
    PLICMode_U,
    PLICMode_S,
    PLICMode_H,
    PLICMode_M
} PLICMode;

typedef struct PLICAddr {
    uint32_t addrid;
    uint32_t hartid;
    PLICMode mode;
} PLICAddr;

struct SiFivePLICState {
    /*< private >*/
    //SysBusDevice parent_obj;

    /*< public >*/
    //MemoryRegion mmio;
    uint32_t num_addrs;
    uint32_t num_harts;
    uint32_t bitfield_words;
    uint32_t num_enables;
    PLICAddr *addr_config;
    uint32_t *source_priority;
    uint32_t *target_priority;
    uint32_t *pending;
    uint32_t *claimed;
    uint32_t *enable;

    /* config */
    char *hart_config;
    uint32_t hartid_base;
    uint32_t num_sources;
    uint32_t num_priorities;
    uint32_t priority_base;
    uint32_t pending_base;
    uint32_t enable_base;
    uint32_t enable_stride;
    uint32_t context_base;
    uint32_t context_stride;
    uint32_t aperture_size;
};

#endif
