#include <sys/time.h>
#include "devices.h"
#include "processor.h"
#include "plic.h"
#include "mmu.h"


static PLICMode char_to_mode(char c)
{
    switch (c) {
    case 'U': return PLICMode_U;
    case 'S': return PLICMode_S;
    case 'H': return PLICMode_H;
    case 'M': return PLICMode_M;
    default:
        fprintf(stderr, "plic: invalid mode '%c'", c);
        exit(1);
    }
}

static char mode_to_char(PLICMode m)
{
    switch (m) {
    case PLICMode_U: return 'U';
    case PLICMode_S: return 'S';
    case PLICMode_H: return 'H';
    case PLICMode_M: return 'M';
    default: return '?';
    }
}


void plic_t::plic_update()
{
    uint32_t addrid;

    /* raise irq on harts where this irq is enabled */
    for (addrid = 0; addrid < plic.num_addrs; addrid++) {
        uint32_t hartid = plic.addr_config[addrid].hartid;
        
        processor_t *cpu = procs[hartid];
        if (!cpu) {
            continue;
        }
        
        PLICMode mode = plic.addr_config[addrid].mode;
        uint32_t level = plic_irqs_pending(addrid);
        switch (mode) {
        case PLICMode_M:
            //riscv_cpu_update_mip(RISCV_CPU(cpu), MIP_MEIP, BOOL_TO_MASK(level));
            break;
        case PLICMode_S:
            //riscv_cpu_update_mip(RISCV_CPU(cpu), MIP_SEIP, BOOL_TO_MASK(level));
            break;
        default:
            break;
        }
    }

}

void plic_t::parse_hart_config()
{
    int addrid, hartid, modes;
    const char *p;
    char c;

    /* count and validate hart/mode combinations */
    addrid = 0, hartid = 0, modes = 0;
    p = plic.hart_config;
    while ((c = *p++)) {
        if (c == ',') {
            addrid += __builtin_popcount(modes);
            modes = 0;
            hartid++;
        } else {
            int m = 1 << char_to_mode(c);
            if (modes == (modes | m)) {
                fprintf(stderr,"plic: duplicate mode '%c' in config: %s",
                             c, plic.hart_config);
                exit(1);
            }
            modes |= m;
        }
    }
    if (modes) {
        addrid += __builtin_popcount(modes);
    }
    hartid++;

    plic.num_addrs = addrid;
    plic.num_harts = hartid;

    /* store hart/mode combinations */
    plic.addr_config = new PLICAddr[plic.num_addrs];
    addrid = 0, hartid = plic.hartid_base;
    p = plic.hart_config;
    while ((c = *p++)) {
        if (c == ',') {
            hartid++;
        } else {
            plic.addr_config[addrid].addrid = addrid;
            plic.addr_config[addrid].hartid = hartid;
            plic.addr_config[addrid].mode = char_to_mode(c);
            addrid++;
        }
    }
}



uint32_t plic_t::plic_claim(uint32_t addrid)
{
    uint32_t i, j;
    uint32_t max_irq = 0;
    uint32_t max_prio = plic.target_priority[addrid];

    for (i = 0; i < plic.bitfield_words; i++) {
        uint32_t pending_enabled_not_claimed =
            (plic.pending[i] & ~plic.claimed[i]) &
            plic.enable[addrid * plic.bitfield_words + i];
        if (!pending_enabled_not_claimed) {
            continue;
        }
        for (j = 0; j < 32; j++) {
            int irq = (i << 5) + j;
            uint32_t prio = plic.source_priority[irq];
            int enabled = pending_enabled_not_claimed & (1 << j);
            if (enabled && prio > max_prio) {
                max_irq = irq;
                max_prio = prio;
            }
        }
    }

    if (max_irq) {
        plic_set_pending(max_irq, false);
        plic_set_claimed(max_irq, true);
    }
    return max_irq;
}

uint32_t plic_t::atomic_set_masked(uint32_t *a, uint32_t mask, uint32_t value)
{
    uint32_t old = *a;
    uint32_t _new = (old & ~mask) | (value & mask);
	*a = _new;
    return _new;
}

void plic_t::plic_set_pending(int irq, bool level)
{
    atomic_set_masked(&plic.pending[irq >> 5], 1 << (irq & 31), -!!level);
}

void plic_t::plic_set_claimed(int irq, bool level)
{
    atomic_set_masked(&plic.claimed[irq >> 5], 1 << (irq & 31), -!!level);
}

int plic_t::plic_irqs_pending(uint32_t addrid)
{
    uint32_t i, j;
    for (i = 0; i < plic.bitfield_words; i++) {
        uint32_t pending_enabled_not_claimed =
            (plic.pending[i] & ~plic.claimed[i]) &
            plic.enable[addrid * plic.bitfield_words + i];
        if (!pending_enabled_not_claimed) {
            continue;
        }
        for (j = 0; j < 32; j++) {
            int irq = (i << 5) + j;
            uint32_t prio = plic.source_priority[irq];
            int enabled = pending_enabled_not_claimed & (1 << j);
            if (enabled && prio > plic.target_priority[addrid]) {
                return 1;
            }
        }
    }
    return 0;
}

plic_t::plic_t(std::vector<processor_t*>& procs, reg_t num_priorities,
               reg_t plic_size, reg_t plic_ndev)
  : procs(procs) 
{
    plic.num_sources = plic_ndev;
    plic.bitfield_words = (plic.num_sources + 31) >> 5;
    plic.num_enables = plic.bitfield_words * plic.num_addrs;
    plic.pending = new uint32_t(plic.bitfield_words);
    plic.source_priority = new uint32_t(plic.num_sources);
    plic.target_priority = new uint32_t(plic.num_addrs);
    plic.claimed = new uint32_t(plic.bitfield_words);
    plic.enable = new uint32_t(plic.num_enables);

    plic.hartid_base = 0;
    plic.num_priorities = num_priorities;
    plic.priority_base = 0x4;
    plic.pending_base = 0x001000;
    plic.enable_base = 0x002000;
    plic.enable_stride = 0x80;
    plic.context_base = 0x200000;
    plic.context_stride = 0x1000;
    plic.aperture_size = plic_size;
}

plic_t::plic_t(std::vector<processor_t*>& procs, 
    char *hart_config,
    uint32_t hartid_base, uint32_t num_sources,
    uint32_t num_priorities, uint32_t priority_base,
    uint32_t pending_base, uint32_t enable_base,
    uint32_t enable_stride, uint32_t context_base,
    uint32_t context_stride, uint32_t aperture_size):
    procs(procs)
{

}

bool plic_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
    /* writes must be 4 byte words */
    if ((addr & 0x3) != 0) {
        return false;
    }

    if (addr >= plic.priority_base && /* 4 bytes per source */
        addr < plic.priority_base + (plic.num_sources << 2))
    {
        uint32_t irq = ((addr - plic.priority_base) >> 2) + 1;
        plic.source_priority[irq];
    } else if (addr >= plic.pending_base && /* 1 bit per source */
               addr < plic.pending_base + (plic.num_sources >> 3))
    {
        uint32_t word = (addr - plic.pending_base) >> 2;
        plic.pending[word];
    } else if (addr >= plic.enable_base && /* 1 bit per source */
             addr < plic.enable_base + plic.num_addrs * plic.enable_stride)
    {
        uint32_t addrid = (addr - plic.enable_base) / plic.enable_stride;
        uint32_t wordid = (addr & (plic.enable_stride - 1)) >> 2;
        if (wordid < plic.bitfield_words) {
            plic.enable[addrid * plic.bitfield_words + wordid];
        }
    } else if (addr >= plic.context_base && /* 1 bit per source */
             addr < plic.context_base + plic.num_addrs * plic.context_stride)
    {
        uint32_t addrid = (addr - plic.context_base) / plic.context_stride;
        uint32_t contextid = (addr & (plic.context_stride - 1));
        if (contextid == 0) {
            plic.target_priority[addrid];
        } else if (contextid == 4) {
            uint32_t value = plic_claim(addrid);
            //sifive_plic_update(plic);
        }
    }

  return true;
}

bool plic_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
    uint64_t value = *(uint64_t *) bytes;
    /* writes must be 4 byte words */
    if ((addr & 0x3) != 0) {
        return false;
    }

    if (addr >= plic.priority_base && /* 4 bytes per source */
        addr < plic.priority_base + (plic.num_sources << 2))
    {
        uint32_t irq = ((addr - plic.priority_base) >> 2) + 1;
        plic.source_priority[irq] = value & 7;
        plic_update();
        return true;
    } else if (addr >= plic.pending_base && /* 1 bit per source */
               addr < plic.pending_base + (plic.num_sources >> 3))
    {
        return true;
    } else if (addr >= plic.enable_base && /* 1 bit per source */
        addr < plic.enable_base + plic.num_addrs * plic.enable_stride)
    {
        uint32_t addrid = (addr - plic.enable_base) / plic.enable_stride;
        uint32_t wordid = (addr & (plic.enable_stride - 1)) >> 2;
        if (wordid < plic.bitfield_words) {
            plic.enable[addrid * plic.bitfield_words + wordid] = value;
            return true;
        }
    } else if (addr >= plic.context_base && /* 4 bytes per reg */
        addr < plic.context_base + plic.num_addrs * plic.context_stride)
    {
        uint32_t addrid = (addr - plic.context_base) / plic.context_stride;
        uint32_t contextid = (addr & (plic.context_stride - 1));
        if (contextid == 0) {
            if (value <= plic.num_priorities) {
                plic.target_priority[addrid] = value;
                plic_update();
            }
            return true;
        } else if (contextid == 4) {
            if (value < plic.num_sources) {
                plic_set_claimed(value, false);
                plic_update();
            }
            return true;
        }
    }

  return true;
}

