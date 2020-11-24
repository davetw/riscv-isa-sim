#include <sys/time.h>
#include "devices.h"
#include "processor.h"
#include "plic.h"
#include "mmu.h"


void plic_t::plic_update()
{
    int addrid;

    /* raise irq on harts where this irq is enabled */
    for (addrid = 0; addrid < plic.num_addrs; addrid++) {
        uint32_t hartid = plic.addr_config[addrid].hartid;
        
        processor_t *cpu = procs[hartid];
        if (!cpu) {
            continue;
        }
        
        PLICMode mode = plic.addr_config[addrid].mode;
        int level = plic_irqs_pending(addrid);
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



uint32_t plic_t::plic_claim(uint32_t addrid)
{
    int i, j;
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
    int i, j;
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



plic_t::plic_t(std::vector<processor_t*>& procs)
  : procs(procs) 
{
    plic.pending = new uint32_t(plic.bitfield_words);
    plic.source_priority = new uint32_t(plic.num_sources);
    plic.target_priority = new uint32_t(plic.num_addrs);
    plic.claimed = new uint32_t(plic.bitfield_words);
    plic.enable = new uint32_t(plic.num_enables);
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

