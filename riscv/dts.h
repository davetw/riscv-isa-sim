// See LICENSE for license details.
#ifndef _RISCV_DTS_H
#define _RISCV_DTS_H

#include "processor.h"
#include "mmu.h"
#include <string>
#include <tuple>
#include <vector>

std::string make_dts(size_t insns_per_rtc_tick, size_t cpu_hz,
                     reg_t initrd_start, reg_t initrd_end,
                     const char* bootargs,
                     std::vector<processor_t*> procs,
                     std::vector<std::pair<reg_t, mem_t*>> mems);

std::string dts_compile(const std::string& dts);

int fdt_get_offset(void *fdt, const char *field);
int fdt_get_first_subnode(void *fdt, int node);
int fdt_get_next_subnode(void *fdt, int node);

int fdt_parse_plic(void *fdt, reg_t *plic_addr, reg_t *plic_size, 
                   reg_t *plic_maxprio, reg_t *plic_ndev, char* plic_config,
                   const char *compatible);
int fdt_parse_clint(void *fdt, reg_t *clint_addr,
                    const char *compatible);
int fdt_parse_pmp_num(void *fdt, int cpu_offset, reg_t *pmp_num);
int fdt_parse_pmp_alignment(void *fdt, int cpu_offset, reg_t *pmp_align);
int fdt_parse_mmu_type(void *fdt, int cpu_offset, char *mmu_type);
int fdt_parse_hartid(void *fdt, int cpu_offset, reg_t *hartid);

/**
 * @param fdt
 * @param val fill the parsed worldguard device
 *            1nd: device base
 *            2rd: device size
 *            3th: hard id for wgMakder, useless for other
 */
int fdt_parse_wg_marker(void *fdt,
                        std::vector<std::tuple<reg_t, reg_t, reg_t>> &devs);

/**
 * @param fdt
 * @param val fill the parsed worldguard filter device
 *            1nd: device base
 *            2rd: device size
 *            3th: client base
 *            4th: clinet size
 */
int fdt_parse_wg_filter(void *fdt,
                        std::vector<std::tuple<reg_t, reg_t, reg_t, reg_t>> &devs);
/**
 * @param fdt
 * @param val fill the parsed worldguard pmp device
 *            1nd: device base
 *            2rd: device size
 *            3th: client base
 *            4th: clinet size
 */
int fdt_parse_wg_pmp(void *fdt,
                        std::vector<std::tuple<reg_t, reg_t, reg_t, reg_t>> &devs);
#endif
