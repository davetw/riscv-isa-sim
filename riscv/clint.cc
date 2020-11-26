#include <sys/time.h>
#include "devices.h"
#include "processor.h"
#include "sim.h"

clint_t::clint_t(std::vector<processor_t*>& procs, uint64_t freq_hz, bool real_time)
  : procs(procs), freq_hz(freq_hz), real_time(real_time), mtime(0), mtimecmp(procs.size())
{
  struct timeval base;

  gettimeofday(&base, NULL);

  real_time_ref_secs = base.tv_sec;
  real_time_ref_usecs = base.tv_usec;
}

/* 0000 msip hart 0
 * 0004 msip hart 1
 * 4000 mtimecmp hart 0 lo
 * 4004 mtimecmp hart 0 hi
 * 4008 mtimecmp hart 1 lo
 * 400c mtimecmp hart 1 hi
 * bff8 mtime lo
 * bffc mtime hi
 */

#define MSIP_BASE	0x0
#define MTIMECMP_BASE	0x4000
#define MTIME_BASE	0xbff8

bool clint_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  increment(0);
  if (addr >= MSIP_BASE && addr + len <= MSIP_BASE + procs.size()*sizeof(msip_t)) {
    std::vector<msip_t> msip(procs.size());
    for (size_t i = 0; i < procs.size(); ++i)
      msip[i] = !!(procs[i]->state.mip & MIP_MSIP);
    memcpy(bytes, (uint8_t*)&msip[0] + addr - MSIP_BASE, len);
  } else if (addr >= MTIMECMP_BASE && addr + len <= MTIMECMP_BASE + procs.size()*sizeof(mtimecmp_t)) {
    memcpy(bytes, (uint8_t*)&mtimecmp[0] + addr - MTIMECMP_BASE, len);
  } else if (addr >= MTIME_BASE && addr + len <= MTIME_BASE + sizeof(mtime_t)) {
    memcpy(bytes, (uint8_t*)&mtime + addr - MTIME_BASE, len);
  } else {
    return false;
  }
  return true;
}

bool clint_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if (addr >= MSIP_BASE && addr + len <= MSIP_BASE + procs.size()*sizeof(msip_t)) {
    std::vector<msip_t> msip(procs.size());
    std::vector<msip_t> mask(procs.size(), 0);
    memcpy((uint8_t*)&msip[0] + addr - MSIP_BASE, bytes, len);
    memset((uint8_t*)&mask[0] + addr - MSIP_BASE, 0xff, len);
    for (size_t i = 0; i < procs.size(); ++i) {
      if (!(mask[i] & 0xFF)) continue;
      procs[i]->state.mip &= ~MIP_MSIP;
      if (!!(msip[i] & 1))
        procs[i]->state.mip |= MIP_MSIP;
    }
  } else if (addr >= MTIMECMP_BASE && addr + len <= MTIMECMP_BASE + procs.size()*sizeof(mtimecmp_t)) {
    memcpy((uint8_t*)&mtimecmp[0] + addr - MTIMECMP_BASE, bytes, len);
  } else if (addr >= MTIME_BASE && addr + len <= MTIME_BASE + sizeof(mtime_t)) {
    memcpy((uint8_t*)&mtime + addr - MTIME_BASE, bytes, len);
  } else {
    return false;
  }
  increment(0);
  return true;
}

void clint_t::increment(reg_t inc)
{
  if (real_time) {
   struct timeval now;
   uint64_t diff_usecs;

   gettimeofday(&now, NULL);
   diff_usecs = ((now.tv_sec - real_time_ref_secs) * 1000000) + (now.tv_usec - real_time_ref_usecs);
   mtime = diff_usecs * freq_hz / 1000000;
  } else {
    mtime += inc;
  }
  for (size_t i = 0; i < procs.size(); i++) {
    procs[i]->state.mip &= ~MIP_MTIP;
    if (mtime >= mtimecmp[i])
      procs[i]->state.mip |= MIP_MTIP;
  }
}

static inline bool is_cover(uint64_t base, uint64_t len, uint64_t req_addr, uint64_t req_len)
{
    if (base <= req_addr && (req_addr + req_len) <= (base + len))
      return true;

    return false;
}

//wg_markker_t
wg_marker_t::wg_marker_t(const sim_t *sim, processor_t *proc,
                         uint32_t wid, uint32_t wid_trusted)
  : sim(sim),
    proc(proc),
    wid(0),
    wid_trusted(wid_trusted),
    lock(0)
{
    if (wid > wid_trusted) {
      fprintf(stderr, "wrong wid(%u), > wid_trusted(%u)\n", wid, wid_trusted);
      exit(1);
    } 

    wid |= 1ul << wid;
}

bool wg_marker_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  if ((sim->get_current_core()->wg_marker->get_wid() & (1ul <<  wid_trusted)) == 0)
    return false;

  if (addr >= 0 && addr + len <= 4) {
    memcpy(bytes, &wid, len);
    return true;
  } else if (addr >= 4 && addr + len <= 8) {
    memcpy(bytes, &lock, len);
    return true;
  }

  return false;
}

bool wg_marker_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if ((sim->get_current_core()->wg_marker->get_wid() & (1ul <<  wid_trusted)) == 0)
    return false;

  if (addr >= 0 && addr + len <= 4) {
    memcpy(&wid, bytes, len);
    return true;
  } else if (addr >= 4 && addr + len <= 8) {
    if (lock)
      return false;

    memcpy(&lock, bytes, len);
    return true;
  }

  return false;
}

//wg_filter_t
wg_filter_t::wg_filter_t(const sim_t *sim, uint32_t wid, uint32_t wid_trusted,
                         uint64_t addr, uint64_t size)
  : sim(sim),
    wid(wid),
    wid_trusted(wid_trusted),
    addr(addr),
    size(size)
{
    if (wid >= wid_trusted) {
      fprintf(stderr, "wrong wid(%u), > wid_trusted(%u)\n", wid, wid_trusted);
      exit(1);
    } 

    wid |= 1ul << wid;
}

bool wg_filter_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  if ((sim->get_current_core()->wg_marker->get_wid() & (1ul << wid_trusted)) == 0)
    return false;

  if (addr >= 0 && addr + len <= 4) {
    memcpy(bytes, &wid, 4);
    return true;
  }

  return false;
}

bool wg_filter_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if ((sim->get_current_core()->wg_marker->get_wid() & (1ul << wid_trusted)) == 0)
    return false;

  if (addr >= 0 && addr + len <= 4) {
    memcpy(&wid, bytes, 4);
    return true;
  }

  return false;
}

bool wg_filter_t::is_valid(uint32_t req_wid, uint64_t req_addr, uint64_t req_len)
{
  if (req_wid == 0)
    return false;

  if (req_wid > wid_trusted)
    return false;

  if (wid & (1ul << req_wid) || req_wid == wid_trusted)
      return true;

  return false;
}

bool wg_filter_t::in_range(uint64_t req_addr, uint64_t req_len)
{
  return is_cover(addr, size, req_addr, req_len);
}

//wg_pmp_t
wg_pmp_t::wg_pmp_t(const sim_t *sim, uint32_t wid_trusted,
                   uint64_t addr, uint64_t size)
  : sim(sim),
    wid_trusted(wid_trusted),
    addr(addr),
    size(size)
{
  for (size_t idx = 0; idx < wid_trusted; ++idx) {
    blks.push_back(std::make_tuple(0, 0, 0, 0));
  }
}

bool wg_pmp_t::load(reg_t addr, size_t len, uint8_t* bytes)
{
  if ((sim->get_current_core()->wg_marker->get_wid() & (1ul << wid_trusted)) == 0)
    return false;

  auto blk_idx = addr / 0x18;

  if (blk_idx >= blks.size())
    return false;

  if ((addr + len) >= blks.size() * 0x18)
    return false;

  auto &blk = blks[blk_idx];
  addr -= blk_idx * 0x18;

  if (addr >= 0 && (addr + len) <= 4) {
    memcpy(bytes, &std::get<0>(blk), len);
    return true;
  } else if (addr >= 0x04 && (addr + len) <= 0x0c) {
    memcpy(bytes, &std::get<0>(blk), len);
    return true;
  } else if (addr >= 0x0c && (addr + len) <= 0x14) {
    memcpy(bytes, &std::get<0>(blk), len);
    return true;
  } else if (addr >= 0x14 && (addr + len) <= 0x18) {
    memcpy(bytes, &std::get<0>(blk), len);
    return true;
  }

  return false;
}

bool wg_pmp_t::store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if ((sim->get_current_core()->wg_marker->get_wid() & (1ul << wid_trusted)) == 0)
    return false;

  auto blk_idx = addr / 0x18;

  if (blk_idx >= blks.size())
    return false;

  if ((addr + len) >= blks.size() * 0x18)
    return false;

  auto &blk = blks[blk_idx];
  addr -= blk_idx * 0x18;

  if (std::get<3>(blk))
    return false;

  if (addr >= 0 && (addr + len) <= 4) {
    memcpy(&std::get<0>(blk), bytes, len);
    return true;
  } else if (addr >= 0x04 && (addr + len) <= 0x0c) {
    memcpy(&std::get<0>(blk), bytes, len);
    return true;
  } else if (addr >= 0x0c && (addr + len) <= 0x14) {
    memcpy(&std::get<0>(blk), bytes, len);
    return true;
  } else if (addr >= 0x14 && (addr + len) <= 0x18) {
    memcpy(&std::get<0>(blk), bytes, len);
    return true;
  }
  return false;
}

bool wg_pmp_t::is_valid(uint32_t req_wid, uint64_t req_addr, uint64_t req_len,
                        access_type type)
{
  if (req_wid == 0)
    return false;

  if (req_wid > wid_trusted)
    return false;

  for (auto blk : blks) {
    auto wid = (std::get<0>(blk) >> (2 * req_wid)) & 0x3;
    auto start = std::get<1>(blk) << 12;
    auto end = (std::get<1>(blk) + std::get<2>(blk)) << 12;
    if (start <= req_addr
        && req_addr + req_len <= end) {
      if (type == STORE) {
        if (wid & 0x1)
          return true;
      } else  {
        if (wid & 0x2)
          return true;
      }
    }
  }

  return false;
}

bool wg_pmp_t::in_range(uint64_t req_addr, uint64_t req_len)
{
  return is_cover(addr, size, req_addr, req_len);
}

