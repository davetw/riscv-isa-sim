#ifndef _RISCV_DEVICES_H
#define _RISCV_DEVICES_H

#include "decode.h"
#include "mmio_plugin.h"
#include "plic.h"
#include "memtracer.h"
#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <stdexcept>

class processor_t;
class sim_t;

class abstract_device_t {
 public:
  virtual bool load(reg_t addr, size_t len, uint8_t* bytes) = 0;
  virtual bool store(reg_t addr, size_t len, const uint8_t* bytes) = 0;
  virtual ~abstract_device_t() {}
};

class bus_t : public abstract_device_t {
 public:
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  void add_device(reg_t addr, abstract_device_t* dev);

  std::pair<reg_t, abstract_device_t*> find_device(reg_t addr);

 private:
  std::map<reg_t, abstract_device_t*> devices;
};

class rom_device_t : public abstract_device_t {
 public:
  rom_device_t(std::vector<char> data);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  const std::vector<char>& contents() { return data; }
 private:
  std::vector<char> data;
};

class mem_t : public abstract_device_t {
 public:
  mem_t(size_t size) : len(size) {
    if (!size)
      throw std::runtime_error("zero bytes of target memory requested");
    data = (char*)calloc(1, size);
    if (!data)
      throw std::runtime_error("couldn't allocate " + std::to_string(size) + " bytes of target memory");
  }
  mem_t(const mem_t& that) = delete;
  ~mem_t() { free(data); }

  bool load(reg_t addr, size_t len, uint8_t* bytes) { return false; }
  bool store(reg_t addr, size_t len, const uint8_t* bytes) { return false; }
  char* contents() { return data; }
  size_t size() { return len; }

 private:
  char* data;
  size_t len;
};

class clint_t : public abstract_device_t {
 public:
  clint_t(std::vector<processor_t*>&, uint64_t freq_hz, bool real_time);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  size_t size() { return CLINT_SIZE; }
  void increment(reg_t inc);
 private:
  typedef uint64_t mtime_t;
  typedef uint64_t mtimecmp_t;
  typedef uint32_t msip_t;
  std::vector<processor_t*>& procs;
  uint64_t freq_hz;
  bool real_time;
  uint64_t real_time_ref_secs;
  uint64_t real_time_ref_usecs;
  mtime_t mtime;
  std::vector<mtimecmp_t> mtimecmp;
};

class plic_t: public abstract_device_t {
 public:
  plic_t(std::vector<processor_t*>& procs, reg_t num_priorities,
         reg_t plic_size, reg_t plic_ndev, char* plic_config);
  plic_t(std::vector<processor_t*>& procs, char *hart_config,
    uint32_t hartid_base, uint32_t num_sources,
    uint32_t num_priorities, uint32_t priority_base,
    uint32_t pending_base, uint32_t enable_base,
    uint32_t enable_stride, uint32_t context_base,
    uint32_t context_stride, uint32_t aperture_size);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);
  uint32_t plic_claim(uint32_t addrid);
  void plic_set_pending(int irq, bool level);
  void plic_set_claimed(int irq, bool level);
  int plic_irqs_pending(uint32_t addrid);
  uint32_t atomic_set_masked(uint32_t *a, uint32_t mask, uint32_t value);
  void plic_update();
  void plic_print_status();
  void parse_hart_config();

 private:
  SiFivePLICState plic;
  std::vector<processor_t*>& procs;
};

class mmio_plugin_device_t : public abstract_device_t {
 public:
  mmio_plugin_device_t(const std::string& name, const std::string& args);
  virtual ~mmio_plugin_device_t() override;

  virtual bool load(reg_t addr, size_t len, uint8_t* bytes) override;
  virtual bool store(reg_t addr, size_t len, const uint8_t* bytes) override;

 private:
  mmio_plugin_t plugin;
  void* user_data;
};

class wg_marker_t : public abstract_device_t {
 public:
  wg_marker_t(const sim_t *sim, processor_t* proc, uint32_t wid, uint32_t wid_trusted);;
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);

  uint32_t get_wid() { return wid;}

 private:
  const sim_t *sim;
  processor_t *proc;
  uint32_t wid;
  uint32_t wid_trusted;
  uint32_t lock;
};

class wg_filter_t : public abstract_device_t {
 public:
  wg_filter_t(const sim_t *sim, uint32_t wid, uint32_t wid_trusted,
              uint64_t addr, uint64_t size);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);

  bool is_valid(uint32_t wid, uint64_t addr, uint64_t len);
  bool in_range(uint64_t addr, uint64_t len);

 private:
  const sim_t *sim;
  uint32_t wid;
  uint32_t wid_trusted;
  uint64_t addr;
  uint64_t size;
};

class wg_pmp_t : public abstract_device_t {
 public:
  wg_pmp_t(const sim_t *sim, uint32_t wid_trusted,
           uint64_t addr, uint64_t size);
  bool load(reg_t addr, size_t len, uint8_t* bytes);
  bool store(reg_t addr, size_t len, const uint8_t* bytes);

  bool is_valid(uint32_t wid, uint64_t addr, uint64_t len, access_type type);
  bool in_range(uint64_t addr, uint64_t len);

 private:
  const sim_t *sim;
  uint32_t wid_trusted;
  std::vector<std::tuple<uint32_t, uint64_t, uint64_t, uint32_t>> blks;
  uint64_t addr;
  uint64_t size;
};
#endif
