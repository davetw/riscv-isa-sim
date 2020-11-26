// See LICENSE for license details.

#include "dts.h"
#include "libfdt.h"
#include <deque>
#include <iostream>
#include <sstream>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

std::string make_dts(size_t insns_per_rtc_tick, size_t cpu_hz,
                     reg_t initrd_start, reg_t initrd_end,
                     const char* bootargs,
                     std::vector<processor_t*> procs,
                     std::vector<std::pair<reg_t, mem_t*>> mems)
{
  std::stringstream s;
  s << std::dec <<
         "/dts-v1/;\n"
         "\n"
         "/ {\n"
         "  #address-cells = <2>;\n"
         "  #size-cells = <2>;\n"
         "  compatible = \"ucbbar,spike-bare-dev\";\n"
         "  model = \"ucbbar,spike-bare\";\n"
         "  chosen {\n";
  if (initrd_start < initrd_end) {
    s << "    linux,initrd-start = <" << (size_t)initrd_start << ">;\n"
         "    linux,initrd-end = <" << (size_t)initrd_end << ">;\n";
    if (!bootargs)
      bootargs = "root=/dev/ram console=hvc0 earlycon=sbi";
  } else {
    if (!bootargs)
      bootargs = "console=hvc0 earlycon=sbi";
  }
    s << "    bootargs = \"";
  for (size_t i = 0; i < strlen(bootargs); i++) {
    if (bootargs[i] == '"')
    s << '\\' << bootargs[i];
    else
    s << bootargs[i];
  }
    s << "\";\n";
    s << "  };\n"
         "  cpus {\n"
         "    #address-cells = <1>;\n"
         "    #size-cells = <0>;\n"
         "    timebase-frequency = <" << (cpu_hz/insns_per_rtc_tick) << ">;\n";
  for (size_t i = 0; i < procs.size(); i++) {
    s << "    CPU" << i << ": cpu@" << i << " {\n"
         "      device_type = \"cpu\";\n"
         "      reg = <" << i << ">;\n"
         "      status = \"okay\";\n"
         "      compatible = \"riscv\";\n"
         "      riscv,isa = \"" << procs[i]->get_isa_string() << "\";\n"
         "      mmu-type = \"riscv," << (procs[i]->get_max_xlen() <= 32 ? "sv32" : "sv48") << "\";\n"
         "      riscv,pmpregions = <16>;\n"
         "      riscv,pmpgranularity = <4>;\n"
         "      clock-frequency = <" << cpu_hz << ">;\n"
         "      CPU" << i << "_intc: interrupt-controller {\n"
         "        #interrupt-cells = <1>;\n"
         "        interrupt-controller;\n"
         "        compatible = \"riscv,cpu-intc\";\n"
         "      };\n"
         "    };\n";
  }
  s <<   "  };\n";
  for (auto& m : mems) {
    s << std::hex <<
         "  memory@" << m.first << " {\n"
         "    device_type = \"memory\";\n"
         "    reg = <0x" << (m.first >> 32) << " 0x" << (m.first & (uint32_t)-1) <<
                   " 0x" << (m.second->size() >> 16 >> 16) << " 0x" << (m.second->size() & (uint32_t)-1) << ">;\n"
         "  };\n";
  }
  s <<   "  soc {\n"
         "    #address-cells = <2>;\n"
         "    #size-cells = <2>;\n"
         "    compatible = \"ucbbar,spike-bare-soc\", \"simple-bus\";\n"
         "    ranges;\n"
         "    clint@" << CLINT_BASE << " {\n"
         "      compatible = \"riscv,clint0\";\n"
         "      interrupts-extended = <" << std::dec;
  for (size_t i = 0; i < procs.size(); i++)
    s << "&CPU" << i << "_intc 3 &CPU" << i << "_intc 7 ";
  reg_t clintbs = CLINT_BASE;
  reg_t clintsz = CLINT_SIZE;
  s << std::hex << ">;\n"
         "      reg = <0x" << (clintbs >> 32) << " 0x" << (clintbs & (uint32_t)-1) <<
                     " 0x" << (clintsz >> 32) << " 0x" << (clintsz & (uint32_t)-1) << ">;\n"
         "    };\n"
         "  };\n"
         "  htif {\n"
         "    compatible = \"ucb,htif0\";\n"
         "  };\n"
         "};\n";
  return s.str();
}

std::string dts_compile(const std::string& dts)
{
  // Convert the DTS to DTB
  int dts_pipe[2];
  pid_t dts_pid;

  fflush(NULL); // flush stdout/stderr before forking
  if (pipe(dts_pipe) != 0 || (dts_pid = fork()) < 0) {
    std::cerr << "Failed to fork dts child: " << strerror(errno) << std::endl;
    exit(1);
  }

  // Child process to output dts
  if (dts_pid == 0) {
    close(dts_pipe[0]);
    int step, len = dts.length();
    const char *buf = dts.c_str();
    for (int done = 0; done < len; done += step) {
      step = write(dts_pipe[1], buf+done, len-done);
      if (step == -1) {
        std::cerr << "Failed to write dts: " << strerror(errno) << std::endl;
        exit(1);
      }
    }
    close(dts_pipe[1]);
    exit(0);
  }

  pid_t dtb_pid;
  int dtb_pipe[2];
  if (pipe(dtb_pipe) != 0 || (dtb_pid = fork()) < 0) {
    std::cerr << "Failed to fork dtb child: " << strerror(errno) << std::endl;
    exit(1);
  }

  // Child process to output dtb
  if (dtb_pid == 0) {
    dup2(dts_pipe[0], 0);
    dup2(dtb_pipe[1], 1);
    close(dts_pipe[0]);
    close(dts_pipe[1]);
    close(dtb_pipe[0]);
    close(dtb_pipe[1]);
    execlp(DTC, DTC, "-O", "dtb", 0);
    std::cerr << "Failed to run " DTC ": " << strerror(errno) << std::endl;
    exit(1);
  }

  close(dts_pipe[1]);
  close(dts_pipe[0]);
  close(dtb_pipe[1]);

  // Read-out dtb
  std::stringstream dtb;

  int got;
  char buf[4096];
  while ((got = read(dtb_pipe[0], buf, sizeof(buf))) > 0) {
    dtb.write(buf, got);
  }
  if (got == -1) {
    std::cerr << "Failed to read dtb: " << strerror(errno) << std::endl;
    exit(1);
  }
  close(dtb_pipe[0]);

  // Reap children
  int status;
  waitpid(dts_pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "Child dts process failed" << std::endl;
    exit(1);
  }
  waitpid(dtb_pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "Child dtb process failed" << std::endl;
    exit(1);
  }

  return dtb.str();
}


static int fdt_get_node_addr_size(void *fdt, int node, reg_t *addr,
                                  unsigned long *size, const char *field)
{
  int parent, len, i;
  int cell_addr, cell_size;
  const fdt32_t *prop_addr, *prop_size;
  uint64_t temp = 0;

  parent = fdt_parent_offset(fdt, node);
  if (parent < 0)
    return parent;

  cell_addr = fdt_address_cells(fdt, parent);
  if (cell_addr < 1)
    return -ENODEV;

  cell_size = fdt_size_cells(fdt, parent);
  if (cell_size < 0)
    return -ENODEV;

  if (!field)
    return -ENODEV;

  prop_addr = (fdt32_t *)fdt_getprop(fdt, node, field, &len);
  if (!prop_addr)
    return -ENODEV;
  prop_size = prop_addr + cell_addr;

  if (addr) {
    for (i = 0; i < cell_addr; i++)
      temp = (temp << 32) | fdt32_to_cpu(*prop_addr++);
    *addr = temp;
  }
  temp = 0;

  if (size) {
    for (i = 0; i < cell_size; i++)
      temp = (temp << 32) | fdt32_to_cpu(*prop_size++);
    *size = temp;
  }

  return 0;
}

static int check_cpu_node(void *fdt, int cpu_offset)
{
  int len;
  const void *prop;

  if (!fdt || cpu_offset < 0)
    return -EINVAL;

  prop = fdt_getprop(fdt, cpu_offset, "device_type", &len);
  if (!prop || !len)
    return -EINVAL;
  if (strncmp ((char *)prop, "cpu", strlen ("cpu")))
    return -EINVAL;

  return 0;
}


int fdt_get_offset(void *fdt, const char *field)
{
  return fdt_path_offset(fdt, field);
}

int fdt_get_first_subnode(void *fdt, int node)
{
  return fdt_first_subnode(fdt, node);
}

int fdt_get_next_subnode(void *fdt, int node)
{
  return fdt_next_subnode(fdt, node);
}

int fdt_parse_plic(void *fdt, reg_t *plic_addr, reg_t *plic_size,
                   reg_t *plic_maxprio, reg_t* plic_ndev, char* plic_config,
                   const char *compatible)
{
  int nodeoffset, rc, len;

  const fdt32_t* prop_maxprio;
  const fdt32_t* prop_ndev;
  const fdt32_t* prop_interrupts_extended;

  nodeoffset = fdt_node_offset_by_compatible(fdt, -1, compatible);
  if (nodeoffset < 0)
    return nodeoffset;

  rc = fdt_get_node_addr_size(fdt, nodeoffset, plic_addr, plic_size, "");
  if (!(rc < 0 || !plic_addr))
    return 0;

  prop_maxprio = (fdt32_t *)fdt_getprop(fdt, nodeoffset, "riscv,max-priority", &len);
  if (!prop_maxprio || !len)
      return -EINVAL;
  *plic_maxprio = fdt32_to_cpu(*prop_maxprio);

  prop_ndev = (fdt32_t *)fdt_getprop(fdt, nodeoffset, "riscv,ndev", &len);
  if (!prop_ndev || !len)
      return -EINVAL;
  *plic_ndev = fdt32_to_cpu(*prop_ndev);

  len = 0;
  prop_interrupts_extended = (fdt32_t *)fdt_getprop(fdt, nodeoffset,
                                                    "interrupts-extended", &len);
  if (!prop_interrupts_extended || !len)
      return -EINVAL;

  std::map<uint32_t, std::string> hart_int_strs;
  std::vector<uint32_t> harts;
  std::deque<std::pair<uint32_t, uint32_t>> hart_int_mode;
  for (uint32_t i=0; i<len/8; ++i) {
    int llen;
    auto cpu_offset = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*(prop_interrupts_extended+i*2)));
    auto int_level = fdt32_to_cpu(*(prop_interrupts_extended+i*2+1));
    auto hart_id = fdt32_to_cpu(*(fdt32_t *)fdt_getprop(fdt,
            fdt_parent_offset(fdt, cpu_offset), "reg", &llen));
    hart_int_mode.push_back(std::make_pair(hart_id, int_level));
    switch(int_level) {
        case 9:
            hart_int_strs[hart_id] += 'S';
            break;
        case 11:
            hart_int_strs[hart_id] += 'M';
            break;
    }
  }
  for(auto hart_pair: hart_int_strs) { harts.push_back(hart_pair.first);}
  std::sort(harts.begin(), harts.end());
  std::string config;
  for (auto hart: harts) {
      if (!config.empty()) { config += ","; }
      config += hart_int_strs[hart];
  }

  strcpy(plic_config, config.c_str());
  return 0;
}

int fdt_parse_clint(void *fdt, reg_t *clint_addr,
                    const char *compatible)
{
  int nodeoffset, rc;

  nodeoffset = fdt_node_offset_by_compatible(fdt, -1, compatible);
  if (nodeoffset < 0)
    return nodeoffset;

  rc = fdt_get_node_addr_size(fdt, nodeoffset, clint_addr, NULL, "reg");
  if (rc < 0 || !clint_addr)
    return -ENODEV;

  return 0;
}

int fdt_parse_pmp_num(void *fdt, int cpu_offset, reg_t *pmp_num)
{
  int rc;

  if ((rc = check_cpu_node(fdt, cpu_offset)) < 0)
    return rc;

  rc = fdt_get_node_addr_size(fdt, cpu_offset, pmp_num, NULL,
                              "riscv,pmpregions");
  if (rc < 0 || !pmp_num)
    return -ENODEV;

  return 0;
}

int fdt_parse_pmp_alignment(void *fdt, int cpu_offset, reg_t *pmp_align)
{
  int rc;

  if ((rc = check_cpu_node(fdt, cpu_offset)) < 0)
    return rc;

  rc = fdt_get_node_addr_size(fdt, cpu_offset, pmp_align, NULL,
                              "riscv,pmpgranularity");
  if (rc < 0 || !pmp_align)
    return -ENODEV;

  return 0;
}

int fdt_parse_mmu_type(void *fdt, int cpu_offset, char *mmu_type)
{
  int len, rc;
  const void *prop;

  if ((rc = check_cpu_node(fdt, cpu_offset)) < 0)
    return rc;

  prop = fdt_getprop(fdt, cpu_offset, "mmu-type", &len);
  if (!prop || !len)
    return -EINVAL;

  strcpy(mmu_type, (char *)prop);

  return 0;
}
