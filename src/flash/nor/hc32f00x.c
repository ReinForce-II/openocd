#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_ERASE_TIMEOUT 100

#define FLASH_BASE 0x00000000
#define FLASH_NUM_SECTORS 64
#define FLASH_SECTOR_SIZE 512

/* register locations */

#define FLASH_REG_BASE_B0 0x40020000

#define HC32_FLASH_TNVS 0x00
#define HC32_FLASH_TPGS 0x04
#define HC32_FLASH_TPROG 0x08
#define HC32_FLASH_TSERASE 0x0C
#define HC32_FLASH_TMERASE 0x10
#define HC32_FLASH_TPRCV 0x14
#define HC32_FLASH_TSRCV 0x18
#define HC32_FLASH_TMRCV 0x1C
#define HC32_FLASH_CR 0x20
#define HC32_FLASH_IFR 0x24
#define HC32_FLASH_ICLR 0x28
#define HC32_FLASH_BYPASS 0x2C
#define HC32_FLASH_SLOCK 0x30

/* FLASH_TNVS register bits */
/* FLASH_TPGS register bits */
/* FLASH_TSERASE register bits */
/* FLASH_TMERASE register bits */
/* FLASH_TPRCV register bits */
/* FLASH_TSRCV register bits */
/* FLASH_TMRCV register bits */
/* FLASH_CR register bits */
#define FLASH_CR_IE_PR (1 << 6)
#define FLASH_CR_IE_PC (1 << 5)
#define FLASH_CR_BUSY (1 << 4)
#define FLASH_CR_WAIT (1 << 2)
// 00:read 01:program 10:sector erase 11:chip erase
#define FLASH_CR_OP_1 (1 << 1)
#define FLASH_CR_OP_0 (1 << 0)
/* FLASH_IFR register bits */
#define FLASH_IF_PR (1 << 1)
#define FLASH_IF_PC (1 << 0)
/* FLASH_ICLR register bits */
// Write 0 to clear
#define FLASH_ICLR_PR (1 << 1)
#define FLASH_ICLR_PC (1 << 0)
/* FLASH_BYPASS register bits */
// write 0x5a5a, then 0xa5a5 to enable write op to flash controller regs
/* FLASH_SLOCK register bits */
// 16bits each references 4 sectors, 0:write protect 1:write allowed

static int hc32_mass_erase(struct flash_bank *bank);

static uint32_t hc32_get_flash_reg(struct flash_bank *bank, uint32_t reg) {
  uint32_t value;
  struct target *target = bank->target;
  target_read_u32(target, FLASH_REG_BASE_B0 + reg, &value);
  return value;
}

static void hc32_set_flash_reg(struct flash_bank *bank, uint32_t reg,
                               uint32_t dat) {
  struct target *target = bank->target;
  target_write_u32(target, FLASH_REG_BASE_B0 + HC32_FLASH_BYPASS, 0x5a5a);
  target_write_u32(target, FLASH_REG_BASE_B0 + HC32_FLASH_BYPASS, 0xa5a5);
  target_write_u32(target, FLASH_REG_BASE_B0 + reg, dat);
}

static int hc32_wait_status_busy(struct flash_bank *bank, int timeout) {
  int retval = ERROR_OK;

  /* wait for busy to clear */
  for (;;) {
    if ((hc32_get_flash_reg(bank, HC32_FLASH_CR) & FLASH_CR_BUSY) == 0)
      break;
    if (timeout-- <= 0) {
      LOG_ERROR("timed out waiting for flash");
      return ERROR_FAIL;
    }
    alive_sleep(1);
  }
  return retval;
}

static int hc32_mass_erase(struct flash_bank *bank) {
  int retval = ERROR_OK;
  struct target *target = bank->target;
  uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, 0xffff);

  uint32_t cr = hc32_get_flash_reg(bank, HC32_FLASH_CR);

  hc32_set_flash_reg(bank, HC32_FLASH_CR, cr | 0x3);
  target_write_u32(target, FLASH_BASE, 0x00000000);
  retval = hc32_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

  hc32_set_flash_reg(bank, HC32_FLASH_CR, cr);
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
  return retval;
}

static int hc32_erase(struct flash_bank *bank, int first, int last) {
  struct target *target = bank->target;
  int i;

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }
  const struct armv7m_common *cm = target_to_armv7m(target);
  uint32_t R_PC;
  cm->load_core_reg_u32(target, ARMV7M_PC, &R_PC);
  cm->store_core_reg_u32(target, ARMV7M_PC, 0x20000000);

  if ((first == 0) && (last == (bank->num_sectors - 1)))
    return hc32_mass_erase(bank);

  uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, 0xffff);
  uint32_t cr = hc32_get_flash_reg(bank, HC32_FLASH_CR);
  int retval = ERROR_OK;
  hc32_set_flash_reg(bank, HC32_FLASH_CR, (cr & 0xfffffffc) | 0x2);
  for (i = first; i <= last; i++) {
    target_write_u32(target, FLASH_BASE + i * FLASH_SECTOR_SIZE, 0x00000000);
    retval = hc32_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
    if (retval != ERROR_OK) {
      LOG_ERROR("Erase error at sector %d", i);
      break;
    }
  }
  hc32_set_flash_reg(bank, HC32_FLASH_CR, cr);
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
  return retval;
}

static int hc32_protect(struct flash_bank *bank, int set, int first, int last) {
  int i;

  uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
  for (i = first; i <= last; i++) {
    if (set) {
      slock &= (~(1 << (i / 4)));
    } else {
      slock |= (1 << (i / 4));
    }
  }
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
  return ERROR_OK;
}

static int hc32_write_block(struct flash_bank *bank, const uint8_t *buffer,
                            uint32_t offset, uint32_t count) {
  struct target *target = bank->target;
  uint32_t buffer_size = 16384;
  struct working_area *write_algorithm;
  struct working_area *source;
  uint32_t address = bank->base + offset;
  struct reg_param reg_params[5];
  struct armv7m_algorithm armv7m_info;
  int retval = ERROR_OK;

  static const uint8_t hc32_flash_write_code[] = {
#include "../../../contrib/loaders/flash/hc32f00x/hc32f00x.inc"
  };

  /* flash write code */
  if (target_alloc_working_area(target, sizeof(hc32_flash_write_code),
                                &write_algorithm) != ERROR_OK) {
    LOG_WARNING("no working area available, can't do block memory writes");
    return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
  }

  retval =
      target_write_buffer(target, write_algorithm->address,
                          sizeof(hc32_flash_write_code), hc32_flash_write_code);
  if (retval != ERROR_OK) {
    target_free_working_area(target, write_algorithm);
    return retval;
  }

  /* memory buffer */
  while (target_alloc_working_area_try(target, buffer_size, &source) !=
         ERROR_OK) {
    buffer_size /= 2;
    buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
    if (buffer_size <= 256) {
      /* we already allocated the writing code, but failed to get a
       * buffer, free the algorithm */
      target_free_working_area(target, write_algorithm);

      LOG_WARNING("no large enough working area available, can't do block "
                  "memory writes");
      return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }
  }

  init_reg_param(&reg_params[0], "r0", 32,
                 PARAM_IN_OUT); /* flash base (in), status (out) */
  init_reg_param(&reg_params[1], "r1", 32,
                 PARAM_OUT); /* count (halfword-16bit) */
  init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* buffer start */
  init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* buffer end */
  init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT); /* target address */

  buf_set_u32(reg_params[0].value, 0, 32, FLASH_REG_BASE_B0);
  buf_set_u32(reg_params[1].value, 0, 32, count);
  buf_set_u32(reg_params[2].value, 0, 32, source->address);
  buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
  buf_set_u32(reg_params[4].value, 0, 32, address);

  armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
  armv7m_info.core_mode = ARM_MODE_THREAD;

  retval = target_run_flash_async_algorithm(
      target, buffer, count, 1, 0, NULL, 5, reg_params, source->address,
      source->size, write_algorithm->address, 0, &armv7m_info);

  if (retval == ERROR_FLASH_OPERATION_FAILED) {
    LOG_ERROR("flash memory not erased before writing");
  }

  target_free_working_area(target, source);
  target_free_working_area(target, write_algorithm);

  destroy_reg_param(&reg_params[0]);
  destroy_reg_param(&reg_params[1]);
  destroy_reg_param(&reg_params[2]);
  destroy_reg_param(&reg_params[3]);
  destroy_reg_param(&reg_params[4]);

  return retval;
}

static int hc32_write(struct flash_bank *bank, const uint8_t *buffer,
                      uint32_t offset, uint32_t count) {
  struct target *target = bank->target;
  uint32_t i;

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }
  const struct armv7m_common *cm = target_to_armv7m(target);
  uint32_t R_PC;
  cm->load_core_reg_u32(target, ARMV7M_PC, &R_PC);
  cm->store_core_reg_u32(target, ARMV7M_PC, 0x20000000);

  uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, 0xffff);
  uint32_t cr = hc32_get_flash_reg(bank, HC32_FLASH_CR);
  int retval = ERROR_OK;
  hc32_set_flash_reg(bank, HC32_FLASH_CR, (cr & 0xfffffffc) | 0x1);
  uint32_t block_size = 1024;
  for (i = 0; i < count; i += block_size) {
    if (i > count - block_size) {
      block_size = count - i;
    }
    retval = hc32_write_block(bank, buffer + i, offset + i, block_size);
    if (retval != ERROR_OK) {
      break;
    }
  }
  hc32_set_flash_reg(bank, HC32_FLASH_CR, cr);
  hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
  return retval;
}

static int hc32_probe(struct flash_bank *bank) {
  int i;
  bank->base = FLASH_BASE;
  bank->size = FLASH_NUM_SECTORS * FLASH_SECTOR_SIZE;
  bank->num_sectors = FLASH_NUM_SECTORS;
  bank->sectors = malloc(sizeof(struct flash_sector) * FLASH_NUM_SECTORS);

  for (i = 0; i < FLASH_NUM_SECTORS; i++) {
    bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
    bank->sectors[i].size = FLASH_SECTOR_SIZE;
    bank->sectors[i].is_erased = -1;
    bank->sectors[i].is_protected = 1;
  }
  return ERROR_OK;
}

static int hc32_auto_probe(struct flash_bank *bank) { return hc32_probe(bank); }

#if 0
COMMAND_HANDLER(hc32_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int hc32_protect_check(struct flash_bank *bank) {
  int i;

  uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
  for (i = 0; i < bank->num_sectors; i += 4) {
    if ((slock & (1 << (i / 4))) == 0) {
      bank->sectors[i].is_protected = 1;
      bank->sectors[i + 1].is_protected = 1;
      bank->sectors[i + 2].is_protected = 1;
      bank->sectors[i + 3].is_protected = 1;
    } else {
      bank->sectors[i].is_protected = 0;
      bank->sectors[i + 1].is_protected = 0;
      bank->sectors[i + 2].is_protected = 0;
      bank->sectors[i + 3].is_protected = 0;
    }
  }
  return ERROR_OK;
}

static int get_hc32_info(struct flash_bank *bank, char *buf, int buf_size) {
  snprintf(buf, buf_size, "Empty");
  return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(hc32_flash_bank_command) {
  if (CMD_ARGC < 6)
    return ERROR_COMMAND_SYNTAX_ERROR;
  bank->driver_priv = NULL;
  return ERROR_OK;
}

struct flash_driver hc32_flash = {
    .name = "hc32f00x",
    .commands = NULL,
    .flash_bank_command = hc32_flash_bank_command,
    .erase = hc32_erase,
    .protect = hc32_protect,
    .write = hc32_write,
    .read = default_flash_read,
    .probe = hc32_probe,
    .auto_probe = hc32_auto_probe,
    .erase_check = default_flash_blank_check,
    .protect_check = hc32_protect_check,
    .info = get_hc32_info,
    .free_driver_priv = default_flash_free_driver_priv,
};
