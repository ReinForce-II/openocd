#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

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
static int hc32_get_device_id(struct flash_bank *bank, uint32_t *device_id);
static int hc32_write_block(struct flash_bank *bank, const uint8_t *buffer,
                            uint32_t offset, uint32_t count);

static uint32_t hc32_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
    struct target *target = bank->target;
    return target_read_u32(FLASH_REG_BASE_B0 + reg);
}

static void hc32_set_flash_reg(struct flash_bank *bank, uint32_t reg, uint32_t dat)
{
    struct target *target = bank->target;
    target_write_u32(target, FLASH_REG_BASE_B0 + HC32_FLASH_BYPASS, 0x5a5a);
    target_write_u32(target, FLASH_REG_BASE_B0 + HC32_FLASH_BYPASS, 0xa5a5);
    target_write_u32(target, FLASH_REG_BASE_B0 + reg, dat);
}

static int hc32_wait_status_busy(struct flash_bank *bank, int timeout)
{
    int retval = ERROR_OK;

    /* wait for busy to clear */
    for (;;)
    {
        if ((hc32_get_flash_reg(bank, HC32_FLASH_CR) & FLASH_CR_BUSY) == 0)
            break;
        if (timeout-- <= 0)
        {
            LOG_ERROR("timed out waiting for flash");
            return ERROR_FAIL;
        }
        alive_sleep(1);
    }
    return retval;
}

static int hc32_mass_erase(struct flash_bank *bank)
{
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

static int hc32_erase(struct flash_bank *bank, int first, int last)
{
    struct target *target = bank->target;
    int i;

    if (bank->target->state != TARGET_HALTED)
    {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if ((first == 0) && (last == (bank->num_sectors - 1)))
        return hc32_mass_erase(bank);

    uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
    hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, 0xffff);
    uint32_t cr = hc32_get_flash_reg(bank, HC32_FLASH_CR);
    int retval = ERROR_OK;
    for (i = first; i <= last; i++)
    {
        hc32_set_flash_reg(bank, HC32_FLASH_CR, (cr & 0xfffffffc) | 0x2);
        target_write_u32(target, FLASH_BASE + i * FLASH_SECTOR_SIZE, 0x00000000);
        retval = hc32_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
        if (retval != ERROR_OK)
        {
            LOG_ERROR("Erase error at sector %d", i);
            break;
        }
    }
    hc32_set_flash_reg(bank, HC32_FLASH_CR, cr);
    hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
    return retval;
}

static int hc32_protect(struct flash_bank *bank, int set, int first, int last)
{
    struct target *target = bank->target;
    int i;

    uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
    for (i = first; i <= last; i++)
    {
        if (set)
        {
            slock &= (~(1 << (i / 4));
        }
        else
        {
            slock |= (1 << (i / 4));
        }
    }
    hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
    return ERROR_OK;
}

static int hc32_write(struct flash_bank *bank, const uint8_t *buffer,
                      uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    int i;

    if (bank->target->state != TARGET_HALTED)
    {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
    hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, 0xffff);
    uint32_t cr = hc32_get_flash_reg(bank, HC32_FLASH_CR);
    int retval = ERROR_OK;
    for (i = 0; i < count; i++)
    {
        hc32_set_flash_reg(bank, HC32_FLASH_CR, (cr & 0xfffffffc) | 0x1);
        target_write_u32(target, FLASH_BASE + offset + i, buffer[i]);
        retval = hc32_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
        if (retval != ERROR_OK)
        {
            LOG_ERROR("Write error at byte %d", i);
            break;
        }
    }
    hc32_set_flash_reg(bank, HC32_FLASH_CR, cr);
    hc32_set_flash_reg(bank, HC32_FLASH_SLOCK, slock);
    return retval;
    return ERROR_OK;
}

static int hc32_probe(struct flash_bank *bank)
{
    bank->base = FLASH_BASE;
    bank->size = FLASH_NUM_SECTORS * FLASH_SECTOR_SIZE;
    bank->num_sectors = FLASH_NUM_SECTORS;
    bank->sectors = malloc(sizeof(struct flash_sector) * FLASH_NUM_SECTORS);

    for (i = 0; i < FLASH_NUM_SECTORS; i++)
    {
        bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
        bank->sectors[i].size = FLASH_SECTOR_SIZE;
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = 1;
    }
    return ERROR_OK;
}

static int hc32_auto_probe(struct flash_bank *bank)
{
    return hc32_probe(bank);
}

#if 0
COMMAND_HANDLER(hc32_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int hc32_protect_check(struct flash_bank *bank)
{
    struct target *target = bank->target;
    int i;

    uint32_t slock = hc32_get_flash_reg(bank, HC32_FLASH_SLOCK);
    for (i = 0; i < bank->num_sectors; i += 4)
    {
        if ((slock & (1 << (i / 4))) == 0)
        {
            bank->sectors[i].is_protected = 1;
            bank->sectors[i + 1].is_protected = 1;
            bank->sectors[i + 2].is_protected = 1;
            bank->sectors[i + 3].is_protected = 1;
        }
        else
        {
            bank->sectors[i].is_protected = 0;
            bank->sectors[i + 1].is_protected = 0;
            bank->sectors[i + 2].is_protected = 0;
            bank->sectors[i + 3].is_protected = 0;
        }
    }
    return ERROR_OK;
}

static int get_hc32_info(struct flash_bank *bank, char *buf, int buf_size)
{
    snprintf(buf, buf_size, "Empty");
    return ERROR_OK;
}

struct flash_driver hc32_flash = {
    .name = "hc32",
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
