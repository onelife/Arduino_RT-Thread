/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author      Notes
 * 2018/08/29     Bernard     first version
 * 2019/04/25     onelife     refactor, reduce ram usage, bug fix
 */

#include "include/rtthread.h"

#ifdef RT_USING_MODULE

#include "dlelf.h"
#include "components/dfs/include/dfs_posix.h"

#ifdef RT_USING_ULOG
# define LOG_LVL                    LOG_LVL_INFO
# define LOG_TAG                    "MO_ELF"
# include "components/utilities/ulog/ulog.h"
#else /* RT_USING_ULOG */
# define LOG_E(format, args...)     rt_kprintf(format "\n", ##args)
# define LOG_W                      LOG_E
# define LOG_I                      LOG_E
# define LOG_D                      LOG_E
#endif /* RT_USING_ULOG */

#define MAX_PADDING_SIZE            (16)

#define BREAK_WITH_WARN(err, msg, args...) {\
    LOG_W(msg, ##args); \
    ret = -err; \
    break; \
}

#define READ_PROG_HDR(fd, elf_hdr, idx, buf) { \
    lseek(fd, elf_hdr->e_phoff + sizeof(Elf32_Phdr) * idx, SEEK_SET); \
    if (sizeof(Elf32_Phdr) != (rt_uint32_t)read(fd, buf, sizeof(Elf32_Phdr))) \
        BREAK_WITH_WARN(RT_EIO, "read prog_hdr err"); \
}

#define READ_SECT_HDR(fd, elf_hdr, idx, buf) { \
    lseek(fd, elf_hdr->e_shoff + sizeof(Elf32_Shdr) * idx, SEEK_SET); \
    if (sizeof(Elf32_Shdr) != (rt_uint32_t)read(fd, buf, sizeof(Elf32_Shdr))) \
        BREAK_WITH_WARN(RT_EIO, "read sect_hdr err"); \
}

#define LOAD_SECT(fd, sect_hdr, buf) { \
    lseek(fd, sect_hdr->sh_offset, SEEK_SET); \
    if (sect_hdr->sh_size != (rt_uint32_t)read(fd, buf, sect_hdr->sh_size)) \
        BREAK_WITH_WARN(RT_EIO, "sect read err"); \
}

#define LOAD_PROG(fd, prog_hdr, buf) { \
    lseek(fd, prog_hdr->p_offset, SEEK_SET); \
    if (prog_hdr->p_filesz != (rt_uint32_t)read(fd, buf, prog_hdr->p_filesz)) \
        BREAK_WITH_WARN(RT_EIO, "prog read err"); \
}

#define LOAD_CONTENT(fd, ofs, buf, sz) { \
    lseek(fd, ofs, SEEK_SET); \
    if (sz != (rt_uint32_t)read(fd, buf, sz)) \
        BREAK_WITH_WARN(RT_EIO, "content read err"); \
}


rt_err_t dlmodule_load_reltab(rt_dlmodule_t *module, Elf32_Rel *reltab,
    rt_uint32_t rel_cnt, Elf32_Sym *symtab, rt_uint8_t *strtab) {
    rt_err_t ret;
    rt_uint32_t idx;

    ret = RT_EOK;
    for (idx = 0; idx < rel_cnt; idx++) {
        Elf32_Addr *where;
        Elf32_Addr sym_val = 0;
        Elf32_Sym *sym = RT_NULL;
        rt_uint8_t rel_type = ELF32_R_TYPE(reltab[idx].r_info);
        rt_uint32_t sym_idx = ELF32_R_SYM(reltab[idx].r_info);

        LOG_D("rel %02d: type %d sym_idx %d", idx, rel_type, sym_idx);
        if (STN_UNDEF == sym_idx) {
            sym_val = 0;
        } else {
            sym = &symtab[sym_idx];
            LOG_D("sym %s, shndx %d", strtab + sym->st_name, sym->st_shndx);
            LOG_D("bind %d type %d value %p", ELF_ST_BIND(sym->st_info),
                ELF_ST_TYPE(sym->st_info), sym->st_value);
            if (SHN_UNDEF != sym->st_shndx) {
                sym_val = (Elf32_Addr)(module->mem_space + \
                    (sym->st_value - module->vstart_addr));
            } else {
                // if (linked) continue;
                /* symbol table lookup */
                sym_val = dlmodule_symbol_find((const char *)\
                    (strtab + sym->st_name));
                if (!sym_val) {
                    BREAK_WITH_WARN(RT_ERROR, "unsolved sym %s",
                        strtab + sym->st_name);
                }
            }
        }
        /* do relocate */
        where = module->mem_space + \
            (reltab[idx].r_offset - module->vstart_addr);
        LOG_D("sym %s: @%p bf %p", strtab + sym->st_name, where,
            sym_val);
        *where = dlmodule_relocate(rel_type, where, sym_val);
        LOG_D("sym %s: @%p af %p", strtab + sym->st_name, where,
            *where);
    } /* for (idx = 0; idx < rel_cnt; idx++) */

    return ret;
}

rt_err_t dlmodule_load_shared_object(int fd, Elf32_Ehdr *elf_hdr,
    rt_dlmodule_t *module) {
    rt_err_t ret;
    Elf32_Phdr *prog_hdr;
    Elf32_Shdr *sect_hdr;

    do {
        rt_uint32_t idx;
        Elf32_Addr init = 0;
        Elf32_Addr fini = 0;
        Elf32_Addr vstart_addr = 0;
        Elf32_Addr vend_addr = 0;
        rt_bool_t has_vstart = RT_FALSE;
        /* TODO: how to make RTMMAG? */
        // rt_bool_t linked = !rt_memcmp(elf_hdr->e_ident, RTMMAG, SELFMAG);
        Elf32_Dyn *dyntab = RT_NULL;
        rt_uint8_t dyn_num = 0;

        ret = RT_EOK;
        module->mem_space = RT_NULL;
        module->symtab = RT_NULL;
        prog_hdr = RT_NULL;
        sect_hdr = RT_NULL;
        // LOG_D("linked %d", linked);
        LOG_D("e_shnum %d", elf_hdr->e_shnum);
        LOG_D("e_phnum %d", elf_hdr->e_phnum);

        /* alloc prog_hdr */
        prog_hdr = (Elf32_Phdr *)rt_malloc(sizeof(Elf32_Phdr));
        if (!prog_hdr) BREAK_WITH_WARN(RT_ENOMEM, "prog mem err");

        /* step 1: get prog size */
        for (idx = 0; idx < elf_hdr->e_phnum; idx++) {
            READ_PROG_HDR(fd, elf_hdr, idx, prog_hdr);
            LOG_D("prog %d: tp %d fg %d, +%p @%p sz %d", idx,
                prog_hdr->p_type, prog_hdr->p_flags, prog_hdr->p_offset,
                prog_hdr->p_vaddr, prog_hdr->p_memsz);
            if (PT_LOAD != prog_hdr->p_type) continue;

            if (prog_hdr->p_memsz < prog_hdr->p_filesz)
                BREAK_WITH_WARN(RT_EIO, "bad prog %d: p_memsz %d, p_filesz %d",
                    idx, prog_hdr->p_memsz, prog_hdr->p_filesz);

            if (!has_vstart) {
                vstart_addr = prog_hdr->p_vaddr;
                vend_addr = prog_hdr->p_vaddr + prog_hdr->p_memsz;
                has_vstart = RT_TRUE;
                if (vend_addr < vstart_addr)
                    BREAK_WITH_WARN(RT_EIO,
                        "bad prog %d: p_vaddr %d, p_memsz %d",
                        idx, prog_hdr->p_vaddr, prog_hdr->p_memsz);
            } else {
                if (vend_addr > prog_hdr->p_vaddr) {
                    BREAK_WITH_WARN(RT_EIO, "bad prog %d: out of order", idx);
                } else if ((vend_addr + MAX_PADDING_SIZE) < prog_hdr->p_vaddr) {
                    BREAK_WITH_WARN(RT_EIO, "bad prog %d: long padding", idx);
                }

                vend_addr = prog_hdr->p_vaddr + prog_hdr->p_memsz;
                if (vend_addr < prog_hdr->p_vaddr)
                    BREAK_WITH_WARN(RT_EIO, "bad prog %d: overflow", idx);
            }
        }
        if (RT_EOK != ret) break;

        module->mem_size = vend_addr - vstart_addr;
        LOG_D("mo @%p (sz %d)", vstart_addr, module->mem_size);
        if (!module->mem_size) BREAK_WITH_WARN(RT_EIO, "bad elf: 0 size");

        /* alloc module->mem_space */
        module->mem_space = (rt_addr_t)rt_malloc(module->mem_size);
        if (!module->mem_space) BREAK_WITH_WARN(RT_ENOMEM, "prog mem err");
        LOG_D("mo mem_space @%p", module->mem_space);
        rt_memset(module->mem_space, 0x00, module->mem_size);
        module->vstart_addr = vstart_addr;
        module->nref = 0;

        /* step 2: load prog and get dyntab */
        for (idx = 0; idx < elf_hdr->e_phnum; idx++) {
            READ_PROG_HDR(fd, elf_hdr, idx, prog_hdr);
            if ((PT_LOAD != prog_hdr->p_type) && \
                (PT_DYNAMIC != prog_hdr->p_type)) continue;

            if (PT_LOAD == prog_hdr->p_type) {
                LOAD_PROG(fd, prog_hdr, module->mem_space + \
                    (prog_hdr->p_vaddr - module->vstart_addr));
            } else if (PT_DYNAMIC == prog_hdr->p_type) {
                if ((vstart_addr > prog_hdr->p_vaddr) || \
                    (vend_addr < prog_hdr->p_vaddr))
                    BREAK_WITH_WARN(RT_EIO, "dyntab no loaded");

                dyntab = (Elf32_Dyn *)(module->mem_space + \
                    (prog_hdr->p_vaddr - module->vstart_addr));
                dyn_num = prog_hdr->p_filesz / sizeof(Elf32_Dyn);
            }
        }
        if (RT_EOK != ret) break;
        LOG_D("dyn_num %d", dyn_num);
        // LOG_HEX("prog", 16, module->mem_space, module->mem_size);
        rt_free(prog_hdr);

        /* step 3: process dyntab */
        if (dyn_num) {
            Elf32_Rel *plttab = RT_NULL;
            Elf32_Rel *reltab = RT_NULL;
            Elf32_Sym *symtab = RT_NULL;
            rt_uint8_t *strtab = RT_NULL;

            rt_uint32_t plt_cnt = 0;
            rt_uint32_t rel_cnt = 0;
            rt_uint32_t sym_cnt = 0;
            rt_uint32_t export_sz = 0;

            for (idx = 0; idx < dyn_num; idx++) {
                LOG_D("dyn %d: tg %d val %p", idx, dyntab[idx].d_tag,
                    dyntab[idx].d_un.d_ptr);
                switch (dyntab[idx].d_tag) {
                case DT_NULL:
                    continue;

                case DT_PLTGOT:
                    LOG_D("PLTGOT @%p", dyntab[idx].d_un.d_ptr);
                    break;

                case DT_JMPREL:
                    if ((vstart_addr > dyntab[idx].d_un.d_ptr) || \
                        (vend_addr < dyntab[idx].d_un.d_ptr))
                            BREAK_WITH_WARN(RT_EIO, "plttab no loaded");
                    plttab = module->mem_space + \
                             (dyntab[idx].d_un.d_ptr - module->vstart_addr);
                    break;
                case DT_PLTREL:
                    if (dyntab[idx].d_un.d_val == DT_REL) {
                        LOG_D("[REL]");
                    } else {
                        LOG_D("[RELA]");
                    }
                    break;
                case DT_PLTRELSZ:
                    plt_cnt = dyntab[idx].d_un.d_val / sizeof(Elf32_Rel);
                    LOG_D("plt_cnt %d", plt_cnt);
                    break;

                case DT_REL:
                    if ((vstart_addr > dyntab[idx].d_un.d_ptr) || \
                        (vend_addr < dyntab[idx].d_un.d_ptr))
                        BREAK_WITH_WARN(RT_EIO, "reltab no loaded");
                    reltab = module->mem_space + \
                             (dyntab[idx].d_un.d_ptr - module->vstart_addr);
                    break;
                case DT_RELSZ:
                    rel_cnt = dyntab[idx].d_un.d_val / sizeof(Elf32_Rel);
                    LOG_D("rel_cnt %d", rel_cnt);
                    break;
                case DT_RELENT:
                    if (sizeof(Elf32_Rel) != dyntab[idx].d_un.d_val)
                        BREAK_WITH_WARN(RT_EIO, "bad DT_RELENT");
                    break;
                case DT_STRTAB:
                    strtab = module->mem_space + \
                             (dyntab[idx].d_un.d_ptr - module->vstart_addr);
                    break;
                case DT_STRSZ:
                    // LOG_D("DT_STRSZ %d", dyntab[idx].d_un.d_val);
                    break;
                case DT_SYMTAB:
                    symtab = module->mem_space + \
                             (dyntab[idx].d_un.d_ptr - module->vstart_addr);
                    break;
                case DT_SYMENT:
                    if (sizeof(Elf32_Sym) != dyntab[idx].d_un.d_val)
                        BREAK_WITH_WARN(RT_EIO, "bad DT_SYMENT");
                    break;

                case DT_INIT:
                    init = dyntab[idx].d_un.d_ptr;
                    LOG_D("init @%p", init);
                    break;
                case DT_FINI:
                    fini = dyntab[idx].d_un.d_ptr;
                    LOG_D("fini @%p", fini);
                    break;

                default:
                    break;
                }
            }
            if (RT_EOK != ret) break;
            if (!rel_cnt && !plttab) BREAK_WITH_WARN(RT_EIO, "bad dyntab");
            if (!symtab || !strtab) BREAK_WITH_WARN(RT_EIO, "bad dyntab");

            ret = dlmodule_load_reltab(module, reltab, rel_cnt, symtab, strtab);
            if (RT_EOK != ret) break;
            ret = dlmodule_load_reltab(module, plttab, plt_cnt, symtab, strtab);
            if (RT_EOK != ret) break;

            /* step 4: get sym_cnt */
            /* alloc sect_hdr */
            sect_hdr = (Elf32_Shdr *)rt_malloc(sizeof(Elf32_Shdr));
            if (!sect_hdr) BREAK_WITH_WARN(RT_ENOMEM, "sect mem err");

            for (idx = 0; idx < elf_hdr->e_shnum; idx++) {
                READ_SECT_HDR(fd, elf_hdr, idx, sect_hdr);
                LOG_D("sect %d: @%p name %d, tp %x, fg %x, if %x", idx,
                    sect_hdr->sh_addr, sect_hdr->sh_name, sect_hdr->sh_type,
                    sect_hdr->sh_flags, sect_hdr->sh_info);
                /* check if .dynsym */
                if (IS_DYNSYM(sect_hdr)) {
                    sym_cnt = sect_hdr->sh_size / sizeof(Elf32_Sym);
                    break;
                }
            }
            if (RT_EOK != ret) break;
            LOG_D("sym_cnt: %p", sym_cnt);
            rt_free(sect_hdr);

            /* step 5: process symtab */
            /* get export symbal table size */
            for (idx = 0, export_sz = 0; idx < sym_cnt; idx++) {
                if ((ELF_ST_BIND(symtab[idx].st_info) != STB_GLOBAL) ||
                    (ELF_ST_TYPE(symtab[idx].st_info) != STT_FUNC)) {
                    continue;
                }
                export_sz++;
            }
            LOG_D("export_sz: %d", export_sz);

            module->nsym = export_sz;
            if (export_sz) {
                /* alloc module->symtab */
                module->symtab = (struct rt_module_symtab *)rt_malloc(
                    export_sz * sizeof(struct rt_module_symtab));
                if (!module->symtab)
                    BREAK_WITH_WARN(RT_ENOMEM, "module->symtab mem err");

                /* load module->symtab */
                for (idx = 0, export_sz = 0; idx < sym_cnt; idx++) {
                    rt_size_t len;

                    if ((ELF_ST_BIND(symtab[idx].st_info) != STB_GLOBAL) ||
                        (ELF_ST_TYPE(symtab[idx].st_info) != STT_FUNC)) {
                        continue;
                    }

                    len = rt_strlen(
                        (const char *)(strtab + symtab[idx].st_name)) + 1;
                    module->symtab[export_sz].addr = (void *)(\
                        module->mem_space + \
                        (symtab[idx].st_value - module->vstart_addr));
                    module->symtab[export_sz].name = (char *)rt_malloc(len);
                    rt_memset((void *)module->symtab[export_sz].name, 0, len);
                    rt_memcpy((void *)module->symtab[export_sz].name,
                        strtab + symtab[idx].st_name, len);
                    LOG_D("sym %d: %s -> %p", export_sz,
                        module->symtab[export_sz].name,
                        module->symtab[export_sz].addr);
                    export_sz++;
                }
            }
        }

        /* set entry_addr */
        module->entry_addr = module->mem_space + \
            (elf_hdr->e_entry - module->vstart_addr);
        if (init) {
            module->init_func = module->mem_space + \
                                (init - module->vstart_addr);
        }
        if (fini) {
            module->cleanup_func = module->mem_space + \
                                   (fini - module->vstart_addr);
        }
        LOG_D("entry_addr: %p", module->entry_addr);
        LOG_D("init_func: %p", module->init_func);
        LOG_D("cleanup_func: %p", module->cleanup_func);
        return ret;
    } while (0);

    if (prog_hdr) rt_free(prog_hdr);
    if (sect_hdr) rt_free(sect_hdr);
    return ret;
}

#ifndef __arm__

rt_err_t dlmodule_load_relocated_object(int fd, Elf32_Ehdr *elf_hdr,
    rt_dlmodule_t *module) {
    rt_err_t ret;
    Elf32_Shdr *sect_hdr;
    rt_uint8_t *shstrab;
    Elf32_Rel *reltab;
    Elf32_Sym *symtab;
    rt_uint8_t *strtab;

    do {
        rt_uint8_t *ptr;
        rt_uint32_t idx;
        rt_uint32_t module_size, text_addr, rodata_addr, data_addr, bss_addr;

        ret = RT_EOK;
        sect_hdr = RT_NULL;
        shstrab = RT_NULL;
        reltab = RT_NULL;
        symtab = RT_NULL;
        strtab = RT_NULL;

        LOG_D("e_shnum %d", elf_hdr->e_shnum);
        LOG_D("e_phnum %d", elf_hdr->e_phnum);

        /* alloc sect_hdr */
        sect_hdr = (Elf32_Shdr *)rt_malloc(sizeof(Elf32_Shdr));
        if (!sect_hdr) BREAK_WITH_WARN(RT_ENOMEM, "sect mem err");

        /* get module size */
        module_size = 0;
        text_addr = 0;
        for (idx = 0; idx < elf_hdr->e_shnum; idx++) {
            READ_SECT_HDR(fd, elf_hdr, idx, sect_hdr);

            if (IS_PROG(sect_hdr) && IS_AX(sect_hdr)) {
                /* text */
                module_size += sect_hdr->sh_size;
                text_addr = sect_hdr->sh_addr;
                LOG_D("text sz 0x%p, addr 0x%p", sect_hdr->sh_size,
                    sect_hdr->sh_addr);
            } else if (IS_PROG(sect_hdr) && IS_ALLOC(sect_hdr)) {
                /* rodata */
                module_size += sect_hdr->sh_size;
                LOG_D("rodata sz 0x%p", sect_hdr->sh_size);
            } else if (IS_PROG(sect_hdr) && IS_AW(sect_hdr)) {
                /* data */
                module_size += sect_hdr->sh_size;
                LOG_D("data sz 0x%p", sect_hdr->sh_size);
            } else if (IS_NOPROG(sect_hdr) && IS_AW(sect_hdr)) {
                /* bss */
                module_size += sect_hdr->sh_size;
                LOG_D("bss sz 0x%p", sect_hdr->sh_size);
            }
        }

        if (!module_size) BREAK_WITH_WARN(RT_EIO, "bad elf: 0 size");

        /* alloc module->mem_space */
        module->mem_space = (rt_addr_t)rt_malloc(module_size);
        if (!module->mem_space) BREAK_WITH_WARN(RT_ENOMEM, "prog mem err");
        LOG_D("so mem_space: %p", module->mem_space);
        rt_memset(module->mem_space, 0x00, module_size);
        module->mem_size = module_size;
        module->vstart_addr = 0;
        module->nref = 0;

        /* load module */
        ptr = module->mem_space;
        rodata_addr = 0;
        data_addr = 0;
        bss_addr = 0;
        for (idx = 0; idx < elf_hdr->e_shnum; idx++) {
            READ_SECT_HDR(fd, elf_hdr, idx, sect_hdr);

            if (IS_PROG(sect_hdr) && IS_AX(sect_hdr)) {
                /* load text */
                LOAD_SECT(fd, sect_hdr, ptr);
                LOG_D("load text 0x%p, sz 0x%p", ptr, sect_hdr->sh_size);
                ptr += sect_hdr->sh_size;
            } else if (IS_PROG(sect_hdr) && IS_ALLOC(sect_hdr)) {
                /* load rodata */
                LOAD_SECT(fd, sect_hdr, ptr);
                LOG_D("load rodata 0x%p, sz 0x%p", ptr, sect_hdr->sh_size);
                rodata_addr = (rt_uint32_t)ptr;
                ptr += sect_hdr->sh_size;
            } else if (IS_PROG(sect_hdr) && IS_AW(sect_hdr)) {
                /* load data */
                LOAD_SECT(fd, sect_hdr, ptr);
                LOG_D("load data 0x%p, sz 0x%p", ptr, sect_hdr->sh_size);
                data_addr = (rt_uint32_t)ptr;
                ptr += sect_hdr->sh_size;
            } else if (IS_NOPROG(sect_hdr) && IS_AW(sect_hdr)) {
                /* load bss */
                rt_memset(ptr, 0x00, sect_hdr->sh_size);
                LOG_D("load bss 0x%p, sz 0x%p", ptr, sect_hdr->sh_size);
                bss_addr = (rt_uint32_t)ptr;
                ptr += sect_hdr->sh_size;
            }
        }
        if (RT_EOK != ret) break;

        /* set entry_addr */
        module->entry_addr = (rt_dlmodule_entry_func_t)\
            ((rt_uint8_t *)module->mem_space + elf_hdr->e_entry - text_addr);

        /* get shstrab sect_hdr */
        READ_SECT_HDR(fd, elf_hdr, elf_hdr->e_shstrndx, sect_hdr);
        LOG_D("shstrab sz %d", sect_hdr->sh_size);

        /* alloc shstrab */
        shstrab = (rt_uint8_t *)rt_malloc(sect_hdr->sh_size);
        if (!shstrab) BREAK_WITH_WARN(RT_ENOMEM, "shstrab mem err");
        /* load shstrab */
        LOAD_SECT(fd, sect_hdr, shstrab);

        /* process relocation */
        for (idx = 0; idx < elf_hdr->e_shnum; idx++) {
            rt_uint32_t i, rel_num;

            READ_SECT_HDR(fd, elf_hdr, idx, sect_hdr);
            if (!IS_REL(sect_hdr)) continue;

            rel_num = (rt_uint32_t)(sect_hdr->sh_size / sizeof(Elf32_Rel));
            LOG_D("rel_num %d", rel_num);

            /* allo reltab */
            LOG_D("reltab sz %d", sect_hdr->sh_size);
            reltab = (Elf32_Rel *)rt_malloc(sect_hdr->sh_size);
            if (!reltab) BREAK_WITH_WARN(RT_ENOMEM, "reltab mem err");
            /* load reltab */
            LOAD_SECT(fd, sect_hdr, reltab);

            /* rel_sect -> symtab */
            READ_SECT_HDR(fd, elf_hdr, sect_hdr->sh_link, sect_hdr);
            LOG_D("symtab sz %d", sect_hdr->sh_size);

            /* alloc symtab */
            symtab = (Elf32_Sym *)rt_malloc(sect_hdr->sh_size);
            if (!symtab) BREAK_WITH_WARN(RT_ENOMEM, "symtab mem err");
            /* load symtab */
            LOAD_SECT(fd, sect_hdr, symtab);

            /* symtab -> strtab */
            READ_SECT_HDR(fd, elf_hdr, sect_hdr->sh_link, sect_hdr);
            LOG_D("strtab sz %d", sect_hdr->sh_size);

            /* alloc strtab */
            strtab = (rt_uint8_t *)rt_malloc(sect_hdr->sh_size);
            if (!strtab) BREAK_WITH_WARN(RT_ENOMEM, "strtab mem err");
            /* load strtab */
            LOAD_SECT(fd, sect_hdr, strtab);

            /* do relocation */
            for (i = 0; i < rel_num; i++) {
                Elf32_Addr addr;
                Elf32_Sym *sym = &symtab[ELF32_R_SYM(reltab[i].r_info)];
                LOG_D("sym %s shndx %d", strtab + sym->st_name, sym->st_shndx);

                addr = 0;
                if (STT_FUNC == ELF_ST_TYPE(sym->st_info)) {
                    addr = (Elf32_Addr)((rt_uint8_t *)module->mem_space - \
                        text_addr + sym->st_value);
                } else if (SHT_NULL != sym->st_shndx) {
                    if ((STT_SECTION == ELF_ST_TYPE(sym->st_info)) ||
                        (STT_OBJECT == ELF_ST_TYPE(sym->st_info))) {
                        READ_SECT_HDR(fd, elf_hdr, sym->st_shndx, sect_hdr);
                        if (!rt_strncmp((char *)sect_hdr->sh_name, 
                            ELF_RODATA, sizeof(ELF_RODATA))) {
                            /* relocate rodata */
                            addr = rodata_addr + sym->st_value;
                            LOG_D("relocate rodata 0x%p", addr);
                        } else if (!rt_strncmp((char *)sect_hdr->sh_name,
                            ELF_BSS, sizeof(ELF_BSS))) {
                            /* relocate bss */
                            addr = bss_addr + sym->st_value;
                            LOG_D("relocate bss 0x%p", addr);
                        } else if (!rt_strncmp((char *)sect_hdr->sh_name,
                            ELF_DATA, sizeof(ELF_DATA))) {
                            /* relocate data */
                            addr = data_addr + sym->st_value;
                            LOG_D("relocate data 0x%p", addr);
                        }
                    }
                } else if (R_ARM_V4BX != ELF32_R_TYPE(reltab[i].r_info)) {
                    /* need to resolve symbol in kernel symbol table */
                    addr = dlmodule_symbol_find(
                        (const char *)(strtab + sym->st_name));
                    if (!addr) {
                        BREAK_WITH_WARN(RT_ERROR, "unsolved sym %s",
                            strtab + sym->st_name);
                    }
                } else {
                    addr = (Elf32_Addr)((rt_uint8_t *)module->mem_space - \
                        text_addr + sym->st_value);
                }

                if (addr) {
                    Elf32_Addr *where = module->mem_space + \
                        (reltab[idx].r_offset - module->vstart_addr);
                    *where = dlmodule_relocate(ELF32_R_TYPE(reltab[idx].r_info),
                        where, addr);
                    LOG_D("sym %s -> %p", strtab + sym->st_name, addr);
                }
            } /* for (i = 0; i < rel_num; i++) */
            if (RT_EOK != ret) break;
        } /* for (idx = 0; idx < elf_hdr->e_shnum; idx++) */
        if (RT_EOK != ret) break;

        rt_free(sect_hdr);
        rt_free(shstrab);
        rt_free(reltab);
        rt_free(symtab);
        rt_free(strtab);
        return ret;
    } while (0);

    if (sect_hdr) rt_free(sect_hdr);
    if (shstrab) rt_free(shstrab);
    if (reltab) rt_free(reltab);
    if (symtab) rt_free(symtab);
    if (strtab) rt_free(strtab);
    if (module->mem_space) rt_free(module->mem_space);
    return ret;
}

#endif /* __arm__ */

#endif /* RT_USING_MODULE */
