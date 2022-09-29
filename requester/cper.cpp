#include "cper.hpp"
#include <string.h>
#include <iostream>

/*
 * Section type definitions, used in SectionType field in struct
 * cper_section_descriptor
 *
 * Processor Generic
 */
/* Processor Specific: ARM */
Guid CPER_SEC_PROC_ARM  = { 0xe19e3d16, 0xbc11, 0x11e4, \
                 { 0x9c, 0xaa, 0xc2, 0x05, 0x1d, 0x5d, 0x46, 0xb0 }};

/* Platform Memory */
Guid CPER_SEC_PLATFORM_MEM = { 0xa5bc1114, 0x6f64, 0x4ede, \
                 { 0xb8, 0x63, 0x3e, 0x83, 0xed, 0x7c, 0x83, 0xb1 }};

/* PCIE */
Guid CPER_SEC_PCIE =  { 0xd995e954, 0xbbc1, 0x430f, \
                 { 0xad, 0x91, 0xb4, 0x4d, 0xcb, 0x3c, 0x6f, 0x35 }};

/* Ampere Specific */
Guid CPER_AMPERE_SPECIFIC = { 0x2826cc9f, 0x448c, 0x4c2b, \
                 { 0x86, 0xb6, 0xa9, 0x53, 0x94, 0xb7, 0xef, 0x33 }};

//Returns one if two EFI GUIDs are equal, zero otherwise.
static inline bool guidEqual(Guid *a, Guid *b)
{
    //Check top base 3 components.
    if (a->Data1 != b->Data1 || a->Data2 != b->Data2 ||
        a->Data3 != b->Data3) {
        return false;
    }

    //Check Data4 array for equality.
    for (int i = 0; i < 8; i++) {
        if (a->Data4[i] != b->Data4[i])
            return false;
    }

    return true;
}

static void decodeSecAmpere(void *section, uint32_t len,
                            AmpereSpecData* ampSpecHdr, FILE *out)
{
    AmpereSpecData *p;
    p = (AmpereSpecData *) section;
    memcpy(ampSpecHdr, section, sizeof(AmpereSpecData));
    fwrite(p, sizeof(AmpereSpecData), 1, out);
    fflush(out);
    /* Unformat data */
    unsigned char *next_pos  = (unsigned char *)(p+1);
    long remain_len = len - sizeof(AmpereSpecData);
    for (int i = 0; i < remain_len; i++) {
        fwrite(next_pos, sizeof(unsigned char), 1, out);
        fflush(out);
        next_pos++;
    }
}

static void decodeArmProcCtx(void *pos, uint16_t type, FILE *out)
{
    switch(type) {
    case ARM_CONTEXT_TYPE_AARCH32_GPR:
        fwrite(pos, sizeof(ARM_V8_AARCH32_GPR), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH32_EL1:
        fwrite(pos, sizeof(ARM_AARCH32_EL1_CONTEXT_REGISTERS), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH32_EL2:
        fwrite(pos, sizeof(ARM_AARCH32_EL2_CONTEXT_REGISTERS), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH32_SECURE:
        fwrite(pos, sizeof(ARM_AARCH32_SECURE_CONTEXT_REGISTERS), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH64_GPR:
        fwrite(pos, sizeof(ARM_V8_AARCH64_GPR), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH64_EL1:
        fwrite(pos, sizeof(ARM_AARCH64_EL1_CONTEXT_REGISTERS), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH64_EL2:
        fwrite(pos, sizeof(ARM_AARCH64_EL2_CONTEXT_REGISTERS), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_AARCH64_EL3:
        fwrite(pos, sizeof(ARM_AARCH64_EL3_CONTEXT_REGISTERS), 1, out);
        fflush(out);
        break;
    case ARM_CONTEXT_TYPE_MISC:
        fwrite(pos, sizeof(ARM_MISC_CONTEXT_REGISTER), 1, out);
        fflush(out);
        break;
    default:
        break;
    }
}

static void decodeSecArm(void *section, AmpereSpecData* ampSpecHdr,
                         FILE *out)
{
    int i, len;
    CPERSecProcArm *proc;
    CPERArmErrInfo *errInfo;
    CPERArmCtxInfo *ctxInfo;
    AmpereSpecData *ampHdr;
    unsigned char * next_pos;

    proc = (CPERSecProcArm*) section;
    fwrite(section, sizeof(CPERSecProcArm), 1, out);
    fflush(out);

    errInfo = (CPERArmErrInfo *)(proc + 1);
    for (i = 0; i < proc->ErrInfoNum; i++) {
        fwrite(errInfo, sizeof(CPERArmErrInfo), 1, out);
        fflush(out);
        errInfo += 1;
    }
    len = proc->SectionLength - (sizeof(*proc) + proc->ErrInfoNum * (sizeof(*errInfo)));
    if (len < 0)
    {
        std::cerr << "section length is too small : " << proc->SectionLength
                  << "\n";
    }

    ctxInfo = (CPERArmCtxInfo *)errInfo;
    for (i = 0; i < proc->ContextInfoNum; i++) {
        fwrite(ctxInfo, sizeof(CPERArmCtxInfo), 1, out);
        fflush(out);
        decodeArmProcCtx((void*)(ctxInfo + 1), ctxInfo->RegisterContextType, out);
        int size = sizeof(*ctxInfo) + ctxInfo->RegisterArraySize;
        len -= size;
        ctxInfo = (CPERArmCtxInfo *)((long)ctxInfo + size);
    }

    if (len > 0) {
        /* Get Ampere Specific header data */
        ampHdr = (AmpereSpecData*)ctxInfo;
        memcpy(ampSpecHdr, ampHdr, sizeof(AmpereSpecData));
        fwrite(ampHdr, sizeof(AmpereSpecData), 1, out);
        fflush(out);
        /* Unformat data */
        next_pos  = (unsigned char *)(ampHdr+1);
        long remain_len = len - sizeof(AmpereSpecData);
        for (i = 0; i < remain_len; i++) {
            fwrite(next_pos, sizeof(unsigned char), 1, out);
            fflush(out);
            next_pos++;
        }
    }
}

static void decodeSecPlatformMemory(void *section, AmpereSpecData* ampSpecHdr,
                                    FILE *out)
{
    CPERSecMemErr *mem = (CPERSecMemErr*) section;
    fwrite(&mem, sizeof(CPERSecMemErr), 1, out);
    fflush(out);
    if (mem->ErrorType == MEM_ERROR_TYPE_PARITY)
    {
        ampSpecHdr->typeId = ERROR_TYPE_ID_MCU;
        ampSpecHdr->subTypeId = SUBTYPE_ID_PARITY;
    }
}

static void decodeSecPcie(void *section, AmpereSpecData* /*ampSpecHdr*/,
                          FILE *out)
{
    CPERSecPcieErr *pcie = (CPERSecPcieErr*) section;
    fwrite(&pcie, sizeof(CPERSecPcieErr), 1, out);
    fflush(out);
}

static void decodeCperSection(FILE *cperFile, long basePos,
                              AmpereSpecData* ampSpecHdr,
                              FILE *out)
{
    CPERSectionDescriptor secDesc;
    if (fread(&secDesc, sizeof(CPERSectionDescriptor), 1, cperFile) != 1)
    {
        std::cerr << "Invalid section descriptor: Invalid length (log too short)."
                  << "\n";
        return;
    }
    //Save our current position in the stream.
    long position = ftell(cperFile);

    //Read section as described by the section descriptor.
    fseek(cperFile, basePos + secDesc.SectionOffset, SEEK_SET);
    void *section = malloc(secDesc.SectionLength);
    if (fread(section, secDesc.SectionLength, 1, cperFile) != 1) {
        std::cerr << "Section read failed: Could not read "
                  << secDesc.SectionLength << "bytes from global offset "
                  << secDesc.SectionOffset << "\n";
        free(section);
        return;
    }
    Guid *ptr = (Guid *) &secDesc.SectionType;
    if (guidEqual(ptr, &CPER_AMPERE_SPECIFIC))
    {
        std::cout << "RAS Section Type : Ampere Specific\n";
        decodeSecAmpere(section, secDesc.SectionLength, ampSpecHdr, out);
    }
    else if (guidEqual(ptr, &CPER_SEC_PROC_ARM))
    {
        std::cout << "RAS Section Type : ARM\n";
        decodeSecArm(section, ampSpecHdr, out);
    }
    else if (guidEqual(ptr, &CPER_SEC_PLATFORM_MEM))
    {
        std::cout << "RAS Section Type : Memory\n";
        decodeSecPlatformMemory(section, ampSpecHdr, out);
    }
    else if (guidEqual(ptr, &CPER_SEC_PCIE))
    {
        std::cout << "RAS Section Type : PCIE\n";
        decodeSecPcie(section, ampSpecHdr, out);
    }
    else
    {
        std::cerr << "Section Type not support\n";
    }
    //Seek back to our original position.
    fseek(cperFile, position, SEEK_SET);
    free(section);
}

void decodeCperRecord(FILE *cperFile, AmpereSpecData* ampSpecHdr,
                      FILE *out)
{
    CPERRecodHeader cperHeader;
    int i;
    long basePos = ftell(cperFile);

    if (fread(&cperHeader, sizeof(CPERRecodHeader), 1, cperFile) != 1) {
        std::cerr << "Invalid CPER header: Invalid length (log too short)." << "\n";
        return;
    }
    //Revert 4 bytes of SignatureStart
    char *sigStr = (char *) &cperHeader.SignatureStart;
    char tmp;
    tmp = sigStr[0];
    sigStr[0] = sigStr[3];
    sigStr[3] = tmp;
    tmp = sigStr[1];
    sigStr[1] = sigStr[2];
    sigStr[2] = tmp;

    fwrite(&cperHeader, sizeof(CPERRecodHeader), 1, out);
    fflush(out);

    CPERSectionDescriptor secDesc;
    //Save our current position in the stream.
    long position = ftell(cperFile);
    for (i = 0; i < cperHeader.SectionCount; i++) {
        if (fread(&secDesc, sizeof(CPERSectionDescriptor), 1, cperFile) != 1)
        {
            std::cerr << "Invalid section descriptor: Invalid length (log too short)."
                      << "\n";
            return;
        }
        fwrite(&secDesc, sizeof(CPERSectionDescriptor), 1, out);
        fflush(out);
    }
    //Seek back to our original position.
    fseek(cperFile, position, SEEK_SET);

    for (i = 0; i < cperHeader.SectionCount; i++) {
        decodeCperSection(cperFile, basePos, ampSpecHdr, out);
    }
}
