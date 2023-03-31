#include "config.h"
#include "bert.hpp"
#include <string.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <filesystem>
#include <fcntl.h>

extern "C"
{
#include <spinorfs.h>
}

#undef BERT_DEBUG

#define PROC_MTD_INFO           "/proc/mtd"
#define HOST_SPI_FLASH_MTD_NAME "hnor"

std::string bertNvp = "ras-crash";
std::string bertFileNvp = "latest.ras";
std::string bertFileNvpInfo = "latest.dump";

static int spinorfsRead(char *file, char *buff, uint32_t offset, uint32_t size)
{
    uint32_t val;

    if (spinorfs_open(file, SPINORFS_O_RDONLY))
        return -1;

    val = spinorfs_read(buff, offset, size);
    spinorfs_close();
    if (val != size)
        return -1;
    else
        return 0;
}

static int spinorfsWrite(char *file, char *buff, uint32_t offset, uint32_t size)
{
    uint32_t val;

    if (spinorfs_open(file, SPINORFS_O_WRONLY | SPINORFS_O_TRUNC))
        return -1;

    val = spinorfs_write(buff, offset, size);
    spinorfs_close();

    if (val != size)
        return -1;
    else
        return 0;
}

static int openSPINorDevice(int *fd)
{
    std::ifstream mtdInfoStream;
    std::string mtdDeviceStr;

    mtdInfoStream.open(PROC_MTD_INFO);
    std::string line;
    while (std::getline(mtdInfoStream, line))
    {
        std::cerr << "Get line: " << line.c_str() << "\n";
        if (line.find(HOST_SPI_FLASH_MTD_NAME) != std::string::npos)
        {
            std::size_t pos = line.find(":");
            mtdDeviceStr = line.substr(0,pos);
            mtdDeviceStr = "/dev/" + mtdDeviceStr;
            *fd = open(mtdDeviceStr.c_str(), O_SYNC | O_RDWR);
            return 0;
        }
    }
    return -1;
}

int bertHandler(bool isBertTrigger, std::string &primaryLogId)
{
    int ret = 0;
    int devFd = -1;
    uint32_t size = 0, offset = 0;
    std::vector<std::string> bertDumpPathList;
    std::string bertDumpPath;
    AmpereBertPartitionInfo bertInfo;
    uint8_t i;

    if (!isBertTrigger)
        return 0;

    ret = openSPINorDevice(&devFd);
    if (ret) {
        std::cerr << "Can not open SPINOR device\n";
        return -1;
    }
    ret = spinorfs_gpt_disk_info(devFd, 0);
    if (ret) {
        std::cerr << "Get GPT Info failure\n";
        return ret;
    }
    ret = spinorfs_gpt_part_name_info((char*) bertNvp.c_str(), &offset, &size);
    if (ret) {
        std::cerr << "Get GPT Partition Info failure\n";
        return ret;
    }
    ret = spinorfs_mount(devFd, size, offset);
    if (ret) {
        std::cerr << "Mount Partition failure\n";
        return ret;
    }

    ret = spinorfsRead((char*) bertFileNvp.c_str(),(char*) &bertInfo,
                       0, sizeof(AmpereBertPartitionInfo));
    if (ret < 0) {
        std::cerr << "Read " << bertFileNvp.c_str() << "failure\n";
        spinorfs_unmount();
        return ret;
    }
#ifdef BERT_DEBUG
   for (i = 0; i < BERT_MAX_NUM_FILE; i++)
   {
       std::cerr << "BERT_PARTITION_INFO size = " <<
                    bertInfo.files[i].size << "\n";
       std::cerr << "BERT_PARTITION_INFO name = " <<
                    bertInfo.files[i].name << "\n";
       std::cerr << "BERT_PARTITION_INFO flags = " <<
                    bertInfo.files[i].flags.reg << "\n";
   }
#endif
    for (i = 0; i < BERT_MAX_NUM_FILE; i++)
    {
        if(!bertInfo.files[i].flags.member.valid ||
           bertInfo.files[i].flags.member.pendingBMC)
            continue;
        /*
         * Valid bert header and BMC flag is not set imply a new
         * bert record for BMC
         */
        bertDumpPath = std::string(BERT_LOG_DIR) +
                       std::string(bertInfo.files[i].name);
        std::vector<char> crashBufVector(bertInfo.files[i].size, 0);
        char *crashBuf = crashBufVector.data();
        ret = spinorfsRead(bertInfo.files[i].name, crashBuf,
                           0, bertInfo.files[i].size);

        if (ret < 0)
        {
            std::cerr << "Read " << bertInfo.files[i].name << "failure\n";
            continue;
        }
        std::ofstream out (bertDumpPath.c_str(), std::ofstream::binary);
        if(!out.is_open())
        {
            std::cerr << "Can not open ofstream for BERT binary file\n";
            ret = -1;
            spinorfs_unmount();
            return ret;
        }
        out.write(crashBuf, bertInfo.files[i].size);
        out.close();
#ifdef BERT_DEBUG
        AmpereBertPayloadSection *bertPayload;
        bertPayload = (AmpereBertPayloadSection *) crashBuf;
        std::cerr << "firmwareVersion = " << bertPayload->firmwareVersion << "\n";
        std::cerr << "totalBertLength = " << bertPayload->totalBertLength << "\n";
        std::cerr << "sectionType = " << bertPayload->header.sectionType << "\n";
        std::cerr << "sectionLength = " << bertPayload->header.sectionLength << "\n";
        std::cerr << "sectionInstance = " << bertPayload->header.sectionInstance << "\n";
        std::cerr << "sectionsValid = " << bertPayload->sectionsValid.reg << "\n";
#endif
        bertDumpPathList.push_back(bertDumpPath);
        /* Set BMC flag to 1 to indicated processed by BMC */
        bertInfo.files[i].flags.member.pendingBMC = 0;
    }

    /* Copy BERT data from SPI to FaultLog location for Redfish
     * Only use BERT payload 0(crash_0). Need to consider how retrieve more BERT
     * payload via Redfish in future.
     */
    std::string faultLogFilePath = std::string(FAULT_LOG_PATH) + primaryLogId;
    std::filesystem::copy(bertDumpPathList[0].c_str(), faultLogFilePath.c_str(),
            std::filesystem::copy_options::overwrite_existing);

    /* Write back to BERT file info to indicate BMC consumed BERT record */
    ret = spinorfsWrite((char*) bertFileNvp.c_str(), (char*) &bertInfo,
                        0, sizeof(AmpereBertPartitionInfo));
    if (ret < 0) {
        std::cerr << "Update " << bertFileNvp.c_str() << "failure\n";
        spinorfs_unmount();
        return ret;
    }

    std::filesystem::remove_all(BERT_LOG_DIR);
    spinorfs_unmount();
    close(devFd);
    return ret;
}

