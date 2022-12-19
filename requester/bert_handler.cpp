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

extern "C"
{
#include "../libnvparm/gpt.h"
#include "../libnvparm/spinor_func.h"
}

#undef BERT_DEBUG

std::string bertNvp = "ras-crash";
std::string bertFileNvp = "latest.ras";
std::string bertFileNvpInfo = "latest.dump";

static int decodeBertPartitionInfo(std::string &inPath,
                                   AmpereBertPartitionInfo *bertInfo)
{
    if (!bertInfo)
        return -1;

    std::ifstream inNvpInfo (inPath.c_str(), std::ifstream::binary);
    if(!inNvpInfo.is_open())
    {
        std::cerr << "Can not open ifstream for " << inPath.c_str() << "\n";
        return -1;
    }

    inNvpInfo.read((char*) bertInfo,
                   sizeof(AmpereBertPartitionInfo));

 #ifdef BERT_DEBUG
    for (int i = 0; i < BERT_MAX_NUM_FILE; i++)
    {
        std::cerr << "BERT_PARTITION_INFO size = " <<
                     bertInfo->files[i].size << "\n";
        std::cerr << "BERT_PARTITION_INFO name = " <<
                     bertInfo->files[i].name << "\n";
        std::cerr << "BERT_PARTITION_INFO flags = " <<
                             bertInfo->files[i].flags.reg << "\n";
    }

#endif

    inNvpInfo.close();

    return 0;
}

[[maybe_unused]] static int decodeBertPayloadSection(std::string &inPath)
{
    AmpereBertPayloadSection bertPayload;
    std::ifstream inPayloadSec (inPath.c_str(), std::ifstream::binary);
    if(!inPayloadSec.is_open())
    {
        std::cerr << "Can not open ifstream for payload section\n";
        return -1;
    }
    inPayloadSec.read((char *) &bertPayload, sizeof(AmpereBertPayloadSection));

#ifdef BERT_DEBUG
    std::cerr << "firmwareVersion = " << bertPayload.firmwareVersion << "\n";
    std::cerr << "totalBertLength = " << bertPayload.totalBertLength << "\n";
    std::cerr << "sectionType = " << bertPayload.header.sectionType << "\n";
    std::cerr << "sectionLength = " << bertPayload.header.sectionLength << "\n";
    std::cerr << "sectionInstance = " << bertPayload.header.sectionInstance << "\n";
    std::cerr << "sectionsValid = " << bertPayload.sectionsValid.reg << "\n";
#endif
    inPayloadSec.close();

    return 0;
}

static int updateBertParititionInfo(std::string &outPath,
                             AmpereBertPartitionInfo *bertInfo)
{
    std::ofstream outNvpInfo (outPath.c_str(), std::ofstream::binary);
    if(!outNvpInfo.is_open())
    {
        std::cerr << "Can not open ofstream for " << outPath.c_str() << "\n";
        return -1;
    }
    outNvpInfo.write((char*) bertInfo, sizeof(AmpereBertPartitionInfo));
    outNvpInfo.close();

    return 0;
}

int bertHandler(bool isBertTrigger, std::string &primaryLogId)
{
    int ret = 0;
    int devFd = -1;
    uint32_t size = 0, offset = 0;
    std::vector<std::string> bertDumpPathList;
    std::string bertDumpPath;

    if (!isBertTrigger)
        return 0;

    ret = find_host_mtd_partition(&devFd);
    if (ret) {
        std::cerr << "Can not find spi partition\n";
        return ret;
    }
    ret = get_gpt_disk_info(devFd, SHOW_GPT_DISABLE);
    if (ret) {
        std::cerr << "Get GPT Info failure\n";
        return ret;
    }
    ret = get_gpt_part_name_info((char*) bertNvp.c_str(), &offset, &size);
    if (ret) {
        std::cerr << "Get GPT Partition Info failure\n";
        return ret;
    }
    ret = spinor_lfs_mount(size, offset);
    if (ret) {
        std::cerr << "Mount Partition failure\n";
        return ret;
    }

    std::string bertFileNvpInfoPath = std::string(BERT_LOG_DIR) + bertFileNvpInfo;
    ret = spinor_lfs_dump_nvp((char*) bertFileNvp.c_str(),
                              (char*) bertFileNvpInfoPath.c_str());
    if (ret) {
        std::cerr << "Read " << bertFileNvp.c_str() << "failure\n";
        return ret;
    }

    AmpereBertPartitionInfo bertInfo;
    ret = decodeBertPartitionInfo(bertFileNvpInfoPath, &bertInfo);
    if (ret) {
        std::cerr << "Decode Bert Partition Info failure\n";
        return ret;
    }

    for (uint8_t i = 0; i < BERT_MAX_NUM_FILE; i++)
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
        ret = spinor_lfs_dump_nvp(bertInfo.files[i].name,
                                  (char*) bertDumpPath.c_str());
        if (!ret)
        {
            bertDumpPathList.push_back(bertDumpPath);
            /* Set BMC flag to 1 to indicated processed by BMC */
            bertInfo.files[i].flags.member.pendingBMC = 1;
        }
    }
#ifdef BERT_DEBUG
    for (uint8_t i = 0; i < bertDumpPathList.size(); i++)
    {
        ret = decodeBertPayloadSection(bertDumpPathList[i]);
        if (ret) {
            std::cerr << "Decode Bert Payload Section failure\n";
        }
    }
#endif

    /* Copy BERT data from SPI to FaultLog location for Redfish
     * Only use BERT payload 0(crash_0). Need to consider how retrieve more BERT
     * payload via Redfish in future.
     */
    std::string faultLogFilePath = std::string(FAULT_LOG_PATH) + primaryLogId;
    std::filesystem::copy(bertDumpPathList[0].c_str(), faultLogFilePath.c_str(),
            std::filesystem::copy_options::overwrite_existing);

    /* Write back to BERT file info to indicate BMC consumed BERT record */
    updateBertParititionInfo(bertFileNvpInfoPath, &bertInfo);
    ret = spinor_lfs_upload_nvp((char*) bertFileNvp.c_str(),
                                (char*) bertFileNvpInfoPath.c_str());
    if (ret) {
        std::cerr << "Update " << bertFileNvp.c_str() << "failure\n";
        return ret;
    }

    std::filesystem::remove_all(BERT_LOG_DIR);
    close(devFd);
    return ret;
}

