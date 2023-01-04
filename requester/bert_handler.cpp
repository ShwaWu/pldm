#include "config.h"
#include "bert.hpp"
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <filesystem>
#include "common/utils.hpp"
#include <variant>
#include <string>
#include <chrono>
#include <sdbusplus/timer.hpp>
#include <fcntl.h>
extern "C"
{
#include <spinorfs.h>
}

#undef BERT_DEBUG

#define BERT_SENSOR_TYPE_OEM         0xC1
#define BERT_EVENT_CODE_OEM          0x04

#define PROC_MTD_INFO           "/proc/mtd"
#define HOST_SPI_FLASH_MTD_NAME "hnor"

std::string bertNvp = "ras-crash";
std::string bertFileNvp = "latest.ras";
std::string bertFileNvpInfo = "latest.dump";

static bool checkBertFlag = false;
static bert_host_status hostStatus = HOST_COMPLETE;
std::unique_ptr<phosphor::Timer> bertHostOffTimer,
                                 bertHostOnTimer,
                                 bertHostFailTimer,
                                 bertClaimSPITimer;

static void bertClaimSPITimeOutHdl(void);
static int handshakeSPI(bert_handshake_cmd val);

using namespace pldm::utils;

static void addBertSELLog(uint8_t crashIndex, uint32_t sectionType,
                          uint32_t subTypeId)
{
    /* Log SEL and Redfish */
    std::vector<uint8_t> evtData;
    std::string msg = "PLDM BERT SEL Event";
    uint8_t recordType = 0xC0;
    uint8_t evtData1, evtData2, evtData3, evtData4, evtData5, evtData6;
    /*
     * OEM IPMI SEL Recode Format for RAS event:
     * evtData1:
     *    Sensor Type: 0xC1 - Ampere OEM Sensor Type
     * evtData2:
     *    Event Code: 0x04 - BMC detects BERT valid on BMC booting
     * evtData3:
     *     crash file index
     * evtData4:
     *     Section Type
     * evtData5:
     *     Error Sub Type ID high byte
     * evtData6:
     *     Error Sub Type ID low byte
     */
    evtData1 = BERT_SENSOR_TYPE_OEM;
    evtData2 = BERT_EVENT_CODE_OEM;
    evtData3 = crashIndex;
    evtData4 = sectionType ;
    evtData5 = subTypeId >> 8;
    evtData6 = subTypeId;
    /*
     * OEM data bytes
     *    Ampere IANA: 3 bytes [0x3a 0xcd 0x00]
     *    event data: 9 bytes [evtData1 evtData2 evtData3
     *                         evtData4 evtData5 evtData6
     *                         0x00     0x00     0x00 ]
     *    sel type: 1 byte [0xC0]
     */
    evtData.reserve(12);
    evtData.push_back(0x3a);
    evtData.push_back(0xcd);
    evtData.push_back(0);
    evtData.push_back(evtData1);
    evtData.push_back(evtData2);
    evtData.push_back(evtData3);
    evtData.push_back(evtData4);
    evtData.push_back(evtData5);
    evtData.push_back(evtData6);
    evtData.push_back(0);
    evtData.push_back(0);
    evtData.push_back(0);

    pldm::utils::addOEMSelLog(msg, evtData, recordType);
}

static int handshakeSPI(bert_handshake_cmd val)
{
    std::stringstream pidStr;
    int ret = 0;

    if (val == STOP_HS)
        bertClaimSPITimer->stop();

    pidStr << getpid();
    std::string hsStr = (val == START_HS) ? "start_handshake" : "stop_handshake";
    std::string cmd = std::string(HANDSHAKE_SPI_SCRIPT) + " " +
                      hsStr + " " + pidStr.str();
    ret = system(cmd.c_str());
    if (ret)
    {
        std::cerr << "Cannot start/stop handshake SPI-NOR\n";
        return ret;
    }
    if (!ret && (val == START_HS))
    {
        bertClaimSPITimer->start(std::chrono::milliseconds(BERT_CLAIMSPI_TIMEOUT));
    }
    return ret;
}

static void bertClaimSPITimeOutHdl(void)
{
    std::cerr << "Timeout " << BERT_CLAIMSPI_TIMEOUT
              << "ms for claiming SPI bus. Release it\n";
    handshakeSPI(STOP_HS);
}

static int enableAccessHostSpiNor(void)
{
    std::stringstream pidStr;
    int ret = 0;

    pidStr << getpid();
    std::string cmd = std::string(HANDSHAKE_SPI_SCRIPT) + " lock " + pidStr.str();
    ret = system(cmd.c_str());
    if (ret)
    {
        std::cerr << "Cannot lock SPI-NOR resource\n";
        return ret;
    }
    cmd = std::string(HANDSHAKE_SPI_SCRIPT) + " bind " + pidStr.str();
    ret = system(cmd.c_str());
    if (ret)
    {
        std::cerr << "Cannot bind SPI-NOR resource\n";
        return ret;
    }

    return ret;
}

static int disableAccessHostSpiNor(void)
{
    std::stringstream pidStr;
     int ret = 0;

     pidStr << getpid();
     std::string cmd = std::string(HANDSHAKE_SPI_SCRIPT) + " unbind " + pidStr.str();
     ret = system(cmd.c_str());
     if (ret)
         std::cerr << "Cannot unbind SPI-NOR resource\n";
     cmd = std::string(HANDSHAKE_SPI_SCRIPT) + " unlock " + pidStr.str();
     ret = system(cmd.c_str());
     if (ret)
     {
         std::cerr << "Cannot unlock SPI-NOR resource\n";
         return ret;
     }

     return ret;
}

static int spinorfsRead(char *file, char *buff, uint32_t offset, uint32_t size)
{
    int ret;

    if (spinorfs_open(file, SPINORFS_O_RDONLY))
        return -1;

    ret = spinorfs_read(buff, offset, size);
    spinorfs_close();

    return ret;
}

static int spinorfsWrite(char *file, char *buff, uint32_t offset, uint32_t size)
{
    int ret;

    if (spinorfs_open(file, SPINORFS_O_WRONLY | SPINORFS_O_TRUNC))
        return -1;

    ret = spinorfs_write(buff, offset, size);
    spinorfs_close();

    return ret;
}

static int handshakeReadSPI(bert_host_state state,
                            char *file, char *buff,
                            uint32_t size)
{
    uint32_t j = 0;

    if (state == HOST_ON)
    {
        for (j = 0; j < size/BLOCK_SIZE; j++)
        {
            if (handshakeSPI(START_HS))
                return -1;
            if (spinorfsRead(file, buff + j*BLOCK_SIZE,
                             j*BLOCK_SIZE, BLOCK_SIZE) < 0)
                goto exit_err;
            handshakeSPI(STOP_HS);
        }
        if (handshakeSPI(START_HS))
            return -1;
        if (spinorfsRead(file, buff + j*BLOCK_SIZE,
                         j*BLOCK_SIZE,
                         size - j*BLOCK_SIZE) < 0)
            goto exit_err;
        handshakeSPI(STOP_HS);
    }
    else
    {
        if (spinorfsRead(file, buff, 0, size) < 0)
            goto exit_err;
    }
    return 0;

exit_err:
    handshakeSPI(STOP_HS);
    return -1;
}

static int handshakeWriteSPI(bert_host_state state,
                             char *file, char *buff,
                             uint32_t size)
{
    uint32_t j = 0;

    if (state == HOST_ON)
    {
        for (j = 0; j < size/BLOCK_SIZE; j++)
        {
            if (handshakeSPI(START_HS))
                return -1;
            if (spinorfsWrite(file, buff + j*BLOCK_SIZE,
                              j*BLOCK_SIZE, BLOCK_SIZE) < 0)
                goto exit_err;
            handshakeSPI(STOP_HS);
        }
        if (handshakeSPI(START_HS))
            return -1;
        if (spinorfsWrite(file, buff + j*BLOCK_SIZE,
                          j*BLOCK_SIZE,
                          size - j*BLOCK_SIZE) < 0)
            goto exit_err;
        handshakeSPI(STOP_HS);
    }
    else
    {
        if (spinorfsWrite(file, buff, 0, size) < 0)
            goto exit_err;
    }

    return 0;

exit_err:
    handshakeSPI(STOP_HS);
    return -1;
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

static int initSPIDevice(bert_host_state state, int *fd)
{
    int ret = 0;
    uint32_t size = 0, offset = 0;

    if (state == HOST_ON)
    {
        ret = handshakeSPI(START_HS);
        if (ret)
            return -1;
    }
    if (openSPINorDevice(fd))
    {
        std::cerr << "Can not open SPINOR device\n";
        ret = -1;
        goto exit;
    }
    if (spinorfs_gpt_disk_info(*fd, 0))
    {
        std::cerr << "Get GPT Info failure\n";
        ret = -1;
        goto exit;
    }
    if (spinorfs_gpt_part_name_info((char*) bertNvp.c_str(), &offset, &size))
    {
        std::cerr << "Get GPT Partition Info failure\n";
        ret = -1;
        goto exit;
    }
    if (spinorfs_mount(*fd, size, offset))
    {
        std::cerr << "Mount Partition failure\n";
        ret = -1;
        goto exit;
    }
exit:
     if (state == HOST_ON)
        handshakeSPI(STOP_HS);
     return ret;
}

static int handshakeSPIHandler(bert_host_state state)
{
    int ret = 0;
    uint8_t i;
    int devFd = -1;
    std::string bertDumpPath, bertFileNvpInfoPath, faultLogFilePath;
    std::string prefix, primaryLogId;
    AmpereBertPartitionInfo bertInfo;
    AmpereBertPayloadSection *bertPayload;
    bool isValidBert = false;

    ret = initSPIDevice(state, &devFd);
    if (ret)
    {
        std::cerr << "Init SPI Device failure\n";
        goto exit;
    }
    /* Read Bert Partition Info from latest.ras */
    ret = handshakeReadSPI(state, (char*) bertFileNvp.c_str(),
                           (char*) &bertInfo, sizeof(AmpereBertPartitionInfo));
    if (ret)
    {
        std::cerr << "Read " << bertFileNvp.c_str() << " failure\n";
        goto exit;
    }
#ifdef BERT_DEBUG
   for (int i = 0; i < BERT_MAX_NUM_FILE; i++)
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
           !bertInfo.files[i].flags.member.pendingBMC)
            continue;
        /*
         * Valid bert header and BMC flag is not set imply a new
         * bert record for BMC
         */
        bertDumpPath = std::string(BERT_LOG_DIR) +
                       std::string(bertInfo.files[i].name);
        std::vector<char> crashBufVector(bertInfo.files[i].size, 0);
        char *crashBuf = crashBufVector.data();
        ret = handshakeReadSPI(state, bertInfo.files[i].name,
                               crashBuf, bertInfo.files[i].size);
        if (!ret)
        {
            std::ofstream out (bertDumpPath.c_str(), std::ofstream::binary);
            if(!out.is_open())
            {
                std::cerr << "Can not open ofstream for "
                          << bertDumpPath.c_str() << "\n";
                continue;
            }
            out.write(crashBuf, bertInfo.files[i].size);
            out.close();
            /* Set BMC flag to 0 to indicated processed by BMC */
            bertInfo.files[i].flags.member.pendingBMC = 0;

#ifdef BERT_DEBUG
            bertPayload = (AmpereBertPayloadSection *) crashBuf;
            std::cerr << "firmwareVersion = " << bertPayload->firmwareVersion << "\n";
            std::cerr << "totalBertLength = " << bertPayload->totalBertLength << "\n";
            std::cerr << "sectionType = " << bertPayload->header.sectionType << "\n";
            std::cerr << "sectionLength = " << bertPayload->header.sectionLength << "\n";
            std::cerr << "sectionInstance = " << bertPayload->header.sectionInstance << "\n";
            std::cerr << "sectionsValid = " << bertPayload->sectionsValid.reg << "\n";
#endif
            std::string prefix = "RAS_BERT_";
            std::string type = "BERT";
            primaryLogId = pldm::utils::getUniqueEntryID(prefix);
            faultLogFilePath = std::string(CRASHDUMP_LOG_PATH) + primaryLogId;
            std::filesystem::copy(bertDumpPath.c_str(), faultLogFilePath.c_str(),
                    std::filesystem::copy_options::overwrite_existing);
            std::filesystem::remove(bertDumpPath.c_str());
            /* Add SEL and Redfish */
            bertPayload = (AmpereBertPayloadSection *) crashBuf;
            AmpereGenericHeader *cperData = &(bertPayload->genericHeader);
            addBertSELLog(i, bertPayload->header.sectionType, cperData->subTypeId);
            pldm::utils::addFaultLogToRedfish(primaryLogId, type);
            isValidBert = true;
        }
        else
        {
            std::cerr << "Read " << bertInfo.files[i].name << " failure\n";
            continue;
        }
    }
    if (!isValidBert)
        goto exit;

    /* Write back to BERT file info to indicate BMC consumed BERT record */
    ret = handshakeWriteSPI(state, (char*) bertFileNvp.c_str(),
                            (char*) &bertInfo,
                            sizeof(AmpereBertPartitionInfo));
    if (ret < 0) {
        std::cerr << "Update " << bertFileNvp.c_str() << " failure\n";
        goto exit;
    }

exit:
    close(devFd);
    spinorfs_unmount();
    return ret;
}

int bertHandler(bool isBertTrigger, bert_host_state state)
{
    int ret = 0;

    if (!isBertTrigger)
        return 0;

    if (enableAccessHostSpiNor())
    {
        std::cerr << "Cannot enable access SPI-NOR\n";
        return -1;
    }

    ret = handshakeSPIHandler(state);

    if (disableAccessHostSpiNor())
    {
        std::cerr << "Cannot disable access SPI-NOR\n";
        return -1;
    }

    return ret;
}

void setBertCheck(bool val)
{
    checkBertFlag = val;
}

bool isBertCheck()
{
    return checkBertFlag;
}

void setHostStatus(bert_host_status val)
{
    hostStatus = val;
}

void checkValidBertRecord(bert_host_state state)
{
    if (!std::filesystem::is_directory(BERT_LOG_DIR))
         std::filesystem::create_directories(BERT_LOG_DIR);
    if (!std::filesystem::is_directory(CRASHDUMP_LOG_PATH))
        std::filesystem::create_directories(CRASHDUMP_LOG_PATH);

    bertHandler(true, state);
}

static void bertHostFailTimeOutHdl(void)
{
    if (hostStatus == HOST_FAILURE)
    {
        std::cerr << "Host boot fail. Read BERT\n";
        checkValidBertRecord(HOST_ON);
    }
}

static void bertHostOnTimeOutHdl(void)
{
    if (hostStatus == HOST_COMPLETE)
    {
        std::cerr << "UEFI already boot completed. Read BERT\n";
        checkValidBertRecord(HOST_ON);
    }
}

void handleBertHostOnEvent(void)
{
    /* Need to delay about 5s to make sure host sent
     * boot progress event to BMC
     */
    bertHostOnTimer = std::make_unique<phosphor::Timer>(
                                 [&](void) { bertHostOnTimeOutHdl(); });
    bertHostOnTimer->start(std::chrono::milliseconds(BERT_HOSTON_TIMEOUT));
    /* Check bert after host boot fails 120s timeout */
    bertHostFailTimer = std::make_unique<phosphor::Timer>(
                                 [&](void) { bertHostFailTimeOutHdl(); });
    bertHostFailTimer->start(std::chrono::milliseconds(BERT_HOSTFAIL_TIMEOUT));
    setBertCheck(true);
    bertClaimSPITimer = std::make_unique<phosphor::Timer>(
                                 [&](void) { bertClaimSPITimeOutHdl(); });
}

static void bertHostOffTimeOutHdl(void)
{
    constexpr auto hostStateInterface =
        "xyz.openbmc_project.State.Host";
    constexpr auto hostStatePath = "/xyz/openbmc_project/state/host0";
    try
    {
        auto propVal = pldm::utils::DBusHandler().getDbusPropertyVariant(
                       hostStatePath, "CurrentHostState", hostStateInterface);
        const auto& currHostState = std::get<std::string>(propVal);
        if ((currHostState == "xyz.openbmc_project.State.Host.HostState.Off"))
        {
            std::cerr << "Host is off. Read SPI to check valid BERT\n";
            checkValidBertRecord(HOST_OFF);
        }
    }
    catch (const sdbusplus::exception::exception& e)
    {
        std::cerr << "Error in getting current host state.\n";
    }
}

void handleBertHostOffEvent(void)
{
    /* pldmd service start early than State Manager service, so we need to delay 20s
     * before checking current host state
     */
    bertHostOffTimer = std::make_unique<phosphor::Timer>(
                                 [&](void) { bertHostOffTimeOutHdl(); });
    bertHostOffTimer->start(std::chrono::milliseconds(BERT_HOSTOFF_TIMEOUT));
}

