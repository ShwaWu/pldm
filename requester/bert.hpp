#ifndef PLDM_REQUESTER_BERT_HPP_
#define PLDM_REQUESTER_BERT_HPP_

#include <stdint.h>
#include <string>

#define BERT_LOG_DIR               "/usr/share/pldm/bert/"
#define BERT_NAME_MAX_SIZE         15
#define BERT_MAX_NUM_FILE          3
#define BERT_CRASH_OCM_SIZE        0x40000
#define BLOCK_SIZE                 65536 /* 64KB */

typedef union {
    struct AmpereBertFileFlagsStruct{
        uint32_t valid        : 1;
        uint32_t defaultBert : 1;
        uint32_t pendingOS   : 1;
        uint32_t pendingBMC   : 1;
        uint32_t rsvd         : 28;
    } member;

    uint32_t reg;
} __attribute__((packed)) AmpereBertFileFlags;

typedef struct {
    AmpereBertFileFlags flags;
    uint32_t size;
    char name[BERT_NAME_MAX_SIZE];
} __attribute__((packed)) AmpereBertFileInfo;

typedef struct {
    AmpereBertFileInfo files[BERT_MAX_NUM_FILE];
} __attribute__((packed)) AmpereBertPartitionInfo;


typedef struct {
    uint32_t sectionType;
    uint32_t sectionLength;
    uint8_t sectionInstance;
    uint8_t rsvd[3];
    uint32_t sectionVersion;
} __attribute__((packed)) AmpereBertSectionHeader;

typedef union {
    struct AmpereGenericHeaderTypeStruct {
        uint16_t ipType      : 11;
        uint16_t isBert      : 1;
        uint16_t payloadType : 4;
    } member;

    uint16_t type;
} __attribute__((packed)) AmpereGenericHeaderType;

typedef struct
{
    AmpereGenericHeaderType typeId;
    uint16_t subTypeId;
    uint32_t instanceId;
}__attribute__((packed)) AmpereGenericHeader;

typedef union {
    struct AmpereBertValidSectionsStruct{
        uint32_t header             : 1;
        uint32_t s0SecproValid    : 1;
        uint32_t s0MproValid      : 1;
        uint32_t s1SecproValid    : 1;
        uint32_t s1MproValid      : 1;
        uint32_t coreChipletValid : 1;
        uint32_t rsvd               : 26;
    } member;

    uint32_t reg;
} __attribute__((packed)) AmpereBertValidSections;

typedef struct {
    AmpereBertSectionHeader header;
    AmpereGenericHeader genericHeader;
    AmpereBertValidSections sectionsValid;
    uint32_t totalBertLength;
    uint32_t firmwareVersion;
} __attribute__((packed)) AmpereBertPayloadSection;


enum bert_host_state{
    HOST_ON = 0,
    HOST_OFF = 1,
};

enum bert_host_status{
    HOST_BOOTING = 0,
    HOST_COMPLETE = 1,
    HOST_FAILURE = 2,
};

enum bert_handshake_cmd{
    START_HS = 0,
    STOP_HS = 1,
};


int bertHandler(bool isBertTrigger, bert_host_state state);
void checkValidBertRecord(bert_host_state state);
void handleBertHostOnEvent(void);
void handleBertHostOffEvent(void);
void setBertCheck(bool val);
bool isBertCheck();
void setHostStatus(bert_host_status val);

#endif /* PLDM_REQUESTER_BERT_HPP_ */
