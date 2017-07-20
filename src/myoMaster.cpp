#include "roboy_managing_node_fpga/myoMaster.hpp"

static BOOL* pfGsOff_l;

PI_IN* MyoMaster::pProcessImageIn_l;
const PI_OUT* MyoMaster::pProcessImageOut_l;
bool MyoMaster::updateControllerConfig = false;
int32_t MyoMaster::setPoints[14];
control_Parameters_t MyoMaster::MotorConfig[NUMBER_OF_CONTROL_MODES][NUMBER_OF_MOTORS_PER_FPGA];
bool MyoMaster::fExit = false;
tOptions MyoMaster::opts;
mutex MyoMaster::mux;

MyoMaster::MyoMaster() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_managing_node_fpga",
                  ros::init_options::NoRosout);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
}

MyoMaster::~MyoMaster() {
    fExit = true;
#ifdef RUN_IN_THREAD
    if(powerLinkThread->joinable())
        powerLinkThread->join();
#endif
    shutdownPowerlink();
    system_exit();
}

void MyoMaster::initialize(int argc, char *argv[]){
    if (getOptions(argc, argv, &opts) < 0)
        ROS_WARN("invalid command line params");

    tOplkError ret = kErrorOk;

    if (system_init() != 0) {
        ROS_ERROR("Error initializing system!");
        powerlink_initialized = false;
    }

    eventlog_init(opts.logFormat,
                  opts.logLevel,
                  opts.logCategory,
                  (tEventlogOutputCb) console_printlogadd);

    pfGsOff_l = new unsigned char;
    *pfGsOff_l = true;

    printf("----------------------------------------------------\n");
    printf("myoFPGA console MN \n");
    printf("Using openPOWERLINK stack: %s\n", oplk_getVersionString());
    printf("----------------------------------------------------\n");

    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "myoMaster: Stack version:%s Stack configuration:0x%08X",
                          oplk_getVersionString(),
                          oplk_getStackConfiguration());

    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Using CDC file: %s",
                          opts.cdcFile);

    const BYTE aMacAddr_l[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    ret = initPowerlink(CYCLE_LEN, opts.cdcFile, opts.devName, aMacAddr_l);

    if (ret != kErrorOk)
        powerlink_initialized = false;

    ret = initProcessImage();
    if (ret != kErrorOk)
        powerlink_initialized = false;
}

void MyoMaster::mainLoop() {
    tOplkError ret = kErrorOk;
    char cKey = 0;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)

#if defined(CONFIG_USE_SYNCTHREAD)
    system_startSyncThread(processSync);
#endif

#endif

    // start stack processing by sending a NMT reset command
    ret = oplk_execNmtCommand(kNmtEventSwReset);
    if (ret != kErrorOk) {
        fprintf(stderr,
                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        return;
    }

    ROS_INFO("-------------------------------");
    ROS_INFO("Press Esc to leave the program");
    ROS_INFO("Press r to reset the node");
    ROS_INFO("-------------------------------");

    while (!fExit) {
        if (console_kbhit()) {
            cKey = (char) console_getch();
            switch (cKey) {
                case 'r':
                    ret = oplk_execNmtCommand(kNmtEventSwReset);
                    if (ret != kErrorOk) {
                        ROS_ERROR(
                                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                                debugstr_getRetValStr(ret),
                                ret);
                        fExit = true;
                    }
                    break;

                case 'c':
                    ret = oplk_execNmtCommand(kNmtEventNmtCycleError);
                    if (ret != kErrorOk) {
                        ROS_ERROR(
                                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                                debugstr_getRetValStr(ret),
                                ret);
                        fExit = true;
                    }
                    break;

                case 0x1B:
                    fExit = true;
                    break;

                default:
                    break;
            }
        }

        if (system_getTermSignalState() == TRUE) {
            fExit = true;
            eventlog_printMessage(kEventlogLevelInfo,
                                  kEventlogCategoryControl,
                                  "Received termination signal, exiting...");
            return;
        }

        if (oplk_checkKernelStack() == FALSE) {
            fExit = true;
            eventlog_printMessage(kEventlogLevelFatal,
                                  kEventlogCategoryControl,
                                  "Kernel stack has gone! Exiting...");
            return;
        }

#if (defined(CONFIG_USE_SYNCTHREAD) || \
     defined(CONFIG_KERNELSTACK_DIRECTLINK))
        system_msleep(100);
#else
        processSync();
#endif
    }
}

void MyoMaster::start(){
    if(powerlink_initialized) {
#ifdef RUN_IN_THREAD
        powerLinkThread = new std::thread(&MyoMaster::mainLoop, this);
        powerLinkThread->detach();
#else
        mainLoop();
#endif
    }
}

bool MyoMaster::changeControl(int motor, int mode){
    if(mode>=POSITION && mode <=DISPLACEMENT) {
        roboy_communication_middleware::MotorConfig msg;
        msg.id = 0;
        msg.motors.push_back(motor);
        msg.control_mode.push_back(mode);
        msg.outputPosMax.push_back(MotorConfig[mode][motor].outputPosMax);
        msg.outputNegMax.push_back(MotorConfig[mode][motor].outputNegMax);
        msg.spPosMax.push_back(MotorConfig[mode][motor].spPosMax);
        msg.spNegMax.push_back(MotorConfig[mode][motor].spNegMax);
        msg.Kp.push_back(MotorConfig[mode][motor].Kp);
        msg.Ki.push_back(MotorConfig[mode][motor].Ki);
        msg.Kd.push_back(MotorConfig[mode][motor].Kd);
        msg.forwardGain.push_back(MotorConfig[mode][motor].forwardGain);
        msg.deadBand.push_back(MotorConfig[mode][motor].deadBand);
        msg.IntegralPosMax.push_back(MotorConfig[mode][motor].IntegralPosMax);
        msg.IntegralNegMax.push_back(MotorConfig[mode][motor].IntegralNegMax);
        motorConfig.publish(msg);
        return true;
    }else{
        ROS_ERROR("invalid controlmode %d requested ", mode);
        return false;
    }
}

bool MyoMaster::changeControl(int mode){
    if(mode>=POSITION && mode <=DISPLACEMENT) {
        roboy_communication_middleware::MotorConfig msg;
        msg.id = 0;
        for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
            msg.motors.push_back(motor);
            msg.control_mode.push_back(mode);
            msg.outputPosMax.push_back(MotorConfig[mode][motor].outputPosMax);
            msg.outputNegMax.push_back(MotorConfig[mode][motor].outputNegMax);
            msg.spPosMax.push_back(MotorConfig[mode][motor].spPosMax);
            msg.spNegMax.push_back(MotorConfig[mode][motor].spNegMax);
            msg.Kp.push_back(MotorConfig[mode][motor].Kp);
            msg.Ki.push_back(MotorConfig[mode][motor].Ki);
            msg.Kd.push_back(MotorConfig[mode][motor].Kd);
            msg.forwardGain.push_back(MotorConfig[mode][motor].forwardGain);
            msg.deadBand.push_back(MotorConfig[mode][motor].deadBand);
            msg.IntegralPosMax.push_back(MotorConfig[mode][motor].IntegralPosMax);
            msg.IntegralNegMax.push_back(MotorConfig[mode][motor].IntegralNegMax);
        }
        motorConfig.publish(msg);
        return true;
    }else{
        ROS_ERROR("invalid controlmode %d requested ", mode);
        return false;
    }
}

bool MyoMaster::changeControl(int motor, control_Parameters_t &params){
    MotorConfig[params.control_mode][motor] = params;
    return changeControl(motor, params.control_mode);
}

bool MyoMaster::changeControl(vector<int> &motors, vector<control_Parameters_t> &params){
    uint i = 0;
    for(auto motor:motors){
        MotorConfig[params[i].control_mode][motor] = params[i];
        i++;
        if(!changeControl(motor, params[i].control_mode))
            return false;
    }
    return true;
}

bool MyoMaster::changeControl(control_Parameters_t &params){
    for(uint motor = 0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++){
        if(!changeControl(motor, params))
            return false;
    }
    return true;
}

void MyoMaster::changeSetPoint(int motor, int setPoint){
    setPoints[motor] = setPoint;
}

void MyoMaster::changeSetPoint(int setPoint){
    fill_n(setPoints,14,setPoint);
}

void MyoMaster::MotorCommand(const roboy_communication_middleware::MotorCommand::ConstPtr &msg){
    uint i = 0;
    for(auto motor:msg->motors){
        setPoints[motor] = msg->setPoints[i++];
    }
}

tOplkError MyoMaster::initProcessImage() {
    tOplkError ret = kErrorOk;
    UINT errorIndex = 0;

    ROS_INFO("Initializing process image...");
    ROS_INFO("Size of process image: Input = %lu Output = %lu",
             (ULONG) sizeof(PI_IN),
            (ULONG) sizeof(PI_OUT));
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Allocating process image: Input:%lu Output:%lu",
                          (ULONG) sizeof(PI_IN),
            (ULONG) sizeof(PI_OUT));

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
        return ret;

    pProcessImageIn_l = (PI_IN *) oplk_getProcessImageIn();
    pProcessImageOut_l = (const PI_OUT *) oplk_getProcessImageOut();

    errorIndex = obdpi_setupProcessImage();
    if (errorIndex != 0) {
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "Setup process image failed at index 0x%04x\n",
                              errorIndex);
        ret = kErrorApiPINotAllocated;
    }

    return ret;
}

int MyoMaster::getOptions(int argc_p, char *const argv_p[], tOptions *pOpts_p) {
    int opt;

    /* setup default parameters */
    strncpy(pOpts_p->cdcFile, "mnobd.cdc", 256);
    strncpy(pOpts_p->devName, "\0", 128);
    pOpts_p->pLogFile = NULL;
    pOpts_p->logFormat = kEventlogFormatReadable;
    pOpts_p->logCategory = 0xffffffff;
    pOpts_p->logLevel = 0xffffffff;

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "c:l:pv:t:d:")) != -1) {
        switch (opt) {
            case 'c':
                strncpy(pOpts_p->cdcFile, optarg, 256);
                break;

            case 'd':
                strncpy(pOpts_p->devName, optarg, 128);
                break;

            case 'p':
                pOpts_p->logFormat = kEventlogFormatParsable;
                break;

            case 'v':
                pOpts_p->logLevel = strtoul(optarg, NULL, 16);
                break;

            case 't':
                pOpts_p->logCategory = strtoul(optarg, NULL, 16);
                break;

            default: /* '?' */
#if defined(CONFIG_USE_PCAP)
                ROS_INFO("Usage: %s [-c CDC-FILE] [-d DEV_NAME] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]", argv_p[0]);
                ROS_INFO(" -d DEV_NAME: Ethernet device name to use e.g. eth1");
#else
                ROS_INFO("Usage: %s [-c CDC-FILE] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]", argv_p[0]);
#endif
                ROS_INFO(" -p: Use parsable log format");
                ROS_INFO("              If option is skipped the program prompts for the interface.");
                ROS_INFO(" -v LOGLEVEL: A bit mask with log levels to be printed in the event logger");
                ROS_INFO(" -t LOGCATEGORY: A bit mask with log categories to be printed in the event logger");
                return -1;
        }
    }

    return 0;
}

tOplkError MyoMaster::initPowerlink(UINT32 cycleLen_p,
                                    const char* cdcFileName_p,
                                    const char* devName_p,
                                    const UINT8* macAddr_p){
    tOplkError          ret = kErrorOk;
    tOplkApiInitParam   initParam;
    static char         devName[128];

    ROS_INFO("Initializing openPOWERLINK stack...");
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryControl,
                          "Initializing openPOWERLINK stack");

#if defined(CONFIG_USE_PCAP)
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Using libpcap for network access");
    if (devName_p[0] == '\0')
    {
        if (selectPcapDevice(devName) < 0)
            return kErrorIllegalInstance;
    }
    else
        strncpy(devName, devName_p, 128);
#else
    UNUSED_PARAMETER(devName_p);
#endif

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    // pass selected device name to Edrv
    initParam.hwParam.pDevName = devName;
    initParam.nodeId = NODEID;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = UINT_MAX;
    initParam.cycleLen                = cycleLen_p;       // required for error detection
    initParam.isochrTxMaxPayload      = 1490;              // const
    initParam.isochrRxMaxPayload      = 1490;             // const
    initParam.presMaxLatency          = 50000;            // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 1400;               // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit     = 1400;               // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 200000;           // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                // required for error detection
    initParam.asyncMtu                = 300;             // required to set up max frame size
    initParam.prescaler               = 2;                // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 1000;
    initParam.deviceType              = UINT_MAX;         // NMT_DeviceType_U32
    initParam.vendorId                = UINT_MAX;         // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = UINT_MAX;         // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = UINT_MAX;         // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = UINT_MAX;         // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = &MyoMaster::processEvents;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    initParam.pfnCbSync  = (tSyncCb)&MyoMaster::processSync;
#else
    initParam.pfnCbSync  = NULL;
#endif

    // Initialize object dictionary
    ret = obdcreate_initObd(&initParam.obdInitParam);
    if (ret != kErrorOk)
    {
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "obdcreate_initObd() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    // initialize POWERLINK stack
    ret = oplk_initialize();
    if (ret != kErrorOk)
    {
        ROS_ERROR(
                "oplk_initialize() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "oplk_init() failed with \"%s\" (0x%04x)",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    ret = oplk_create(&initParam);
    if (ret != kErrorOk)
    {
        ROS_ERROR(
                "oplk_create() failed with \"%s\" (0x%04x)",
                debugstr_getRetValStr(ret),
                ret);
        return ret;
    }

    ret = oplk_setCdcFilename(cdcFileName_p);
    if (ret != kErrorOk)
    {
        ROS_ERROR(
                "oplk_setCdcFilename() failed with \"%s\" (0x%04x)",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "oplk_setCdcFilename() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    return kErrorOk;
}

tOplkError MyoMaster::processSync() {
    tOplkError ret;

    ret = oplk_waitSyncEvent(100000);
    if (ret != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

//    static int iter = 1;
//    if ((iter++) % 50 == 0) {
//        printf("\n############## myoFPGA ###################\n");
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_1);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_2);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_3);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_4);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_5);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_6);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_7);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_8);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_9);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_10);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_11);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_12);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_13);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_14);
//    }
    {
        std::lock_guard<std::mutex> lock(mux);
        // setpoints for 16 motors
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_1 = setPoints[0];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_2 = setPoints[1];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_3 = setPoints[2];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_4 = setPoints[3];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_5 = setPoints[4];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_6 = setPoints[5];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_7 = setPoints[6];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_8 = setPoints[7];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_9 = setPoints[8];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_10 = setPoints[9];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_11 = setPoints[10];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_12 = setPoints[11];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_13 = setPoints[12];
        pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_14 = setPoints[13];
    }
    ret = oplk_exchangeProcessImageIn();

    return ret;
}

void MyoMaster::shutdownPowerlink() {
    UINT i;
    tOplkError ret = kErrorOk;

    // NMT_GS_OFF state has not yet been reached
    fGsOff_l = FALSE;

#if (!defined(CONFIG_KERNELSTACK_DIRECTLINK) && \
     defined(CONFIG_USE_SYNCTHREAD))
    system_stopSyncThread();
    system_msleep(100);
#endif

    // halt the NMT state machine so the processing of POWERLINK frames stops
    ret = oplk_execNmtCommand(kNmtEventSwitchOff);
    if (ret != kErrorOk) {
        fprintf(stderr,
                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
    }

    // small loop to implement timeout waiting for thread to terminate
    for (i = 0; i < 1000; i++) {
        if (fGsOff_l)
            break;
    }

    ROS_WARN("Stack is in state off ... Shutdown");
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryControl,
                          "Stack is in state off ... Shutdown openPOWERLINK");

    oplk_destroy();
    oplk_exit();
}

tOplkError MyoMaster::processEvents(tOplkApiEventType EventType_p,
                                    const tOplkApiEventArg *pEventArg_p,
                                    void *pUserArg_p){
    tOplkError  ret = kErrorOk;

    // check if NMT_GS_OFF is reached
    switch (EventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(&pEventArg_p->nmtStateChange, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(&pEventArg_p->internalError, pUserArg_p);
            break;

        case kOplkApiEventHistoryEntry:
            ret = processHistoryEvent(&pEventArg_p->errorHistoryEntry, pUserArg_p);
            break;

        case kOplkApiEventNode:
            ret = processNodeEvent(&pEventArg_p->nodeEvent, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(&pEventArg_p->pdoChange, pUserArg_p);
            break;

        case kOplkApiEventCfmProgress:
            ret = processCfmProgressEvent(&pEventArg_p->cfmProgress, pUserArg_p);
            break;

        case kOplkApiEventCfmResult:
            ret = processCfmResultEvent(&pEventArg_p->cfmResult, pUserArg_p);
            break;

        default:
            break;
    }

    return ret;
}

tOplkError MyoMaster::processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                              void* pUserArg_p){
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        console_printlog("Application event module is not initialized!\n");
        return kErrorGeneralError;
    }

    eventlog_printStateEvent(pNmtStateChange_p);

    switch (pNmtStateChange_p->newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process() and main()
            ret = kErrorShutdown;

            printf("Stack received kNmtGsOff!\n");

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsResetCommunication:
            break;

        case kNmtGsResetConfiguration:
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:        // Implement
        case kNmtMsNotActive:               // handling of
        case kNmtMsPreOperational1:         // different
        case kNmtMsPreOperational2:         // states here
        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
        case kNmtMsBasicEthernet:           // no break

        default:
            ROS_INFO("Stack entered state: %s",
                     debugstr_getNmtStateStr(pNmtStateChange_p->newNmtState));
            break;
    }

    return ret;
}

tOplkError MyoMaster::processErrorWarningEvent(const tEventError* pInternalError_p, void* pUserArg_p){
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printErrorEvent(pInternalError_p);

    return kErrorOk;
}

tOplkError MyoMaster::processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                          void* pUserArg_p) {
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printHistoryEvent(pHistoryEntry_p);

    return kErrorOk;
}

tOplkError MyoMaster::processNodeEvent(const tOplkApiEventNode* pNode_p,
                                       void* pUserArg_p){
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printNodeEvent(pNode_p);

    // check additional argument
    switch (pNode_p->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            break;

        case kNmtNodeEventUpdateConf:
            break;

        case kNmtNodeEventNmtState:
            ROS_INFO("Node %d entered state %s\n",
                     pNode_p->nodeId,
                     debugstr_getNmtStateStr(pNode_p->nmtState));
            break;

        case kNmtNodeEventError:
            break;

        case kNmtNodeEventFound:
            ROS_INFO("Stack found node %d\n",
                     pNode_p->nodeId);
            break;

        case kNmtNodeEventAmniReceived:
            break;

        default:
            break;
    }

    return kErrorOk;
}

tOplkError MyoMaster::processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                              void* pUserArg_p){
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmProgressEvent(pCfmProgress_p);

    return kErrorOk;
}

tOplkError MyoMaster::processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                            void* pUserArg_p){
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmResultEvent(pCfmResult_p->nodeId, pCfmResult_p->nodeCommand);

    switch (pCfmResult_p->nodeCommand)
    {
        case kNmtNodeCommandConfOk:
            break;

        case kNmtNodeCommandConfErr:
            break;

        case kNmtNodeCommandConfReset:
            break;

        case kNmtNodeCommandConfRestored:
            break;

        default:
            break;
    }

    return kErrorOk;
}

tOplkError MyoMaster::processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                            void* pUserArg_p){
    UINT        subIndex;
    UINT64      mappObject;
    tOplkError  ret;
    UINT        varLen;

    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printPdoEvent(pPdoChange_p);

    for (subIndex = 1; subIndex <= pPdoChange_p->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange_p->mappParamIndex,
                                   subIndex,
                                   &mappObject,
                                   &varLen);
        if (ret != kErrorOk)
        {
            eventlog_printMessage(kEventlogLevelError,
                                  kEventlogCategoryObjectDictionary,
                                  "Reading 0x%X/%d failed with %s(0x%X)",
                                  pPdoChange_p->mappParamIndex,
                                  subIndex,
                                  debugstr_getRetValStr(ret),
                                  ret);
            continue;
        }

        eventlog_printPdoMap(pPdoChange_p->mappParamIndex,
                             subIndex,
                             mappObject);
    }

    return kErrorOk;
}