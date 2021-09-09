/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */

#include "posix_thread.h"
#include "ecat_settings.h"
#include "ethercat.h"

#define ELMO_DOF 33
#define EC_TIMEOUTMON 500
#define START_N 0
#define Q_START 15

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
bool reachedInitial[ELMO_DOF] = {false};

struct elmo_gold_rx rxPDO[ELMO_DOF];
struct elmo_gold_tx txPDO[ELMO_DOF];

//Data Layer//
//Receiver
// from elmo
float q_elmo_[ELMO_DOF];
float q_dot_elmo_[ELMO_DOF];
float torque_elmo_[ELMO_DOF];
float q_ext_elmo_[ELMO_DOF];
int joint_state_elmo_[ELMO_DOF];
int hommingElmo[ELMO_DOF];

// to robot
float q_[ELMO_DOF];
float q_dot_[ELMO_DOF];
float torque_[ELMO_DOF];
float q_ext_[ELMO_DOF];
int joint_state_[ELMO_DOF];

int8_t state_elmo_[ELMO_DOF];
int8_t state_zp_[ELMO_DOF];
int8_t state_safety_[ELMO_DOF];

//Sender
float torque_desired_elmo_[ELMO_DOF] = {0.0}; //get torque command
float q_desired_elmo_[ELMO_DOF] = {0.0};      //get joint command
float torque_desired_[ELMO_DOF] = {0.0};      //get torque command
float q_desired_[ELMO_DOF] = {0.0};           //get joint command

//
float q_zero_point[ELMO_DOF] = {0.0};
float q_zero_elmo_[ELMO_DOF] = {0.0};
float q_zero_mod_elmo_[ELMO_DOF] = {0.0};

const char cred[] = "\033[0;31m";
const char creset[] = "\033[0m";
const char cblue[] = "\033[0;34m";
const char cgreen[] = "\033[0;32m";
const char cyellow[] = "\033[0;33m";

volatile bool de_operation_ready;
volatile bool de_emergency_off;
volatile bool de_shutdown;
volatile bool de_ecat_lost;
volatile bool de_ecat_lost_before;
volatile bool de_ecat_recovered;
volatile bool de_initialize;
volatile bool de_commutation_done;
volatile bool de_zp_sequence;
volatile bool de_zp_upper_switch;
volatile bool de_zp_lower_switch;
volatile int de_debug_level;
volatile bool de_zp_upper;
volatile bool de_zp_lower;

void redtest(char *ifname, char *ifname2)
{
    int cnt, i, j, oloop, iloop;

    printf("Starting Redundant test\n");

    /* initialise SOEM, bind socket to ifname */
    (void)ifname2;
    if (ec_init_redundant(ifname, ifname2))
    //if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //0x1605 :  Target Position             32bit
                //          Target Velocity             32bit
                //          Max Torque                  16bit
                //          Control word                16bit
                //          Modes of Operation          16bit
                uint16 map_1c12[2] = {0x0001, 0x1605};

                //0x1a00 :  position actual value       32bit
                //          Digital Inputs              32bit
                //          Status word                 16bit
                //0x1a11 :  velocity actual value       32bit
                //0x1a13 :  Torque actual value         16bit
                //0x1a1e :  Auxiliary position value    32bit
                uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                //uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};
                int os;
                os = sizeof(map_1c12);
                ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, map_1c12, EC_TIMEOUTRXM);
                os = sizeof(map_1c13);
                ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, map_1c13, EC_TIMEOUTRXM);
            }

            //printf("ELMO %d : config init\n", g_init_args.ecat_device);
            ec_config_map(&IOmap);

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /* configure DC options for every DC capable slave found in the list */
            ec_configdc();

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for (cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                printf("         Out:%p,%4d In:%p,%4d\n",
                       ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
                /* check for EL2004 or EL2008 */
                if (!digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052)))
                {
                    digout = ec_slave[cnt].outputs;
                }
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* request OP state for all slaves */
            ec_writestate(0);
            /* activate cyclic process data */
            dorun = 1;
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;
                /* acyclic loop 5000 x 20ms = 10s */
                while (TRUE)
                {
                    printf("Processdata cycle %5d , Wck %3d, DCtime %12" PRId64 ", dt %12" PRId64 ", O:",
                           dorun, wkc, ec_DCtime, gl_delta);
                    for (j = 0; j < oloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].outputs + j));
                    }
                    printf(" I:");
                    for (j = 0; j < iloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].inputs + j));
                    }
                    printf("\r");
                    fflush(stdout);
                    osal_usleep(20000);
                }
                dorun = 0;
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End redundant test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % SEC_IN_NSEC;
    sec = (addtime - nsec) / SEC_IN_NSEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= SEC_IN_NSEC)
    {
        nsec = ts->tv_nsec % SEC_IN_NSEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / SEC_IN_NSEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

bool controlWordGenerate(const uint16_t statusWord, uint16_t *controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (statusWord & (1 << FAULT_BIT))
                {
                    *controlWord = 0x80;
                    return false;
                }
                else
                {
                    *controlWord = CW_SHUTDOWN;
                    return false;
                }
            }
            else
            {
                *controlWord = CW_SWITCHON;
                return true;
            }
        }
        else
        {
            *controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        *controlWord = CW_ENABLEOP;
        return true;
    }
    *controlWord = 0;
    return false;
}

void sendJointState()
{

}

void getJointCommand()
{

}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= SEC_IN_NSEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= SEC_IN_NSEC;
    }
    cycletime = *(int *)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    ec_send_processdata();

    while (1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
            dorun++;
            wkc = ec_receive_processdata(200);
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if (controlWordGenerate(rxPDO[slave - 1].statusWord, &txPDO[slave - 1].controlWord))
                {
                    reachedInitial[slave - 1] = true;
                }

                if (reachedInitial[slave - 1])
                {
                    q_elmo_[START_N + slave - 1] = rxPDO[slave - 1].positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                    hommingElmo[START_N + slave - 1] =
                        (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                    q_dot_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[10]) +
                         ((int32_t)ec_slave[slave].inputs[11] << 8) +
                         ((int32_t)ec_slave[slave].inputs[12] << 16) +
                         ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                        CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                    torque_elmo_[START_N + slave - 1] =
                        (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
                                  ((int16_t)ec_slave[slave].inputs[15] << 8));
                    q_ext_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[16]) +
                         ((int32_t)ec_slave[slave].inputs[17] << 8) +
                         ((int32_t)ec_slave[slave].inputs[18] << 16) +
                         ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
                        EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];
                    if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
                    {
                        hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
                    }
                    txPDO[slave - 1].maxTorque = (uint16)500; // originaly 1000
                }
            }

            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }
            ec_send_processdata();
        }
    }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    (void)ptr;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
    int ctime;

    printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");

    if (argc > 3)
    {
        dorun = 0;
        ctime = atoi(argv[3]);

        /* create RT thread */
        osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&ctime);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

        /* start acyclic part */
        redtest(argv[1], argv[2]);
    }
    else
    {
        printf("Usage: red_test ifname1 ifname2 cycletime\nifname = eth0 for example\ncycletime in us\n");
    }

    printf("End program\n");

    return (0);
}