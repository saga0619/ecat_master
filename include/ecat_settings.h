#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>
#include <stdbool.h>

#define CNT_TO_RAD_46 (3.141592 * 2 / (8192 * 100)) //819200
#define CNT_TO_RAD_80 (3.141592 * 2 / (8000 * 100)) //819200

#define DEG2RAD (3.141592f / 180.0f)

#define EXT_CNT_TO_RAD_46 (3.141592 * 2 / 8192) //819200
#define EXT_CNT_TO_RAD_80 (3.141592 * 2 / 8192) //819200

#define RAD_TO_CNT_46 (1 / (CNT_TO_RAD_46))
#define RAD_TO_CNT_80 (1 / (CNT_TO_RAD_80))

#define EXT_RAD_TO_CNT_46 (1 / (EXT_CNT_TO_RAD_46))
#define EXT_RAD_TO_CNT_80 (1 / (EXT_CNT_TO_RAD_80))

#define EC_TIMEOUTMON 500

#define ELMO_DOF 33

#define ELMO_DOF_UPPER 18

#define ELMO_DOF_LOWER ELMO_DOF - ELMO_DOF_UPPER

#define LEG_DOF 12

#define CL_LOCK 20

#define UPPERBODY_DOF 21

#define PERIOD_NS 500000
#define SEC_IN_NSEC 1000000000UL

#define FORCE_CONTROL_MODE false

extern const char ifname_lower[];
extern char ifname_lower2[];
extern const char ifname_upper[];
extern char ifname_upper2[];

extern const int starting_point;
enum 
{
    READY_TO_SWITCH_ON_BIT,
    SWITCHED_ON_BIT,
    OPERATION_ENABLE_BIT,
    FAULT_BIT,
};

enum
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
};

enum ELMO
{
    ELMO_Head_Joint,
    ELMO_Neck_Joint,
    ELMO_R_Wrist1_Joint,
    ELMO_R_Wrist2_Joint,
    ELMO_L_Wrist2_Joint,
    ELMO_L_Wrist1_Joint,
    ELMO_L_Shoulder3_Joint,
    ELMO_L_Armlink_Joint,
    ELMO_R_Armlink_Joint,
    ELMO_R_Shoulder3_Joint,
    ELMO_R_Elbow_Joint,
    ELMO_R_Forearm_Joint,
    ELMO_L_Forearm_Joint,
    ELMO_L_Elbow_Joint,
    ELMO_L_Shoulder1_Joint,
    ELMO_L_Shoulder2_Joint,
    ELMO_R_Shoulder2_Joint,
    ELMO_R_Shoulder1_Joint,
    ELMO_Upperbody_Joint,
    ELMO_Waist2_Joint,
    ELMO_R_HipYaw_Joint,
    ELMO_R_HipRoll_Joint,
    ELMO_R_HipPitch_Joint,
    ELMO_R_Knee_Joint,
    ELMO_R_AnklePitch_Joint,
    ELMO_R_AnkleRoll_Joint,
    ELMO_Waist1_Joint,
    ELMO_L_HipYaw_Joint,
    ELMO_L_HipRoll_Joint,
    ELMO_L_HipPitch_Joint,
    ELMO_L_Knee_Joint,
    ELMO_L_AnklePitch_Joint,
    ELMO_L_AnkleRoll_Joint
};

enum MODEL
{
    MODEL_L_HipYaw_Joint,
    MODEL_L_HipRoll_Joint,
    MODEL_L_HipPitch_Joint,
    MODEL_L_Knee_Joint,
    MODEL_L_AnklePitch_Joint,
    MODEL_L_AnkleRoll_Joint,
    MODEL_R_HipYaw_Joint,
    MODEL_R_HipRoll_Joint,
    MODEL_R_HipPitch_Joint,
    MODEL_R_Knee_Joint,
    MODEL_R_AnklePitch_Joint,
    MODEL_R_AnkleRoll_Joint,
    MODEL_Waist1_Joint,
    MODEL_Waist2_Joint,
    MODEL_Upperbody_Joint,
    MODEL_L_Shoulder1_Joint,
    MODEL_L_Shoulder2_Joint,
    MODEL_L_Shoulder3_Joint,
    MODEL_L_Armlink_Joint,
    MODEL_L_Elbow_Joint,
    MODEL_L_Forearm_Joint,
    MODEL_L_Wrist1_Joint,
    MODEL_L_Wrist2_Joint,
    MODEL_Neck_Joint,
    MODEL_Head_Joint,
    MODEL_R_Shoulder1_Joint,
    MODEL_R_Shoulder2_Joint,
    MODEL_R_Shoulder3_Joint,
    MODEL_R_Armlink_Joint,
    MODEL_R_Elbow_Joint,
    MODEL_R_Forearm_Joint,
    MODEL_R_Wrist1_Joint,
    MODEL_R_Wrist2_Joint
};

enum MODE_OF_OPERATION
{
    ProfilePositionmode = 1,
    ProfileVelocitymode = 3,
    ProfileTorquemode = 4,
    Homingmode = 6,
    InterpolatedPositionmode = 7,
    CyclicSynchronousPositionmode = 8,
    CyclicSynchronousVelocitymode = 9,
    CyclicSynchronousTorquemode = 10,
    CyclicSynchronousTorquewithCommutationAngle = 11
};

struct elmo_gold_tx
{
    int32_t targetPosition;
    int32_t targetVelocity;
    int16_t targetTorque;
    uint16_t maxTorque;
    uint16_t controlWord;
    int8_t modeOfOperation;
};
struct elmo_gold_rx
{
    int32_t positionActualValue;
    //int32_t positionFollowingErrrorValue;
    uint32_t hommingSensor;
    uint16_t statusWord;
    //int8_t modeOfOperationDisplay;
    int32_t velocityActualValue;
    int16_t torqueActualValue;
    //int16_t torqueDemandValue;
    int32_t positionExternal;
};

enum
{
    ELMO_FAULT = 0,
    ELMO_NOTFAULT = 2,
    ELMO_READY_TO_SWITCH_ON = 3,
    ELMO_SWITCHED_ON = 4,
    ELMO_OPERATION_ENABLE = 1,
};

struct ElmoState
{
    int boot_sequence;           // = 0;
    int state;                   // = 0;
    int state_before;            // = 0;
    uint16_t check_value;        // = 0;
    uint16_t check_value_before; // = 0;

    bool commutation_ok;           // = false;
    bool commutation_required;     // = false;
    bool commutation_not_required; // = false;
    bool first_check;              // = true;
};

struct ElmoHomming
{
    bool hommingElmo;
    bool hommingElmo_before;
    bool startFound;      // = false;
    bool endFound;        // = false;
    int findZeroSequence; // = 0;
    double initTime;
    double initPos;
    double desPos;
    double posStart;
    double posEnd;
    double req_length; // = 0.2;
    double firstPos;
    double init_direction; // = 1;
    int status;
    int result;
};

enum
{
    FZ_CHECKHOMMINGSTATUS,
    FZ_FINDHOMMINGSTART,
    FZ_FINDHOMMINGEND,
    FZ_FINDHOMMING,
    FZ_GOTOZEROPOINT,
    FZ_HOLDZEROPOINT,
    FZ_FAILEDANDRETURN,
    FZ_MANUALDETECTION,
    FZ_TORQUEZERO,
};

enum
{
    EM_POSITION = 11,
    EM_TORQUE = 22,
    EM_DEFAULT = 33,
    EM_COMMUTATION = 44,
};

extern const char ELMO_NAME[ELMO_DOF][20];

//pos[i] = pos_elmo[JointMap[i]]
extern const int JointMap[ELMO_DOF];

//pos_elmo[i] = pos[JointMap2[i]]
extern const int JointMap2[ELMO_DOF];

extern const double CNT2RAD[ELMO_DOF];
extern const double EXTCNT2RAD[ELMO_DOF];

extern const double RAD2CNT[ELMO_DOF];

extern const double EXTRAD2CNT[ELMO_DOF];

extern const double NM2CNT[ELMO_DOF];

extern const int q_ext_mod_elmo_[ELMO_DOF];
//right -> left
//right hippitch front -> modval +
//left knee pitch front -> modval -
//right ankle pitch up -> modval

extern const double joint_velocity_limit[ELMO_DOF];

extern const double joint_upper_limit[ELMO_DOF];

extern const double joint_lower_limit[ELMO_DOF];

extern const double elmo_axis_direction[ELMO_DOF];

extern const double elmo_ext_axis_direction[ELMO_DOF];

extern const double upper_homming_minimum_required_length[ELMO_DOF];

extern const double pos_p_gain[ELMO_DOF];

extern const double pos_d_gain[ELMO_DOF];