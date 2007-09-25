char *statusTxt[] = 
{
   "STATUS_OFFLINE",
   "STATUS_RESET",
   "STATUS_ERR",
   "STATUS_READY",
};

struct defaultStruct
{
   int *key;
   long val;
};

struct defaultStruct taterDefs[] = 
{
   &TIE, 0,
   &ACCEL, 100,
   &AP, 0,
   &CT, 4096,
   &CTS, 40960,
   &DP, 0,
   &MT, 3300,
   &MV, 100,
   &MCV, 100,
   &MOV, 100,
   &OT, 0,
   &_DS, 256,
   &KP, 2500,
   &KD, 8000,
   &KI, 12,
   &HOLD, 0,
   &TSTOP, 1000,
   &OTEMP, 60,
   &PTEMP, 0,
   &OT, 0,
   &CT, 4096,
   
   NULL, 0
};

struct defaultStruct wraptorDefs[] = 
{
   &TIE, 0,
   &ACCEL, 100,
   &AP, 0,
   &CT, 750,
   &CTS, 4096,
   &DP, 0,
   &MT, 990,
   &MV, 1500,
   &MCV, 1500,
   &MOV, 1500,
   &DP, 0,
   &OT, 0,
   &CT, 1E6,
   &_DS, 2560,
   &KP, 2000,
   &KD, 8000,
   &KI, 0,
   &IKP, 4096,
   &IKI, 819,
   &IKCOR, 819,
   &GRPA, 0,
   &GRPB, 1,
   &GRPC, 4,
   &POLES, 6,
   &IPNM, 20000,
   
   NULL, 0
};

struct defaultStruct safetyDefs[] = 
{
   &TIE, 0,
   &VOLTL1, 36,
   &VOLTL2, 30,
   &VOLTH1, 54,
   &VOLTH2, 57,
   &GRPA, 1,
   &GRPB, 2,
   &GRPC, 3,
   
   NULL, 0
};

struct fcnStruct{
   void (*f)(void *data);
   char title[64];
};

struct watchStruct{
   int puckID;
   int prop;
};
