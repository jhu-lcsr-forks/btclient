char *statusTxt[] = 
{
   "STATUS_OFFLINE",
   "STATUS_RESET",
   "STATUS_ERR",
   "STATUS_READY",
};

struct
{
   int *key;
   long val;
}defaults[] = 
{
   &TIE, 0,
   &ACCEL, 32,
   &AP, 0,
   &CT, 750,
   &CTS, 4096,
   &DP, 0,
   //         EN, 0x00EE,
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
   
   
   NULL, 0
};

struct fcnStruct{
   void (*f)(void *data);
   char title[64];
};

struct watchStruct{
   int puckID;
   int param;
};
