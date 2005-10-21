/*
   Module .............libbt                                
   File ...............gimbals.c                                          
   Author .............Brian Zenowich                                 
   Creation Date ......Oct 31, 2003                                 

 ***********************************************************************  
 *                                                                      
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:                                                              
 *   Reads the gimbles angles, initializes the gimbals if necessary                        
 *                                                                      
 *  REVISION HISTORY:                                                   
 *  031031 - BZ - File created                                          
 *                                                                      
 *======================================================================*/

/* \file gimbals.c  
   \brief Reads the gimbles angles, initializes the gimbals if necessary
    
    
*/ 

#include "btcan.h"
#include "btsystem.h"
#include "stdio.h"
#include "syslog.h"
#include "btwam.h"
#define TWOPI 6.283185
int gimbalsInit = 0;
double gimbalsOffset[3];
double gimbalsGain[3];

int getGimbalsAngles(double *gimbals)
{
    wam_struct *WAM;
    
    WAM = GetWAM();
#if 0    
    /* Read the gimbles puck for its three positions (A/D readings) */
    getPositions(
        act[0].bus   /** Pointer to the CANdev_t structure */, 
        5                   /** Group number of pucks whose property we want */, 
        3                           /** Number of pucks in this group (number of responses to wait for) */, 
        
        temp                     /** An array of replies indexed by the Node ID */);
#endif

    /* Extract the gimbals position information from the temp array */
    /* Gimbals position is returned in Q4.12, so we must divide by 2^12 to get radians */
    gimbals[0] = TWOPI * WAM->act[0].puck.position / WAM->act[0].motor.counts_per_rev;// * gimbalsGain[0] + gimbalsOffset[0];
    gimbals[1] = TWOPI * WAM->act[1].puck.position / WAM->act[1].motor.counts_per_rev;// * gimbalsGain[1] + gimbalsOffset[1];
    gimbals[2] = TWOPI * WAM->act[2].puck.position / WAM->act[2].motor.counts_per_rev;// * gimbalsGain[2] + gimbalsOffset[2];
    
    return(0); /* Return success */
}

int initGimbals(void)
{
    FILE *inFile;
    int i;
    wam_struct *WAM;
    
    WAM = GetWAM();
    if(!gimbalsInit)
    {
        /* Wake the gimbals puck */
        wakePuck(0, 5);
        
        //Wait 100ms for the pucks to come online
        usleep(100000);
        setProperty(0,5,DIG0,FALSE,1);
        setProperty(0,5,DIG1,FALSE,1);
        WAM->act[4].motor.counts_per_rev = 4096.0;
        WAM->act[5].motor.counts_per_rev = 4096.0;
        WAM->act[6].motor.counts_per_rev = 4096.0;
#if 0          
        /* Read the gain/offset information from the gimbals.dat file */
        if((inFile = fopen("gimbals.dat", "r")) == NULL)
        {
            syslog(LOG_ERR, "Could not open gimbals.dat file!");
            return(1);
        }
        for(i = 0; i < 3; i++)
        {
            fscanf(inFile, "%lf %lf", &gimbalsGain[i], &gimbalsOffset[i]);
        }
        fclose(inFile);
#endif        
        gimbalsInit = 1;
    }
}















