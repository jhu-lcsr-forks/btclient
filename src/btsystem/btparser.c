/*======================================================================*
 *  Module .............libbtsystem
 *  File ...............btparser.c
 *  Author .............Brian Zenowich
 *  Creation Date ......15 Feb 2003
 *  Addtl Authors ......
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 * Copyright (C) 2005   Barrett Technology <support@barrett.com>
 *
 *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *  16 Dec 2004 - BZ, SC, TH
 *    Initial port to linux + RTAI
 *                                                                      *
 *======================================================================*/



/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btparser.h"

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
int parseFile(char *fn);
int parseGetVal(int type, char *str, void *loc);

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
char 	hdr[255];
FILE	*outFile;

/*==============================*
 * Functions                    *
 *==============================*/
/* Strip everything after a hash mark, unless hash is preceeded by an escape char ('\') */
void stripComments(char *str)
{
    char *c;
    
    if((c = strchr(str, '#')) != NULL)
    {
    	if(c != str && *(c-1) == '\\') // Special case
    		stripComments(c+1);
    	else
    		*c = '\0';
    }
}

/* Get the nested key value and update the header */
void nestLine(char *line)
{
    char *h;
    
    h = hdr + strlen(hdr);
    
    while(*line == ' ' || *line == '\t') ++line; // Skip initial whitespace
    while(*line != ' ' && *line != '\t' && *line != '{') *h++ = *line++; // Copy the key value
    
    *h++ = '.'; // Add a dot to the hdr
    *h = '\0'; // Terminate the hdr
}

/* Extract the key */
void getKey(char *key, char *line)
{
	char *k = key;
    
    while(*line == ' ' || *line == '\t') ++line; // Skip initial whitespace
    while(*line != ' ' && *line != '\t' && *line != '=') *k++ = *line++; // Copy the key value
    *k = '\0'; // Terminate the key
}

/* Write out the assignment statement */
void assignLine(char *line)
{
    char key[255];
    char *val;
    char *k;
    
    getKey(key, line);
	line = strchr(line, '=') + 1; // Get on the right side of =
	
	if((val = strpbrk(line, "<\0x22")) != NULL) // String or Vector
		fprintf(outFile, "%s%s = %s\n", hdr, key, val);
	else // Array
	{
		do{
			val = line;
			k = strchr(line, ',');
			if(k != NULL) 
			{
				*k = '\0';
				line = k+1;
			}
			fprintf(outFile, "%s%s = %s\n", hdr, key, val);
		}while(k);
	}
}

/* Update the header when exiting a nested statement */
void killLine(char *line)
{
    char *h;
    h = hdr + strlen(hdr) - 2;
    while(h > hdr && *(h-1) != '.') --h; // Go back to the prev dot or start
    *h = '\0'; // Terminate the hdr   
}

/** Create a value-lookup file from a structured configuration file */
int parseFile(char *fn)
{
    FILE    *inFile;
    char    line[255], srch[255], key1[255], key2[255];
    int		addBrace, index;
    long	stepPos, srchPos;
    
    if((outFile=fopen("temp.dat","w"))==NULL){
    	return(1);
    }
    	
    if((inFile=fopen(fn,"r"))==NULL)
    {
        fprintf(outFile, "Cannot open %s for reading\n", fn);
        fclose(outFile);
        return(1);
    }
    
    // Pass 1 = Structure -> Flat File
    hdr[0] = '\0';
    while(1)
    {
    	if(fgets(line, 255, inFile) == NULL) break;
    	line[strlen(line)-1] = '\0'; 	// Overwrite newline with termination
        stripComments(line); 			// Strip the comments
        //fprintf(outFile, "%s\n", line);
        if(strchr(line, '{') != NULL) nestLine(line); // NEST
        else if(strchr(line, '=') != NULL) assignLine(line); // ASGN
        else if(strchr(line, '}') != NULL) killLine(line); // KILL
    }
    fclose(outFile);
    fclose(inFile);
    
    // Pass 2 = Add braces to non-brace duplicates, apply indices
    inFile = fopen("temp.dat", "r+");
    outFile = fopen("config.dat","w");
    while(1)
    {
    	index = 0;
    	if(fgets(line, 255, inFile) == NULL) break;
    	line[strlen(line)-1] = '\0'; 	// Overwrite newline with termination
    	stepPos = ftell(inFile);
    	getKey(key1, line);
    	if(key1[0] == '.') continue; // Already processed this line
    	addBrace = (strchr(key1, '[') == NULL); // If dup found, add brace?
    	while(1)
    	{
    		srchPos = ftell(inFile);
    		if(fgets(srch, 255, inFile) == NULL) break;
    		srch[strlen(srch)-1] = '\0'; 	// Overwrite newline with termination
    		getKey(key2, srch);
    		if(!strcmp(key1,key2)) // If the keys are identical
    		{
    			if(!index) // First match?
    			{
    				if(addBrace) // Append brace to end of key
    					fprintf(outFile, "%s[%d] = %s\n", key2, index, strchr(line,'=')+1);
    				else // Use existing brace in key
    				{
    					*(strchr(key2,']')) = '\0'; // Overwrite brace
    					fprintf(outFile, "%s%d]%s = %s\n", key2, index, key2+strlen(key2)+1, strchr(line,'=')+1); 
    					key2[strlen(key2)] = ']'; // Replace brace
    				}
    				++index;
    			}
    			if(addBrace) // Append brace to end of key
    				fprintf(outFile, "%s[%d] = %s\n", key2, index, strchr(srch,'=')+1);
				else // Use existing brace in key
				{
					*(strchr(key2,']')) = '\0'; // Overwrite brace
					fprintf(outFile, "%s%d]%s = %s\n", key2, index, key2+strlen(key2)+1, strchr(srch,'=')+1); 
				}
				++index;
				
				// Mark the line as processed
				fseek(inFile, srchPos, SEEK_SET);
				fputc('.', inFile);
				fseek(inFile, srchPos + strlen(srch) + 1, SEEK_SET);
    		}
    	}
    	if(!index) fprintf(outFile, "%s\n", line); 	// No dups found, output line
    	fseek(inFile, stepPos, SEEK_SET); 			// Return to present step location
    }
    fclose(outFile);
    fclose(inFile);
    
    return(0);
}

/** Look up the value of a configuration key. parseFile() must be called prior to this function. */
int parseGetVal(int type, char *find, void *loc)
{
	int intVal;
	long longVal;
	double doubleVal;
	char key[255];
	char str[255];
	char *val = NULL, *s;
	FILE *inFile;
	
	inFile = fopen("config.dat","r");
	while(1)
    {
    	if(fgets(str, 255, inFile) == NULL) break;
    	getKey(key, str);
    	if(!strcmp(key,find)) // Keys match
    	{
    		val = strchr(str,'=') + 1; // Get right of =
    		break;
    	}
    }
    fclose(inFile);
	if(!val) return -1; // If key not found, return err
	switch(type){
		case INT:
			intVal = atoi(val); // Convert
			*(int*)loc = intVal; // Assign
		break;
		case LONG:
			longVal = atol(val); // Convert
			*(long*)loc = longVal; // Assign
		break;
		case DOUBLE:
			doubleVal = strtod(val,NULL); // Convert
			*(double*)loc = doubleVal; // Assign
		break;
		case STRING:
			val = strchr(val,0x22)+1; // Move beyond first dbl quote
			s = (char*)loc;
			intVal = 0;
			while(1)
			{
				if(*val == '\\') // If escape, copy next char
					val++;
				else if(*val == 0x22) // If dbl quote, quit
					break;
					
				*s++ = *val++; // Copy the string
			}
			*s = '\0'; // Terminate the string
		break;
		case VECTOR:
		    strto_vn((vect_n *)loc, val, "<>")
		break;
		default:
			return(1);
		break;
	}
	return(0);
}

