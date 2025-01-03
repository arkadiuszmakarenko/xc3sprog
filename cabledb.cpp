/* Programming cable database

Copyright (C) 2011, 2012 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#ifndef CABLEDB
#define CABLEDB "cablelist.txt"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "cabledb.h"
#include "cables.h"
#include "utilities.h"

CableDB::CableDB(const char *cf_name)
{
    cable_t cable;
    char alias[64];
    char cabletype[64];
    char freq_string[64];
    char options[256];
    FILE *fp;
    
    if(!cf_name)
    {
        if(!(cf_name = getenv("CABLEDB")))
            cf_name = CABLEDB;
    }

    fp = fopen(cf_name,"rt");
    if (fp)
    {
        cablename = cf_name;
        while(!feof(fp))
        {
          int i;
	  char buffer[256];
	  fgets(buffer,256,fp);  // Get next line from file
          i = strlen(buffer);         
          while (i > 0 && isspace(buffer[i-1]))
            i--;
          buffer[i] = 0;
          if(buffer[0] == '#')
              continue;
	  if (sscanf(buffer,"%64s %64s %64s %255[^;]", 
                     alias, cabletype, freq_string, options) == 4)
          {
              if (strchr(freq_string, ':'))
              {
                  fprintf(stderr,"%s has wrong format!\n", cf_name);
                  break;
              }
              else
                  cable.freq = atoi(freq_string);                  
              cable.alias = new char[strlen(alias)+1];
              strcpy(cable.alias,alias);
              cable.cabletype = getCableType(cabletype);
              cable.optstring = new char[strlen(options)+1];
              strcpy(cable.optstring,options);
              cable_db.push_back(cable);
	    }
	}
      fclose(fp);
    }
    else
        /* Read from built-in structure */
    {
        char buffer[512];
        const char *p = cabledb_string;
        
        cablename = "built-in cable list";
        while(*p)
	{
            int i;
            for(i=0; p[i] && (p[i] != ';'); i++)
	    {
                buffer[i] = p[i];
	    }
            p += i;
            while(*p && *p == ';')
                p++;
            while(i>0 && isspace(buffer[i-1]))
                i--;
            buffer[i] = 0;
            if(buffer[0] == '#')
                continue;
	  if (sscanf(buffer,"%64s %64s %d %255[^;]", 
                     alias, cabletype, &cable.freq, options) == 4)
	    {
                cable.alias = new char[strlen(alias)+1];
                strcpy(cable.alias,alias);
                cable.cabletype = getCableType(cabletype);
                cable.optstring = new char[strlen(options)+1];
                strcpy(cable.optstring,options);
                cable_db.push_back(cable);
 	    }
	}
    }
}
CableDB::~CableDB()
{
    unsigned int i;
    
    for(i = 0; i < cable_db.size(); i++)
    {
        if (cable_db[i].alias != 0)
            delete [] cable_db[i].alias;
        if( cable_db[i].optstring != 0)
            delete []  cable_db[i].optstring;
    }
}
/* Return 0 on match*/
int CableDB::getCable(const char *name, struct cable_t *cable)
{
    unsigned int i;

    for(i = 0; i < cable_db.size(); i++)
    {
        if (!(strcasecmp(cable_db[i].alias, name)))
        {
            cable->alias = cable_db[i].alias;
            cable->cabletype = cable_db[i].cabletype;
            cable->optstring = cable_db[i].optstring;
            cable->freq = cable_db[i].freq;
            return 0;
        }
    }
    return 1;
}

CABLES_TYPES CableDB::getCableType(const char *given_name)
{
  if (strcasecmp(given_name, "pp") == 0)
    return CABLE_PP;
  if (strcasecmp(given_name, "ftdi") == 0)
    return CABLE_FTDI;
  if (strcasecmp(given_name, "fx2") == 0)
    return CABLE_FX2;
  if (strcasecmp(given_name, "xpc") == 0)
    return CABLE_XPC;
  if (strcasecmp(given_name, "gpiod_creator") == 0)
    return CABLE_GPIOD_CREATOR;
    if (strcasecmp(given_name, "gpiod_voice") == 0)
    return CABLE_GPIOD_VOICE;

  return CABLE_UNKNOWN;
}

const char *CableDB::getCableName(const CABLES_TYPES type )
{
    switch (type)
    {
    case CABLE_PP: return "pp";
    case CABLE_FTDI: return "ftdi";
    case CABLE_FX2: return  "fx2";
    case CABLE_XPC: return "xpc";
    case CABLE_GPIOD_CREATOR: return "gpiod_creator";
    case CABLE_GPIOD_VOICE: return "gpiod_voice";
    case CABLE_NONE: return "none";
    case CABLE_UNKNOWN: return "unknown";
    }
    return "UNKNOWN";
}

int CableDB::dumpCables(FILE *fp_out)
{
    unsigned int i;

    if (!fp_out)
    {
        fprintf(stderr," No valid file to dump Cablelist\n");
        return 1;
    }
    for(i = 0; i < cable_db.size(); i++)
        fprintf(fp_out,"%-20s%-8s%-10d%-60s\n",
                cable_db[i].alias,
                getCableName(cable_db[i].cabletype),
                cable_db[i].freq,
                cable_db[i].optstring);
    return 0;
}       
