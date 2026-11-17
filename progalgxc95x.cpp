/* XC95 CPLD JTAG programming algorithms

Copyright (C) 2008-2009 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de
(C) Copyright 2001 Nahitafu,Naitou Ryuji

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

#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include "progalgxc95x.h"

const byte ProgAlgXC95X::ISC_NOOP=0xff;
const byte ProgAlgXC95X::ISC_DISABLE=0xf0;
const byte ProgAlgXC95X::ISC_ERASE=0xed;
const byte ProgAlgXC95X::ISC_PROGRAM=0xea;
const byte ProgAlgXC95X::ISC_READ=0xee;
/* ISPEN, as Xilinx iMPACT uses. 0xe9 is ISPENC. */
const byte ProgAlgXC95X::ISC_ENABLE=0xe8;

const byte ProgAlgXC95X::XSC_BLANK_CHECK=0xe5;

const byte ProgAlgXC95X::BYPASS=0xff;

#define deltaT(tvp1, tvp2) (((tvp2)->tv_sec-(tvp1)->tv_sec)*1000000 + \
                              (tvp2)->tv_usec - (tvp1)->tv_usec)

ProgAlgXC95X::ProgAlgXC95X(Jtag &j, int s)
{
  jtag=&j;

  switch (s)
   {
   case  1: DRegLength = 2;
     break;
   case  2: DRegLength = 4;
     break;
   case  4: DRegLength = 8;
     break;
   case 11: DRegLength = 16;
     break;
   default: fprintf(stderr, "Unknown device\n");
   }
}

/* Stay in Run-Test/Idle for at least tck clocks and at least usec
   microseconds. iMPACT's SVF counts its waits in TCK at 1 MHz; a bit-banged
   cable runs at an unknown rate, so meet both the count and the time. */
void ProgAlgXC95X::wait_idle(int tck, unsigned int usec)
{
  struct timeval tv[2];
  long done;

  gettimeofday(tv, NULL);
  jtag->cycleTCK(tck);
  gettimeofday(tv+1, NULL);
  done = deltaT(tv, tv + 1);
  if (done < (long)usec)
    jtag->Usleep(usec - done);
}

void ProgAlgXC95X::flow_enable()
{
  byte data[1];

  jtag->shiftIR(&ISC_ENABLE);
  data[0]=0x05;
  jtag->shiftDR(data,0,6);
  jtag->cycleTCK(1);
}

void ProgAlgXC95X::flow_disable()
{
  jtag->shiftIR(&ISC_DISABLE);
  wait_idle(100, 100);
  jtag->shiftIR(&BYPASS);
  jtag->cycleTCK(1);
}


void ProgAlgXC95X::flow_error_exit()
{
  jtag->shiftIR(&ISC_NOOP);
  jtag->cycleTCK(1);
}

/* The status shifts below send the operation's word again with control
   bits 01, as iMPACT does, and the captured low two bits read 01 when the
   operation finished. Words are 18 bits, least significant byte first. */

/* Returns 0 when the device is blank. */
int ProgAlgXC95X::flow_blank_check()
{
  byte i_data[3]={0x03,0,0};  /* 0x00003 */
  byte s_data[3]={0x01,0,0};  /* 0x00001 */
  byte o_data[3];
  jtag->shiftIR(&XSC_BLANK_CHECK);
  jtag->shiftDR(i_data, 0,18);
  wait_idle(500, 500);
  jtag->shiftDR(s_data,o_data,18);
  if ((o_data[0] & 0x03) != 0x01)
    {
      fprintf(stderr, "Device is not blank (status %02x)\n", o_data[0] & 0x03);
      return 1;
    }
  if(jtag->getVerbose())
    fprintf(stderr, "Device is blank\n");
  return 0;
}

/* Bulk erase. Returns 0 when the status says the erase finished. */
int ProgAlgXC95X::flow_erase()
{
  byte i_data[3] = {0xff, 0xff, 0x03};  /* 0x3ffff: every address bit set */
  byte s_data[3] = {0xfd, 0xff, 0x03};  /* 0x3fffd */
  byte o_data[3];
  jtag->shiftIR(&ISC_ERASE);
  jtag->shiftDR(i_data,0,18);
  wait_idle(200000, 500000);
  jtag->shiftDR(s_data,o_data,18);
  if((o_data[0]& 0x03) != 0x01)
    {
      fprintf(stderr, "Erase failed (status %02x)\n", o_data[0] & 0x03);
      return 1;
    }
  return 0;
}
  
#define MaxSector 108
#define MaxDRegLength 16

int ProgAlgXC95X::flow_array_program(JedecFile &file)
{
  byte preamble[1]= {0x01};
  byte i_data[MaxDRegLength+2];
  byte o_data[MaxDRegLength+3];
  struct timeval tv[2];
  unsigned long Addr=0;
  int bitlen;
  int k, sec,l,m;
  unsigned char data;
  unsigned int idx=0;
     

  gettimeofday(tv, NULL);
  for(sec=0;sec < MaxSector;sec++)
    {
    if(jtag->getVerbose())
      fprintf(stderr, "                            \r"
	      "Programming Sector %3d", sec);
      preamble[0]= 0x01;
      for(l=0;l<3;l++){
	for(m=0;m<5;m++){
	  Addr = sec*0x20 + l*0x08 + m;
	  i_data[DRegLength] = (byte) (Addr &0xff);
	  i_data[DRegLength+1] = (byte) ((Addr>>8) &0xff);
	  if(l*5+m >= 9){
	    bitlen=6;
	  }
	  else{
	    bitlen=8;
	  }
	  for(int j=0;j<DRegLength;j++)
	    {
	      data = 0;
	      for(int i=0;i<bitlen;i++)
		{
		  data |=(file.get_fuse(idx++)<<i);
		}
	      i_data[j] = data;
	    }
	  if ((l == 2) && (m == 4))
	    preamble[0] = 0x03;
	  jtag->Usleep(1000); /*FIXME: IOFTDI Buffer causes abort with high
			        rate on some board otherwise*/
	  jtag->shiftIR(&ISC_PROGRAM);
	  jtag->shiftDR(preamble,0,2,0,false);
	  jtag->shiftDR(i_data,0,(DRegLength+2)*8);
	  if ((l == 2) && (m == 4))
	    {
	      /* Control bits 11 programmed the sector. Read its status as
		 iMPACT does: shift the word again with control bits 00; the
		 two bits captured first read 01 when programming finished. */
	      wait_idle(20000, 50000);
	      preamble[0]= 0x00;
	      for(k=0; k< 32; k++)
		{
		  jtag->shiftDR(preamble, o_data, 2, 0, false);
		  jtag->shiftDR(i_data, 0,(DRegLength+2)*8);
		  if(jtag->getVerbose())
		    {
		      fprintf(stderr, ".");
		      fflush(stderr);
		    }
		  if ((o_data[0] & 0x03) == 0x01)
		    break;
		  wait_idle(20000, 50000);
		}
	      if (k == 32)
		{
		  fprintf(stderr, "\nProgramming failed at sector %d (status %02x)\n",
			  sec, o_data[0] & 0x03);
		  return 1;
		}
	    }
	  else
	    jtag->cycleTCK(1);
	}
      }
    }
  gettimeofday(tv+1, NULL);
  if(jtag->getVerbose())
    fprintf(stderr, "\nProgramming  time %.1f ms\n",
	    (double)deltaT(tv, tv + 1)/1.0e3);
  return 0;
}

void ProgAlgXC95X::flow_array_read(JedecFile &rbfile)
{
  byte preamble[1]= {0x03};
  byte i_data[MaxDRegLength+2];
  byte o_data[MaxDRegLength+2];
  struct timeval tv[2];

  unsigned long Addr=0;
  int bitlen;
  int sec,l,m;
  unsigned char data;
  unsigned int idx=0;

  memset(i_data, 0, sizeof(i_data)); /* only the address is set below */
  gettimeofday(tv, NULL);
  for(sec=0;sec < MaxSector;sec++)
    {
      if(jtag->getVerbose())
	{
	  fprintf(stderr, "\rReading Sector %3d", sec);
	  fflush(stderr);
	}
      for(l=0;l<3;l++){
	for(m=0;m<5;m++){
	  Addr = sec*0x20 + l*0x08 + m;
	  i_data[DRegLength] = (byte) (Addr &0xff);
	  i_data[DRegLength+1] = (byte) ((Addr>>8) &0xff);
	  jtag->shiftIR(&ISC_READ);
	  jtag->shiftDR(preamble,0,2,0,false);
	  jtag->shiftDR(i_data,o_data,(DRegLength+2)*8);
	  jtag->cycleTCK(1);
	  if(sec | l | m )
	    {
	      for(int j=0;j<DRegLength;j++)
		{
		  data = o_data[j];
		  for(int i=0;i<bitlen;i++)
		    {
		      rbfile.set_fuse(idx++, (data & 0x01));
		      data = data >> 1;
		    }
		}
	    }
	  if(l*5+m >= 9){
	    bitlen=6;
	  }
	  else{
	    bitlen=8;
	  }
	}
      }
    }
  /* Now read the security fuses*/
  jtag->shiftIR(&ISC_READ);
  jtag->shiftDR(preamble,0,2,0,false);
  jtag->shiftDR(i_data,o_data,(DRegLength+2)*8);
  for(int j=0;j<DRegLength;j++)
    {
      data = o_data[j];
      for(int i=0;i<bitlen;i++){
	rbfile.set_fuse(idx++, (data & 0x01));
	data = data >> 1;
      }
    }
  
  gettimeofday(tv+1, NULL);
  if(jtag->getVerbose())
    fprintf(stderr, "\nReadback time %.1f ms\n",
	    (double)deltaT(tv, tv + 1)/1.0e3);
}

int ProgAlgXC95X::flow_array_verify(JedecFile &file)
{
  byte preamble[1]= {0x03};
  byte i_data[MaxDRegLength+2];
  byte o_data[MaxDRegLength+2];
  struct timeval tv[2];

  unsigned long Addr=0;
  int bitlen;
  int sec,l,m;
  unsigned char data;
  unsigned int idx=0;

  memset(i_data, 0, sizeof(i_data)); /* only the address is set below */
  gettimeofday(tv, NULL);
  for(sec=0;sec < MaxSector;sec++)
    {
      if(jtag->getVerbose())
	{
	  fprintf(stderr, "\rVerify Sector %3d", sec);
	  fflush(stderr);
	}
      for(l=0;l<3;l++){
	for(m=0;m<5;m++){
	  Addr = sec*0x20 + l*0x08 + m;
	  i_data[DRegLength] = (byte) (Addr &0xff);
	  i_data[DRegLength+1] = (byte) ((Addr>>8) &0xff);
	  jtag->shiftIR(&ISC_READ);
	  jtag->shiftDR(preamble,0,2,0,false);
	  jtag->shiftDR(i_data,o_data,(DRegLength+2)*8);
	  jtag->cycleTCK(1);
	  if(sec | l | m )
	    {
	      for(int j=0;j<DRegLength;j++)
		{
		  data = o_data[j];
		  for(int i=0;i<bitlen;i++)
		    {
		      if ((data& 0x01) != file.get_fuse(idx))
			{
			  fprintf(stderr, "\nMismatch at fuse %6d: %d vs %d\n",
				 idx, data& 0x01, file.get_fuse(idx));
			  return 1;
			}
		      data = data >> 1;
		      idx++;
		    }
		}
	    }
	  if(l*5+m >= 9){
	    bitlen=6;
	  }
	  else{
	    bitlen=8;
	  }
	}
      }
    }
  /* Now read the security fuses*/
  jtag->shiftIR(&ISC_READ);
  jtag->shiftDR(preamble,0,2,0,false);
  jtag->shiftDR(i_data,o_data,(DRegLength+2)*8);
  for(int j=0;j<DRegLength;j++)
    {
      data = o_data[j];
      for(int i=0;i<bitlen;i++){
	if ((data& 0x01) != file.get_fuse(idx++))
	  {
	    idx--;
	    fprintf(stderr, "\nMismatch at security fuse %6d: %d vs %d\n",
		   idx, data& 0x01, file.get_fuse(idx));
	    return 1;
	  }
	data = data >> 1;
      }
    }
  
  gettimeofday(tv+1, NULL);
  if(jtag->getVerbose())
    fprintf(stderr, "\nSuccess! Verify time %.1f ms\n",
	    (double)deltaT(tv, tv + 1)/1.0e3);
  return 0;
}	    

void ProgAlgXC95X::array_read(JedecFile &file)
{
  file.setLength(108*108* DRegLength);
  flow_enable();
  flow_array_read(file);
  flow_disable();
}

/* Returns 0 when the device erased and is blank. */
int ProgAlgXC95X::erase()
{
  int ret;
  flow_enable();
  ret = flow_erase();
  if (ret == 0)
    ret = flow_blank_check();
  if (ret)
    flow_error_exit();
  flow_disable();
  return ret;
}

/* Returns 0 when the device is blank. */
int ProgAlgXC95X::blank_check()
{
  int ret;
  flow_enable();
  ret = flow_blank_check();
  flow_disable();
  return ret;
}

/* Returns 0 when every sector programmed. */
int ProgAlgXC95X::array_program(JedecFile &file)
{
  int ret;
  flow_enable();
  ret = flow_array_program(file);
  if (ret)
    flow_error_exit();
  flow_disable();
  return ret;
}

int ProgAlgXC95X::array_verify(JedecFile &file)
{
  int ret;
  flow_enable();
  ret = flow_array_verify(file);
  flow_disable();
  return ret;
}
