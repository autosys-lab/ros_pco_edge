//-----------------------------------------------------------------//
// Name        | file12.cpp                  | Type: (*) source    //
//-------------------------------------------|       ( ) header    //
// Project     | pco.camera                  |       ( ) others    //
//-----------------------------------------------------------------//
// Platform    | Linux, Windows                                    //
//-----------------------------------------------------------------//
// Environment |                                                   //
//             |                                                   //
//-----------------------------------------------------------------//
// Purpose     | library for b16 and tif files                     //
//-----------------------------------------------------------------//
// Author      | MBL, PCO AG                                       //
//-----------------------------------------------------------------//
// Revision    | rev. 1.04 rel. 0.00                               //
//-----------------------------------------------------------------//
// Notes       | must be linked together with libpcolog            //
//             |                                                   //
//             |                                                   //
//-----------------------------------------------------------------//
// (c) 2012 PCO AG * Donaupark 11 *                                //
// D-93309      Kelheim / Germany * Phone: +49 (0)9441 / 2005-0 *  //
// Fax: +49 (0)9441 / 2005-20 * Email: info@pco.de                 //
//-----------------------------------------------------------------//


#ifdef LINUX
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>

#include <unistd.h>

#include <pco_common/pco_include/defs.h>
#include <pco_common/pco_include/PCO_err.h>

#define FD_O_BIN 0

#endif
#ifdef WIN32

#include "stdafx.h"

#define _CRT_SECURE_NO_WARNINGS

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <io.h>
#include <fcntl.h>
#include "C:\PCO_Include\srccommon\pco\pco_err.h"

#define open _open
#define read _read
#define write _write
#define close _close
#define lseek _lseek

#define FD_O_BIN O_BINARY

#endif


#include <pco_common/pco_include/file12.h>


#define FPVERS "1.30"
#define FPVER  130

#define FILEISOK        1
#define FILEISMACFORMAT 2

#include <pco_common/pco_include/Cpco_log.h>

#ifdef CLASSLOG
CPco_Log* file12log=NULL;
#else
void* file12log=NULL;
#endif

FILE* file12log_out=NULL;

char pcotiff_text[70]="";


static void writelog(DWORD lev,const char *str,...)
{
  if((file12log==NULL)&&(file12log_out==NULL))
   return;

  char *txt;
  txt=(char*)malloc(1000);
  if(txt)
  {
   va_list arg;
   va_start(arg,str);
   vsprintf(txt,str,arg);

#ifdef CLASSLOG
   if(file12log)
    file12log->writelog(lev,txt);
#endif

   if(file12log_out)
    fprintf(file12log_out,"%s\n",txt);

   va_end(arg);
   free(txt);
  }
  return;
}



extern "C" int get_b16_fileparams(char *filename,int *width,int *height,int *colormode)
{
  unsigned char *cptr;
  unsigned char *c1;
  unsigned int *b1;
  int hfread;
  int e;

  cptr=(unsigned char *)malloc(200);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"get_b16_fileparams() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfread = open(filename,O_RDONLY|FD_O_BIN);
  if(hfread == -1)
  {
   free(cptr);
   writelog(ERROR_M,"get_b16_fileparams() Open file %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr;
  e=read(hfread,cptr,200);

  if(e<200)
  {
   close(hfread);
   free(cptr);
   writelog(ERROR_M,"get_b16_fileparams() read header from %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr;
  if((*c1 != 'P')||(*(c1+1) != 'C')||(*(c1+2) != 'O')||(*(c1+3) != '-'))
  if(e<200)
  {
   close(hfread);
   free(cptr);
   writelog(ERROR_M,"get_b16_fileparams() Missing b16 intro 'PCO-' in %s",filename);
   return PCO_ERROR_NOFILE;
  }

/* read FILEHEADER width and height
*/
  c1=cptr+12;                          /*Width*/
  b1=(unsigned int*)c1;
  *width   = *b1;

  c1=cptr+16;                          /*Height*/
  b1=(unsigned int*)c1;
  *height  = *b1;

  c1=cptr+20;

  if(*c1!='C')
  {
   switch(*b1)
   {
      case 0:
      case 1:
       *colormode  = *b1;
       break;

      case 4294967295:
       b1++;
       *colormode  = *b1;
       break;

      default:
       *colormode=0;
       break;
   }
  }
  else
  {
    c1=cptr+35;
    if(*c1=='5')
     *colormode=1;
    else
     *colormode=0;
  }

  close(hfread);
  free(cptr);

  writelog(INFO_M,"get_b16_fileparams() %s done",filename);

  return PCO_NOERROR;
}

int GET_B16_FILEPARAMS(char *filename,int *width,int *height,int *colormode)
{
 return get_b16_fileparams(filename,width,height,colormode);
}


typedef struct _LUTVAL
{
  int bwmin;
  int bwmax;
  int bwlin;

  int rmin;
  int rmax;
  int gmin;
  int gmax;
  int bmin;
  int bmax;
  int clin;
}LUTVAL;


int get_b16_lutset(char *filename,LUTVAL *lut)
{
  unsigned char *cptr;
  unsigned char *c1;
  unsigned int *b1;
  int hfread;
  int e;

  cptr=(unsigned char *)malloc(400);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"get_b16_lutset() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfread = open(filename,O_RDONLY|FD_O_BIN);
  if(hfread == -1)
  {
   free(cptr);
   writelog(ERROR_M,"get_b16_lutset() Open file %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr;
  e=read(hfread,cptr,400);

  if(e<400)
  {
   close(hfread);
   free(cptr);
   writelog(ERROR_M,"get_b16_lutset() read header from %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr;
  if((*c1 != 'P')||(*(c1+1) != 'C')||(*(c1+2) != 'O')||(*(c1+3) != '-'))
  {
   close(hfread);
   free(cptr);
   writelog(ERROR_M,"get_b16_lutset() Missing b16 intro 'PCO-' in %s",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr+20;                          /* colormode */
  b1=(unsigned int*)c1;
  if(*b1!=0xFFFFFFFF)
   return PCO_NOERROR;
  else
   {
    b1++;
    b1++;   //colormode

    lut->bwmin=*b1++;
    lut->bwmax=*b1++;
    lut->bwlin=*b1++;

    lut->rmin=*b1++;
    lut->rmax=*b1++;
    lut->gmin=*b1++;
    lut->gmax=*b1++;
    lut->bmin=*b1++;
    lut->bmax=*b1++;
    lut->clin=*b1;

/*
    err=setdialogbw(*b1   ,*(b1+1),*(b1+2));
//              bwmin,bwmax ,bwlin

    if(err==NOERR)
     err=setdialogcol(*(b1+3),*(b1+4),*(b1+5),*(b1+6),*(b1+7),*(b1+8),*(b1+9));
//                        rmin  ,rmax  ,gmin  ,gmax  ,bmin  ,bmax  ,collin
*/
   }

  close(hfread);
  free(cptr);

  return PCO_NOERROR;
}

extern "C" int read_b16(char *filename, void *buf)
{
  unsigned char *cptr;
  unsigned char *c1;
  unsigned int *b1;
  int hfread;
  int e,z;
  int fsize;
  int headerl;
  int err;

  cptr=(unsigned char *)malloc(200);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"read_b16() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfread = open(filename,O_RDONLY|FD_O_BIN);
  if(hfread == -1)
  {
   free(cptr);
   writelog(ERROR_M,"read_b16() Open file %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr;
  e=read(hfread,cptr,200);

  if(e<200)
  {
   close(hfread);
   free(cptr);
   writelog(ERROR_M,"read_b16() File read header from %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  c1=cptr;
  if((*c1 != 'P')||(*(c1+1) != 'C')||(*(c1+2) != 'O')||(*(c1+3) != '-'))
  {
   close(hfread);
   free(cptr);
   writelog(ERROR_M,"read_b16() Missing b16 intro 'PCO-' in %s",filename);
   return PCO_ERROR_NOFILE;
  }

/*read FILEHEADER*/
  c1=cptr+4;                  /* filesize */
  b1=(unsigned int*)c1;
  fsize=*b1;

  c1=cptr+8;                  /* headerlength */
  b1=(unsigned int*)c1;
  headerl =*b1;

  z=fsize-headerl;

  err=PCO_NOERROR;
  
/* read data */
  lseek(hfread,headerl,SEEK_SET);

  e=read(hfread,buf,z);

  if(e<z)
  {
   writelog(ERROR_M,"read_b16() read %d bytes from %s failed",z,filename);
   err= PCO_ERROR_NOFILE;
  }

  close(hfread);
  free(cptr);

  writelog(INFO_M,"read_b16() %s done",filename);
  return err;
}

extern "C" int store_b16(char *filename,int width,int height,int colormode,void *buf)
{
  unsigned char *cptr;
  unsigned char *c1;
  unsigned int *b1;
  int hfstore;
  int e,z;
  int headerl;

//  printf("store %s w%d h%d size %d buf%p\n",filename,width,height,width*height*2,buf);

  cptr=(unsigned char *)malloc(2000);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"store_b16() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfstore = open(filename,O_CREAT|O_WRONLY|O_TRUNC|FD_O_BIN,0666);//S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH);
  if(hfstore == -1)
  {
   writelog(ERROR_M,"store_b16() create file %s failed",filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  headerl = 128;

  c1=cptr;
  *c1++ = 'P';           //Begin PCO-Header PCO-
  *c1++ = 'C';
  *c1++ = 'O';
  *c1++ = '-';

  b1 = (unsigned int*)c1;

/* Daten fuer Header */
  *b1++ = (width*height*2)+headerl;
  *b1++ = headerl;
  *b1++ = width;
  *b1++ = height;
  *b1++ = 0;

  *b1++ = colormode;
/*
  getdialogbw(b1   ,(b1+1),(b1+2));
//              bwmin,bwmax ,bwlin
  getdialogcol((b1+3),(b1+4),(b1+5),(b1+6),(b1+7),(b1+8),(b1+9));
//               rmin  ,rmax  ,gmin  ,gmax  ,bmin  ,bmax  ,collin

  b1+=10;
*/
/* Fill Header */
  c1=(unsigned char *)b1;
  for(;c1<cptr+128;)
   *c1++=0;

  z=headerl;
  e=write(hfstore,(void *)cptr,z);
  if(e==-1)
  {
   writelog(ERROR_M,"store_b16() write header to %s failed",filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  z=width*height*2;

  e=write(hfstore,(void *)buf,z);
  if(e == -1)
  {
   writelog(ERROR_M,"store_b16() write %d bytes to %s failed",z,filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }


  close(hfstore);
  free(cptr);

  writelog(INFO_M,"store_b16() %s done",filename);
  return PCO_NOERROR;
}


extern "C" int store_tiff_v(char *filename,int width,int height,
               int colormode,void *bufadr,char *apptext)
{
  unsigned short *cptr;
  unsigned short *c1;
  unsigned int *b1;
  int hfstore;
  int e,z,x;
  int headerl;
  int slen,txtlen;
  char *ch;


  cptr=(unsigned short *)malloc(65536);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"store_tiff() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfstore = open(filename,O_CREAT|O_WRONLY|O_TRUNC|FD_O_BIN,0666);//S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH);
  if(hfstore == -1)
  {
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  slen=(int)strlen(apptext);
  txtlen=slen+1;
  txtlen=(txtlen/16)*16+16;

  c1=cptr;
  *c1++ = 0x4949;           /* Begin TIFF-Header II */
  *c1++ = 0x002A;

  *c1++ = 0x0010;           /* Pointer to IFD  */
  *c1++ = 0;


  *c1++ = 0;
  *c1++ = 0;
  *c1 = 0;

/* create  IFD */
  c1=cptr+8;

  *c1++ = 0x000F;             /* Entry Count */

  *c1++ = 0x00FE;             /* NewSubfileType */
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x00000000;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0100;             /* ImageWidth */
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = width;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0101;             /* ImageHeight */
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = height;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0102;             /* BitsPerPixel */
  *c1++ = 0x0003;             /* SHORT */
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0010;             /* 16 */
  *c1++ = 0x0000;

  *c1++ = 0x0103;             /* Compression */
  *c1++ = 0x0003;             /* SHORT */
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;             /* 1 */
  *c1++ = 0x0000;

  *c1++ = 0x0106;             /* PhotometricInterpretation */
  *c1++ = 0x0003;             /* SHORT */
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;             /* 1 */
  *c1++ = 0x0000;


  *c1++ = 0x0111;             /* StripOffset */
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = height;             /* 1 Zeilen pro */
  *b1++ = 0x00E0;              /* pointer */
  c1 = (unsigned short *)b1;

  *c1++ = 0x0115;             /* SamplePerPixel */
  *c1++ = 0x0003;             /* SHORT */
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;             /* 1 */
  *c1++ = 0x0000;

  *c1++ = 0x0116;             /* RowsPerStrip */
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x00000001;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0117;              /* StripByteCounts */
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = height;
  *b1++ = 0x0E0+height*4;      /* pointer */
  c1 = (unsigned short *)b1;

  *c1++ = 0x011A;              /* X-Resolution */
  *c1++ = 0x0005;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x0E0+height*8;      /* pointer */
  c1 = (unsigned short *)b1;

  *c1++ = 0x011B;              /* Y-Resolution */
  *c1++ = 0x0005;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x0E0+height*8+8;    /* pointer */
  c1 = (unsigned short *)b1;

  *c1++ = 0x011C;              /* PlanarConfiguration */
  *c1++ = 0x0003;              /* SHORT */
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;              /* 1 */
  *c1++ = 0x0000;

  *c1++ = 0x0128;              /* ResolutionUnit */
  *c1++ = 0x0003;              /* SHORT */
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;              /* 1 */
  *c1++ = 0x0000;

  *c1++ = 0x0131;              /* Software */
  *c1++ = 0x0002;
  b1 = (unsigned int *)c1;
  *b1++ = slen+1;
  *b1++ = 0x0E0+height*8+16;   /* pointer */


//fill with 0 until strips at 0xE0
  c1 = (unsigned short *)b1;
  for(;c1<cptr+0xE0/2;)
   *c1++ = 0;                   /* ende */

/* beginn der stripadressen vom Fileanfang */
  z=0x0E0+height*8+16+txtlen;     /*  textlength */

  c1=cptr+0x70;                /* 0x70=0xE0/2 */
  b1 = (unsigned int *)c1;
  for(x=0;x<height;x++)
   *b1++=z+x*width*2;

  for(x=0;x<height;x++)
   *b1++=width*2;

  *b1++=0x00000004;
  *b1++=width;
  *b1++=0x00000004;
  *b1++=height;

  strcpy((char*)b1,apptext);
  ch=(char*)b1;
  ch+=slen;
  for(;ch<(char*)cptr+z;)
   *ch++=0;

  headerl = (int)(ch-(char*)cptr);

  e=write(hfstore,(void *)cptr,headerl);
  if(e== -1)
  {
   writelog(ERROR_M,"store_tiff() write header to %s failed",filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  z=width*height*2;
  e=write(hfstore,(void *)bufadr,z);
  if(e==-1)
  {
   writelog(ERROR_M,"store_tiff() write %d bytes to %s failed",z,filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  close(hfstore);
  free(cptr);
  writelog(INFO_M,"store_tiff() %s done",filename);

  return PCO_NOERROR;
}

int store_tiff(char *filename,int width,int height,int colormode,void *bufadr)
{
  if(strlen(pcotiff_text)==0)
  {
   int x;
   sprintf(pcotiff_text,"PCO File_R/W-Library %s Copyright (C)2012 PCO ",FPVERS);
   if(colormode==0)
    strcat(pcotiff_text,"InputImage: B/W  ");
   else
    strcat(pcotiff_text,"InputImage: COLOR");
   for(x=(int)strlen(pcotiff_text);x<70-1;x++)
    pcotiff_text[x]=0x20;
   pcotiff_text[x]=0;
  }
  return store_tiff_v(filename,width,height,colormode,bufadr,pcotiff_text);
}


extern "C" int store_tif(char *filename,int width,int height,int colormode,void *bufadr)
{
  return store_tiff(filename,width,height,colormode,bufadr);
}


static void swapbytes(int W,int H,unsigned short* outbuf,unsigned short* inbuf)
{
  int x=0,y=0;
  for (y=0;y<H;y++)
  {
   for (x=0;x<W;x++)
   {
    *outbuf=((*inbuf&0xFF)<<8) + ((*inbuf&0xFF00)>>8);
    outbuf++;
    inbuf++;
   }
  }
}



/********************************************************************************************************************
*                                            FITS-Datei schreiben                                                   *
*********************************************************************************************************************/

#define FITSBLOCK 2880

extern "C" int store_fits_exp(char *filename,int W, int H, void *img_data,int _exp_time_ms)
{
  int z=0,e=0,i=0;
  char buffer[100];
  FILE *fitsfile;
  char *headerline;
  unsigned short *imgbuf;

  fitsfile=fopen(filename,"w+");
  if (fitsfile==NULL)
  {
   writelog(ERROR_M,"store_fits_exp() create file %s failed",filename);
   remove(filename);
   return PCO_ERROR_NOFILE;
  }

  imgbuf=(unsigned short *)malloc(W*H*sizeof(unsigned short));
  if(imgbuf==NULL)
  {
   fclose(fitsfile);
   remove(filename);
   writelog(ERROR_M,"store_fits_exp() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  headerline =(char *)malloc(2881);
  if(headerline==NULL)
  {
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   writelog(ERROR_M,"store_fits_exp() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  swapbytes(W,H,imgbuf,(unsigned short*)img_data);

  memset(headerline,0,2881);//to avoid confusion

//now we can insert the keywords
//don't forget to provide a line length of 80 characters
  sprintf(buffer,"SIMPLE  =                    T                                                  ");
  strcat(headerline,buffer);
  sprintf(buffer,"BITPIX  =                   16                                                  ");
  strcat(headerline,buffer);
  sprintf(buffer,"NAXIS   =                    2                                                  ");
  strcat(headerline,buffer);
  sprintf(buffer,"NAXIS1  =                 %4d                                                  ",W);
  strcat(headerline,buffer);
  sprintf(buffer,"NAXIS2  =                 %4d                                                  ",H);
  strcat(headerline,buffer);
 //end of header:
  sprintf(buffer,"END                                                                             ");
  strcat(headerline,buffer);
  for (i=(int)strlen(headerline);i<FITSBLOCK;i++)
  {
   strcat(headerline," ");
  }

//write the header to fitsfile
  e=0;
  e=fputs(headerline,fitsfile);
  if (e==EOF)
  {
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   free(headerline);
   writelog(ERROR_M,"store_fits_exp() write header to %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

//write the data to 'fitsfile'
  e=(int)fwrite(imgbuf,sizeof(unsigned short),W*H,fitsfile);
  if (e==0)
  {
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   free(headerline);
   writelog(ERROR_M,"store_fits_exp() write %d bytes to %s failed",z,filename);
   return PCO_ERROR_NOFILE;
  }

//we have to fill up the last 2880 byte block with blanks
  memset(headerline,' ',2881);

  z=W*H;
  z=(H+4)*W*2;
  z%=FITSBLOCK;
  z=FITSBLOCK-z;

  e=(int)fwrite(headerline,sizeof(char),z,fitsfile);
  if (e==0)
  {
   writelog(ERROR_M,"store_fits_exp() write %d bytes to %s failed",z,filename);
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   free(headerline);
   return PCO_ERROR_NOFILE;
  }

  fclose(fitsfile);
  free(imgbuf);
  free(headerline);

  writelog(INFO_M,"store_fits_exp() %s done",filename);

  return PCO_NOERROR;
}

extern "C" int store_fits(char *filename,int W, int H,int colormode, void *img_data)
{
  int z=0,e=0,i=0;
  char buffer[100];
  FILE *fitsfile;
  char *headerline;
  unsigned short *imgbuf;


  fitsfile=fopen(filename,"w+");
  if (fitsfile==NULL)
  {
   writelog(ERROR_M,"store_fits() create file %s failed",filename);
   remove(filename);
   return PCO_ERROR_NOFILE;
  }

  imgbuf=(unsigned short *)malloc(W*H*sizeof(unsigned short));
  if(imgbuf==NULL)
  {
   fclose(fitsfile);
   remove(filename);
   writelog(ERROR_M,"store_fits() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  headerline =(char *)malloc(2881);
  if(headerline==NULL)
  {
   fclose(fitsfile);
   remove(filename);
   free(imgbuf);
   writelog(ERROR_M,"store_fits() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  swapbytes(W,H,imgbuf,(unsigned short*)img_data);

//  memcpy(imgbuf,img_data,W*H*sizeof(unsigned short));
//  chbytes(W,H,(unsigned short*)imgbuf);

  memset(headerline,0,2881);//to avoid confusion
//now we can insert the keywords
//don't forget to provide a line length of 80 characters
  sprintf(buffer,"SIMPLE  =                    T                                                  ");
  strcat(headerline,buffer);
  sprintf(buffer,"BITPIX  =                   16                                                  ");
  strcat(headerline,buffer);
  sprintf(buffer,"NAXIS   =                    2                                                  ");
  strcat(headerline,buffer);
  sprintf(buffer,"NAXIS1  =                 %4d                                                  ",W);
  strcat(headerline,buffer);
  sprintf(buffer,"NAXIS2  =                 %4d                                                  ",H);
  strcat(headerline,buffer);
 //end of header:
  sprintf(buffer,"END                                                                             ");
  strcat(headerline,buffer);
  for (i=(int)strlen(headerline);i<FITSBLOCK;i++)
  {
  strcat(headerline," ");
  }

  //write the header to fitsfile
  e=0;
  e=fputs(headerline,fitsfile);
  if(e==EOF)
  {
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   free(headerline);
   writelog(ERROR_M,"store_fits() write header to %s failed",filename);
   return PCO_ERROR_NOFILE;
  }

  e=0;
  //write the data to 'fitsfile'
  e=(int)fwrite(imgbuf,sizeof(unsigned short),W*H,fitsfile);
  if (e==0)
  {
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   free(headerline);
   writelog(ERROR_M,"store_fits() write %d bytes to %s failed",z,filename);
   return PCO_ERROR_NOFILE;
  }

  //we have to fill up the last 2880 byte block with blanks
  memset(headerline,' ',2881);

  z=W*H;
  z=(H+4)*W*2;
  z%=FITSBLOCK;
  z=FITSBLOCK-z;

  e=(int)fwrite(headerline,sizeof(char),z,fitsfile);
  if (e==0)
  {
   fclose(fitsfile);
   remove(filename);

   free(imgbuf);
   free(headerline);
   writelog(ERROR_M,"store_fits() write %d bytes to %s failed",z,filename);
   return PCO_ERROR_NOFILE;
  }

  fclose(fitsfile);
  free(imgbuf);
  free(headerline);

  writelog(INFO_M,"store_fits() %s done",filename);
  return PCO_NOERROR;
}




extern "C" int store_bmp(char *filename,int width,int height,int colormode,void *buf)
{
  unsigned char *cptr;
  unsigned char *c1;
  unsigned int *b1;
  int hfstore;
  int e,z,x;
  int headerl;


  if(colormode>2)
  {
   writelog(ERROR_M,"store_bmp() wrong colormode value");
   return PCO_ERROR_WRONGVALUE;
  }

  cptr=(unsigned char *)malloc(2000L);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"store_bmp() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfstore = open(filename,O_CREAT|O_WRONLY|O_TRUNC|FD_O_BIN,0666);//S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH); |O_SYNC
  if(hfstore == -1)
  {
   writelog(ERROR_M,"store_bmp() create file %s failed",filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

//calculate size of LUT-Table
  if(colormode==0)
   x=256;
  else
   x=0;

  headerl=x*4 + 0x28 + 0x0E;

  c1=cptr;
  *c1++ = 'B';           //Begin BMP-Header BM
  *c1++ = 'M';

  b1 = (unsigned int *)c1;

// Data for header
  if(colormode==0)
   *b1++ = width*height+headerl;
  if(colormode==1)
   *b1++ = width*height*3+headerl;
  if(colormode==2)
   *b1++ = width*height*4+headerl;
  *b1++ = 0L;
  *b1++ = headerl;
  *b1++ = 0x028;               //sizeof(BITMAPAPINFOHEADER);
  *b1++ = width;
  *b1++ = -height;
  if(colormode==0)
   *b1++ = 1+(8<<16);
  if(colormode==1)
   *b1++ = 1+(24<<16);
  if(colormode==2)
   *b1++ = 1+(32<<16);
  *b1++ = 0;                  // BI_RGB;
  *b1++ = width*height;
  *b1++ = 0;
  *b1++ = 0;
  *b1++ = 0;
  *b1++ = 0;

  c1 = (unsigned char *)b1;

//write  LUT-Table
  if(colormode==0)
  {
   for(z=0;z<x;z++)
   {
    *c1++ = (unsigned char)z;
    *c1++ = (unsigned char)z;
    *c1++ = (unsigned char)z;
    *c1++ = 0;
   }
  }


  z=headerl;
  e=write(hfstore,(void *)cptr,z);
  if(e==-1)
  {
   writelog(ERROR_M,"store_bmp() write header to %s failed",filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

//write data to file
  switch(colormode)
   {
    case 0:
     z=width*height;
     break;

    case 1:
     z=width*height*3;
     break;

    case 2:
     z=width*height*4;
     break;

    default:
     break;
   }

  e=write(hfstore,(void *)buf,z);
  if(e==-1)
  {
   writelog(ERROR_M,"store_bmp() write %d bytes to %s failed",z,filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  close(hfstore);
  free(cptr);

  writelog(INFO_M,"store_bmp() %s done",filename);
  return PCO_NOERROR;
}


extern "C"  int store_tif8bw_v(char *filename,int width,int height,int colormode,void *bufadr,char* apptext)
{
  unsigned short *cptr;
  unsigned short *c1;
  unsigned int *b1;
  int hfstore;
  int e,z,x;
  int headerl;
  int slen,txtlen;
  char *ch;

  cptr=(unsigned short *)malloc(65536);
  if(cptr==NULL)
  {
   writelog(ERROR_M,"store_tif8bw() memory allocation failed");
   return PCO_ERROR_NOMEMORY;
  }

  hfstore = open(filename,O_CREAT|O_WRONLY|O_TRUNC|FD_O_BIN,0666);//S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH);
  if(hfstore == -1)
  {
   writelog(ERROR_M,"store_btif8bw() create file %s failed",filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  slen=(int)strlen(apptext);
  txtlen=slen+1;
  txtlen=(txtlen/16)*16+16;

  c1=cptr;
  *c1++ = 0x4949;           //Begin TIFF-Header II
  *c1++ = 0x002A;

  *c1++ = 0x0010;            //Pointer to IFD
  *c1++ = 0;


  *c1++ = 0;
  *c1++ = 0;
  *c1 = 0;

// create  IFD
  c1=cptr+8;                 //8 Words

  *c1++ = 0x000F;             //Entry Count

  *c1++ = 0x00FE;             //NewSubfileType
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x00000000;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0100;             //ImageWidth
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = width;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0101;             //ImageHeight
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = height;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0102;             //BitsPerPixel
  *c1++ = 0x0003;             //SHORT
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0008;             //8
  *c1++ = 0x0000;

  *c1++ = 0x0103;             //Compression
  *c1++ = 0x0003;             //SHORT
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;             //1
  *c1++ = 0x0000;

  *c1++ = 0x0106;             //PhotometricInterpretation
  *c1++ = 0x0003;             //SHORT
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;             //1 min is black
  *c1++ = 0x0000;

  *c1++ = 0x0111;             //StripOffset
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = height;              //1 Zeile pro
  *b1++ = 0x0E0;               //pointer
  c1 = (unsigned short *)b1;

  *c1++ = 0x0115;             //SamplePerPixel
  *c1++ = 0x0003;             //SHORT
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;             //1
  *c1++ = 0x0000;

  *c1++ = 0x0116;             //RowsPerStrip
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x00000001;
  c1 = (unsigned short *)b1;

  *c1++ = 0x0117;              //StripByteCounts
  *c1++ = 0x0004;
  b1 = (unsigned int *)c1;
  *b1++ = height;
  *b1++ = 0x0E0+height*4;     //pointer;
  c1 = (unsigned short *)b1;

  *c1++ = 0x011A;              //X-Resolution
  *c1++ = 0x0005;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x0E0+height*8;   //pointer;
  c1 = (unsigned short *)b1;

  *c1++ = 0x011B;              //Y-Resolution
  *c1++ = 0x0005;
  b1 = (unsigned int *)c1;
  *b1++ = 0x00000001;
  *b1++ = 0x0E0+height*8+8; //pointer;
  c1 = (unsigned short *)b1;

  *c1++ = 0x011C;              //PlanarConfiguration
  *c1++ = 0x0003;              //SHORT
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;              //1
  *c1++ = 0x0000;

  *c1++ = 0x0128;              //ResolutionUnit
  *c1++ = 0x0003;              //SHORT
  *c1++ = 0x0001;
  *c1++ = 0x0000;
  *c1++ = 0x0001;              //1
  *c1++ = 0x0000;


  *c1++ = 0x0131;             //Software
  *c1++ = 0x0002;
  b1 = (unsigned int *)c1;
  *b1++ = slen+1;
  *b1++ = 0x0E0+height*8+16; //pointer;

  c1 = (unsigned short *)b1;
  for(;c1<cptr+0xE0/2;)
   *c1++ = 0;                   /* ende */

//beginn der stripadressen 224byte vom Fileanfang
  z=0x0E0+height*8+16+txtlen;     //  txtlength

  c1=cptr+0x70;                //0x70=0xE0/2
  b1 = (unsigned int *)c1;     //write line offsets
  for(x=0;x<height;x++)
   *b1++=z+x*width;

  for(x=0;x<height;x++)        //write line bytes
   *b1++=width;

  *b1++=0x00000004;
  *b1++=width;
  *b1++=0x00000004;
  *b1++=height;

  ch=(char*)b1;
  strcpy(ch,apptext);
  ch+=slen;
  for(;ch<(char*)cptr+z;)
   *ch++=0;

  headerl = (int)(ch-(char*)cptr);

  e=write(hfstore,(void *)cptr,headerl);
  if(e== -1)
  {
   writelog(ERROR_M,"store_tif8bw() write header to %s failed",filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  z=width*height;
  e=write(hfstore,(void *)bufadr,z);
  if(e==-1)
  {
   writelog(ERROR_M,"store_tif8bw() write %d bytes to %s failed",z,filename);
   close(hfstore);
   remove(filename);
   free(cptr);
   return PCO_ERROR_NOFILE;
  }

  close(hfstore);
  free(cptr);

  writelog(INFO_M,"store_tif8bw() %s done",filename);
  return PCO_NOERROR;
}

extern "C" int store_tif8bw(char *filename,int width,int height,int colormode,void *bufadr)
{
  if(strlen(pcotiff_text)==0)
  {
   int x;
   sprintf(pcotiff_text,"PCO File_R/W-Library %s Copyright (C)2012-2017 PCO ",FPVERS);
   if(colormode==0)
    strcat(pcotiff_text,"InputImage: B/W  ");
   else
    strcat(pcotiff_text,"InputImage: COLOR");
   for(x=(int)strlen(pcotiff_text);x<70-1;x++)
    pcotiff_text[x]=0x20;
   pcotiff_text[x]=0;
  }
  return store_tif8bw_v(filename,width,height,colormode,bufadr,pcotiff_text);
}






