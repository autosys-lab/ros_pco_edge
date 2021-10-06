//-----------------------------------------------------------------//
// Name        | Cpcolog.cpp                 | Type: (*) source    //
//-------------------------------------------|       ( ) header    //
// Project     | pco.camera                  |       ( ) others    //
//-----------------------------------------------------------------//
// Platform    | LINUX                                             //
//-----------------------------------------------------------------//
// Environment |                                                   //
//-----------------------------------------------------------------//
// Purpose     | pco.camera - non functional logging class         //
//-----------------------------------------------------------------//
// Author      | MBL, PCO AG                                       //
//-----------------------------------------------------------------//
// Revision    | rev. 0.01 rel. 0.00                               //
//-----------------------------------------------------------------//
// Notes       | Common functions                                  //
//             |                                                   //
//             |                                                   //
//-----------------------------------------------------------------//
// (c) 2010 PCO AG * Donaupark 11 *                                //
// D-93309      Kelheim / Germany * Phone: +49 (0)9441 / 2005-0 *  //
// Fax: +49 (0)9441 / 2005-20 * Email: info@pco.de                 //
//-----------------------------------------------------------------//


//-----------------------------------------------------------------//
// Revision History:                                               //
//-----------------------------------------------------------------//
// Rev.:     | Date:      | Changed:                               //
// --------- | ---------- | ---------------------------------------//
//  0.01     | 16.06.2010 |  new file                              //
//-----------------------------------------------------------------//
//  0.02     | 11.04.2019 |  write to file or stdout               //
//-----------------------------------------------------------------//
//  0.0x     | xx.xx.200x |                                        //
//-----------------------------------------------------------------//

#include <fcntl.h>      /* open */
#include <unistd.h>     /* exit */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <memory.h>
#include <ctype.h>
#include <time.h>
#include <sched.h>

#include <sys/time.h>
#include <sys/syscall.h>
#include <sys/types.h>

#include <pco_common/pco_include/Cpco_log.h>

//to output logging use either 
//writing to file
//#define OUTPUT_FILE  

//writing to stdout
//#define OUTPUT_CON


const char crlf[3]={0x0d,0x0a,0x00};

CPco_Log::CPco_Log(const char *name)
{

#if defined _DEBUG
  log_bits=0x0003FFFF;
#else
  log_bits=ERROR_M|INIT_M;
#endif

  hflog=-1;
  tzset();

  QueryPerformanceFrequency((LARGE_INTEGER*)&lpFrequency);
  lpPCount1=lpPCount2=0;
  QueryPerformanceCounter((LARGE_INTEGER*)&lpPCount1);

#if defined OUTPUT_FILE
  if(name==NULL)
    sprintf(logname,"pco_log.log");
  else
    strcpy(logname,name);

  hflog=open(logname,O_CREAT|O_WRONLY|O_TRUNC,0666);
  if(hflog!=-1)
  {
    char fname[MAX_PATH+100];
    SYSTEMTIME  st;
    DWORD z;

    GetLocalTime(&st);
    sprintf(fname,"%s logfile started at "
      "%02d.%02d.%04d %02d:%02d:%02d \r\n",logname,st.wDay,st.wMonth,st.wYear,st.wHour,st.wMinute,st.wSecond);
#ifdef VERSTR
    strcat(fname,"PCO Common Lib VERSION: ");
    strcat(fname,VERSTR);
#endif
    strcat(fname,crlf);
    strcat(fname,crlf);

    lseek(hflog,0,SEEK_END);
    z=(DWORD)strlen(fname);
    write(hflog,fname,z);
  }
#endif
}

CPco_Log::~CPco_Log()
{

#if defined OUTPUT_FILE
  if(hflog)
  {
    char fname[MAX_PATH+100];
    SYSTEMTIME  st;
    DWORD z;

    GetLocalTime(&st);
    sprintf(fname,"Log ended at %02d.%02d.%04d %02d:%02d.%02d",st.wDay,st.wMonth,st.wYear,st.wHour,st.wMinute,st.wSecond);
    strcat(fname,crlf);
    strcat(fname,crlf);

    lseek(hflog,0,SEEK_END);
    z=(DWORD)strlen(fname);
    write(hflog,fname,z);
    close(hflog);
    hflog=-1;
  }
#endif
}


void CPco_Log::writelog(DWORD lev,const char *str,...)
{

#if defined OUTPUT_FILE || defined OUTPUT_CON
  va_list arg;

  QueryPerformanceCounter((LARGE_INTEGER*)&lpPCount2);

  if(lev==0)
    lev+=ERROR_M;
  if(lev&log_bits)
  {
    char buf[300];
    SYSTEMTIME  st;
    DWORD z;

    z=0;
    memset(buf,0,300);

    if(log_bits&TIME_M)
    {
      GetLocalTime(&st);
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%02d:%02d:%02d.%03d ",st.wHour,st.wMinute,st.wSecond,st.wMilliseconds);
    }
    if(log_bits&TIME_MD)
    {
      double time;
      time=(double)(lpPCount2-lpPCount1);
      time=time/lpFrequency;
      time*=1000; //ms
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%6d.%03dms ",(int)time%1000000,(int)((time-(int)time)*1000));
    }
    if(log_bits&THREAD_ID)
    {
     pid_t t=syscall(SYS_gettid);
     pid_t p=getpid();
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%05d:%05d ",p,t);
    }
    if(log_bits&CPU_ID)
    {
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%02d ",sched_getcpu());
    }

    if(lev&ERROR_M)
    {
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"ERROR ");
    }

    va_start(arg,str);
    z=(DWORD)strlen(buf);
    vsprintf(buf+z,str,arg);
    va_end(arg);

#if defined OUTPUT_FILE
    if(hflog)
    {
      strcat(buf,crlf);
      lseek(hflog,0,SEEK_END);
      z=(DWORD)strlen(buf);
      write(hflog,buf,z);
    }
#elif defined OUTPUT_CON
    fprintf(stdout,"%s\n",buf);
#endif 
    lpPCount1=lpPCount2;
 }
#endif 
}

void CPco_Log::writelog(DWORD lev,const char *str,  va_list arg)
{

#if defined OUTPUT_FILE || defined OUTPUT_CON
  QueryPerformanceCounter((LARGE_INTEGER*)&lpPCount2);

  if(lev==0)
    lev+=ERROR_M;
  if(lev&log_bits)
  {
    char buf[300];
    SYSTEMTIME  st;
    DWORD z;

    z=0;
    memset(buf,0,300);

    if(log_bits&TIME_M)
    {
      GetLocalTime(&st);
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%02d:%02d:%02d.%03d ",st.wHour,st.wMinute,st.wSecond,st.wMilliseconds);
    }
    if(log_bits&TIME_MD)
    {
      double time;
      time=(double)(lpPCount2-lpPCount1);

      time=time/lpFrequency;
      time*=1000; //ms
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%6d.%03dms ",(int)time%1000000,(int)((time-(int)time)*1000));
    }
    if(log_bits&THREAD_ID)
    {
     pid_t t=syscall(SYS_gettid);
     pid_t p=getpid();
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%05d:%05d ",p,t);
    }
    if(log_bits&CPU_ID)
    {
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%02d ",sched_getcpu());
    }

    if(lev&ERROR_M)
    {
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"ERROR ");
    }

    z=(DWORD)strlen(buf);
    vsprintf(buf+z,str,arg);

#if defined OUTPUT_FILE
    if(hflog)
    {
      strcat(buf,crlf);
      lseek(hflog,0,SEEK_END);
      z=(DWORD)strlen(buf);
      write(hflog,buf,z);
    }
#elif defined  OUTPUT_CON
    fprintf(stdout,"%s\n",buf);
#endif 
    lpPCount1=lpPCount2;
  }
#endif
}


void CPco_Log::writelog(DWORD lev,PCO_HANDLE hdriver,const char *str,...)
{
#if defined OUTPUT_FILE || defined OUTPUT_CON
  va_list arg;

  QueryPerformanceCounter((LARGE_INTEGER*)&lpPCount2);

  if(lev==0)
    lev+=ERROR_M;
  if(lev&log_bits)
  {
    char buf[300];
    SYSTEMTIME  st;
    DWORD z;

    z=0;
    memset(buf,0,300);

    if(log_bits&TIME_M)
    {
      GetLocalTime(&st);
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%02d:%02d:%02d.%03d ",st.wHour,st.wMinute,st.wSecond,st.wMilliseconds);
    }
    if(log_bits&TIME_MD)
    {
      double time;
      time=(double)(lpPCount2-lpPCount1);

      time=time/lpFrequency;
      time*=1000; //ms
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%6d.%03dms ",(int)time%1000000,(int)((time-(int)time)*1000));
    }
    if(log_bits&THREAD_ID)
    {
     pid_t t=syscall(SYS_gettid);
     pid_t p=getpid();
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%05d:%05d ",p,t);
    }
    if(log_bits&CPU_ID)
    {
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%02d ",sched_getcpu());
    }

    z=(DWORD)strlen(buf);
    sprintf(buf+z,"0x%04x ",(DWORD) hdriver);

    if(lev&ERROR_M)
    {
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"ERROR ");
    }

    va_start(arg,str);
    z=(DWORD)strlen(buf);
    vsprintf(buf+z,str,arg);
    va_end(arg);

#if defined OUTPUT_FILE
    if(hflog)
    {
      strcat(buf,crlf);
      lseek(hflog,0,SEEK_END);
      z=(DWORD)strlen(buf);
      write(hflog,buf,z);
    }
#elif defined  OUTPUT_CON
    fprintf(stdout,"%s\n",buf);
#endif 
    lpPCount1=lpPCount2;
 }
#endif
}


void CPco_Log::writelog(DWORD lev,PCO_HANDLE hdriver,const char *str,va_list args)
{

#if defined OUTPUT_FILE || defined OUTPUT_CON
  QueryPerformanceCounter((LARGE_INTEGER*)&lpPCount2);

  if(lev==0)
    lev+=ERROR_M;
  if(lev&log_bits)
  {
    char buf[300];
    SYSTEMTIME  st;
    DWORD z;

    z=0;
    memset(buf,0,300);

    if(log_bits&TIME_M)
    {
      GetLocalTime(&st);
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%02d:%02d:%02d.%03d ",st.wHour,st.wMinute,st.wSecond,st.wMilliseconds);
    }
    if(log_bits&TIME_MD)
    {
     double time;
     time=(double)(lpPCount2-lpPCount1);
 
     time=time/lpFrequency;
     time*=1000; //ms
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"%6d.%03dms ",(int)time%1000000,(int)((time-(int)time)*1000));
    }
    if(log_bits&THREAD_ID)
    {
     pid_t t=syscall(SYS_gettid);
     pid_t p=getpid();
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%05d:%05d ",p,t);
    }
    if(log_bits&CPU_ID)
    {
     z=(DWORD)strlen(buf);
     sprintf(buf+z,"%02d ",sched_getcpu());
    }

    z=(DWORD)strlen(buf);
    sprintf(buf + z, "0x%04x ", (DWORD) hdriver);

    if(lev&ERROR_M)
    {
      z=(DWORD)strlen(buf);
      sprintf(buf+z,"ERROR ");
    }

    z=(DWORD)strlen(buf);
    vsprintf(buf+z,str,args);

#if defined OUTPUT_FILE
    if(hflog)
    {
      strcat(buf,crlf);
      lseek(hflog,0,SEEK_END);
      z=(DWORD)strlen(buf);
      write(hflog,buf,z);
    }
#elif defined  OUTPUT_CON
    fprintf(stdout,"%s\n",buf);
#endif 

    lpPCount1=lpPCount2;
  }
#endif
}



void CPco_Log::set_logbits(DWORD logbit)
{
  log_bits=logbit;
}

DWORD CPco_Log::get_logbits(void)
{
  return log_bits;
}


void CPco_Log::start_time_mess(void)
{
  QueryPerformanceCounter((LARGE_INTEGER*)&stamp1);
}

double CPco_Log::stop_time_mess(void)
{
  double time;
  QueryPerformanceCounter((LARGE_INTEGER*)&stamp2);
  time=(double)(stamp2-stamp1);
  time=time/lpFrequency;
  time*=1000; //ms
  return time;
}

#ifndef WIN32
void CPco_Log::GetLocalTime(SYSTEMTIME* st)
{
  struct timeval t1;
  struct tm timeinfo;

  gettimeofday(&t1,NULL);
  localtime_r(&t1.tv_sec,&timeinfo);

  st->wSecond    = timeinfo.tm_sec;
  st->wMinute    = timeinfo.tm_min;
  st->wHour      = timeinfo.tm_hour;
  st->wDay       = timeinfo.tm_mday;
  st->wDayOfWeek = timeinfo.tm_wday;
  st->wMonth     = timeinfo.tm_mon+1;
  st->wYear      = timeinfo.tm_year+1900;

  st->wMilliseconds = t1.tv_usec/1000;

}

void CPco_Log::QueryPerformanceCounter(LARGE_INTEGER* lpCount)
{
    struct timeval t;

    gettimeofday(&t,NULL);
    *lpCount = t.tv_sec*1000000LL + t.tv_usec; //time in useconds
/*
    timespec t;
    long long nanotime;
    clock_gettime(CLOCK_MONOTONIC,&t);

    nanotime = t.tv_sec*1000000000LL + t.tv_nsec; //time in nanoseconds
    *lpCount = nanotime / 1000 + (nanotime % 1000 >= 500); //round up halves
*/
}

void CPco_Log::QueryPerformanceFrequency(LARGE_INTEGER* frequency)
{
    *frequency = 1000000;
}

#endif
