//-----------------------------------------------------------------//
// Name        | main.cpp                    | Type: (*) source    //
//-------------------------------------------|       ( ) header    //
// Project     | pco.camera                  |       ( ) others    //
//-----------------------------------------------------------------//
// Platform    | Linux, Windows                                    //
//-----------------------------------------------------------------//
// Environment |                                                   //
//             |                                                   //
//-----------------------------------------------------------------//
// Purpose     | Qt project part                                   //
//-----------------------------------------------------------------//
// Author      | MBL,EIJ, PCO AG                                   //
//-----------------------------------------------------------------//
// Revision    | rev. 1.00                                         //
//-----------------------------------------------------------------//
// Notes       |                                                   //
//             |                                                   //
//             |                                                   //
//             |                                                   //
//             |                                                   //
//-----------------------------------------------------------------//
// (c) 2015 PCO AG                                                 //
// Donaupark 11 D-93309  Kelheim / Germany                         //
// Phone: +49 (0)9441 / 2005-0   Fax: +49 (0)9441 / 2005-20        //
// Email: info@pco.de                                              //
//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// This program is free software; you can redistribute it and/or   //
// modify it under the terms of the GNU General Public License as  //
// published by the Free Software Foundation; either version 2 of  //
// the License, or (at your option) any later version.             //
//                                                                 //
// This program is distributed in the hope that it will be useful, //
// but WITHOUT ANY WARRANTY; without even the implied warranty of  //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the    //
// GNU General Public License for more details.                    //
//                                                                 //
// You should have received a copy of the GNU General Public       //
// License along with this program; if not, write to the           //
// Free Software Foundation, Inc., 59 Temple Place- Suite 330,     //
// Boston, MA 02111-1307, USA                                      //
//-----------------------------------------------------------------//


#include <QtWidgets/QApplication>
#include "interface_includes.h"
#include "qt_pco_camera.h"

/* Global Variables  */
//MBL no globals used
//CPco_Log Clog(LOGNAME);
//CPco_camera camera;
//CPco_grabber grabber(&camera);

/* End Global Variables */

int main(int argc, char *argv[])
{
  int ret;
  int help=0;

  CPco_Log *mylog=NULL;

  for(int i=argc;i>1;)
  {
   char *a;

   i--;

   if(strstr(argv[i],"-h"))
    help=1;
   if(strstr(argv[i],"-?"))
    help=1;
   if(strstr(argv[i],"?"))
    help=1;
/*
   if((a=strstr(argv[i],"-b")))
   {
    board=atoi(a+2);
   }

*/
   if((a=strstr(argv[i],"-l")))
   {
    unsigned int x=strtol(a+2,NULL,0);
    printf("Logging to '%s' enabled, ",LOGNAME);
    mylog=new CPco_Log(LOGNAME);
    if(x>0)
    {
     x|=0x03;
     mylog->set_logbits(x);
    }
    printf("logbits set to 0x%x\n",mylog->get_logbits());
   }
  }

  if(help)
  {
   printf("usage: %s options\n"
          "options: \n"
          "-l[0...]  enable logging, set loglevel\n"
          "-h,-?,? this message\n",argv[0]);

   if(mylog)
    delete mylog;
   return 0;
  }


  QApplication a(argc, argv);
  qt_pco_camera w(mylog);
  w.show();
  ret=a.exec();

  if(mylog)
   delete mylog;
  return ret;
}

