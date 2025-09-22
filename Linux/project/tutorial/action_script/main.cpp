/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"

#include "Action.h"
#include "Head.h"
#include "Walking.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "LinuxActionScript.h"
#include "LinuxDARwIn.h"

#define MOTION_FILE_PATH    "../../../../Data/motion_4096.bin"

#define INI_FILE_PATH       "../../../../Data/config.ini"


LinuxArbotixPro linux_arbotixpro("/dev/ttyUSB0");
ArbotixPro arbotixpro(&linux_arbotixpro);
LinuxMotionTimer linuxMotionTimer;


void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}
int main(void)
{

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    char filename[128];

    minIni* ini = new minIni(INI_FILE_PATH);

    change_current_dir();
        strcpy(filename, MOTION_FILE_PATH); // Set default motion file path

    /////////////// Load/Create Action File //////////////////
    if (Action::GetInstance()->LoadFile(filename) == false)
    {
        printf("Can not open %s\n", filename);
        exit(0);
    }
    ////////////////////////////////////////////////////////////

    //    PS3Controller_Start();
    //////////////////// Framework Initialize ////////////////////////////
    if (MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
    {
        printf("Initializing Motion Manager failed!\n");
        exit(0);
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->SetEnable(false);
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Stop();
    /////////////////////////////////////////////////////////////////////
   

    Action::GetInstance()->m_Joint.SetEnable(true,true);
    MotionManager::GetInstance()->SetEnable(true);
    linuxMotionTimer.Start();

    Action::GetInstance()->Start(1);    /* Init(stand up) pose */
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

    printf("Press the ENTER key to begin!\n");
    getchar();

    LinuxActionScript::ScriptStart("script.asc");
    while(LinuxActionScript::m_is_running == 1) sleep(10);

    return 0;
}
