#include <stdio.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <signal.h>
#include <ncurses.h>
#include <libgen.h>
#include "LinuxDARwIn.h"

using namespace Robot;
using namespace std;

#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#define INI_FILE_PATH       "../../../Data/config.ini"

#define VERSION					"1.000"
#define TCPIP_PORT				6501
#define ROBOPLUS_JOINT_MAXNUM	26
#define ARGUMENT_NAXNUM			30


LinuxArbotixPro linux_arbotixpro("/dev/ttyUSB0");
ArbotixPro arbotixpro(&linux_arbotixpro);

int _getch()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

string* string_split(string str_org, string str_tok)
{
    int cutAt;
    int index = 0;

    string* str_result = new string[ARGUMENT_NAXNUM];

    while((cutAt = str_org.find_first_of(str_tok)) != str_org.npos)
    {
        if(cutAt > 0)
            str_result[index++] = str_org.substr(0, cutAt);
        str_org = str_org.substr(cutAt + 1);
    }
	
    if(str_org.length() > 0)
        str_result[index++] = str_org.substr(0, cutAt);

    return str_result;
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
	exit(0);
}

int main(int argc, char *argv[])
{
	signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
	signal(SIGQUIT, &sighandler);
	signal(SIGINT, &sighandler);

	int ch;
	char filename[128];

    fprintf(stderr, "\n***********************************************************************\n");
    fprintf(stderr,   "*                      RoboPlus Server Program                        *\n");
    fprintf(stderr,   "***********************************************************************\n\n");

    minIni* ini = new minIni(INI_FILE_PATH);

	change_current_dir();
	if(argc < 2)
		strcpy(filename, MOTION_FILE_PATH); // Set default motion file path
	else
		strcpy(filename, argv[1]);

	/////////////// Load/Create Action File //////////////////
	if(Action::GetInstance()->LoadFile(filename) == false)
	{
		printf("Can not open %s\n", filename);
		printf("Do you want to make a new action file? (y/n) ");
		ch = _getch();
		if(ch != 'y')
		{
			printf("\n");
			return 0;
		}

		if(Action::GetInstance()->CreateFile(filename) == false)
		{
			printf("Fail to create %s\n", filename);
			return 0;
		}
	}
	////////////////////////////////////////////////////////////


	//////////////////// Framework Initialize ////////////////////////////	
	if(MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
    MotionManager::GetInstance()->LoadINISettings(ini);
	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());	
	LinuxMotionTimer linuxMotionTimer;
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Stop();
	/////////////////////////////////////////////////////////////////////

    int firm_ver = 0;
    if(arbotixpro.ReadByte(JointData::ID_HEAD_PAN, AXDXL::P_VERSION, &firm_ver, 0)  != ArbotixPro::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if(27 > firm_ver)
    {
        fprintf(stderr, "The RoboPlus(ver 1.0.23.0 or higher) Motion is not supported 1024 resolution..\n\n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
    }

	cout << "[Running....]\n";
    try
    {
        // Create the socket
        //LinuxSocket new_sock;
        LinuxServer new_sock;
        LinuxServer server ( TCPIP_PORT );

        while ( true )
        {
            cout << "[Waiting..]" << endl;            
            server.accept ( new_sock );
            cout << "[Accepted..]" << endl;

            try
            {
                while ( true )
                {
                    string data;
					Action::PAGE page;

                    new_sock >> data;
					cout << data << endl;

                    string* p_str_tok = string_split(data, " ");

                    if(p_str_tok[0] == "v")
                    {
                        new_sock << "{[DARwIn:" << VERSION << "]}\n";
                    }
                    else if(p_str_tok[0] == "E")
                    {
                        new_sock << "{[DARwIn:1.000]}";
                        new_sock << "{[PC:TCP/IP][DXL:1000000(BPS)]}";
						new_sock << "{";
						for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
							new_sock << "[" << id << ":024(AX-12)]";
						new_sock << "}";
                        new_sock << "{[DXL:" << JointData::NUMBER_OF_JOINTS << "(PCS)]}";
                        new_sock << "{[ME]}\n";
                    }
                    else if(p_str_tok[0] == "exit")
                    {
						Action::GetInstance()->Stop();
                    }
                    else if(p_str_tok[0] == "List")
                    {
                        new_sock << "{";
                        for(int i = 0; i < 256; i++)
						{
							Action::GetInstance()->LoadPage(i, &page);
                                                        //new_sock << "[" << i << ":";
                                                        //str[strcspn(reinterpret_cast<const char*>(page.header.name), "\n")] = '\0';
							//new_sock.send(page.header.name, 14);
							//new_sock << ":" << (int)page.header.stepnum << ":" << (int)page.header.next << ":" << (int)page.header.exit << "]";
    int j =0;
    while (page.header.name[j] != '\n'){
        j++;
    }
    page.header.name[j] = '\0';
                                                        new_sock  << "[" << i << ":" << (char*)page.header.name << ":" << (int)page.header.next << ":" << (int)page.header.exit << ":" << (int)page.header.stepnum << "]";
                                                        //new_sock  << "[" << i << ":" << (char*)str << ":" << (int)page.header.stepnum << ":" << (int)page.header.next << ":" << (int)page.header.exit << "]";
						}
                        /*char str[15] = "hellohellohell"; 
                        int i = 1;
                        int step = 3;
                        int next = 0;
                        int exit = 0;
                        new_sock  << "[" << i << ":" << (char*)str << ":" << (int)step << ":" << (int)next << ":" << (int)exit << "]";
                        */
                        new_sock << "}";
                        new_sock << "{[ME]}\n";
                    }
                    else if(p_str_tok[0] == "Get" || p_str_tok[0] == "on" || p_str_tok[0] == "off")
                    {
						if(p_str_tok[0] == "on" || p_str_tok[0] == "off")
						{
							int torque = 0;
							if(p_str_tok[0] == "on")
							{
								torque = 1;
								
							}
							else if(p_str_tok[0] == "off")
							{
								torque = 0;
							}

							int i;
							for(i=1; i<ARGUMENT_NAXNUM; i++)
							{
								if(p_str_tok[i].length() > 0)
								{
									arbotixpro.WriteByte(atoi(p_str_tok[i].c_str()), AXDXL::P_TORQUE_ENABLE, torque, 0);
								}
								else
									break;
							}

							if(i == 1)
							{
								for(int id=1; id<JointData::NUMBER_OF_JOINTS; id++)
									arbotixpro.WriteByte(id, AXDXL::P_TORQUE_ENABLE, torque, 0);
							}
						}

						int torque, position;
						new_sock << "{";
						for(int id=0; id<ROBOPLUS_JOINT_MAXNUM; id++)
						{
							new_sock << "[";
							cout << "[";
							if(id >= 1 && id < JointData::NUMBER_OF_JOINTS)
							{
								if(arbotixpro.ReadByte(id, AXDXL::P_TORQUE_ENABLE, &torque, 0) == ArbotixPro::SUCCESS)
								{
									if(torque == 1)
									{
										if(arbotixpro.ReadWord(id, AXDXL::P_GOAL_POSITION_L, &position, 0) == ArbotixPro::SUCCESS)
										{
											new_sock << position;
											cout << position;
										}
										else
										{
											new_sock << "----";
											cout << "Fail to read present position ID(" << id << ")";
										}
									}
									else
									{
										new_sock << "????";
										cout << "????";
									}
								}
								else
								{
									new_sock << "----";
									cout << "Fail to read torque ID(" << id << ")";
								}
							}
							else
							{
								new_sock << "----";
								cout << "----";
							}
							new_sock << "]";
							cout << "]";
						}
						cout << endl;
						new_sock << "}";
                        new_sock << "{[ME]}\n";
                    }
                    else if(p_str_tok[0] == "go")
                    {
						int GoalPosition, StartPosition, Distance;
						
						new_sock << "{";
						for(int id=0; id<ROBOPLUS_JOINT_MAXNUM; id++)
						{
							if(id >= 1 && id < JointData::NUMBER_OF_JOINTS)
							{
								if(arbotixpro.ReadWord(id, AXDXL::P_PRESENT_POSITION_L, &StartPosition, 0) == ArbotixPro::SUCCESS)
								{
									GoalPosition = (int)atoi(p_str_tok[id + 1].c_str());

									if( StartPosition > GoalPosition )
										Distance = StartPosition - GoalPosition;
									else
										Distance = GoalPosition - StartPosition;

									Distance >>= 2;
									if( Distance < 8 )
										Distance = 8;

									arbotixpro.WriteWord(id, AXDXL::P_MOVING_SPEED_L, Distance, 0);
									if(arbotixpro.WriteWord(id, AXDXL::P_GOAL_POSITION_L, GoalPosition, 0) == ArbotixPro::SUCCESS)
									{
										new_sock << "[" << GoalPosition << "]";
										cout << "[" << GoalPosition << "]";
									}
									else
									{
										new_sock << "[----]";
										cout << "[----]";
									}
								}
								else
								{
									new_sock << "[----]";
									cout << "[----]";
								}
							}
							else
							{
								new_sock << "[----]";
								cout << "[----]";
							}
						}
						cout << endl;
						new_sock << "}";
						new_sock << "{[ME]}\n";
                    }
                    else if(p_str_tok[0] == "set")
                    {
						int id = (int)atoi(p_str_tok[1].c_str());
						int position = (int)atoi(p_str_tok[2].c_str());

						if(arbotixpro.WriteWord(id, AXDXL::P_GOAL_POSITION_L, position, 0) == ArbotixPro::SUCCESS)
						{
							if(arbotixpro.ReadWord(id, AXDXL::P_GOAL_POSITION_L, &position, 0) == ArbotixPro::SUCCESS)
							{
								cout << "{[" << position << "]}";
								new_sock << "{[" << position << "]}";
							}
							else
							{
								cout << "[Fail to read goal position ID(" << id << ")]";
								new_sock << "{[----]}";
							}
						}
						else
						{
							cout << "[Fail to write goal position ID(" << id << ")]";
							new_sock << "{[----]}";
						}
						cout << endl;
						new_sock << "{[ME]}\n";
                    }
                    else if(p_str_tok[0] == "play" || p_str_tok[0] == "rplay")
                    {						
						int value;

						for(int id=0; id<ROBOPLUS_JOINT_MAXNUM; id++)
						{
							if(id >= JointData::ID_R_SHOULDER_PITCH && id <= JointData::ID_HEAD_TILT)
                                                        //if(id >= JointData::ID_R_SHOULDER_PITCH && id < JointData::NUMBER_OF_JOINTS)
							{
								if(arbotixpro.ReadWord(id, AXDXL::P_TORQUE_ENABLE, &value, 0) == ArbotixPro::SUCCESS)
								{
									if(value == 0)
									{
										if(arbotixpro.ReadWord(id, AXDXL::P_PRESENT_POSITION_L, &value, 0) == ArbotixPro::SUCCESS)
											MotionStatus::m_CurrentJoints.SetValue(id, value);
										else
											cout << "[Fail to communication ID(" << id << ")]" << endl;
									}
									else
									{
										if(arbotixpro.ReadWord(id, AXDXL::P_GOAL_POSITION_L, &value, 0) == ArbotixPro::SUCCESS)
											MotionStatus::m_CurrentJoints.SetValue(id, value);
										else
											cout << "[Fail to communication ID(" << id << ")]" << endl;
									}
								}
								else
									cout << "[Fail to communication ID(" << id << ")]" << endl;
							}
						}														
						
					    linuxMotionTimer.Start();
						MotionManager::GetInstance()->SetEnable(true);
						
						int index = (int)atoi(p_str_tok[1].c_str());
						if(p_str_tok[0] == "play")
							Action::GetInstance()->Start(index);
						else
							Action::GetInstance()->Start(index, &page);
                    }
					else if(p_str_tok[0] == "info")
					{
						int ipage, istep;
						if(Action::GetInstance()->IsRunning(&ipage, &istep) == 1)
						{
							new_sock << "{[" << ipage << ":" << istep << "]}\n";
							cout << "[" << ipage << ":" << istep << "]" << endl;
						}
						else
						{
							new_sock << "{[OK]}\n";
							cout << "[END]" << endl;
							MotionManager::GetInstance()->SetEnable(false);
							linuxMotionTimer.Stop();
						}
					}
                    else if(p_str_tok[0] == "stop")
                    {
						Action::GetInstance()->Stop();
                    }
                    else if(p_str_tok[0] == "stopinfo")
                    {
                        Action::GetInstance()->Stop();

                        int ipage, istep;
                        if(Action::GetInstance()->IsRunning(&ipage, &istep) == 1)
                        {
                            new_sock << "{[" << ipage << ":" << istep << "]}\n";
                            cout << "[" << ipage << ":" << istep << "]" << endl;
                        }
                        else
                        {
                            new_sock << "{[OK]}\n";
                            cout << "[END]" << endl;
                            MotionManager::GetInstance()->SetEnable(false);
                            linuxMotionTimer.Stop();
                        }
                    }
					else if(p_str_tok[0] == "break")
                    {
						Action::GetInstance()->Brake();
                    }
                    else if(p_str_tok[0] == "breakinfo")
                    {
                        Action::GetInstance()->Brake();

                        int ipage, istep;
                        if(Action::GetInstance()->IsRunning(&ipage, &istep) == 1)
                        {
                            new_sock << "{[" << ipage << ":" << istep << "]}\n";
                            cout << "[" << ipage << ":" << istep << "]" << endl;
                        }
                        else
                        {
                            new_sock << "{[OK]}\n";
                            cout << "[END]" << endl;
                            MotionManager::GetInstance()->SetEnable(false);
                            linuxMotionTimer.Stop();
                        }
                    }
                    else if(p_str_tok[0] == "RDownload")
                    {
						int rcv_len = (int)sizeof(Action::PAGE);
						int i_data = 0;
						unsigned char *data = (unsigned char*)&page;

                        new_sock << "{[READY]}\n";
						while(rcv_len > 0)
						{
							i_data += new_sock.recv(data, rcv_len);
							data += i_data;
							rcv_len -= i_data;
						}
                        new_sock << "{[SUCCESS]}\n";
                        new_sock << "{[ME]}\n";
                    }                    
                    else if(p_str_tok[0] == "upload")
                    {
                        // send page data
						int index = (int)(atoi(p_str_tok[1].c_str()) / sizeof(Action::PAGE));
						int num = (int)(atoi(p_str_tok[2].c_str()) / sizeof(Action::PAGE));
						
						for(int i=0; i<num; i++)
						{
							Action::GetInstance()->LoadPage(index + i, &page);
							new_sock.send((unsigned char*)&page, (int)sizeof(Action::PAGE));
						}

                        new_sock << "{[ME]}\n";
                    }
                    else if(p_str_tok[0] == "ld")
                    {
						int index = (int)(atoi(p_str_tok[1].c_str()) / sizeof(Action::PAGE));
						int num = (int)(atoi(p_str_tok[2].c_str()) / sizeof(Action::PAGE));
						int rcv_len;
						int i_data;
						unsigned char *data;

						cout << "[READY]" << endl;
                        new_sock << "{[READY]}\n";

                        // byte stream
						for(int i=0; i<num; i++)
						{
							i_data = 0;
							rcv_len = (int)sizeof(Action::PAGE);
							data = (unsigned char*)&page;
							while(rcv_len > 0)
							{
								i_data += new_sock.recv(data, rcv_len);
								data += i_data;
								rcv_len -= i_data;
							}
							Action::GetInstance()->SavePage(index + i, &page);
							cout << "[SAVE:" << index + i << "]" << endl;
						}
                        new_sock << "{[ME]}\n";
                    }
					else
					{
						cout << " [Invalid:" << p_str_tok[0] << "]" << endl;
					}
                }
            }
            catch ( LinuxSocketException& )
			{
				cout << "[Disconnected]" << endl;

				if(Action::GetInstance()->IsRunning() == 1)
				{
					Action::GetInstance()->Stop();
					while(Action::GetInstance()->IsRunning() == 1)
						usleep(1);
					MotionManager::GetInstance()->SetEnable(false);
					linuxMotionTimer.Stop();
				}
			}
        }
    }
    catch ( LinuxSocketException& e )
    {
        cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

	return 0;
}
