﻿#include <iostream>
#include <iomanip> 
#include <cstring>


#include "aris_core_socket.h"
#include "aris_core_msg_loop.h"

enum ClientMessage
{
	VisualSystemConnected,
	VisualSystemDataReceived,
	VisualSystemLost,
	ControlSystemConnected,
	ControlTrajectoryFinished,
	ControlSystemLost,
};

using namespace std;


#ifdef UNIX
#include <unistd.h>
#endif

/*以下用于VS内存泄露检测*/
#ifdef WIN32  
#include <Windows.h>

#define CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>  
_CrtMemState s1, s2, s3;
#endif
/*内存检测泄露完毕*/


aris::core::Socket VisualSystem, ControlSystem;

int main()
{
	
	/*以下用于VS内存泄露检测*/
#ifdef WIN32  
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtMemCheckpoint(&s1);
	//_CrtSetBreakAlloc(203);
#endif

	/*内存检测泄露完毕*/
	{	
		/*注册所有的消息函数*/
		aris::core::registerMsgCallback(VisualSystemConnected, [](aris::core::Msg &msg)
		{
			cout << "Received connection from vision system:" << endl;
			cout << "    Remote IP is: " << msg.data() + 4 << endl;
			cout << "    Port is     : " << *((int*)msg.data()) << endl << endl;

			/*像视觉客户端发送数据，索要地图*/
			aris::core::Msg data(0, 0);
			VisualSystem.sendMsg(data);

			return 0;
		});
		aris::core::registerMsgCallback(VisualSystemDataReceived, [](aris::core::Msg &msg)
		{
			cout << "Received data from vision system:" << endl;

			double map[9];
			msg.paste(map, sizeof(map));

			for (int i = 0; i < 9; ++i)
			{
				cout << "    " << map[i] << endl;
			}


			/*询问用户该地图是否满足需求*/
			cout << "Is map OK?" << endl;

			char answer[16];

			while (1)
			{
				cin >> answer;


				if (strcmp(answer, "yes") == 0)
				{
					aris::core::Msg data;
					ControlSystem.sendMsg(data);
					cout << "command robot to walk" << endl;
					break;
				}
				else if (strcmp(answer, "no") == 0)
				{
					aris::core::Msg data;
					VisualSystem.sendMsg(data);
					break;
				}

			}

			return 0;
		});
		aris::core::registerMsgCallback(ControlSystemConnected, [](aris::core::Msg &msg)
		{
			cout << "Received connection from control system:" << endl;
			cout << "    Remote IP is: " << msg.data() + 4 << endl;
			cout << "    Port is     : " << *((int*)msg.data()) << endl << endl;

			return 0;
		});
		aris::core::registerMsgCallback(ControlTrajectoryFinished, [](aris::core::Msg &msg)
		{
			/*只要收到数据就认为控制客户端已经走完对应的轨迹*/
			cout << "Robot walk finished, Now ask for map" << endl;
			aris::core::Msg data;
			VisualSystem.sendMsg(data);


			return 0;
		});

		/*设置所有Socket类型的回调函数*/
		VisualSystem.setOnReceivedConnection([](aris::core::Socket *pConn, const char *addr, int port)
		{
			aris::core::Msg msg;

			msg.setMsgID(VisualSystemConnected);
			msg.copy(&port, sizeof(int));
			msg.copyMore(addr, strlen(addr) + 1);

			postMsg(msg);

			return 0;
		});
		VisualSystem.setOnReceivedMsg([](aris::core::Socket *pConn, aris::core::Msg &msg)
		{
			msg.setMsgID(VisualSystemDataReceived);
			aris::core::postMsg(msg);
			return 0;
		});
		VisualSystem.setOnLoseConnection([](aris::core::Socket *pConn)
		{
			cout << "Vision system connection lost" << endl;
			pConn->startServer("5688");
			return 0;
		});
		ControlSystem.setOnReceivedConnection([](aris::core::Socket *pConn, const char *addr, int port)
		{
			aris::core::Msg msg;
			
			msg.setMsgID(ControlSystemConnected);
			msg.copy(&port, sizeof(int));
			msg.copyMore(addr, strlen(addr) + 1);

			postMsg(msg);

			return 0;
		});
		ControlSystem.setOnReceivedMsg([](aris::core::Socket *pConn, aris::core::Msg &msg)
		{
			msg.setMsgID(ControlTrajectoryFinished);
			aris::core::postMsg(msg);
			return 0;
		});
		ControlSystem.setOnLoseConnection([](aris::core::Socket *pConn)
		{
			cout << "control system connection lost" << endl;
			pConn->startServer("5689");
			return 0;
		});
		
		ControlSystem.setOnReceivedRequest([](aris::core::Socket *pConn,aris::core::Msg)
		{
			cout << "received request" << endl;
#ifdef UNIX  
			usleep(1000000);
#endif
#ifdef WIN32  
			aris::core::msSleep(1000);
#endif
			

			aris::core::Msg m;
			m.copy("12345");

			return m;
		});

		/*打开客户端*/
		VisualSystem.startServer("5688");
		ControlSystem.startServer("5689");

		/*开始消息循环*/
		aris::core::runMsgLoop();
	}

#ifdef WIN32  
	_CrtMemCheckpoint(&s2);
	if (_CrtMemDifference(&s3, &s1, &s2))
		_CrtMemDumpStatistics(&s3);
	//_CrtDumpMemoryLeaks();
#endif
	
	
}
