#include "keyHead.h"
#include "BLAstraGCamera.h"
#include "tcp_client.h"
#include "BLCalibration.h"

void control(BLAstraCamrea* cam, int run=1) 
{
	char bufSend[BUF_SIZE] = { 0 };//发送缓冲区
	char bufRecv[BUF_SIZE] = { 0 };//接受缓冲区
	cv::Mat_<double> ToolPose;
	while (run==1)
	{
		
		
		printf("\nEnter command:");
		gets_s(bufSend);

	//	std::cout << "bufSend = "<<bufSend;

		switch (*bufSend)
		{

		case '0':
		{
			run = 0;
			std::cout << "cmd close....";
			break;
		}

		case '1':
		{
		
			// 获取机器人位姿
		   /* memset(bufSend, 0, BUF_SIZE);
			memset(bufRecv, 0, BUF_SIZE);*/
			bufSend[0] = 'A';
			tcp_send(bufSend);
			int ok = tcp_recv(bufRecv);
			if (ok < 0) break;
			printf("%s\n\n", bufRecv);
			//解析位姿
			ToolPose = bl::analyzePose(bufRecv, sizeof(bufRecv), ToolPose);

			break;
		}

		case '2':
		{
			// 保存机器人位姿
			bl::writeRobotPos(ToolPose,EH_CAL_ROBOT_PATH_TXT);



			break;
		}
		
		default:
			tcp_send(bufSend);
			tcp_recv(bufRecv);
			printf("收到信息: %s\n\n", bufRecv);
			break;
		}

		memset(bufSend, 0, BUF_SIZE);
		memset(bufRecv, 0, BUF_SIZE);

	}



}