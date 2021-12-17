#include "BLFactory.h"
#include "BLCalibration.h"
#include "keyHead.h"
#include "tcp_client.h"


BLAstraCamrea* cam;

int tcp_run = 1;
int con_run = 1;

// AstraGCamera ��Ļص����������������ȡ����ɫͼ�����ͼ
void keyCallback(cv::Mat colorImg, cv::Mat depthImg)
{

	int key = cv::waitKey(30) & 0xFF;

//	showPos(cam);
	

	switch (key)
	{
	case ACTION_A:
	{
		// ��ȡ���ص��Ӧ���������
	
		cv::Point2d p(300, 200);
		cv::Point3d p2;
		piexl2Cam(p, p2, depthImg, cam->RgbParamMat);
		std::cout << "p2" << p2;

		break;

	}

	case ACTION_B:
	{

		cout << cam->RgbParamMat;
		cout << "\n" << cam->RgbDistCoeffs;
		//test(cam);
		break;
	
	}

	case ACTION_C:
	{
		// ����궨
		cv::Mat camParm, camK;
		camCalibration(camParm, camK);
		break;


	}

	case ACTION_D:
	{
		if (depthImg.cols > 0)
			// ��ȡ���ͼ
			saveImg(colorImg.clone(), 3, depthImg.clone());
		else
			std::cout << "���ͼδ��..";
		break;
	}

	case ACTION_E:
	{
		// ���۱궨
		eye2handCal();

		break;

	}

	case ACTION_H:
	{
		std::cout << " ������\nA ��ȡ������������� \nC ����궨 \nD ��ȡ���ͼ \nE ���۱궨 \nSPACE ���� \nESC(Q) �˳�" << std::endl;
		break;
	}

	case ACTION_SPACE:
	{
		// �����ļ�
		saveImg(colorImg.clone(), MODE);
		break;
	}

	case ACTION_Q:
	case ACTION_ESC:
	{
		// �رճ���

		close(cam, tcp_run, con_run);


		break;
	}
	default:
		break;
	}


}



int main()
{
	BLfactory b;
	cam = b.createBLAstraCamera();
//	cam->setkeyCallback(keyCallback);
	cam->setShowMode(1);
	cam->start();

	std::thread threadTcp([&] {tcp_Robotic_Arm(tcp_run); });

	//Sleep(2000);
	//char recv_msg[1024];
	//char send_msg[1024];
	//send_msg[0] = 'A';

	//tcp_send(send_msg);
	//tcp_recv(recv_msg);

	//printf("main: %s\n\n", recv_msg);
	
	control(cam,tcp_run,con_run);

	threadTcp.join();

	cam->join();
	b.deleteObject(cam);


}

