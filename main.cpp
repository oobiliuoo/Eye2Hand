
#include "BLFactory.h"
#include "BLCalibration.h"
#include "keyHead.h"


BLAstraCamrea* cam;



// AstraGCamera ��Ļص����������������ȡ����ɫͼ�����ͼ
void keyCallback(cv::Mat colorImg, cv::Mat depthImg)
{

	int key = cv::waitKey(30) & 0xFF;

	switch (key)
	{
	case ACTION_A:
	{
		// ��ȡ���ص��Ӧ���������
		cv::Point2d p(300, 200);
		cv::Point3f p2;
		piexl2Cam(p, p2, depthImg, cam->RgbParamMat);
		std::cout << "p2" << p2;

		break;

	}

	case ACTION_B:
	{
		test(cam);
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
		cam->close();
		cv::destroyAllWindows();
		std::cout << "��������....\n";
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
	cam->setkeyCallback(keyCallback);
	cam->setShowMode(3);
	cam->start();

	cam->join();
	b.deleteObject(cam);


}

