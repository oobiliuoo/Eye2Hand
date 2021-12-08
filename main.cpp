
#include "BLFactory.h"
#include "BLCalibration.h"
#include "keyHead.h"


BLAstraCamrea* cam;



// AstraGCamera 类的回调函数，用来处理获取的颜色图和深度图
void keyCallback(cv::Mat colorImg, cv::Mat depthImg)
{

	int key = cv::waitKey(30) & 0xFF;

	switch (key)
	{
	case ACTION_A:
	{
		// 获取像素点对应的相机坐标
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
		// 相机标定
		cv::Mat camParm, camK;
		camCalibration(camParm, camK);
		break;


	}

	case ACTION_D:
	{
		if (depthImg.cols > 0)
			// 获取深度图
			saveImg(colorImg.clone(), 3, depthImg.clone());
		else
			std::cout << "深度图未打开..";
		break;
	}

	case ACTION_E:
	{
		// 手眼标定
		eye2handCal();

		break;

	}

	case ACTION_H:
	{
		std::cout << " 帮助：\nA 获取特征点相机坐标 \nC 相机标定 \nD 获取深度图 \nE 手眼标定 \nSPACE 拍照 \nESC(Q) 退出" << std::endl;
		break;
	}

	case ACTION_SPACE:
	{
		// 保存文件
		saveImg(colorImg.clone(), MODE);
		break;
	}

	case ACTION_Q:
	case ACTION_ESC:
	{
		// 关闭程序
		cam->close();
		cv::destroyAllWindows();
		std::cout << "结束程序....\n";
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

