#pragma once
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/types_c.h>
#include <fstream>

using std::cout;
using std::endl;

// 按键定义
static const int ACTION_ESC = 27;
static const int ACTION_SPACE = 32;

static const int ACTION_A = 65;
static const int ACTION_B = 66;
static const int ACTION_C = 67;
static const int ACTION_D = 68;
static const int ACTION_E = 69;
static const int ACTION_H = 72;
static const int ACTION_Q = 81;



static std::string NOR_CAL_FILE_PATH = "../img/normal/"; // 保存相机图片的路径
static std::string CAM_CAL_FILE_PATH = "../img/camCal/"; // 保存相机标定机图片的路径
static std::string EH_CAL_FILE_PATH = "../img/eyeHandCal/"; // 保存手眼图片的路径
static std::string DEPTH_IMG_FILE_PATH = "../img/depth/"; // 保存手眼图片的路径

static std::string FILE_PATH[4] = {NOR_CAL_FILE_PATH,CAM_CAL_FILE_PATH ,EH_CAL_FILE_PATH, DEPTH_IMG_FILE_PATH };

static std::string IMG_TYPE = ".PNG"; // 保存图片的格式

// 保存图片路径的文件名
static std::string CAM_CAL_IMG_PATH_TXT = "../img/camCalImageName.txt";
static std::string EH_CAL_IMG_PATH_TXT = "../img/eye2HandCalImageName.txt";
static std::string EH_CAL_ROBOT_PATH_TXT = "../img/eye2HandCalPos.txt";


// 保存图片的下标
int saveImgNum = 0;
// 拍照模式选择
int MODE = 0;

std::ofstream fout;

void saveImg(cv::Mat colorImg,int mode,cv::Mat depthImg=cv::Mat::zeros(3,3,CV_32FC1))
{
	// 保存文件
	std::ostringstream s;
	// 按索引定义文件名存储图片
	s << FILE_PATH[mode] << saveImgNum << IMG_TYPE;
	std::string imageName(s.str());
	cv::imwrite(imageName, colorImg);

	if (mode == 0)
		std::cout << "拍照中...";
	else if (mode == 1)
	{
		std::cout << "相机标定数据获取中...";
		fout << imageName << std::endl;
	}
	else if (mode == 2)
	{
		std::cout << "手眼标定数据获取中...";
		fout << imageName << std::endl;
	}
	else if (mode == 3)
	{
		std::cout << "深度数据获取中...";
		std::ostringstream s2;
		s2 << FILE_PATH[mode]<<"depth_"<< saveImgNum << IMG_TYPE;
		std::string imageName_d(s2.str());
		cv::imwrite(imageName_d, depthImg);

	}

	saveImgNum++;
	std::cout << "保存成功....  :" << saveImgNum << std::endl;

}

void camCalibration(cv::Mat camPram,cv::Mat camK)
{
	if (MODE==1)
	{
		fout.close();
		std::cout << "共获取 " << saveImgNum << " 张图片\n 开始标定" << std::endl;
		bl::cameraCalibration(CAM_CAL_IMG_PATH_TXT, camPram, camK);
	}
	else
	{
		MODE = 1;
		saveImgNum = 0;
		fout.open(CAM_CAL_IMG_PATH_TXT);
		std::cout << "开始获取标定图片，请保存标定板在图像内，按空格拍照\n";

	}

}

void piexl2Cam(cv::Point2d piexlPoint,cv::Point3f& camPoint,const cv::Mat depthImg,const cv::Mat camParm)
{
	float zc = (float)depthImg.at<uint16_t>(piexlPoint.x, piexlPoint.y);
	bl::piexl2Cam(piexlPoint,camPoint, zc, camParm);

}

void eye2handCal()
{

	if (MODE==2)
	{
		fout.close();
		std::cout << "共获取 " << saveImgNum << " 张图片\n 开始手眼标定" << std::endl;
		cv::Mat t;
	//	MyCalibration::eye2HandCalibration(EH_CAL_IMG_PATH_TXT, EH_CAL_ROBOT_PATH_TXT,t);
	}
	else
	{
		MODE = 2;
		saveImgNum = 0;
		fout.open(EH_CAL_FILE_PATH);
		std::cout << "开始获取标定图片，请保存标定板在图像内，按空格拍照\n";

	}


}

void test(BLAstraCamrea* cam)
{
	cv::Mat color = cam->getColor();
//	cv::imshow("test", color);

	std::vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	cv::Size board_size = cv::Size(11, 8);    /* 标定板上每行、列的角点数 */
	/*棋盘三维信息*/
	cv::Size square_size = cv::Size(5, 5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
		/* 提取角点 */
	if (0 == findChessboardCorners(color, board_size, image_points_buf))
	{
		std::cout << "can not find chessboard corners!\n"; //找不到角点
		return;
	}
	else
	{
		cv::Mat view_gray;
		cvtColor(color, view_gray, CV_RGB2GRAY);
		/* 亚像素精确化 */
		find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //对粗提取的角点进行精确化

	}
	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	std::vector<cv::Point3f> object_points;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			cv::Point3f realPoint;
			/* 假设标定板放在世界坐标系中z=0的平面上 */
			realPoint.x = i * square_size.width;
			realPoint.y = j * square_size.height;
			realPoint.z = 0;
			object_points.push_back(realPoint);
		}
	}


	std::vector<cv::Point3f> p3d;
	std::vector<cv::Point2f> p2d;

	p3d.push_back(object_points[0]);
	p3d.push_back(object_points[4]);
	p3d.push_back(object_points[33]);
	p3d.push_back(object_points[37]);


	p2d.push_back(image_points_buf[0]);
	p2d.push_back(image_points_buf[4]);
	p2d.push_back(image_points_buf[33]);
	p2d.push_back(image_points_buf[37]);

	cv::Mat r1, t1,h1;
	bl::getImgRT(color, r1, t1, cam->getRgbParamMat(), cam->getRgbDistCoeffs());
	bl::R_T2H(r1, t1, h1);
//	std::cout << "r1\n" << r1 << std::endl;
//	std::cout << "t1\n" << t1 << std::endl;
	std::cout << "h\n" << h1 << std::endl;



	cv::Mat R, T, H;

	bl::dealP3P(p3d, p2d, cam->getRgbParamMat(), cam->getRgbDistCoeffs(), R, T, bl::SOLVEPNP_P3P);
//	std::cout << R << std::endl;
//	std::cout << T << std::endl;

	bl::R_T2H(R, T, H);
	std::cout << "H: \n" << H << std::endl;


	int index = 87;

	cv::Point3f p = cam->piexl2cam(image_points_buf[index]);
	cv::Mat pc = (cv::Mat_<float>(4, 1) << p.x, p.y, p.z, 1);
	cv::Mat pw = (cv::Mat_<float>(4, 1) << object_points[index].x, object_points[index].y, object_points[index].z, 1);

	std::cout << "Pc\n" << p << endl;
	std::cout << "Pw\n" << object_points[index] << std::endl;

	std::cout << "world pos 1:" << H.inv() * pc<<endl;
	std::cout << "world pos 2:" << h1.inv() * pc << endl;


	std::cout << "cam pos 1:" << H * pw<<endl;
	std::cout << "cam pos 2:" << h1 * pw << endl;



}
