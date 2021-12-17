#pragma once
#ifndef keyhead
#define keyhead
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/types_c.h>
#include <fstream>
#include "BLFactory.h"
#include "BLCalibration.h"
#include "tcp_client.h"

using std::cout;
using std::endl;

// ��������
static const int ACTION_ESC = 27;
static const int ACTION_SPACE = 32;

static const int ACTION_A = 65;
static const int ACTION_B = 66;
static const int ACTION_C = 67;
static const int ACTION_D = 68;
static const int ACTION_E = 69;
static const int ACTION_H = 72;
static const int ACTION_Q = 81;



static std::string NOR_CAL_FILE_PATH = "../img/normal/"; // �������ͼƬ��·��
static std::string CAM_CAL_FILE_PATH = "../img/camCal/"; // ��������궨��ͼƬ��·��
static std::string EH_CAL_FILE_PATH = "../img/eyeHandCal/"; // ��������ͼƬ��·��
static std::string DEPTH_IMG_FILE_PATH = "../img/depth/"; // �������ͼƬ��·��

static std::string FILE_PATH[4] = { NOR_CAL_FILE_PATH,CAM_CAL_FILE_PATH ,EH_CAL_FILE_PATH, DEPTH_IMG_FILE_PATH };

static std::string IMG_TYPE = ".PNG"; // ����ͼƬ�ĸ�ʽ

// ����ͼƬ·�����ļ���
// ����궨�ļ�
static std::string CAM_CAL_IMG_PATH_TXT = "../img/camCalImageName.txt";
// ���۱궨ͼ���ļ�
static std::string EH_CAL_IMG_PATH_TXT = "../img/eye2HandCalImageName.txt";
// ���۱궨λ���ļ�
static std::string EH_CAL_ROBOT_PATH_TXT = "../img/eye2HandCalPos.txt";




// ����ͼƬ���±�
int saveImgNum = 0;
// ����ģʽѡ��
int MODE = 0;

std::ofstream fout;
cv::Size board_size = cv::Size(11, 8);    /* �궨����ÿ�С��еĽǵ��� */
cv::Mat cur_save_img;

const int X_ADD = 17;
const int Y_ADD = -19;
const int Z_ADD = 5;

int saveImg(cv::Mat colorImg, int mode, cv::Mat depthImg = cv::Mat::zeros(3, 3, CV_32FC1))
{
	cur_save_img = colorImg.clone();
	// �����ļ�
	std::ostringstream s;
	// �����������ļ����洢ͼƬ
	s << FILE_PATH[mode] << saveImgNum << IMG_TYPE;
	std::string imageName(s.str());


	if (mode == 0)
	{
		std::cout << "������...";
		cv::imwrite(imageName, cur_save_img);
	}
	else if (mode == 1)
	{
		std::cout << "����궨���ݻ�ȡ��...";
		if (!bl::checkImg(cur_save_img, board_size))
		{
			std::cout << "��Ƭ��ⲻ���ǵ㣬���������㣡";
			return 1;
		}
		cv::imwrite(imageName, cur_save_img);
		fout << imageName << std::endl;
	}
	else if (mode == 2)
	{
		std::cout << "���۱궨���ݻ�ȡ��...";
		if (!bl::checkImg(cur_save_img, board_size))
		{
			std::cout << "��Ƭ��ⲻ���ǵ㣬���������㣡";
			return 1;
		}
		cv::imwrite(imageName, cur_save_img);
		fout << imageName << std::endl;
	}
	else if (mode == 3)
	{
		std::cout << "������ݻ�ȡ��...";
		std::ostringstream s2;
		s2 << FILE_PATH[mode] << "depth_" << saveImgNum << IMG_TYPE;
		std::string imageName_d(s2.str());
		cv::imwrite(imageName_d, depthImg);

	}

	saveImgNum++;
	std::cout << "����ɹ�....  :" << saveImgNum << std::endl;
	return 0;

}

void camCalibration(cv::Mat camPram, cv::Mat camK)
{
	if (MODE == 1)
	{
		fout.close();
		std::cout << "����ȡ " << saveImgNum << " ��ͼƬ\n ��ʼ�궨" << std::endl;
		bl::cameraCalibration(CAM_CAL_IMG_PATH_TXT, camPram, camK);
		MODE == 0;
	}
	else
	{
		MODE = 1;
		saveImgNum = 0;
		fout.open(CAM_CAL_IMG_PATH_TXT);
		std::cout << "��ʼ��ȡ�궨ͼƬ���뱣��궨����ͼ���ڣ����ո�����\n";

	}

}

void piexl2Cam(cv::Point2d piexlPoint, cv::Point3d& camPoint, const cv::Mat depthImg, const cv::Mat camParm)
{
	float zc = (float)depthImg.at<uint16_t>(piexlPoint.y, piexlPoint.x);
	bl::piexl2Cam(piexlPoint, camPoint, zc, camParm);

}

void eye2handCal()
{

	if (MODE == 2)
	{
		fout.close();
		std::cout << "����ȡ " << saveImgNum << " ��ͼƬ\n ��ʼ���۱궨" << std::endl;
		//cv::Mat t;
		//	MyCalibration::eye2HandCalibration(EH_CAL_IMG_PATH_TXT, EH_CAL_ROBOT_PATH_TXT,t);
		saveImgNum = 0;
		MODE = 0;
	}
	else
	{
		MODE = 2;
		saveImgNum = 0;
		fout.open(EH_CAL_IMG_PATH_TXT);
		std::cout << "��ʼ��ȡ�궨ͼƬ���뱣��궨����ͼ���ڣ����ո�����\n";

	}


}


void showPos(BLAstraCamrea* cam)
{
	cv::Point2d p(300, 300);
	cv::Mat temp = cam->getColor();
	cv::circle(temp, p, 10, cv::Scalar(0, 0, 255));
	cv::Point3f p2;
	//	piexl2Cam(p, p2, depth, RgbParam);
	
	cv::Mat t = cam->getDepth();
	cv::normalize(t, t, 255, 1, cv::NORM_INF);
	t.convertTo(t, CV_8UC1);
	cv::circle(t, p, 10, cv::Scalar(255, 0, 255), -1);
	cv::imshow("t", t);

	p2 = cam->piexl2cam(p);
//	std::cout << "s p2" << p2<<std::endl;
	std::ostringstream s2;
	s2 << "P(" << p.x << "," << p.y << ")" << "\tdepth: x:" << p2.x << ",y:" << p2.y << ",z:" << p2.z;
	std::string str_pos(s2.str());
	cv::Point pos(0, temp.rows - 10);
	cv::putText(temp, str_pos, pos, cv::FONT_HERSHEY_DUPLEX, 0.6f, cv::Scalar(0, 0, 255));
	cv::imshow("temp", temp);


}


void close(BLAstraCamrea* cam, int& tcp_run, int& con_run) 
{
	// �رճ���
	cam->close();
	tcp_run = 0;
	con_run = 0;
//	cv::destroyAllWindows();
	std::cout << "��������....\n";

}

void control(BLAstraCamrea* cam, int& tcp_run, int& con_run)
{
	char bufSend[BUF_SIZE] = { 0 };//���ͻ�����
	char bufRecv[BUF_SIZE] = { 0 };//���ܻ�����
	cv::Mat_<double> ToolPose;
	cv::Mat_<double> TargetPose;
	std::vector<cv::Point3d> robotPoints;
	std::vector<cv::Point3d> cameraPoints;

	cv::Mat H_cam2base;
	bool i_flog = 0;

	while (con_run == 1)
	{


		printf("\nEnter command:");
		gets_s(bufSend);

		//	std::cout << "bufSend = "<<bufSend;

		switch (*bufSend)
		{

		case '0':
		{
			close(cam, tcp_run, con_run);
			std::cout << "cmd close....";
			break;
		}

		case '1':
		{

			// ��ȡ��ǰ��Ƭ
			if (!i_flog)
			{
				eye2handCal();
				i_flog = 1;
			}

			if (0 != saveImg(cam->getColor(), 2))
			{
				break;
			}


			// ��ȡ������λ��
		   /* memset(bufSend, 0, BUF_SIZE);
			memset(bufRecv, 0, BUF_SIZE);*/
			bufSend[0] = 'A';
			tcp_send(bufSend);
			int ok = tcp_recv(bufRecv);
			if (ok < 0) break;
			printf("%s\n\n", bufRecv);
			//����λ��
			ToolPose = bl::analyzePose(bufRecv, sizeof(bufRecv), ToolPose);


			break;
		}

		case '2':
		{
			i_flog = 0;
			// ���������λ��
		//	bl::writeRobotPos(ToolPose, EH_CAL_ROBOT_PATH_TXT);
			// �޸����״̬
		//	eye2handCal();
			// ��ʼ���۱궨
			cv::Mat H = cv::Mat(4, 4, CV_64F, cv::Scalar(0));
			bl::hand2eyeCalibration(EH_CAL_IMG_PATH_TXT, EH_CAL_ROBOT_PATH_TXT, H, cam->getRgbParamMat(), cam->getRgbDistCoeffs());
			H_cam2base = H.clone();
			std::cout << "H:" << H << std::endl;

			break;
		}

		case '3':
		{
			// ��ȡͼ���еĵ�
			std::vector<cv::Point3d> camPoints = cam->getCamPoints();
			cv::Point3d p(-19, 94, 295);
			if(camPoints.size()!=0)
				p = camPoints[0];
			//
			cv::Mat pMat = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1);
			std::cout << "\n pMat:\n" << pMat << std::endl;
			// ���Ӧ�Ļ�������
			cv::Mat pMat2 = H_cam2base * pMat;
			cv::Point3d p2(pMat2.at<double>(0, 0), pMat2.at<double>(1, 0), pMat2.at<double>(2, 0));
			p2.x += X_ADD;
			p2.y += Y_ADD;
			p2.z += Z_ADD;
			std::cout << "world p: " << p2;


			memset(bufSend, 0, BUF_SIZE);
			sprintf_s(bufSend, "DA %3.3f %3.3f %3.3f", p2.x, p2.y, p2.z);
			tcp_send(bufSend);
			break;
		}
		case '4':
		{
			
			// ��ȡp
			bufSend[0] = 'A';
			tcp_send(bufSend);
			int ok = tcp_recv(bufRecv);
			if (ok < 0) break;
			printf("%s\n\n", bufRecv);
			//����λ��
			TargetPose = bl::analyzePose(bufRecv, sizeof(bufRecv), TargetPose);
			int rows = TargetPose.rows - 1;
			cv::Point3d tempP;
			tempP.x = TargetPose.at<double>(rows, 0);
			tempP.y = TargetPose.at<double>(rows, 1);
			tempP.z = TargetPose.at<double>(rows, 2);
			robotPoints.push_back(tempP);
			std::cout << "aa:" << tempP;
			break;
		}

		case '5':
		{
		
			cameraPoints = cam->getCamPoints();

			// ��������
			for (int i = 0; i < cameraPoints.size(); i++) 
			{
				cv::Mat pMat = (cv::Mat_<double>(4, 1) << cameraPoints[i].x, cameraPoints[i].y, cameraPoints[i].z, 1);
				// ���Ӧ�Ļ�������
				cv::Mat pMat2 = H_cam2base * pMat;
				cameraPoints[i].x = pMat2.at<double>(0, 0) + X_ADD;
				cameraPoints[i].y = pMat2.at<double>(1, 0) + Y_ADD;
				cameraPoints[i].z = pMat2.at<double>(2, 0) + Z_ADD;
			}

			cv::Mat R, t,H;
			bl::ICP(cameraPoints,robotPoints,R,t);
			bl::R_T2H(R, t, H);
			std::cout << "\nH:\n" << H << std::endl;
			break;
		}

		case '6':
		{
		
			if (0 != saveImg(cam->getColor(), 0))
			{
				break;
			}
			cv::Mat R, t, H;
			bl::getImgRT(cur_save_img, R, t, cam->getRgbParamMat(), cam->getRgbDistCoeffs());
			bl::R_T2H(R, t, H);
			std::cout << "\nH:\n" << H << std::endl;

			break;
		}


		default:
			tcp_send(bufSend);
			tcp_recv(bufRecv);
			printf("�յ���Ϣ: %s\n\n", bufRecv);
			break;
		}

		memset(bufSend, 0, BUF_SIZE);
		memset(bufRecv, 0, BUF_SIZE);

	}



}


void test(BLAstraCamrea* cam)
{
	cv::Mat color = cam->getColor();
	//	cv::imshow("test", color);

	std::vector<cv::Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	cv::Size board_size = cv::Size(11, 8);    /* �궨����ÿ�С��еĽǵ��� */
	/*������ά��Ϣ*/
	cv::Size square_size = cv::Size(5, 5);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
		/* ��ȡ�ǵ� */
	if (0 == findChessboardCorners(color, board_size, image_points_buf))
	{
		std::cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
		return;
	}
	else
	{
		cv::Mat view_gray;
		cvtColor(color, view_gray, CV_RGB2GRAY);
		/* �����ؾ�ȷ�� */
		find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��

	}
	/* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	std::vector<cv::Point3d> object_points;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			cv::Point3d realPoint;
			/* ����궨�������������ϵ��z=0��ƽ���� */
			realPoint.x = i * square_size.width;
			realPoint.y = j * square_size.height;
			realPoint.z = 0;
			object_points.push_back(realPoint);
		}
	}


	std::vector<cv::Point3d> p3d;
	std::vector<cv::Point2f> p2d;

	p3d.push_back(object_points[0]);
	p3d.push_back(object_points[4]);
	p3d.push_back(object_points[33]);
	p3d.push_back(object_points[37]);


	p2d.push_back(image_points_buf[0]);
	p2d.push_back(image_points_buf[4]);
	p2d.push_back(image_points_buf[33]);
	p2d.push_back(image_points_buf[37]);

	cv::Mat r1, t1, h1;
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

	std::cout << "world pos 1:" << H.inv() * pc << endl;
	std::cout << "world pos 2:" << h1.inv() * pc << endl;


	std::cout << "cam pos 1:" << H * pw << endl;
	std::cout << "cam pos 2:" << h1 * pw << endl;

}

#endif // !keyhead


