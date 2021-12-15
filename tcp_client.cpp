#include "tcp_client.h"
#include "opencv2/opencv.hpp"
SOCKET sock;


	
int tcp_Robotic_Arm(int& run) {


	//初始化DLL
	WSADATA wsaData;

	WSAStartup(MAKEWORD(2, 2), &wsaData);

	//向服务器发起请求
	sockaddr_in sockAddr;
	memset(&sockAddr, 0, sizeof(sockAddr));//每个字节都用0填充
	sockAddr.sin_family = PF_INET;//使用ipv4地址

	//sockAddr.sin_addr.s_addr = inet_pton("192.168.0.253");//具体的IP地址 这里是本机地址

	inet_pton(AF_INET, "192.168.0.144", &sockAddr.sin_addr.s_addr);   //  代替 foo.sin_addr.addr=inet_addr(ip);
	sockAddr.sin_port = htons(5150);//端口

	//inet_pton(AF_INET, "127.0.0.1", &sockAddr.sin_addr.s_addr);
	//sockAddr.sin_port = htons(2001);//端口

	initialization();
	//创建套接字
	sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (connect(sock, (SOCKADDR*)&sockAddr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}

	//thread thread_tcp_rob_cmd([&] {tcp_rob_cmd(); });

	//发送,接收数据
	while (1)
	{
		if (run != 1)
			break;
		//定义长度变量
		//int send_len = 0;
		//int recv_len = 0;
		//char bufSend[BUF_SIZE] = { 0 };//发送缓冲区
		//char bufRecv[BUF_SIZE] = { 0 };//接受缓冲区
		//char CRLF[] = "\r\n";

		//printf("Enter command:");
		//gets_s(bufSend);
		//拼接结束标识
		//strcat_s(bufSend, CRLF);
		//std::cout << "bufSend = "<<bufSend;
		//send_len = send(sock, bufSend, BUF_SIZE, 0);
		//send_len = tcp_send(bufSend);
		//if (send_len < 0) {
		//	cout << "发送失败！" << endl;
		//	break;
		//}

		//接受服务器传回的数据
		//recv_len = tcp_recv(bufRecv);
		//if (recv_len < 0) {
		//	cout << "接受失败！" << endl;
		//	break;
		//}
		//else {
		//	//cout << "服务端信息:" << bufRecv << endl;
		//	//输出接收到的数据
		//	printf("%s\n\n", bufRecv);
		////}

		//memset(bufSend, 0, BUF_SIZE);
		//memset(bufRecv, 0, BUF_SIZE);
		//Sleep(10);
	}


	//关闭套接字
	closesocket(sock);
	//释放DLL资源
	WSACleanup();
	return 0;
}
void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		//cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		//cout << "套接字库版本正确！" << endl;
	}
	//填充服务端地址信息

}

int tcp_send(char* bufSend) {

	//定义长度变量
	int send_len = 0;
	//char Send[BUF_SIZE] = { 0 };//发送缓冲区

	//gets_s(Send);

	send_len = send(sock, bufSend, BUF_SIZE, 0);
	if (send_len < 0) {
		cout << "发送失败！" << endl;
	}

	return 0;
}

int tcp_recv(char* bufRecv) {

	//定义长度变量
	int recv_len = 0;
	recv_len = recv(sock, bufRecv, BUF_SIZE, 0);
	if (recv_len < 0) {
		cout << "接受失败！" << endl;
	}
	return 0;
}
