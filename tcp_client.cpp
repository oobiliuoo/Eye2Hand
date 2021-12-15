#include "tcp_client.h"
#include "opencv2/opencv.hpp"
SOCKET sock;


	
int tcp_Robotic_Arm(int& run) {


	//��ʼ��DLL
	WSADATA wsaData;

	WSAStartup(MAKEWORD(2, 2), &wsaData);

	//���������������
	sockaddr_in sockAddr;
	memset(&sockAddr, 0, sizeof(sockAddr));//ÿ���ֽڶ���0���
	sockAddr.sin_family = PF_INET;//ʹ��ipv4��ַ

	//sockAddr.sin_addr.s_addr = inet_pton("192.168.0.253");//�����IP��ַ �����Ǳ�����ַ

	inet_pton(AF_INET, "192.168.0.144", &sockAddr.sin_addr.s_addr);   //  ���� foo.sin_addr.addr=inet_addr(ip);
	sockAddr.sin_port = htons(5150);//�˿�

	//inet_pton(AF_INET, "127.0.0.1", &sockAddr.sin_addr.s_addr);
	//sockAddr.sin_port = htons(2001);//�˿�

	initialization();
	//�����׽���
	sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (connect(sock, (SOCKADDR*)&sockAddr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "����������ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "���������ӳɹ���" << endl;
	}

	//thread thread_tcp_rob_cmd([&] {tcp_rob_cmd(); });

	//����,��������
	while (1)
	{
		if (run != 1)
			break;
		//���峤�ȱ���
		//int send_len = 0;
		//int recv_len = 0;
		//char bufSend[BUF_SIZE] = { 0 };//���ͻ�����
		//char bufRecv[BUF_SIZE] = { 0 };//���ܻ�����
		//char CRLF[] = "\r\n";

		//printf("Enter command:");
		//gets_s(bufSend);
		//ƴ�ӽ�����ʶ
		//strcat_s(bufSend, CRLF);
		//std::cout << "bufSend = "<<bufSend;
		//send_len = send(sock, bufSend, BUF_SIZE, 0);
		//send_len = tcp_send(bufSend);
		//if (send_len < 0) {
		//	cout << "����ʧ�ܣ�" << endl;
		//	break;
		//}

		//���ܷ��������ص�����
		//recv_len = tcp_recv(bufRecv);
		//if (recv_len < 0) {
		//	cout << "����ʧ�ܣ�" << endl;
		//	break;
		//}
		//else {
		//	//cout << "�������Ϣ:" << bufRecv << endl;
		//	//������յ�������
		//	printf("%s\n\n", bufRecv);
		////}

		//memset(bufSend, 0, BUF_SIZE);
		//memset(bufRecv, 0, BUF_SIZE);
		//Sleep(10);
	}


	//�ر��׽���
	closesocket(sock);
	//�ͷ�DLL��Դ
	WSACleanup();
	return 0;
}
void initialization() {
	//��ʼ���׽��ֿ�
	WORD w_req = MAKEWORD(2, 2);//�汾��
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "��ʼ���׽��ֿ�ʧ�ܣ�" << endl;
	}
	else {
		//cout << "��ʼ���׽��ֿ�ɹ���" << endl;
	}
	//���汾��
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "�׽��ֿ�汾�Ų�����" << endl;
		WSACleanup();
	}
	else {
		//cout << "�׽��ֿ�汾��ȷ��" << endl;
	}
	//������˵�ַ��Ϣ

}

int tcp_send(char* bufSend) {

	//���峤�ȱ���
	int send_len = 0;
	//char Send[BUF_SIZE] = { 0 };//���ͻ�����

	//gets_s(Send);

	send_len = send(sock, bufSend, BUF_SIZE, 0);
	if (send_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}

	return 0;
}

int tcp_recv(char* bufRecv) {

	//���峤�ȱ���
	int recv_len = 0;
	recv_len = recv(sock, bufRecv, BUF_SIZE, 0);
	if (recv_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	return 0;
}
