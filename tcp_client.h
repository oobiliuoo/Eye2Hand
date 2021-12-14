#pragma once

//头文件主体
#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib,"ws2_32.lib")//加载ws2_32.dll
#define BUF_SIZE 100
#define PI acos(-1)

using namespace std;

int tcp_Robotic_Arm(int&);
void initialization();

int tcp_send(char* bufSend);
int tcp_recv(char* bufRecv);

