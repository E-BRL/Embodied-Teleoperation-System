#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <exception>
#include <stdexcept>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <ctime>
#include <thread>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <errno.h>
#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <mutex>
using namespace std;

//=======================server=========================================
#define PACKET_SIZE 1024
#define serverPort 8080
list<int> clnt_list;
struct ClientInfo { // 클라이언트 정보 필드
	int socket;
	sockaddr_in clientAddress;
};
vector<pthread_t> clientThreads;    // 스레드들을 관리할 vector 선언
vector<ClientInfo*> clientPool;     // 클라이언트 정보 구조체를 관리할 vector 선언

int server_socket;                  // 서버 소켓
void* handle_client(void* data);    // 클라이언트 관리 thread 함수
void* client_accept(void* data);    // 클라이언트 연결 thread 함수
//==========================phantom==============================================
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <vector>
#include <algorithm> 
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

int getch()
{
    int c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr);           // 현재 터미널 설정 읽음
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);         // CANONICAL과 ECHO 끔
    newattr.c_cc[VMIN] = 1;                      // 최소 입력 문자 수를 1로 설정
    newattr.c_cc[VTIME] = 0;                     // 최소 읽기 대기 시간을 0으로 설정
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // 터미널에 설정 입력
    c = getchar();                               // 키보드 입력 읽음
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // 원래의 설정으로 복구
    return c;
}

#define M_PI 3.14159265358979323846

struct UserData {
	int dynamixel_process;
	int sORf;
	int button;
	int clutch;
	hduVector3Dd position;
	hduVector3Dd eulerZYX;
};

bool exit_flag = false;
void error_handling(const char *message);

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
//==========================main==============================================
int main() {

    char answer;
    std::cout << "Do you want to start server? (Y/N): ";
    std::cin >> answer;
    string sORf;
    std::cout << "Enter the number (1: Stylus | 2: Finger): ";
    std::cin >> sORf;

    //****************server**************
    int optvalue = 1;
    struct sockaddr_in serverAddr;

    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        cerr << "서버 소켓 생성 실패" << endl;
        perror("socket");
        
        return 1;
    }
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &optvalue, sizeof(optvalue));
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(serverPort);
    int bindResult = bind(server_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    if (bindResult < 0) {
        cerr << "바인딩 실패" << endl;
        perror("bind");

        close(server_socket);
        return 1;
    }
    listen(server_socket, 5); // 5는 백로그 크기 = 동시에 처리 가능한 연결 요청의 최대 수
    cout << "서버가 "<< serverPort << " 포트에서 대기 중..." << endl;
    pthread_t thread_listen;
    if (pthread_create(&thread_listen, nullptr, client_accept, nullptr) != 0) {
        cerr << "스레드 생성 실패" << endl;
        return 1;
    }
    // 서버, 클라이언트 소켓의 연결을 해제
    pthread_join(thread_listen, nullptr);
    /* 메모리 할당 해제 */
    for (ClientInfo* c : clientPool){
        cout << "clientPool 메모리 할당 해제: " << c->socket << endl;
        delete c;
    }
    for(pthread_t hThread : clientThreads){
        cout << "clientThreads 메모리 할당 해제: " << hThread << endl;
        pthread_detach(hThread);
    }
    /* 메모리 할당 해제 */

    close(server_socket);
    return 0;
}

void* client_accept(void* data){    // 클라이언트 연결 thread 함수
    while(1) {
        int client_socket;
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);

        client_socket = accept(server_socket, (struct sockaddr*)&clientAddr, &clientAddrLen);
        if (client_socket == -1) {
            // 클라이언트 연결 실패 시 소켓을 닫고 다음 while 회차로 넘어감
            std::cerr << "클라이언트 연결 수락 실패" << std::endl;
            perror("client");
            close(client_socket);
            continue;
        }

        // 클라이언트 소켓을 논블로킹 모드로 설정
        int flags = fcntl(client_socket, F_GETFL, 0);
        fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);


        cout << "클라이언트와 연결이 완료되었습니다" << endl;
        cout << "Client IP: " << inet_ntoa(clientAddr.sin_addr) << endl;
		    cout << "Port: " << ntohs(clientAddr.sin_port) << endl;
        if (clnt_list.size()==0){
            clnt_list.push_back(clientAddr.sin_port);
        }
        else if (clnt_list.size()==1 || clnt_list.front()!=clientAddr.sin_port){
            clnt_list.push_back(clientAddr.sin_port);
        }
        
        // 클라이언트의 데이터를 담은 구조체를 생성하고
        // 연결된 클라이언트의 구조체 정보를 관리하기 위해 vector를 사용
        ClientInfo* clientData = new ClientInfo;
        clientData->socket = client_socket;
        clientData->clientAddress = clientAddr;

        clientPool.push_back(clientData);

        // 클라이언트를 처리할 스레드 생성
        pthread_t thread_client;
        pthread_create(&thread_client, nullptr, handle_client, (void*)clientData);
        clientThreads.push_back(thread_client);
    }
    return nullptr;
}
mutex clientPoolMutex;

void* handle_client(void* data){
    ClientInfo* clientData = static_cast<ClientInfo*>(data);
    int clientSocket = clientData->socket;
    while (clientPool[3] ==nullptr || clientPool[2] ==nullptr || clientPool[1] ==nullptr || clientPool[0] ==nullptr){
      cout<<"not yet"<<endl;
    }

      //공통
      string totalmsg;

      // Ard
      int clientSocket_Ard = clientPool[2]->socket;
      sockaddr_in clientAddr_Ard = clientPool[2]->clientAddress;
      char buffer_Ard[130];
      string accumulatedData_Ard;
      int recvSize_Ard;
      string line_Ard="";

      //Pha
      int clientSocket_Pha = clientPool[3]->socket;
      sockaddr_in clientAddr_Pha = clientPool[3]->clientAddress;
      char buffer_Pha[130];
      string accumulatedData_Pha;
      int recvSize_Pha;
      string line_Pha="";


    while(1){
        recvSize_Ard = recv(clientSocket_Ard, buffer_Ard, sizeof(buffer_Ard) - 1, 0);
        if (recvSize_Ard == -1) {
              if (errno == EWOULDBLOCK || errno == EAGAIN) {
                  usleep(100000);
                  continue;
              } else {
                  std::cerr << "데이터 수신 오류" << std::endl;
                  break;
              }
          } else if (recvSize_Ard == 0) {
              std::cout << "클라이언트 연결이 종료되었습니다: " << inet_ntoa(clientAddr_Ard.sin_addr) << ":" << ntohs(clientAddr_Ard.sin_port) << std::endl;
              break;
          }
          buffer_Ard[recvSize_Ard] = '\0'; // Null-terminate the received data
          accumulatedData_Ard += buffer_Ard; // Append the received data to the accumulated data
          size_t startPos_Ard;
          size_t endPos_Ard;
        if (startPos_Ard = accumulatedData_Ard.find("999") != string::npos){
          startPos_Ard = accumulatedData_Ard.find("999");
          string accum2_Ard = accumulatedData_Ard.substr(startPos_Ard);
          if (endPos_Ard = accum2_Ard.find('\n') != string::npos){
            endPos_Ard = accum2_Ard.find('\n');
            line_Ard = accum2_Ard.substr(0,endPos_Ard);
            {lock_guard<mutex> lock(clientPoolMutex);}
            }
          }
          accumulatedData_Ard="";
      recvSize_Pha = recv(clientSocket_Pha, buffer_Pha, sizeof(buffer_Pha) - 1, 0);
        if (recvSize_Pha == -1) {
              if (errno == EWOULDBLOCK || errno == EAGAIN) {
                  usleep(100000);
                  continue;
              } else {
                  std::cerr << "데이터 수신 오류" << std::endl;
                  break;
              }
          } else if (recvSize_Pha == 0) {
              std::cout << "클라이언트 연결이 종료되었습니다: " << inet_ntoa(clientAddr_Pha.sin_addr) << ":" << ntohs(clientAddr_Pha.sin_port) << std::endl;
              break;
          }
          buffer_Pha[recvSize_Pha] = '\0'; // Null-terminate the received data
          accumulatedData_Pha += buffer_Pha; // Append the received data to the accumulated data
          size_t startPos_Pha;
          size_t endPos_Pha;
        if (startPos_Pha = accumulatedData_Pha.find("999") != string::npos){
          startPos_Pha = accumulatedData_Pha.find("999");
          string accum2_Pha = accumulatedData_Pha.substr(startPos_Pha);
          if (endPos_Pha = accum2_Pha.find('\n') != string::npos){
            endPos_Pha = accum2_Pha.find('\n');
            line_Pha = accum2_Pha.substr(0,endPos_Pha);
            {lock_guard<mutex> lock(clientPoolMutex);}
          }
        }
        accumulatedData_Pha="";
      {
        lock_guard<mutex> lock(clientPoolMutex);
        int firstClientSocket = clientPool.at(0)->socket;
        int SecondClientSocket = clientPool.at(1)->socket;
        totalmsg=line_Ard+","+line_Pha;
        cout<<totalmsg<<endl;
        send(firstClientSocket, totalmsg.c_str(), totalmsg.size(), 0);
        send(SecondClientSocket, totalmsg.c_str(), totalmsg.size(), 0);
      }
    }

    for (size_t i = 0; i < clientPool.size(); ++i) {
		if (clientPool[i]->socket == clientSocket) {
			// 클라이언트 정보 삭제
			cout << "클라이언트와 연결이 종료되었습니다: " << inet_ntoa(clientPool[i]->clientAddress.sin_addr) << ":" << ntohs(clientPool[i]->clientAddress.sin_port) << endl;
			clientPool.erase(clientPool.begin() + i);
			break;
		}
	}
    delete clientData;
    close(clientSocket);
    return nullptr;
}
