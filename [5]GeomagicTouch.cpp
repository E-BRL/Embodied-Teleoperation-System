#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <fcntl.h>
#include <fstream> 
#include <iostream>
#include <limits>
#include <netdb.h>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace std;


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

/*******************************************************************************
 RotationMatrix to EulerZYX
*******************************************************************************/
typedef struct {
	double m00, m01, m02, m03;
	double m10, m11, m12, m13;
	double m20, m21, m22, m23;
	double m30, m31, m32, m33;
} Matrix4x4;
typedef struct {
	double m00, m01, m02;
	double m10, m11, m12;
	double m20, m21, m22;
} Matrix3x3;

std::tuple<double, double> cancel_jump_eul1(double& eul1, double& old_eul1) {  //rad여야 함
    if (std::abs(eul1 - old_eul1) > 50 * M_PI / 180) {
        cout<<"------------------jump1-----------------------  "<<eul1* 180 / M_PI <<"  "<<old_eul1* 180 / M_PI<<endl;
        if (abs(abs(eul1)-abs(old_eul1))<25* M_PI / 180){
            cout<<"11111"<<endl;
            return std::make_tuple(-eul1, -eul1);
        }
        else{
            cout<<"EULER 111111 | ELSE"<<endl;
            return std::make_tuple(eul1, eul1);
        }
    } else {
        return std::make_tuple(eul1, eul1);
    }
}
std::tuple<double, double> cancel_jump_eul2(double& eul2, double& old_eul2) {  //rad여야 함
    if (std::abs(eul2 - old_eul2) > 50 * M_PI / 180) {
        cout<<"------------------jump2-----------------------"<<eul2* M_PI / 180 <<"  "<<old_eul2* M_PI / 180<<endl;
        if (abs(abs(eul2)-abs(old_eul2))<25* M_PI / 180){
            cout<<"11111"<<endl;
            return std::make_tuple(-eul2, -eul2);
        }
        else{
            if (eul2>=old_eul2){cout<<"22222"<<endl;return std::make_tuple(eul2-std::abs(eul2 - old_eul2), old_eul2);}
            else{cout<<"33333"<<endl;return std::make_tuple(eul2+std::abs(eul2 - old_eul2), old_eul2);}
        }
    } else {
        return std::make_tuple(eul2, eul2);
    }
}
std::tuple<double, double> cancel_jump_eul3(double& eul3, double& old_eul3) {  //rad여야 함
    if (eul3*old_eul3<0 and std::abs(eul3 - old_eul3) > 50 * M_PI / 180) {
        cout<<"------------------jump3-----------------------  "<<eul3* 180 / M_PI <<"  "<<old_eul3* 180 / M_PI<<endl;
        return std::make_tuple(-eul3, -eul3);
    } else {
        return std::make_tuple(eul3, eul3);
    }
}

std::array<double, 6>  r2e_ZX_with_jump(const Matrix3x3& R, double& old_eul1, double& old_eul2, double& old_eul3, int& k) {
    double eul1, eul2, eul3;
	std::array<double, 6> result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double tol = 0.1;
    if (k == 0) {
        eul1 = std::atan2(R.m10, R.m00);
        eul2 = 0.0;
        eul3 = std::atan2(R.m21, R.m22);
        result[0] = eul1 * 180.0 / M_PI;
        result[1] = eul2 * 180.0 / M_PI;
        result[2] = eul3 * 180.0 / M_PI;
        result[3] = eul1 * 180.0 / M_PI;
        result[4] = eul2 * 180.0 / M_PI;
        result[5] = eul3 * 180.0 / M_PI;
        return result;
    } else {
        eul1 = std::atan2(R.m10, R.m00);
        eul2 = 0.0;
        eul3 = std::atan2(R.m21, R.m22);
        std::tie(eul1, old_eul1) = cancel_jump_eul1(eul1, old_eul1);
        std::tie(eul3, old_eul3) = cancel_jump_eul3(eul3, old_eul3);
        result[0] = eul1 * 180.0 / M_PI;
        result[1] = eul2 * 180.0 / M_PI;
        result[2] = eul3 * 180.0 / M_PI;
        result[3] = eul1 * 180.0 / M_PI;
        result[4] = eul2 * 180.0 / M_PI;
        result[5] = eul3 * 180.0 / M_PI;
        return result;
    }
}


/*******************************************************************************
 Main function
 ******************************************************************************/
std::array<double, 3> phantom_value(double x, double y, double z, int sORf, const std::vector<std::vector<double>> tfref_np) {
    std::array<double, 3> result = {0.0, 0.0, 0.0};
    std::array<std::array<double, 3>, 3> R_URtoPh = {{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}};
    if (sORf == 1){
        std::array<double, 3> p = {x, y, z};
        std::array<double, 3> intermediate;
        std::array<std::array<double, 3>, 3> transform = {{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}};

        for (int i = 0; i < 3; ++i) {
            intermediate[i] = transform[i][0] * p[0] + transform[i][1] * p[1] + transform[i][2] * p[2];
        }
        for (int i = 0; i < 3; ++i) {
            result[i] = R_URtoPh[i][0] * intermediate[0] + R_URtoPh[i][1] * intermediate[1] + R_URtoPh[i][2] * intermediate[2];
        }
    } else if (sORf == 2) {
        std::array<std::array<double, 4>, 4> T = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 163}, {0, 0, 0, 1}}};
        std::array<std::array<double, 4>, 4> R_URtoPh = {{{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
        std::array<std::array<double, 4>, 4> Phantom_Base_ref = {{{0, 0, 1, 0}, {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0, 1}}};
        std::array<std::array<double, 4>, 4> tfref_np_2={{{tfref_np[0][0],tfref_np[0][1],tfref_np[0][2],tfref_np[0][3]},
                                                            {tfref_np[1][0],tfref_np[1][1],tfref_np[1][2],tfref_np[1][3]},
                                                            {tfref_np[2][0],tfref_np[2][1],tfref_np[2][2],tfref_np[2][3]},
                                                            {tfref_np[3][0],tfref_np[3][1],tfref_np[3][2],tfref_np[3][3]}}};

        Matrix4x4_2 temp1 = multiply(R_URtoPh, Phantom_Base_ref);
        Matrix4x4_2 temp2 = multiply(temp1, tfref_np_2);
        Matrix4x4_2 new_tfref = multiply(temp2, T);

        result[0] = new_tfref[0][3];
        result[1] = new_tfref[1][3];
        result[2] = new_tfref[2][3];
    }
    return result;
}

Matrix4x4 SpanDoubleArrayToMatrix4x4(const double* T) {
	Matrix4x4 result;
	result.m00 = T[0];
	result.m01 = T[1];
	result.m02 = T[2];
	result.m03 = T[3];

	result.m10 = T[4];
	result.m11 = T[5];
	result.m12 = T[6];
	result.m13 = T[7];

	result.m20 = T[8];
	result.m21 = T[9];
	result.m22 = T[10];
	result.m23 = T[11];

	result.m30 = T[12];
	result.m31 = T[13];
	result.m32 = T[14];
	result.m33 = T[15];

	return result;
}


double computeMatrixDifference(const Matrix3x3& mat1, const Matrix3x3& mat2) {
    double maxDiff = 0.0;
    
    double diff = std::abs(mat1.m00 - mat2.m00);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m01 - mat2.m01);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m02 - mat2.m02);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m10 - mat2.m10);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m11 - mat2.m11);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m12 - mat2.m12);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m20 - mat2.m20);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m21 - mat2.m21);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(mat1.m22 - mat2.m22);
    maxDiff = std::max(maxDiff, diff);

    return maxDiff;
}

double computeAngleDifference(hduVector3Dd jointAngles, hduVector3Dd jointAngles_old, hduVector3Dd gimbalAngles, hduVector3Dd gimbalAngles_old) {
    double maxDiff = 0.0;
    
    double diff = std::abs(jointAngles[0] - jointAngles_old[0]);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(jointAngles[1] - jointAngles_old[1]);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(jointAngles[2] - jointAngles_old[2]);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(gimbalAngles[0] - gimbalAngles_old[0]);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(gimbalAngles[1] - gimbalAngles_old[1]);
    maxDiff = std::max(maxDiff, diff);
    
    diff = std::abs(gimbalAngles[2] - gimbalAngles_old[2]);
    maxDiff = std::max(maxDiff, diff);

    return maxDiff;
}

int main(int argc, char* argv[]) {
	//  client
	char *serverIp = (char*)"NETWORK HOST"; 
	int port = 8080; 
	struct hostent* host = gethostbyname(serverIp); 
    sockaddr_in sendSockAddr;   
    bzero((char*)&sendSockAddr, sizeof(sendSockAddr)); 
    sendSockAddr.sin_family = AF_INET; 
    sendSockAddr.sin_addr.s_addr = 
        inet_addr(inet_ntoa(*(struct in_addr*)*host->h_addr_list));
    sendSockAddr.sin_port = htons(port);
    int clientSd = socket(AF_INET, SOCK_STREAM, 0);

	// 소켓을 논블로킹 모드로 설정
    int flags = fcntl(clientSd, F_GETFL, 0);
    fcntl(clientSd, F_SETFL, flags | O_NONBLOCK);


    //try to connect...
    int status = connect(clientSd,(sockaddr*) &sendSockAddr, sizeof(sendSockAddr));
    if (status < 0 && errno != EINPROGRESS)
    {
        cout<<"Error connecting to socket!"<<endl;
        return -1;
    }
    cout << "Connected to the server!" << endl;
    int bytesRead, bytesWritten = 0;

    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    while (!kbhit() || getch() != 27) {
	
        // Initialize UserData struct
        UserData userData;
        userData.position = hduVector3Dd(0, 0, 0);
        userData.eulerZYX = hduVector3Dd(0, 0, 0);
        userData.sORf = 0;
        userData.dynamixel_process = 0;
        userData.button = 0;
        userData.clutch = 0;

        userData.dynamixel_process = 1;
        string sORf;
        std::cout << "Enter the number (1: Stylus | 2: Finger): ";
        std::cin >> sORf;
            
        userData.button = 0;
        userData.clutch = 0;

        char msg[1500]; 
        hduVector3Dd position;
        hduVector3Dd gimbalAngles;
        hduVector3Dd jointAngles;
        hduVector3Dd gimbalAngles_old;
        hduVector3Dd jointAngles_old;
        HDlong encoders[6];
        hduVector3Dd reset;
        hduVector3Dd eulerZYX = {0,0,0};
        Matrix3x3 RM;
        Matrix3x3 RM_old;
        std::array<double, 3> position_send;
        std::array<double, 6> euler_send;

        std::array<double, 3> old_eul = {0.0, 0.0, 24.0};
        int k=0;

        std::vector<std::vector<double>> tfref_np;
        std::vector<std::vector<double>> tfref_np_old;
        std::vector<std::vector<double>> tfref_finger;
        std::vector<std::vector<double>> tfref_finger_old;

        
        while (!kbhit() || getch() != 27) {
            double T[16];
            hdBeginFrame(hHD);
            /* INKWELl Calibration */
            HDboolean inkwell;
            hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH,&inkwell);
            hdGetDoublev(HD_CURRENT_POSITION, position);
            hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbalAngles);
            hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);
            hdGetDoublev(HD_CURRENT_TRANSFORM, T);
            if (inkwell){std::cout<<" "<<std::endl;}
            else{std::cout<<"INKWELL"<<std::endl; hdUpdateCalibration(HD_CALIBRATION_INKWELL); jointAngles_old=jointAngles; gimbalAngles_old=gimbalAngles;}
            /* Get the current position of the device. */
            hdGetLongv(HD_CURRENT_ENCODER_VALUES, encoders);
            hdGetDoublev(HD_CALIBRATION_ENCODER_RESET, reset);
            hdEndFrame(hHD);
            
            if (k==0) {
                jointAngles_old=jointAngles;
                gimbalAngles_old=gimbalAngles;
            } else{
                double threshold=1.0;
                double difference = computeAngleDifference(jointAngles, jointAngles_old, gimbalAngles, gimbalAngles_old);
                std::cout<<"difference: "<<difference<<std::endl;
                if (difference>=threshold){
                    jointAngles=jointAngles_old;
                    gimbalAngles=gimbalAngles_old;
                }
                else{
                jointAngles_old=jointAngles;
                gimbalAngles_old=gimbalAngles;
                }
            }

            RM.m00=(-((-sin(jointAngles[0])*sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) + sin(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[0]) - sin(gimbalAngles[0])*cos(jointAngles[0]))*sin(gimbalAngles[1]) + (-sin(jointAngles[0])*sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin(jointAngles[0])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*cos(gimbalAngles[1]))*sin(gimbalAngles[2]) + ((-sin(jointAngles[0])*sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) + sin(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*sin(gimbalAngles[0]) + cos(gimbalAngles[0])*cos(jointAngles[0]))*cos(gimbalAngles[2]);
            RM.m01=(-((-sin(jointAngles[0])*sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) + sin(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[0]) - sin(gimbalAngles[0])*cos(jointAngles[0]))*sin(gimbalAngles[1]) + (-sin(jointAngles[0])*sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin(jointAngles[0])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*cos(gimbalAngles[1]))*cos(gimbalAngles[2]) - ((-sin(jointAngles[0])*sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) + sin(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*sin(gimbalAngles[0]) + cos(gimbalAngles[0])*cos(jointAngles[0]))*sin(gimbalAngles[2]);
            RM.m02=-((-sin(jointAngles[0])*sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) + sin(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[0]) - sin(gimbalAngles[0])*cos(jointAngles[0]))*cos(gimbalAngles[1]) - (-sin(jointAngles[0])*sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin(jointAngles[0])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*sin(gimbalAngles[1]);
            RM.m10=((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) - cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[1]) - (-sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*sin(gimbalAngles[1])*cos(gimbalAngles[0]))*sin(gimbalAngles[2]) + (-sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*sin(gimbalAngles[0])*cos(gimbalAngles[2]);
            RM.m11=((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) - cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[1]) - (-sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*sin(gimbalAngles[1])*cos(gimbalAngles[0]))*cos(gimbalAngles[2]) - (-sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*sin(gimbalAngles[0])*sin(gimbalAngles[2]);
            RM.m12=-(sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1])) - cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*sin(gimbalAngles[1]) - (-sin(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])) - sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[1]))*cos(gimbalAngles[0])*cos(gimbalAngles[1]);
            RM.m20=(-((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0]) - cos(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[0]) - sin(gimbalAngles[0])*sin(jointAngles[0]))*sin(gimbalAngles[1]) + (sin(jointAngles[1])*cos(jointAngles[0])*cos((jointAngles[2]-jointAngles[1])) + sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0])*cos(jointAngles[1]))*cos(gimbalAngles[1]))*sin(gimbalAngles[2]) + ((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0]) - cos(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*sin(gimbalAngles[0]) + sin(jointAngles[0])*cos(gimbalAngles[0]))*cos(gimbalAngles[2]);
            RM.m21=(-((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0]) - cos(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[0]) - sin(gimbalAngles[0])*sin(jointAngles[0]))*sin(gimbalAngles[1]) + (sin(jointAngles[1])*cos(jointAngles[0])*cos((jointAngles[2]-jointAngles[1])) + sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0])*cos(jointAngles[1]))*cos(gimbalAngles[1]))*cos(gimbalAngles[2]) - ((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0]) - cos(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*sin(gimbalAngles[0]) + sin(jointAngles[0])*cos(gimbalAngles[0]))*sin(gimbalAngles[2]);
            RM.m22=-((sin(jointAngles[1])*sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0]) - cos(jointAngles[0])*cos(jointAngles[1])*cos((jointAngles[2]-jointAngles[1])))*cos(gimbalAngles[0]) - sin(gimbalAngles[0])*sin(jointAngles[0]))*cos(gimbalAngles[1]) - (sin(jointAngles[1])*cos(jointAngles[0])*cos((jointAngles[2]-jointAngles[1])) + sin((jointAngles[2]-jointAngles[1]))*cos(jointAngles[0])*cos(jointAngles[1]))*sin(gimbalAngles[1]);// euler_send=r2e_ZYX_new(RM, k, old_eul[0],old_eul[1],old_eul[2]);
            tfref_np = {
                    { RM.m00, RM.m01, RM.m02, position[0] },
                    { RM.m10, RM.m11, RM.m12, position[1] },
                    { RM.m20, RM.m21, RM.m22, position[2] },
                    { T[12], T[13], T[14], T[15] }
                };
            tfref_finger={
                    { T[0], T[1], T[2], position[0] },
                    { T[4], T[5], T[6], position[1] },
                    { T[7], T[8], T[10], position[2] },
                    { T[12], T[13], T[14], T[15] }};

            
            if (stoi(sORf)==1){
                position_send=phantom_value(position[0], position[1], position[2], stoi(sORf), tfref_np);}
            else if (stoi(sORf)==2){
                position_send=phantom_value(position[0], position[1], position[2], stoi(sORf), tfref_finger);
            }
            euler_send= r2e_ZX_with_jump(RM, old_eul[0],old_eul[1],old_eul[2], k);

            k++;
            old_eul[0]=euler_send[3]*M_PI/180;
            old_eul[1]=euler_send[4]*M_PI/180;
            old_eul[2]=euler_send[5]*M_PI/180;

            userData.position = position;
            userData.eulerZYX = eulerZYX;

            std::string stringdata;

            stringdata="999,"+std::to_string(position_send[0])+","+std::to_string(position_send[1])+","+std::to_string(position_send[2])+","+std::to_string(euler_send[2])+","+std::to_string(euler_send[1])+","+std::to_string(-euler_send[0])+","+sORf+","+"666\n";
            memset(&msg, 0, sizeof(msg));
            strcpy(msg, stringdata.c_str());
            std::cout<<stringdata<<std::endl;
            
            ssize_t result = send(clientSd, (char*)&msg, strlen(msg), 0);
            if (result == -1)
            {
                if (errno == EWOULDBLOCK)
                {
                    // 소켓이 아직 전송할 준비가 되지 않은 경우
                    cout << "Socket is not ready to send data yet." << endl;
                    usleep(100000);
                    continue;
                }
                else
                {
                    // 전송 오류 발생
                    cerr << "Failed to send data. Error: " << strerror(errno) << endl;
                    break;
                }
            }
            bytesWritten += result;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
        // hdEndFrame(hHD);
    hdStopScheduler();
    hdDisableDevice(hHD);
    close(clientSd);

    return 0;
}


/*******************************************************************************
 Error function
 ******************************************************************************/

 void error_handling(const char *message) {
	fputs(message, stderr);
	fputc('\n', stderr);
}
