#include <Windows.h>
#include <vector>
#include<fstream>//csv�ɮץ�
#include<io.h>// �p���ɮץΪ�
#include <queue>// �p���ɮץΪ�
#include <opencv2/core/types.hpp> // Point��
#include <opencv2/core/utility.hpp> // �p��ɶ�(��fps)
#include <opencv2/features2d.hpp> // �S�x�ǰt
#include "opencv2/calib3d.hpp"// �p��z���x�}��
#include <opencv2/imgproc.hpp>// �p�⭱�n
#include <iomanip>// std::cout�ƪ���


using namespace std;
using namespace cv;

class CamLaser
{
private:
	 
public:
	// �ŧi�ܼ�
	static string End_code;

	
	/*---�\��Τp�{��---*/
	/* ���@�w�C�ӳ��|�Q�Ψ� */

	// ��Xvector<Point2f>��Csv
	static void Out_Data_Csv_2f(String path, vector<Point2f> Out_Point, int Point_number);
	// ��Xvector<Point3f>��Csv
	static void Out_Data_Csv_3f(String path, vector<Point3f> Out_Point, int Point_number);
	// ��J��Csv�ɮ׬�vector<Point2f>
	static void Read_Data_Csv_2f(String path, vector<Point2f>& Out_Point);
	// ��J��Csv�ɮ׬�vector<Point3f>
	static void Read_Data_Csv_3f(String path, vector<Point3f>& Out_Point);
	// Ū��CSV�নMat��X
	static void Read_Csv2Mat(String path, Mat& Out_Mat);
	// �NMat�g��csv�ɮ׿�X
	static void Write_Csv2Mat(string path, Mat Input_Mat);
	// �����|���I��m�A��X�@��r��A����Vector�A��Xx1���O�r��
	static void Xcode_ctrl(vector<Point2f>& Input_Point, String& Out_Xcode);
	// int �� string�A�N�ƭȸɨ����|��ơA�ʶ���0
	static String int2string_4ch(int input_num);
	// ���Φr��
	static const vector<string> split(const string& str, const char& delimiter);
	// �N��J���ƭȮե���0��2047�H��
	static int cal_0to2048(int input_num);
	// �N�ƭȸɨ�0�A��X�� cv::String
	static String NameCreate(int ZerosNum, int Value);
	// ��Xvector�����̤j�γ̤p��
	static void Max_P2f(vector<Point2f> input_vector, float out_x, float out_y);
	// �Ω�p����I�������Z��(Point2f)
	static float length_2P(Point2f p1, Point2f p2);
	// �p���J���x�Ϊ��̤j�ȤΥL�����ޭ�
	static void square_max(vector<vector<Point>>square_input, double& square_max, int& square_max_local);
	// �p�⨤��
	static double angle(Point pt1, Point pt2, Point pt0);
	// �Ω���v�������x��
	static void findSquares(const Mat& image, vector<vector<Point> >& squares, int N, int thresh);
	// �Ω�p���Ƨ��`��
	static int GetFileNum(const std::string& inPath);


	/*---�p��ε{��---*/
	// LSM �B��A�@����
	static void LineFitLeastSquares(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& r);
	// LSM �B��A�G�������X
	static void LineFitLeastSquares_quadratic(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& c);
	// ��C���A�T��
	static float Determinant_tri(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3);
	// LSM�ഫ�A�@����
	static void LSM_Trans(Point2f& input_point, Point2f& out_point, double x_a, double x_b, double y_a, double y_b);
	// LSM�A�G�����ഫ
	static void LSM_Trans_quadratic(Point2f& input_point, Point2f& out_point, double x_a, double x_b, double x_c, double y_a, double y_b, double y_c);
	// �S�x�ǰt�A�Ω����i�v���A�æ^��4���I��m
	static void Feature_ORB(int, static void*, Mat img1, Mat img2, vector<Point2f>& get_point4);

	/* -----�֤ߵ{��----- */



	/*-----���b���աA�֤ߵ{��-----*/


	

};