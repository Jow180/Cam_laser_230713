#include <Windows.h>
#include <vector>
#include<fstream>//csv檔案用
#include<io.h>// 計算檔案用的
#include <queue>// 計算檔案用的
#include <opencv2/core/types.hpp> // Point用
#include <opencv2/core/utility.hpp> // 計算時間(算fps)
#include <opencv2/features2d.hpp> // 特徵匹配
#include "opencv2/calib3d.hpp"// 計算透視矩陣用
#include <opencv2/imgproc.hpp>// 計算面積
#include <iomanip>// std::cout排版用


using namespace std;
using namespace cv;

class CamLaser
{
private:
	 
public:
	// 宣告變數
	static string End_code;

	
	/*---功能用小程式---*/
	/* 不一定每個都會被用到 */

	// 輸出vector<Point2f>為Csv
	static void Out_Data_Csv_2f(String path, vector<Point2f> Out_Point, int Point_number);
	// 輸出vector<Point3f>為Csv
	static void Out_Data_Csv_3f(String path, vector<Point3f> Out_Point, int Point_number);
	// 輸入的Csv檔案為vector<Point2f>
	static void Read_Data_Csv_2f(String path, vector<Point2f>& Out_Point);
	// 輸入的Csv檔案為vector<Point3f>
	static void Read_Data_Csv_3f(String path, vector<Point3f>& Out_Point);
	// 讀取CSV轉成Mat輸出
	static void Read_Csv2Mat(String path, Mat& Out_Mat);
	// 將Mat寫成csv檔案輸出
	static void Write_Csv2Mat(string path, Mat Input_Mat);
	// 接收四個點位置，輸出一串字串，接收Vector，輸出x1指令字串
	static void Xcode_ctrl(vector<Point2f>& Input_Point, String& Out_Xcode);
	// int 轉 string，將數值補足為四位數，缺項補0
	static String int2string_4ch(int input_num);
	// 分割字串
	static const vector<string> split(const string& str, const char& delimiter);
	// 將輸入的數值校正到0到2047以內
	static int cal_0to2048(int input_num);
	// 將數值補足0，輸出為 cv::String
	static String NameCreate(int ZerosNum, int Value);
	// 找出vector中的最大及最小值
	static void Max_P2f(vector<Point2f> input_vector, float out_x, float out_y);
	// 用於計算兩點之間的距離(Point2f)
	static float length_2P(Point2f p1, Point2f p2);
	// 計算輸入的矩形的最大值及他的索引值
	static void square_max(vector<vector<Point>>square_input, double& square_max, int& square_max_local);
	// 計算角度
	static double angle(Point pt1, Point pt2, Point pt0);
	// 用於找到影像中的矩形
	static void findSquares(const Mat& image, vector<vector<Point> >& squares, int N, int thresh);
	// 用於計算資料夾總數
	static int GetFileNum(const std::string& inPath);


	/*---計算用程式---*/
	// LSM 運算，一階級
	static void LineFitLeastSquares(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& r);
	// LSM 運算，二次方擬合
	static void LineFitLeastSquares_quadratic(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& c);
	// 行列式，三階
	static float Determinant_tri(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3);
	// LSM轉換，一階級
	static void LSM_Trans(Point2f& input_point, Point2f& out_point, double x_a, double x_b, double y_a, double y_b);
	// LSM，二次方轉換
	static void LSM_Trans_quadratic(Point2f& input_point, Point2f& out_point, double x_a, double x_b, double x_c, double y_a, double y_b, double y_c);
	// 特徵匹配，用於比對兩張影像，並回傳4個點位置
	static void Feature_ORB(int, static void*, Mat img1, Mat img2, vector<Point2f>& get_point4);

	/* -----核心程式----- */



	/*-----仍在測試，核心程式-----*/


	

};