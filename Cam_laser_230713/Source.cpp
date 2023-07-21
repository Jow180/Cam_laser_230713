#include <iostream>
#include <opencv2/videoio.hpp>// camera用
#include <opencv2/highgui.hpp>// imshow、waitkey用
#include <librealsense2/rs.hpp>// realsense用
#include<io.h>// 建立資料夾用
#include "SerialPort.h"
#include "camlaser.h"

/*---快捷鍵功能---
// ctrl + m + l展開或縮小所有程式
// ctrl + k + c 註解
// ctrl + k + u取消註解
*/

/*---命名空間宣告---*/
using namespace std; // std標準函式
using namespace cv;  // OpenCV
using namespace rs2; // Realsense

// 類別宣告
CSerialPort mySerialPort;
CamLaser CL;

/*--- 參數宣告-- - */
String FileDate = "230721";//存檔日期與版本名字，可加上當日版本號碼，同時也是建立資料夾的名稱
String FilePath = "D:\\59.恩哲的研究\\221016laser校正與通訊整合測試\\";// 存檔路徑
int port_cam = 2;//相機的port編號
int apiID = cv::CAP_DSHOW;// 相機api ID，宣告比較不會找不到相機
int port_laser = 6;//雷射振鏡所連接的port的編號

// 校正用參數
int Points_Pro_Range = 2048;//雷射投影的轉換後係數解析度
int Points_R_Num = 4;//校正點列數量，請輸入4的倍數
int Points_C_Num = 4;//校正點行數量，請輸入4的倍數
int Points_Num_total = (Points_R_Num + 1) * (Points_C_Num + 1);

// 設定相機解析度
int Width = 1280;
int Height = 720;

/*--------副程式宣告-------*/
// 雷射振鏡連接用的Port口程式 
void PortOpen();

/*--------主程式宣告-------*/

/*-----編譯完畢-----*/
// 2D校正，雷射投影後拍照，紀錄影像
bool Cam_Laser_Calibration_v2();
// 2D校正，將拍攝完的影像班點偵測後，進行儲存
bool Cam_Laser_Calibration_v2_BOLB();
// 2D校正，將拍照完的數據LSM計算
bool Cam_CAL_LSM();
// 2D校正，將拍照完的數據LSM計算，二階級擬和
bool Cam_CAL_LSM_quadratic();
// 2D，重複性驗證
bool Cam_Laser_Calibration_repeatability();
// 2D，重複性驗證結果
bool Cam_Laser_repeatability_result();
// 2D，讀取patten圖片，並根據校正參數進行投影
int Feature_Match_Online_v5();

/*-----仍在測試-----*/
// 3D，用於製作理論點XML資料
int Cam_Laser_Calibration_stereo_v2();
// 3D 校正，拍照，並儲存深度資訊
int Cam_Laser_Calibration_stereo_v3();
// 3D 校正，將拍攝完的影像班點偵測後，儲存深度及影像座標等資訊
int Cam_Laser_Calibration_stereo_Blob_v3();

/*-------其他程式宣告-------*/
// ROI 矩形辨識，offline
int ROI_square_offline();
// ROI 矩形辨識，online
int ROI_square_online();
// XML 寫入
bool Write_XML();
// XML 讀取
bool Read_XML();
// 多圖特徵匹配，online
int Feature_Match_most_pic();

/*-------其他測試程式宣告-------*/
// 230720，將3D量測後的資料轉成csv檔案
bool Read_2_XML_230720();


int main() {
	//Cam_Laser_Calibration_v2();
	Cam_Laser_Calibration_stereo_v3();
	//Cam_Laser_Calibration_stereo_Blob_v3();
	//Read_2_XML_230720();
	return 0;
}

/* ---副程式--- */
void PortOpen() {
	/*std::cout << "Please insert your port number : " << endl;
	cin >> port_laser;*/

	if (!mySerialPort.InitPort(port_laser, 115200, 'N', 8, 1, EV_RXCHAR))
	{
		std::cout << "initPort fail !" << std::endl;
		PortOpen();
	}
	else
	{
		std::cout << "initPort success !" << std::endl;
	}
}

/* ---主程式--- */
bool Cam_Laser_Calibration_v2() {
	// 雷射投影後拍照，再進行Bolb Dection，如果掉點也沒關西
	// 230105 註解 : 採用直接關閉相機去拍照的迴圈控制
	//1.先打開相機確認校正板位置
	VideoCapture cap;//打開SerialPort 0 的相機

	//2.雷射投影及拍照 參數宣告
	vector<double> Points_R_Laser;
	vector<double> Points_C_Laser;
	vector<double> Points_R_Cam;
	vector<double> Points_C_Cam;
	vector<int> Points_Laser_Theory_ID(Points_Num_total, 0);
	vector<Point2f>Points_Laser_Theory;
	vector<Point2f>Points_Laser_Detect;
		// 傳送的點的容器
	int x1 = 0;
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	string code_out;
	char* code_out_ch;
	int Point_Numbers = 0;//用於紀錄總點數的起始值
	char read_ch;// 讀取確認碼用的
	string read_str;// 組合接收到的指令

	Mat frame;// 相機照片容器
	//圖片存檔位置
	String Savepath = FilePath;
	String Im_Savepath = Savepath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//CSV檔案存檔位置
	String Csv_Savepath = Savepath + FileDate + "\\";
	String Csv_Filename_Dectection = FileDate + "_detected.csv";
	String Csv_Filename_Theory = FileDate + "_theory.csv";

	// 先建立資料夾
	string Folder_Path = Savepath + FileDate;
	string command;
	command = "mkdir -p " + Folder_Path;// 以dos建立資料夾
	system(command.c_str());

	// 理論點
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}
	PortOpen();// 打開雷射振鏡的port口
	//mySerialPort.OpenListenThread();// 打開串聯通訊監聽
	Sleep(1 * 1000);
	cap.open(port_cam, apiID);// 打開相機
	if (!cap.isOpened()) {
		std::cout << "Can't open the camera, port number is " << port_cam << std::endl;
	}
	else {
		std::cout << "Camera is opened, port number is " << port_cam << std::endl;
		cap.set(cv::CAP_PROP_FRAME_WIDTH, Width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);
	}

	Sleep(2 * 1000);

	for (int i = 0; i < Points_Num_total; i++) {
		//控制雷射點到指定點位
		x1 = CL.cal_0to2048((int)Points_Laser_Theory[i].x);
		y1 = CL.cal_0to2048((int)Points_Laser_Theory[i].y);
		code_out = "01" + CL.int2string_4ch(x1) + CL.int2string_4ch(y1) + CL.int2string_4ch(x2) + \
			CL.int2string_4ch(y2) + CL.int2string_4ch(x3) + CL.int2string_4ch(y3) + CL.int2string_4ch(x4) + CL.int2string_4ch(y4);//太長，分行
		code_out_ch = (char*)code_out.data();

		for (int i = 0; i < 5; i++) {
			mySerialPort.WriteData(code_out_ch, 34);//傳送字串給Arduino
		}
		std::cout << "PC Send :" << endl;
		std::cout << code_out_ch << endl;
		std::cout << "send points : " << i << endl;
		Sleep(200);

		cap.read(frame);
		if (frame.empty() == 1) {
			std::cout << "圖片不存在，請確認圖片" << endl;// 檢查有拍到照片
			//return 1;
		}
		else {
			std::cout << "test拍照，OK!" << endl;
		}
		UINT BytesInQue = mySerialPort.GetBytesInCOM();// 確認緩衝區有無資料
		while (1) {
			BytesInQue = mySerialPort.GetBytesInCOM();// 重新確認緩衝區有無資料
			mySerialPort.ReadChar(read_ch);// 重新讀取資料
			if (BytesInQue != 0 && read_ch == 'N') {
				std::cout << "接收到了 : " << read_ch << endl;
				cap.read(frame);
				if (frame.empty() == 1) {
					std::cout << "圖片不存在，請確認圖片" << endl;// 檢查有拍到照片
				}
				else {
					std::cout << "已拍照" << endl;
				}
				imshow("Frame", frame);
				waitKey(1);
				Im_Name = CL.NameCreate(4, i);
				Im_Filename_All = Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype;
				imwrite(Im_Filename_All, frame);
				std::cout << "已存圖，第 " << i << "張" << endl;
				break;
			}
		}
	}
	// 關閉雷射
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);
	// 釋放容器
	cap.release();
	//輸出CSV
	CL.Out_Data_Csv_2f(Csv_Savepath + Csv_Filename_Theory, Points_Laser_Theory, Points_Num_total);//理論值
	std::cout << "Finsih!" << endl;
	return true;
}
bool Cam_Laser_Calibration_v2_BOLB() {
	// 連續讀取圖片，劃分ROI區域，Bolb Dection，將辨識到的點進行儲存
	//圖片存檔位置
	String Im_Savepath = FilePath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//CSV檔案存檔位置
	String Csv_Savepath = FilePath + FileDate + "\\";
	String Csv_Filename_Dectection = FileDate + "_detected.csv";
	String Csv_Filename_Theory = FileDate + "_theory.csv";
	String Csv_Filename_Theory_v2 = FileDate + "_theory_v2.csv";

	// 宣告變數
	Mat frame;
	Mat im_Crop; // ROI切割的圖片
	Mat im_Gray;
	Mat im_HSV;
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	vector<Point2f> Point_Theory_v2;//檢測完後的可用理論點
	vector<Point2f>	Points_Laser_Detect;
	int get_number = 0; // 找到的點數量
	int lose_number = 0;// 遺失的點數量

	//ROI參數
	Rect2d r;//r為Roi的值
	//Rect2d r2;
	bool showCrosshair = false;
	bool fromCenter = false;
	int ROI_x, ROI_y, ROI_width, ROI_height;//ROI的各項值儲存

	// Bolb Dection 各項參數
	SimpleBlobDetector::Params params;

	// Blob Color // 設定顏色
	//params.blobColor = 150;

	// Change thresholds	//探索的像素上下限
	params.minThreshold = 80;
	params.maxThreshold = 120;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1;

	// Filter by Circularity //圓度
	params.filterByCircularity = false;
	params.minCircularity = 0.5;

	// Filter by Convexity	//凸度
	params.filterByConvexity = false;
	params.minConvexity = 0.01;

	// Filter by Inertia	//冠量比
	params.filterByInertia = false;
	params.minInertiaRatio = 0.3;

	// Storage for blobs
	vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	//藉由讀取理論點數量，得到對應的照片數
	CL.Read_Data_Csv_2f(Csv_Savepath + Csv_Filename_Theory, Point_Theory);

	// 先顯示第一張照片，選取ROI區域
	Im_Name = "0000";// 起始照片編號
	frame = imread(Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype);
	r = cv::selectROI("請選擇ROI，ESC 結束", frame, showCrosshair, fromCenter);//r為Roi的值
	ROI_x = r.x;
	ROI_y = r.y;
	ROI_width = r.width;
	ROI_height = r.height;
	std::cout << "ROI Select Succes!" << endl;
	std::cout << "ROI size is" << ROI_width << "," << ROI_height << endl;
	Rect2d r2(ROI_x, ROI_y, ROI_width, ROI_height);
	im_Crop = frame(r2);
	if (im_Crop.empty() == 1) {
		std::cout << "ROI切割失敗!" << endl;
		return 1;
	}
	for (int i = 0; i < Point_Theory.size(); i++) {
		Im_Name = CL.NameCreate(4, i);
		frame = imread(Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype);

		im_Crop = frame(r2);
		im_Crop.convertTo(im_Crop, -1, 1.2, -150);// plan a 調整對比度與亮度
		cvtColor(im_Crop, im_HSV, cv::COLOR_BGR2HSV);//轉HSV色域
		//std::cout << "imcrop to hsv finish" << endl;
		cvtColor(im_HSV, im_Gray, cv::COLOR_BGR2GRAY);//圖片轉灰階
		//std::cout << "hsv to Gray finish" << endl;
		detector->detect(im_Gray, keypoints);//Bolb Dection
		//判斷是否單點
		if (keypoints.size() > 1) {
			std::cout << "Bolb Dection 第" << i << "偵測到多於 1 點" << endl;
			lose_number++;
		}
		else if (keypoints.size() < 1) {
			std::cout << "Bolb Dection 第" << i << "偵測到少於 1 點" << endl;
			lose_number++;
		}
		else {
			std::cout << "Bolb Dection正在偵測 第" << i << "點 為[" <<
				keypoints[0].pt.x + ROI_x << "," << keypoints[0].pt.y + ROI_y << "]" << endl;//基於原圖影像座標
			Point_Theory_v2.push_back(Point2f(Point_Theory[i].x, Point_Theory[i].y));
			Points_Laser_Detect.push_back(Point2f(keypoints[0].pt.x + ROI_x, keypoints[0].pt.y + ROI_y));//將找到的點儲存
			circle(frame, Point2f(keypoints[0].pt.x + ROI_x, keypoints[0].pt.y + ROI_y), 0.2, Scalar(0, 255, 0), 10);// 如果找到，就繪圖
			get_number++;
		}
		imshow("Result", frame);
		waitKey(3);
	}
	std::cout << "共找到 " << get_number << " 點， 未找到 " << lose_number << " 點 " << endl;
	for (int i = 0; i < get_number; i++) {
		circle(frame, Points_Laser_Detect[i], 1, Scalar(0, 0, 255), 5);
	}

	while (1) {
		imshow("Result", frame);
		waitKey(1);
		int waitkey = waitKey(5);
		if (waitkey == 27) {
			std::cout << "結束程式" << endl;
			break;
		}
	}
	Im_Name = FileDate + "_Dection_Result";
	Im_Filename_All = Im_Savepath + Im_Name + Im_Savetype;
	//Im_Filename_All = "D:\\59.恩哲的研究\\221016laser校正與通訊整合測試\\230208\\230208_dection_result.png";
	imwrite(Im_Filename_All, frame);
	CL.Out_Data_Csv_2f(Csv_Savepath + Csv_Filename_Dectection, Points_Laser_Detect, get_number);//量測結果
	CL.Out_Data_Csv_2f(Csv_Savepath + Csv_Filename_Theory_v2, Point_Theory_v2, get_number);//更新後的理論點
	std::cout << "共找到 " << get_number << " 點， 未找到 " << lose_number << " 點 " << endl;
	return true;
}
bool Cam_CAL_LSM() {
	//檔案路徑
	String Csv_Path = FilePath + FileDate + "\\";
	String Csv_Header_Get = FileDate + "_detected.csv";
	String Csv_Header_Theory = FileDate + "_theory_v2.csv";
	String Csv_Header_Parameter_x = FileDate + "_x_Cal_Parameter.csv";// x方向的校正參數
	String Csv_Header_Parameter_y = FileDate + "_y_Cal_Parameter.csv";// y方向的校正參數
	//宣告變數
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	//讀取檔案
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Get, Point_Get);
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Theory, Point_Theory);
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "第" << i << "點為 " << Point_Get[i].x << "," << Point_Get[i].y << endl;
		std::cout << "第" << i << "點為 " << Point_Theory[i].x << "," << Point_Theory[i].y << endl;
	}
	//LSM參數宣告
	vector<float> x_get, x_thy;
	vector<float> y_get, y_thy;
	float x_a, x_b, x_r;
	float y_a, y_b, y_r;
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		x_get.push_back(Point_Get[i].x);
		x_thy.push_back(Point_Theory[i].x);
		y_get.push_back(Point_Get[i].y);
		y_thy.push_back(Point_Theory[i].y);
	}
	CL.LineFitLeastSquares(x_get, x_thy, x_get.size(), x_a, x_b, x_r);
	CL.LineFitLeastSquares(y_get, y_thy, y_get.size(), y_a, y_b, y_r);
	std::cout << "X A值 為" << x_a << endl;
	std::cout << "X B值 為" << x_b << endl;
	std::cout << "y A值 為" << y_a << endl;
	std::cout << "y B值 為" << y_b << endl;
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "影像 x: " << x_get[i] << " 理論x : " << x_thy[i] << ", 傳送給雷射x : " << (x_get[i] * x_a + x_b) << endl;
		std::cout << "影像 y: " << y_get[i] << " 理論y : " << y_thy[i] << ", 傳送給雷射y : " << (y_get[i] * y_a + y_b) << endl;
		std::cout << endl;
	}
	std::cout << "相關性係數 X R : " << x_r << endl;
	std::cout << "相關性係數 Y R : " << y_r << endl;

	//宣告儲存用的向量容器
	vector<Point2f> Parameter_LSM_x;
	vector<Point2f> Parameter_LSM_y;
	Parameter_LSM_x.push_back(Point2f(x_a, x_b));
	Parameter_LSM_y.push_back(Point2f(y_a, y_b));
	CL.Out_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_x, Parameter_LSM_x, Parameter_LSM_x.size());
	CL.Out_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_y, Parameter_LSM_y, Parameter_LSM_y.size());
	return true;
}
bool Cam_CAL_LSM_quadratic() {
	//檔案路徑
	String Csv_Path = FilePath + FileDate + "\\";
	String Csv_Header_Get = FileDate + "_detected.csv";
	String Csv_Header_Theory = FileDate + "_theory_v2.csv";
	String Csv_Header_Parameter_x = FileDate + "_x_Cal_Parameter.csv";// x方向的校正參數
	String Csv_Header_Parameter_y = FileDate + "_y_Cal_Parameter.csv";// y方向的校正參數
	//宣告變數
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	//讀取檔案
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Get, Point_Get);
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Theory, Point_Theory);
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "第" << i << "點為 " << Point_Get[i].x << "," << Point_Get[i].y << endl;
		std::cout << "第" << i << "點為 " << Point_Theory[i].x << "," << Point_Theory[i].y << endl;
	}
	//LSM參數宣告
	vector<float> x_get, x_thy;
	vector<float> y_get, y_thy;
	float x_a, x_b, x_c;
	float y_a, y_b, y_c;
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		x_get.push_back(Point_Get[i].x);
		x_thy.push_back(Point_Theory[i].x);
		y_get.push_back(Point_Get[i].y);
		y_thy.push_back(Point_Theory[i].y);
	}
	CL.LineFitLeastSquares_quadratic(x_get, x_thy, x_get.size(), x_a, x_b, x_c);
	CL.LineFitLeastSquares_quadratic(y_get, y_thy, y_get.size(), y_a, y_b, y_c);
	std::cout << "X A值 為" << x_a << endl;
	std::cout << "X B值 為" << x_b << endl;
	std::cout << "X C值 為" << x_c << endl;
	std::cout << "y A值 為" << y_a << endl;
	std::cout << "y B值 為" << y_b << endl;
	std::cout << "y C值 為" << y_c << endl;
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "影像 x: " << x_get[i] << " 理論x : " << x_thy[i] << ", 傳送給雷射x : " << (x_get[i] * x_get[i] * x_a + x_get[i] * x_b + x_c) << endl;
		std::cout << "影像 y: " << y_get[i] << " 理論y : " << y_thy[i] << ", 傳送給雷射y : " << (y_get[i] * y_get[i] * y_a + y_get[i] * y_b + y_c) << endl;
		std::cout << endl;
	}


	return true;
}
bool Cam_Laser_Calibration_repeatability() {
	// 230610 : 重複性測試，online
	// 同一點偵測多次
	// 宣告變數
	Mat frame;// 相機照片容器
	Mat frame_copy;
	Mat im_Crop;
	Mat im_Gray;
	Mat im_HSV;
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	vector<Point2f> Point_Theory_v2;//檢測完後的可用理論點
	vector<Point2f>	Points_Laser_Detect;
	vector<vector<Point2f>> Point_Theory_v2_time;
	vector<vector<Point2f>>Points_Laser_Detect_time;

	//ROI參數
	Rect2d roi;//Roi容器
	bool showCrosshair = false;
	bool fromCenter = false;

	// 建立資料夾
	string Folder_Path = FilePath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// 如果檔案不存在
		command = "mkdir -p " + Folder_Path;// 以dos建立資料夾
		system(command.c_str());

	}

	// XML檔案
	String Xml_Savepath = Folder_Path + "\\";
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml(Xml_Savepath + Xml_Filename, FileStorage::WRITE);// 儲存容器
	if (!Fs_Xml.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}

	//-----雷射參數-----//
	string code_out;// 先祖合成"字串"
	char* code_out_ch;// 再分割成'字元'傳送
	int Point_Numbers = 0;//用於紀錄總點數的起始值
	char read_ch;// 讀取確認碼用的
	string read_str;// 組合接收到的指令
	int Lasertimes = 30;// 雷射拍幾個循環的，一個循環是一整面
	vector<Point2f>Points_Laser_Theory;// 理論雷射點的容器(多循環)
	int x1 = 0;// 傳送的點的容器
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	UINT BytesInQue;// 緩衝區容器

	// Bolb Dection 各項參數
	SimpleBlobDetector::Params params;
	// Change thresholds	//探索的像素上下限
	params.minThreshold = 80;
	params.maxThreshold = 120;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1;

	// Filter by Circularity //圓度
	params.filterByCircularity = false;
	params.minCircularity = 0.5;

	// Filter by Convexity	//凸度
	params.filterByConvexity = false;
	params.minConvexity = 0.01;

	// Filter by Inertia	//冠量比
	params.filterByInertia = false;
	params.minInertiaRatio = 0.3;

	// Storage for blobs
	vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// 相機參數
	VideoCapture cap;//打開SerialPort 0 的相機


	// 建立雷射理論點
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}

	PortOpen();// 打開雷射振鏡的port口
	//mySerialPort.OpenListenThread();// 打開串聯通訊監聽
	Sleep(1 * 1000);
	cap.open(port_cam, apiID);// 打開相機
	if (!cap.isOpened()) {
		std::cout << "Can't open the camera, port number is " << port_cam << std::endl;
	}
	else {
		std::cout << "Camera is opened, port number is " << port_cam << std::endl;
		cap.set(cv::CAP_PROP_FRAME_WIDTH, Width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);
	}

	Sleep(2 * 1000);

	// 選取ROI
	cap.read(frame);// 相機拍照
	if (frame.empty() == 1) {
		std::cout << "Error while camera read frame!" << endl;// 檢查有拍到照片
		return 1;
	}
	roi = cv::selectROI("請選擇ROI，ESC 結束", frame, showCrosshair, fromCenter);

	for (int j = 0; j < Lasertimes; j++) {
		for (int i = 0; i < Points_Num_total; i++) {
			//控制雷射點到指定點位
			x1 = CL.cal_0to2048((int)Points_Laser_Theory[i].x);
			y1 = CL.cal_0to2048((int)Points_Laser_Theory[i].y);
			code_out = "01" + CL.int2string_4ch(x1) + CL.int2string_4ch(y1) + CL.int2string_4ch(x2) + \
				CL.int2string_4ch(y2) + CL.int2string_4ch(x3) + CL.int2string_4ch(y3) + CL.int2string_4ch(x4) + CL.int2string_4ch(y4);//太長，分行
			code_out_ch = (char*)code_out.data();

			for (int i = 0; i < 5; i++) {
				mySerialPort.WriteData(code_out_ch, 34);//傳送字串給Arduino
			}
			Sleep(200);

			BytesInQue = mySerialPort.GetBytesInCOM();// 確認緩衝區有無資料
			while (1) {
				BytesInQue = mySerialPort.GetBytesInCOM();// 重新確認緩衝區有無資料
				mySerialPort.ReadChar(read_ch);// 重新讀取資料
				if (BytesInQue != 0 && read_ch == 'N') {
					std::cout << "接收到了 : " << read_ch << endl;
					while (1) {
						cap.read(frame);// 相機拍照
						frame_copy = frame.clone();
						if (frame.empty() == 1) {
							std::cout << "Error while camera read frame!" << endl;// 檢查有拍到照片
							return 1;
						}

						// Blob detector
						im_Crop = frame(roi);
						im_Crop.convertTo(im_Crop, -1, 1.2, -150);// plan a 調整對比度與亮度
						cvtColor(im_Crop, im_HSV, cv::COLOR_BGR2HSV);//轉HSV色域
						//std::cout << "imcrop to hsv finish" << endl;
						cvtColor(im_HSV, im_Gray, cv::COLOR_BGR2GRAY);//圖片轉灰階
						//std::cout << "hsv to Gray finish" << endl;
						detector->detect(im_Gray, keypoints);//Bolb Dection
						//判斷是否單點

						if (keypoints.size() > 1) {
							std::cout << "Bolb Dection 第" << i << "偵測到多於 1 點" << endl;
						}
						else if (keypoints.size() < 1) {
							std::cout << "Bolb Dection 第" << i << "偵測到少於 1 點" << endl;
						}
						else {
							std::cout << "Bolb Dection正在偵測 第" << i << "點 為[" <<
								keypoints[0].pt.x + roi.x << "," << keypoints[0].pt.y + roi.y << "]" << endl;//基於原圖影像座標
							Point_Theory_v2.push_back(Point2f(Points_Laser_Theory[i].x, Points_Laser_Theory[i].y));
							Points_Laser_Detect.push_back(Point2f(keypoints[0].pt.x + roi.x, keypoints[0].pt.y + roi.y));//將找到的點儲存
							circle(frame_copy, Point2f(keypoints[0].pt.x + roi.x, keypoints[0].pt.y + roi.y), 0.2, Scalar(0, 255, 0), 10);// 如果找到，就繪圖
							break;
						}
					}

					imshow("Frame", frame_copy);
					waitKey(1);
					break;
				}
				else {
					cout << "No data in buffer, read again";
				}

			}
		}
		Point_Theory_v2_time.push_back(Point_Theory_v2);
		Points_Laser_Detect_time.push_back(Points_Laser_Detect);
		Point_Theory_v2.clear();
		Points_Laser_Detect.clear();

	}
	// 輸出資料
	Fs_Xml << "Point_Theory_v2_time" << Point_Theory_v2_time;
	Fs_Xml << "Points_Laser_Detect_time" << Points_Laser_Detect_time;

	// 關閉雷射
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);
	std::cout << "Finish!" << std::endl;

	return true;
}
bool Cam_Laser_repeatability_result() {
	// 230622 : 讀取data分析重複性
	// 檔案路徑
	string Folder_Path = FilePath + FileDate;
	// XML檔案
	String Xml_Savepath = Folder_Path + "\\";
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml_read(Xml_Savepath + Xml_Filename, FileStorage::READ);// 儲存容器
	if (!Fs_Xml_read.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}
	// 宣告變數
	vector<vector<Point2f>> Point_Theory_v2_time;
	vector<vector<Point2f>>Points_Laser_Detect_time;

	// 讀取資料
	Fs_Xml_read["Point_Theory_v2_time"] >> Point_Theory_v2_time;
	Fs_Xml_read["Points_Laser_Detect_time"] >> Points_Laser_Detect_time;

	// 測試顯示讀取資料
	/*for (int i = 0; i < Points_Laser_Detect_time.size(); i++) {
		for (int j = 0; j < Points_Laser_Detect_time[0].size(); j++) {

			cout << "time : "<<i<<", point : "<<j<<" result : "<<Points_Laser_Detect_time[i][j] << endl;

		}
	}*/
	// 分析結果
	// 1.平均值
	float x_point = 0; float y_point = 0;
	float x_average, y_average;
	float x_error = 0; float y_error = 0;
	float x_error_max = 0; float y_error_max = 0;
	float x_error_min = 1; float y_error_min = 1;
	float x_error_total = 0;
	float y_error_total = 0;
	float x_error_average = 0;
	float y_error_average = 0;

	vector<Point2f> point_dec_average;// 平均值
	vector<Point2f> point_dec_mean_error;// 平均誤差
	vector<Point2f> point_dec_max_error;// 平均誤差
	Point2f point_average;

	for (int i = 0; i < Points_Laser_Detect_time[0].size(); i++) {
		for (int j = 0; j < Points_Laser_Detect_time.size(); j++) {

			cout << "point : " << i << "time : " << j << " result : " << Points_Laser_Detect_time[j][i] << endl;
			x_point = x_point + Points_Laser_Detect_time[j][i].x;
			y_point = y_point + Points_Laser_Detect_time[j][i].y;
		}
		x_average = x_point / Points_Laser_Detect_time.size();
		y_average = y_point / Points_Laser_Detect_time.size();
		point_dec_average.push_back(Point2f(x_average, y_average));
		x_point = 0;// 用完之後，變數歸零
		y_point = 0;
	}
	// 顯示平均值結果
	for (int i = 0; i < point_dec_average.size(); i++) {
		cout << "第" << i << "點，平均" << point_dec_average[i] << endl;
	}
	// 2.計算誤差
	for (int i = 0; i < Points_Laser_Detect_time[0].size(); i++) {
		for (int j = 0; j < Points_Laser_Detect_time.size(); j++) {

			//cout << "point : " << i << "time : " << j << " result : " << Points_Laser_Detect_time[j][i] << endl;
			x_error = fabs(Points_Laser_Detect_time[j][i].x - point_dec_average[i].x);
			y_error = fabs(Points_Laser_Detect_time[j][i].y - point_dec_average[i].y);
			x_error_total = x_error_total + x_error;
			y_error_total = y_error_total + y_error;
			x_error_max = max(x_error_max, x_error);
			y_error_max = max(y_error_max, y_error);

		}

		x_error_average = x_error_total / Points_Laser_Detect_time.size();
		y_error_average = y_error_total / Points_Laser_Detect_time.size();
		point_dec_mean_error.push_back(Point2f(x_error_average, y_error_average));
		point_dec_max_error.push_back(Point2f(x_error_max, y_error_max));
		/*cout << "第" << i << "點，平均" << point_dec_average[i] << endl;*/
		x_error_total = 0;// 用完之後，變數歸零
		y_error_total = 0;
		x_error_max = 0;
		y_error_max = 0;
	}

	// 顯示平均誤差結果
	for (int i = 0; i < point_dec_mean_error.size(); i++) {
		cout << "第" << i << "點，平均誤差" << point_dec_mean_error[i] << endl;
	}
	// 顯示最大誤差
	for (int i = 0; i < point_dec_mean_error.size(); i++) {
		cout << "第" << i << "點，最大誤差" << point_dec_max_error[i] << endl;
	}
	// 計算全場重複性誤差，要忽略過大的點

	// 輸出成CSV檔案
	String Filename_mean_error = FileDate + "_mean_error";
	String Filename_max_error = FileDate + "_max_error";
	String Filename_theory = FileDate + "_theory";
	String File_type = ".csv";

	CL.Out_Data_Csv_2f(Folder_Path + "\\" + Filename_mean_error + File_type, point_dec_mean_error, point_dec_mean_error.size());
	CL.Out_Data_Csv_2f(Folder_Path + "\\" + Filename_theory + File_type, Point_Theory_v2_time[0], Point_Theory_v2_time[0].size());// 輸出理論點以供參考
	return true;
}
int Feature_Match_Online_v5() {
	Mat img1;// img1 : 要拿來比對的照片，pattam
	Mat frame;// 相機當下拍的影像
	Mat frame_copy;// 複製的影像，用於繪圖
	int num = 0;
	vector<Point2f> coner_point(4);// 影像座標上的Feature Match四個點
	vector<Point2f> coner_point_Laser(4);// 轉換到雷射座標的四個點
	img1 = imread(FilePath + "221121_特徵匹配\\pic008.jpg");
	String code_out;// 輸出的X1 code指令
	char* code_out_ch;// 輸出的X1 code指令 char型態

	// 讀取CSV檔案各項校正後參數
	String Csv_Path = FilePath + FileDate + "\\";
	String Csv_Header_Parameter_x = FileDate + "_x_Cal_Parameter.csv";// x方向的校正參數
	String Csv_Header_Parameter_y = FileDate + "_y_Cal_Parameter.csv";// y方向的校正參數

	// 校正參數 容器
	vector<Point2f>Parameter_x;// x 方向的校正參數， .x為 a， .y為 b
	vector<Point2f>Parameter_y;// y 方向的校正參數， .x為 a， .y為 b

	// 讀取校正參數
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_x, Parameter_x);
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_y, Parameter_y);

	// 打開各項設備
	PortOpen();// 打開連接雷射振鏡的連接口
	mySerialPort.OpenListenThread();
	VideoCapture cap;// 打開SerialPort 0 的相機
	cap.open(port_cam, apiID);
	if (!cap.isOpened()) {
		std::cout << "Can't open the camera, port number is " << port_cam << std::endl;
	}
	else {
		std::cout << "Camera is opened, port number is " << port_cam << std::endl;
	}
	cap.set(cv::CAP_PROP_FRAME_WIDTH, Width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);

	while (1) {
		int waitkey = waitKey(3);
		cap >> frame;
		imshow("frame", frame);
		num++;
		if (frame.empty() == 1) {
			std::cout << "圖片不存在，請確認圖片" << endl;
			return -1;
		}

		if (!img1.data || !frame.data)
		{
			std::cout << "圖片未找到！" << endl;
			return -1;
		}

		//imshow("Feature Match", frame);
		CL.Feature_ORB(0, 0, img1, frame, coner_point);//Feature Match得到四個點
		frame_copy = frame.clone();
		for (int j = 0; j < 4; j++) {
			line(frame_copy, coner_point[j], coner_point[(j + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);// 更改後
			//std::cout << num << "     第" << j << "點" << coner_point[j];
		}
		for (int k = 0; k < 4; k++) {
			CL.LSM_Trans(coner_point[k], coner_point_Laser[k], Parameter_x[0].x, Parameter_x[0].y, Parameter_y[0].x, Parameter_y[0].y);
		}
		std::cout << endl;
		CL.Xcode_ctrl(coner_point_Laser, code_out);
		code_out_ch = (char*)code_out.data();// 轉換成字元
		mySerialPort.WriteData(code_out_ch, 34);
		//std::cout << "已傳送字串" << code_out << endl;

		coner_point.clear();
		imshow("img1", img1);
		imshow("Frame result", frame_copy);
		namedWindow("Frame result", WINDOW_NORMAL);

		if (waitkey == 27) {
			std::cout << "結束程式" << endl;

			break;
		}
	}
	Sleep(1000);// 等待1秒後，再送出指令
	// 關閉雷射
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);
	// 釋放容器
	frame.release();
	cap.release();
	return 0;
}
int Cam_Laser_Calibration_stereo_v2() {
	// 230306，測試立體校正

	// 先建立資料夾
	string Folder_Path = FilePath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// 如果檔案不存在
		command = "mkdir -p " + Folder_Path;// 以dos建立資料夾
		system(command.c_str());
	}

	//-----rs參數-----//
	rs2::pipeline pipe; // rs用於接收影像的容器
	rs2::config cfg;// rs自訂參數
	float dist; // rs儲存深度的容器
	static Point p;// rs單點像素點容器
	String Csv_Filename_Depth;
	String Csv_File_Path;
	// 宣告參數
	int w = 1280;
	int h = 720;
	int save_im_number = 0;// 用於計算照片數量

	// 建立CSV檔案
	String Csv_Savepath = FilePath + FileDate + "\\";
	ofstream Outfile;// Csv檔案容器

	// XML檔案
	String Xml_Savepath = Csv_Savepath;
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml(Xml_Savepath + Xml_Filename, FileStorage::WRITE);// 儲存容器
	if (!Fs_Xml.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}


	//-----雷射參數-----//
	string code_out;// 先祖合成"字串"
	char* code_out_ch;// 再分割成'字元'傳送
	int Point_Numbers = 0;//用於紀錄總點數的起始值
	char read_ch;// 讀取確認碼用的
	string read_str;// 組合接收到的指令
	int Lasertimes = 3;// 雷射拍幾個循環的，一個循環是一整面
	vector<Point2f>Points_Laser_Theory;// 理論雷射點的容器
	int x1 = 0;// 傳送的點的容器
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	// 建立雷射理論點
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}

	//-----儲存影像位置-----//
	String Savepath = FilePath;
	String Im_Savepath = Savepath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;


	// 儲存理論點到XML
	for (int k = 0; k < Lasertimes - 1; k++) {
		for (int i = 0; i <= Points_R_Num; i++) {
			for (int j = 0; j <= Points_C_Num; j++) {
				Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
			}
		}
	}
	Fs_Xml << "Point_Laser_Theory" << Points_Laser_Theory;
	// 輸出照片總數 
	save_im_number = Lasertimes * (Points_R_Num + 1) * (Points_R_Num + 1);
	Fs_Xml << "save_im_number" << save_im_number;
	Fs_Xml.release();
	// 關閉雷射
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);

	return 0;
}
int Cam_Laser_Calibration_stereo_v3() {
	// 230712，測試立體校正

	// 先建立資料夾
	string Folder_Path = FilePath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// 如果檔案不存在
		command = "mkdir -p " + Folder_Path;// 以dos建立資料夾
		system(command.c_str());
	}

	//-----rs參數-----//
	rs2::pipeline pipe; // rs用於接收影像的容器
	rs2::config cfg;// rs自訂參數
	rs2::frameset frames; // 等待下一組影像
	rs2::frame color_frame;// 取得彩色影像

	float dist; // rs儲存深度的容器
	static Point p;// rs單點像素點容器
	String Csv_Filename_Depth;
	String Csv_File_Path;
	// 宣告參數
	int w = 1280;
	int h = 720;
	int save_im_number = 0;// 用於計算照片數量

	// 建立CSV檔案
	String Csv_Savepath = FilePath + FileDate + "\\";
	ofstream Outfile;// Csv檔案容器

	// XML檔案
	String Xml_Savepath = Csv_Savepath;
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml(Xml_Savepath + Xml_Filename, FileStorage::WRITE);// 儲存容器
	if (!Fs_Xml.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}

	//讀取的影像格式
	cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30); // BGR888格式彩色影像 30fps
	cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 30); // 16 bit格式灰階深度影像 30fps

	//-----雷射參數-----//
	string code_out;// 先祖合成"字串"
	char* code_out_ch;// 再分割成'字元'傳送
	int Point_Numbers = 0;//用於紀錄總點數的起始值
	char read_ch;// 讀取確認碼用的
	string read_str;// 組合接收到的指令
	int Lasertimes = 3;// 雷射拍幾個循環的，一個循環是一整面
	vector<Point2f>Points_Laser_Theory;// 理論雷射點的容器(多循環)
	int x1 = 0;// 傳送的點的容器
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	// 建立雷射理論點
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}
	PortOpen();// 打開雷射振鏡的port口

	Sleep(3 * 1000);

	//-----儲存影像位置-----//
	String Savepath = FilePath;
	String Im_Savepath = Savepath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	// 其他變數
	rs2_intrinsics intr;
	int cycle;// 用於分割每次校正循環

	// 開始讀取影像
	pipe.start(cfg);


	int waitkey = waitKey(3);
	for (int i = 0; i < Lasertimes; i++) {
		for (int j = 0; j < Points_Num_total; j++) {
			//控制雷射點到指定點位
			x1 = CL.cal_0to2048((int)Points_Laser_Theory[j].x);
			y1 = CL.cal_0to2048((int)Points_Laser_Theory[j].y);
			code_out = "01" + CL.int2string_4ch(x1) + CL.int2string_4ch(y1) + CL.int2string_4ch(x2) + \
				CL.int2string_4ch(y2) + CL.int2string_4ch(x3) + CL.int2string_4ch(y3) + CL.int2string_4ch(x4) + CL.int2string_4ch(y4);//太長，分行
			code_out_ch = (char*)code_out.data();
			for (int k = 0; k < 5; k++) {
				mySerialPort.WriteData(code_out_ch, 34);//傳送字串給Arduino
			}
			std::cout << "PC Send :" << endl;
			std::cout << code_out_ch << endl;
			std::cout << "send points : " << j << endl;
			Sleep(200);
			UINT BytesInQue = mySerialPort.GetBytesInCOM();// 確認緩衝區有無資料
			while (1) {

				if (BytesInQue != 0) {
					mySerialPort.ReadChar(read_ch);// 重新讀取資料
					if (read_ch == 'N') {
						std::cout << "接收到了 : " << read_ch << endl;
						// 開始讀取影像
						frames = pipe.wait_for_frames();// 等待下一組影像
						color_frame = frames.get_color_frame();// 取得彩色影像
						rs2::depth_frame depth = frames.get_depth_frame();// 取得深度

						// 轉換為opencv格式
						Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP); // 彩色影像

						if (color_image.empty() == 1) {
							std::cout << "圖片不存在，請確認圖片" << endl;// 檢查有拍到照片
							return 1;
						}
						else {
							// 以OpenCV函式顯示擷取到影像
							imshow("Color Image", color_image); // 彩色影像

							// 儲存照片
							waitKey(1);
							Im_Name = CL.NameCreate(4, save_im_number);
							Im_Filename_All = Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype;
							imwrite(Im_Filename_All, color_image);
							std::cout << "儲存第" << save_im_number << "張照片" << endl;

							// 儲存深度資訊
							Csv_Filename_Depth = FileDate + "_depth" + CL.int2string_4ch(save_im_number) + ".csv";
							Csv_File_Path = Csv_Savepath + Csv_Filename_Depth;
							Outfile.open(Csv_File_Path, ios::out);

							// 每列填完換一列，由左到右，再由上到下
							for (int row = 0; row < w; row++) {
								for (int col = 0; col < h; col++) {
									p = Point(row, col);
									dist = depth.get_distance(p.x, p.y);
									//dist = 1;
									Outfile << dist << ",";
								}
								Outfile << endl;// 換行
							}
							std::cout << "儲存第" << save_im_number << "個 .csv檔案" << endl;
							Outfile.close();
							save_im_number++;
							break;
						}

					}

				}
				else {
					cout << "Didn't read.." << endl;
					BytesInQue = mySerialPort.GetBytesInCOM();// 重新確認緩衝區有無資料
				}
				Sleep(100);

			}
		}
		cout <<"完成第 "<<i<< " 個循環，請輸入任意數字以繼續" << endl;
		cin >> cycle;
	}
	// 輸出內參
	intr = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Realsense內參
	int intr_width = intr.width;
	int intr_height = intr.height;
	float intr_ppx = intr.ppx;
	float intr_ppy = intr.ppy;
	float intr_fx = intr.fx;
	float intr_fy = intr.fy;
	String intr_model = rs2_distortion_to_string(intr.model);
	vector<float> intr_coeffs;
	for (int i = 0; i < 5; i++) {
		intr_coeffs.push_back(intr.coeffs[i]);
	}

	Fs_Xml << "intr_width" << intr_width;
	Fs_Xml << "intr_height" << intr_height;
	Fs_Xml << "intr_ppx" << intr_ppx;
	Fs_Xml << "intr_ppy" << intr_ppy;
	Fs_Xml << "intr_fx" << intr_fx;
	Fs_Xml << "intr_fy" << intr_fy;
	Fs_Xml << "intr_model" << intr_model;
	Fs_Xml << "intr_coeff" << intr_coeffs;

	// 儲存理論點到XML
	for (int k = 0; k < Lasertimes - 1; k++) {
		for (int i = 0; i <= Points_R_Num; i++) {
			for (int j = 0; j <= Points_C_Num; j++) {
				Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
			}
		}
	}
	Fs_Xml << "Point_Laser_Theory" << Points_Laser_Theory;
	// 輸出照片總數
	Fs_Xml << "save_im_number" << save_im_number;
	Fs_Xml.release();


	// 關閉雷射
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);

	return 0;
}
int Cam_Laser_Calibration_stereo_Blob_v3() {
	/*
	* 230307，讀取拍攝檔案，辨識影像的雷射
	*將其辨識的位置及像素深度計算出來
	*!!!需要計算該點的真實座標，而非直接當像素座標為XY部分
	*/
	// 連續讀取圖片位置
	// 先從可以連續切割ROI開始
	//

	// 圖片路徑
	String Im_Savepath = FilePath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	//-----儲存偵測到影像位置-----//
	String Im_Savepath_Dection = Im_Savepath + "dection\\";
	String Im_Filename_Title_Dection = FileDate + "_image_dection";
	String Im_Name;
	String Im_Filename_All;

	// 建立偵測到點的資料夾
	string Folder_Path = FilePath + FileDate + "\\dection";
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// 如果檔案不存在
		command = "mkdir -p " + Folder_Path;// 以dos建立資料夾
		system(command.c_str());
	}

	// XML檔案
	String Data_savepath = Im_Savepath;
	String Filename = FileDate + ".xml";
	cout << Data_savepath + Filename << endl;

	FileStorage Fs_Xml_Read(Data_savepath + Filename, FileStorage::READ); // 儲存容器
	
	if (!Fs_Xml_Read.isOpened()) {
		std::cout << "Fail to read file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}
	


	// csv檔案
	String Csv_Savepath = Im_Savepath;
	String Csv_filename;

	// ROI參數
	vector<vector<Point> > squares;// 找到的矩形容器
	Rect2d r;// ROI容器
	Mat Im_crop;
	Mat Im_gray;
	Mat Im_HSV;
	int thresh = 5, N = 10;// Canny檢測的參數，沒事先不要動好了，請參考Opencv的範例
	double square_max_area = 0;
	int square_local = 0;


	// Bolb Dection 各項參數
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.blobColor = uchar(0);
	params.minThreshold = 85;// Change thresholds	//探索的像素上下限
	params.maxThreshold = 120;
	params.filterByArea = true;// Filter by Area.
	params.minArea = 1;
	params.filterByCircularity = false;// Filter by Circularity //圓度
	params.minCircularity = 0.4;
	params.filterByConvexity = false;// Filter by Convexity	//凸度
	params.minConvexity = 0.4;
	params.filterByInertia = false;// Filter by Inertia	//冠量比
	params.minInertiaRatio = 0.3;

	// Storage for blobs
	vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// 其他變數
	Mat Im;// 讀入的影像
	Mat Im_clone;// 複製的影像
	int save_im_number;// 照片總數
	int get_number = 0; // 找到的點數量
	int lose_number = 0;// 遺失的點數量
	Point2f Point_source;// 在原圖上的雷射點位置，避免誤使用到ROI的位置
	vector<int>Im_get_number;// 找到的照片順序編號
	vector<Point2f> Point_Theory;
	vector<Point2f> Point_Theory_Used;//檢測完後的可用理論點，雷射座標
	vector<Point2f>Points_Laser_Detect;// 檢測完的點，像素
	vector<Point3f>Points_Laser_Detect_RW;// 檢測完的點，3D
	vector<double>Depth_vector;// 檢測完的點對應的深度
	Mat_<double> Depth_mat;// 從csv檔案讀取深度矩陣的容器

	//rs變數
	rs2_intrinsics intr;
	int intr_width;
	int intr_height;
	float intr_ppx;
	float intr_ppy;
	float intr_fx;
	float intr_fy;
	String intr_model;
	vector<float> intr_coeffs;

	// 讀取資料
	if (!Fs_Xml_Read.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}
	Fs_Xml_Read["save_im_number"] >> save_im_number;
	Fs_Xml_Read["Point_Laser_Theory"] >> Point_Theory;
	Fs_Xml_Read["intr_width"] >> intr_width;
	Fs_Xml_Read["intr_height"] >> intr_height;
	Fs_Xml_Read["intr_ppx"] >> intr_ppx;
	Fs_Xml_Read["intr_ppy"] >> intr_ppy;
	Fs_Xml_Read["intr_fx"] >> intr_fx;
	Fs_Xml_Read["intr_fy"] >> intr_fy;
	Fs_Xml_Read["intr_model"] >> intr_model;
	Fs_Xml_Read["intr_coeff"] >> intr_coeffs;

	Fs_Xml_Read.release();// 讀取完釋放容器

	intr.width = intr_width;
	intr.height = intr_height;
	intr.ppx = intr_ppx;
	intr.ppy = intr_ppy;
	intr.fx = intr_fx;
	intr.fy = intr_fy;
	if (intr_model == "Inverse Brown Conrady") {
		intr.model = RS2_DISTORTION_BROWN_CONRADY;
	}
	for (int i = 0; i < 5; i++) {
		intr.coeffs[i] = intr_coeffs[i];
	}

	for (int i = 0; i < Point_Theory.size(); i++) {
		std::cout << Point_Theory[i] << endl;
	}
	// 主程式
	for (int Im_number = 0; Im_number < save_im_number; Im_number++) {
		Im = imread(Im_Savepath + Im_Filename_Title + CL.int2string_4ch(Im_number) + Im_Savetype);
		Im_clone = Im.clone();
		CL.findSquares(Im_clone, squares, N, thresh);
		CL.square_max(squares, square_max_area, square_local);
		if (CL.length_2P(squares[square_local][1], squares[square_local][0]) < 900 && CL.length_2P(squares[square_local][2], squares[square_local][1]) < 900) {
			r = boundingRect(squares[square_local]);
			for (int j = 0; j < 4; j++) {
				// 繪製矩形辨識結果
				//circle(im, squares[square_local][j], 1, Scalar(0, 255, 0), 5);
				//line(im, squares[square_local][j], squares[square_local][(j + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);
				//// roi畫圖，黃色
				//line(im, point2f(r.x, r.y), point2f(r.x + r.width, r.y), scalar(0, 255, 255), 2, 8, 0);
				//line(im, point2f(r.x + r.width, r.y), point2f(r.x + r.width, r.y + r.height), scalar(0, 255, 255), 2, 8, 0);
				//line(im, point2f(r.x + r.width, r.y + r.height), point2f(r.x, r.y + r.height), scalar(0, 255, 255), 2, 8, 0);
				//line(im, point2f(r.x, r.y + r.height), point2f(r.x, r.y), scalar(0, 255, 255), 2, 8, 0);
			}
		}

		// 測試先將RGB影像膨脹，提高亮部分，降低黑暗部分
		Im_crop = Im_clone(r);
		convertScaleAbs(Im_crop, Im_crop, 1.2, -100);
		Mat element_1 = getStructuringElement(MORPH_RECT, Size(1, 1));
		Mat element_2 = getStructuringElement(MORPH_ELLIPSE, Size(1, 1));
		Mat element_3 = getStructuringElement(MORPH_RECT, Size(6, 6));
		Mat element_4 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

		erode(Im_crop, Im_crop, element_1);// 侵蝕	
		erode(Im_crop, Im_crop, element_2);// 侵蝕	
		dilate(Im_crop, Im_crop, element_3);// 膨脹			
		dilate(Im_crop, Im_crop, element_4);// 膨脹			
		cvtColor(Im_crop, Im_HSV, cv::COLOR_BGR2HSV);//轉HSV色域
		cvtColor(Im_HSV, Im_gray, cv::COLOR_BGR2GRAY);//圖片轉灰階

		//---Bolb Dection---//
		detector->detect(Im_gray, keypoints);//Bolb Dection

		if (keypoints.size() == 1) {

			for (int i = 0; i < keypoints.size(); i++) {
				Point_source = Point2f(keypoints[i].pt.x + r.x, keypoints[i].pt.y + r.y);// 先修正為原圖的位置
				circle(Im_clone, Point_source, 0.2, Scalar(0, 255, 0), 10);// 如果找到，就繪圖
			}
			get_number++;
			// 儲存找到雷射點的圖片
			Im_Filename_All = Im_Savepath_Dection + Im_Filename_Title_Dection + "_" + CL.int2string_4ch(Im_number) + Im_Savetype;
			imwrite(Im_Filename_All, Im_clone);

			// 將找到點的資料進行儲存
			Points_Laser_Detect.push_back(Point_source);
			Point_Theory_Used.push_back(Point_Theory[Im_number]);
			Im_get_number.push_back(Im_number);

			Csv_filename = FileDate + "_depth" + CL.int2string_4ch(Im_number) + ".csv";
			CL.Read_Csv2Mat(Csv_Savepath + Csv_filename, Depth_mat);
			if (Depth_mat.empty() == 1) {
				std::cout << "Error in read csv to mat!" << endl;
				std::cout << "Csv_Savepath : " << Csv_Savepath << endl;
				std::cout << "Csv_filename :" << Csv_filename << endl;
				return 1;
			}
			Depth_vector.push_back(Depth_mat(Point_source.x, Point_source.y));

			imshow("Result of on point", Im_clone);
			cv::waitKey(1);
		}
		else {
			lose_number++;
		}
		std::cout << "共" << save_im_number << "張，目前第" << Im_number << "張" << endl;
	}
	// 將找到的點，轉換為3D座標
	for (int i = 0; i < get_number; i++) {
		float upoint[3];
		float upixel[2] = { Points_Laser_Detect[i].x, Points_Laser_Detect[i].y };
		float udist;
		udist = Depth_vector[i];
		rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
		Points_Laser_Detect_RW.push_back(Point3f(upoint[0], upoint[1], upoint[2]));
	}

	for (int i = 0; i < get_number; i++) {
		std::cout << setw(25) << "Points_Laser_Detect : " << Points_Laser_Detect[i] << endl;
		std::cout << setw(25) << "Point_Theory_Used : " << Point_Theory_Used[i] << endl;
		std::cout << setw(25) << "Depth_vector : " << Depth_vector[i] << endl;
		std::cout << setw(25) << "Points_Laser_Detect_RW : " << Points_Laser_Detect_RW[i] << endl;
		std::cout << "------------------------------------------" << endl;
	}
	FileStorage Fs_Xml_Write(Data_savepath + Filename, FileStorage::WRITE); // 儲存容器
	if (!Fs_Xml_Write.isOpened()) {
		std::cout << "Fail to write file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}
	// 先覆寫舊的資料
	Fs_Xml_Write << "save_im_number" << save_im_number;
	Fs_Xml_Write << "Point_Laser_Theory" << Point_Theory;
	Fs_Xml_Write << "intr_width" << intr_width;
	Fs_Xml_Write << "intr_height" << intr_height;
	Fs_Xml_Write << "intr_ppx" << intr_ppx;
	Fs_Xml_Write << "intr_ppy" << intr_ppy;
	Fs_Xml_Write << "intr_fx" << intr_fx;
	Fs_Xml_Write << "intr_fy" << intr_fy;
	Fs_Xml_Write << "intr_model" << intr_model;
	Fs_Xml_Write << "intr_coeff" << intr_coeffs;

	// 新增的資料
	Fs_Xml_Write << "Points_Laser_Detect" << Points_Laser_Detect;
	Fs_Xml_Write << "Point_Theory_Used" << Point_Theory_Used;
	Fs_Xml_Write << "Depth_vector" << Depth_vector;
	Fs_Xml_Write << "Points_Laser_Detect_RW" << Points_Laser_Detect_RW;
	Fs_Xml_Write << "get_number" << get_number;

	std::cout << "共　　" << save_im_number << "點" << endl;
	std::cout << "共找到" << get_number << "點" << endl;
	std::cout << "共遺失" << lose_number << "點" << endl;
	return 0;
}

/*-----其他程式-----*/
int Feature_Match_most_pic() {
	vector<Mat> Pattern_pics;
	vector<String> Pattern_list;
	Mat frame;
	Mat frame_copy;
	int Pattern_number = 2;// 清單上讀取幾張，以前面排到後面
	//vector<Point2f>coner_point_list[];
	vector<vector<Point2f>> coner_point_list;
	vector<Point2f>coner_point;
	String window_name = "pattern";
	// 讀取照片清單，預計以xml檔案讀取
	Pattern_list.push_back(FilePath + "221121_特徵匹配\\pic009.jpg");
	Pattern_list.push_back(FilePath + "221121_特徵匹配\\pic010.jpg");

	if (Pattern_number > Pattern_list.size()) {
		std::cout << "ERROR!設定讀取照片數大於清單!" << std::endl;
		return 1;
	}

	// 將清單上的Pattan照片都讀取進來
	for (int i = 0; i < Pattern_number; i++) {
		Mat im_read = imread(Pattern_list[i]);
		if (im_read.empty() == 1) {
			std::cout << "ERROR when read Pattern!" << endl;
			return 1;
		}
		Pattern_pics.push_back(im_read);
	}

	// 開始辨識
	VideoCapture cap;// 打開SerialPort 0 的相機
	cap.open(port_cam, apiID);
	if (!cap.isOpened()) {
		std::cout << "Can't open the camera, port number is " << port_cam << std::endl;
	}
	else {
		std::cout << "Camera is opened, port number is " << port_cam << std::endl;
	}
	cap.set(cv::CAP_PROP_FRAME_WIDTH, Width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);

	while (1) {
		int waitkey = waitKey(3);
		cap >> frame;
		frame_copy = frame.clone();
		if (frame.empty() == 1) {
			std::cout << "ERROR，相機拍照!" << endl;
			return 1;
		}

		for (int i = 0; i < Pattern_number; i++) {
			imshow(window_name + CL.int2string_4ch(i), Pattern_pics[i]);
		}
		for (int i = 0; i < Pattern_number; i++) {
			CL.Feature_ORB(0, 0, Pattern_pics[i], frame, coner_point);//Feature Match得到四個點
			coner_point_list.push_back(coner_point);
			coner_point.clear();
			/*std::cout <<"coner_point"<< coner_point << std::endl;
			std::cout <<"coner_point_list" << coner_point_list[i] << std::endl;
			std::cout << "----------------------------" << std::endl;*/
		}
		for (int i = 0; i < Pattern_number; i++) {
			std::cout << "coner_point_list (Pattern_number) = " << i << std::endl;
			for (int j = 0; j < 4; j++) {
				std::cout << coner_point_list[i][j] << std::endl;
			}
		}
		for (int i = 0; i < coner_point_list.size(); i++) {
			for (int j = 0; j < 4; j++) {
				line(frame_copy, coner_point_list[i][j], coner_point_list[i][(j + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);//更改後	

				//line(frame_copy, coner_point_list[1][j], coner_point_list[1][(j + 1) % 4], Scalar(0, 255, 0), 2, 8, 0);//更改後	
			}
		}
		coner_point_list.clear();
		imshow("frame", frame_copy);
		cv::waitKey(3);
		if (waitkey == 27) {
			std::cout << "結束程式" << endl;
			break;
		}
	}
	return 0;
}
int ROI_square_offline() {
	// 連續讀取圖片，劃分ROI區域，在ROI區域辨識矩形
	//圖片存檔位置
	String Im_Savepath = "E:\\恩哲\\43.人機協作\\圖檔與影片_實驗用\\230408_ROI區域矩形辨識_測試影片\\out2";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//ROI參數
	Rect2d r;//r為Roi的值
	//Rect2d r2;
	bool showCrosshair = false;
	bool fromCenter = false;
	int ROI_x, ROI_y, ROI_width, ROI_height;//ROI的各項值儲存

	Mat Im;
	Mat Im_crop;
	vector<vector<Point> > squares;
	int thresh = 125, N = 10;
	double square_max_area = 2;
	int square_local = 0;
	float length_width, length_height;
	float length_width_min = 1;// 檢測矩形邊界下限
	float length_width_max = 500;// 檢測矩形邊界上限
	float length_height_min = 1;// 檢測矩形邊界下限
	float length_height_max = 100;// 檢測矩形邊界上限

	Im_Filename_All = Im_Savepath + "\\" + CL.int2string_4ch(0) + Im_Savetype;
	Im = imread(Im_Filename_All);
	if (Im.empty()) {
		cout << "error in read image" << endl;
		cout << "path : " << Im_Filename_All << endl;
		return 1;
	}
	r = cv::selectROI("請選擇ROI，ESC 結束", Im, showCrosshair, fromCenter);//r為Roi的值
	/*r.x = 260;
	r.y = 174;
	r.width = 232;
	r.height = 168;*/
	cout << r.x << endl;
	cout << r.y << endl;
	cout << r.width << endl;
	cout << r.height << endl;
	Im_crop = Im(r);
	imshow("Im_crop", Im_crop);
	waitKey(30 * 1000);
	int pic_num = CL.GetFileNum(Im_Savepath);
	for (int i = 0; i < pic_num; i++) {
		Im_Filename_All = Im_Savepath + "\\" + CL.int2string_4ch(i) + Im_Savetype;
		Im = imread(Im_Filename_All);
		Im_crop = Im(r);
		CL.findSquares(Im_crop, squares, N, thresh);
		if (squares.empty() == 0) {// 如果有存在矩形，才計算
			CL.square_max(squares, square_max_area, square_local);
			length_width = CL.length_2P(squares[square_local][1], squares[square_local][0]);
			length_height = CL.length_2P(squares[square_local][2], squares[square_local][1]);
			length_width_max = r.width - 1;
			length_height_max = r.height - 1;
			if (length_width > length_width_min && length_width < length_width_max) {
				if (length_height > length_height_min && length_height < length_height_max) {
					for (int j = 0; j < 4; j++) {
						Point P1 = Point(squares[square_local][j].x + r.x, squares[square_local][j].y + r.y);
						Point P2 = Point(squares[square_local][(j + 1) % 4].x + r.x, squares[square_local][(j + 1) % 4].y + r.y);
						line(Im, P1, P2, Scalar(0, 0, 255), 2, 8, 0);//更改後
					}
				}
			}

		}
		imshow("Im_crop", Im_crop);
		imshow("Frame", Im);
		waitKey(5);

	}
	waitKey(0);
	return 0;

}
int ROI_square_online() {
	// 連續讀取圖片，劃分ROI區域，在ROI區域辨識矩形
	//圖片存檔位置
	String Im_Savepath = "E:\\恩哲\\43.人機協作\\圖檔與影片_實驗用\\230408_ROI區域矩形辨識_測試影片\\out2";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//ROI參數
	Rect2d r;//r為Roi的值
	//Rect2d r2;
	bool showCrosshair = false;
	bool fromCenter = false;
	int ROI_x, ROI_y, ROI_width, ROI_height;//ROI的各項值儲存

	Mat frame;
	Mat Im_crop;
	vector<vector<Point> > squares;
	int thresh = 125, N = 10;
	double square_max_area = 2;
	int square_local = 0;
	float length_width, length_height;
	float length_width_min = 1;// 檢測矩形邊界下限
	float length_width_max = 500;// 檢測矩形邊界上限
	float length_height_min = 1;// 檢測矩形邊界下限
	float length_height_max = 100;// 檢測矩形邊界上限

	VideoCapture cap;//打開SerialPort 0 的相機
	cap.open(port_cam, apiID);
	if (!cap.isOpened()) {
		std::cout << "Can't open the camera, port number is " << port_cam << std::endl;
	}
	else {
		std::cout << "Camera is opened, port number is " << port_cam << std::endl;
		cap.set(cv::CAP_PROP_FRAME_WIDTH, Width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);
	}
	cap.read(frame);
	if (frame.empty() == 1) {
		std::cout << "圖片不存在，請確認圖片" << endl;
		//return 1;
	}
	r = cv::selectROI("請選擇ROI，ESC 結束", frame, showCrosshair, fromCenter);//r為Roi的值
	/*r.x = 260;
	r.y = 174;
	r.width = 232;
	r.height = 168;*/
	cout << r.x << endl;
	cout << r.y << endl;
	cout << r.width << endl;
	cout << r.height << endl;
	Im_crop = frame(r);
	imshow("Im_crop", Im_crop);
	waitKey(30 * 1000);
	int pic_num = CL.GetFileNum(Im_Savepath);
	while (1) {
		cap.read(frame);
		if (frame.empty() == 1) {
			std::cout << "圖片不存在，請確認圖片" << endl;
			//return 1;
		}

		Im_crop = frame(r);
		CL.findSquares(Im_crop, squares, N, thresh);
		if (squares.empty() == 0) {// 如果有存在矩形，才計算
			CL.square_max(squares, square_max_area, square_local);
			length_width = CL.length_2P(squares[square_local][1], squares[square_local][0]);
			length_height = CL.length_2P(squares[square_local][2], squares[square_local][1]);
			length_width_max = r.width - 1;
			length_height_max = r.height - 1;
			if (length_width > length_width_min && length_width < length_width_max) {
				if (length_height > length_height_min && length_height < length_height_max) {
					for (int j = 0; j < 4; j++) {
						Point P1 = Point(squares[square_local][j].x + r.x, squares[square_local][j].y + r.y);
						Point P2 = Point(squares[square_local][(j + 1) % 4].x + r.x, squares[square_local][(j + 1) % 4].y + r.y);
						line(frame, P1, P2, Scalar(0, 0, 255), 2, 8, 0);//更改後
					}
				}
			}

		}
		imshow("Im_crop", Im_crop);
		imshow("Frame", frame);
		int waitkey = waitKey(5);
		if (waitkey == 27) {
			break;
		}

	}

	return 0;

}
bool Write_XML() {
	
	String Savepath = FilePath;
	String Data_savepath = Savepath + FileDate + "\\";
	String Filename = FileDate + "_test.xml";

	// 先建立資料夾
	string Folder_Path = Savepath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// 如果檔案不存在
		command = "mkdir -p " + Folder_Path;// 以dos建立資料夾
		system(command.c_str());
	}

	// 設定為XML儲存模式
	FileStorage fs(Data_savepath + Filename, FileStorage::WRITE);// 儲存容器

	if (!fs.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}

	String test = "String001";
	int num1 = 10;
	Mat test_mat = (Mat_<double>(3, 3) <<
		1.0, 2.0, 3.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0);
	fs << "title1" << test;
	fs << "INT" << num1;
	fs << "Rotation_matrix" << test_mat;
	fs.release();
	return true;
}
bool Read_XML() {
	String Data_savepath = FilePath + FileDate + "\\";
	String Filename = FileDate + "_test.xml";
	FileStorage fs(Data_savepath + Filename, FileStorage::READ);// 儲存容器
	if (!fs.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}
	// read string
	string read_string;
	Mat read_mat;
	fs["Rotation_matrix"] >> read_mat;
	fs["title1"] >> read_string;
	//std::cout << "title1 = " << read_string << endl;
	std::cout << "read_mat = " << read_mat << endl;

	// read int
	int num2;
	fs["INT"] >> num2;
	std::cout << "read int = " << num2 << endl;
	fs.release();
	return true;
}
bool Read_2_XML_230720() {

	// XML 檔案 讀取
	String XML_File_Path = FilePath + FileDate+"\\";
	String XML_File_Name = FileDate + ".xml";
	FileStorage Fs_Xml_Read(XML_File_Path + XML_File_Name, FileStorage::READ); // 儲存容器
	if (!Fs_Xml_Read.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << XML_File_Path << endl;
		std::cout << "File Name : " << XML_File_Name << endl;
		return false;
	}

	// CSV 檔案 儲存
	String Csv_file_path = XML_File_Path;
	String Csv_file_name_theroy = FileDate + "_csv_theory" + ".csv";
	String Csv_file_name_detect = FileDate + "_csv_detect" + ".csv";
	String Csv_file_name_rw = FileDate + "_csv_rw" + ".csv";
	// 讀取容器
	vector<Point2f> Point_Theory_Used;//檢測完後的可用理論點，雷射座標
	vector<Point2f>Points_Laser_Detect;// 檢測完的點，像素
	vector<Point3f>Points_Laser_Detect_RW;// 檢測完的點，3D
	int get_number;// 檢測的有效data數量

	// 讀取資料
	Fs_Xml_Read["Point_Theory_Used"] >> Point_Theory_Used;
	Fs_Xml_Read["Points_Laser_Detect"] >> Points_Laser_Detect;
	Fs_Xml_Read["Points_Laser_Detect_RW"] >> Points_Laser_Detect_RW;
	Fs_Xml_Read["get_number"] >> get_number;
	// 輸出成csv
	CL.Out_Data_Csv_2f(Csv_file_path + Csv_file_name_theroy, Point_Theory_Used, get_number);
	CL.Out_Data_Csv_2f(Csv_file_path + Csv_file_name_detect, Points_Laser_Detect, get_number);
	CL.Out_Data_Csv_3f(Csv_file_path + Csv_file_name_rw, Points_Laser_Detect_RW, get_number);

	cout << "Out Finish!" << endl;

	// 讀取進來
	CL.Read_Data_Csv_2f(Csv_file_path + Csv_file_name_theroy, Point_Theory_Used);
	CL.Read_Data_Csv_2f(Csv_file_path + Csv_file_name_detect, Points_Laser_Detect);
	CL.Read_Data_Csv_3f(Csv_file_path + Csv_file_name_rw, Points_Laser_Detect_RW);

	cout << "Read Finish!" << endl;

	return true;
}