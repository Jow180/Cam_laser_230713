#include <iostream>
#include <opencv2/videoio.hpp>// camera��
#include <opencv2/highgui.hpp>// imshow�Bwaitkey��
#include <librealsense2/rs.hpp>// realsense��
#include<io.h>// �إ߸�Ƨ���
#include "SerialPort.h"
#include "camlaser.h"

/*---�ֱ���\��---
// ctrl + m + l�i�}���Y�p�Ҧ��{��
// ctrl + k + c ����
// ctrl + k + u��������
*/

/*---�R�W�Ŷ��ŧi---*/
using namespace std; // std�зǨ禡
using namespace cv;  // OpenCV
using namespace rs2; // Realsense

// ���O�ŧi
CSerialPort mySerialPort;
CamLaser CL;

/*--- �Ѽƫŧi-- - */
String FileDate = "230721";//�s�ɤ���P�����W�r�A�i�[�W��骩�����X�A�P�ɤ]�O�إ߸�Ƨ����W��
String FilePath = "D:\\59.��������s\\221016laser�ե��P�q�T��X����\\";// �s�ɸ��|
int port_cam = 2;//�۾���port�s��
int apiID = cv::CAP_DSHOW;// �۾�api ID�A�ŧi������|�䤣��۾�
int port_laser = 6;//�p�g����ҳs����port���s��

// �ե��ΰѼ�
int Points_Pro_Range = 2048;//�p�g��v���ഫ��Y�ƸѪR��
int Points_R_Num = 4;//�ե��I�C�ƶq�A�п�J4������
int Points_C_Num = 4;//�ե��I��ƶq�A�п�J4������
int Points_Num_total = (Points_R_Num + 1) * (Points_C_Num + 1);

// �]�w�۾��ѪR��
int Width = 1280;
int Height = 720;

/*--------�Ƶ{���ŧi-------*/
// �p�g����s���Ϊ�Port�f�{�� 
void PortOpen();

/*--------�D�{���ŧi-------*/

/*-----�sĶ����-----*/
// 2D�ե��A�p�g��v���ӡA�����v��
bool Cam_Laser_Calibration_v2();
// 2D�ե��A�N���᧹���v���Z�I������A�i���x�s
bool Cam_Laser_Calibration_v2_BOLB();
// 2D�ե��A�N��ӧ����ƾ�LSM�p��
bool Cam_CAL_LSM();
// 2D�ե��A�N��ӧ����ƾ�LSM�p��A�G�������M
bool Cam_CAL_LSM_quadratic();
// 2D�A���Ʃ�����
bool Cam_Laser_Calibration_repeatability();
// 2D�A���Ʃ����ҵ��G
bool Cam_Laser_repeatability_result();
// 2D�AŪ��patten�Ϥ��A�îھڮե��Ѽƶi���v
int Feature_Match_Online_v5();

/*-----���b����-----*/
// 3D�A�Ω�s�@�z���IXML���
int Cam_Laser_Calibration_stereo_v2();
// 3D �ե��A��ӡA���x�s�`�׸�T
int Cam_Laser_Calibration_stereo_v3();
// 3D �ե��A�N���᧹���v���Z�I������A�x�s�`�פμv���y�е���T
int Cam_Laser_Calibration_stereo_Blob_v3();

/*-------��L�{���ŧi-------*/
// ROI �x�ο��ѡAoffline
int ROI_square_offline();
// ROI �x�ο��ѡAonline
int ROI_square_online();
// XML �g�J
bool Write_XML();
// XML Ū��
bool Read_XML();
// �h�ϯS�x�ǰt�Aonline
int Feature_Match_most_pic();

/*-------��L���յ{���ŧi-------*/
// 230720�A�N3D�q���᪺����নcsv�ɮ�
bool Read_2_XML_230720();


int main() {
	//Cam_Laser_Calibration_v2();
	Cam_Laser_Calibration_stereo_v3();
	//Cam_Laser_Calibration_stereo_Blob_v3();
	//Read_2_XML_230720();
	return 0;
}

/* ---�Ƶ{��--- */
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

/* ---�D�{��--- */
bool Cam_Laser_Calibration_v2() {
	// �p�g��v���ӡA�A�i��Bolb Dection�A�p�G���I�]�S����
	// 230105 ���� : �ĥΪ��������۾��h��Ӫ��j�鱱��
	//1.�����}�۾��T�{�ե��O��m
	VideoCapture cap;//���}SerialPort 0 ���۾�

	//2.�p�g��v�Ω�� �Ѽƫŧi
	vector<double> Points_R_Laser;
	vector<double> Points_C_Laser;
	vector<double> Points_R_Cam;
	vector<double> Points_C_Cam;
	vector<int> Points_Laser_Theory_ID(Points_Num_total, 0);
	vector<Point2f>Points_Laser_Theory;
	vector<Point2f>Points_Laser_Detect;
		// �ǰe���I���e��
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
	int Point_Numbers = 0;//�Ω�����`�I�ƪ��_�l��
	char read_ch;// Ū���T�{�X�Ϊ�
	string read_str;// �զX�����쪺���O

	Mat frame;// �۾��Ӥ��e��
	//�Ϥ��s�ɦ�m
	String Savepath = FilePath;
	String Im_Savepath = Savepath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//CSV�ɮצs�ɦ�m
	String Csv_Savepath = Savepath + FileDate + "\\";
	String Csv_Filename_Dectection = FileDate + "_detected.csv";
	String Csv_Filename_Theory = FileDate + "_theory.csv";

	// ���إ߸�Ƨ�
	string Folder_Path = Savepath + FileDate;
	string command;
	command = "mkdir -p " + Folder_Path;// �Hdos�إ߸�Ƨ�
	system(command.c_str());

	// �z���I
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}
	PortOpen();// ���}�p�g���誺port�f
	//mySerialPort.OpenListenThread();// ���}���p�q�T��ť
	Sleep(1 * 1000);
	cap.open(port_cam, apiID);// ���}�۾�
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
		//����p�g�I����w�I��
		x1 = CL.cal_0to2048((int)Points_Laser_Theory[i].x);
		y1 = CL.cal_0to2048((int)Points_Laser_Theory[i].y);
		code_out = "01" + CL.int2string_4ch(x1) + CL.int2string_4ch(y1) + CL.int2string_4ch(x2) + \
			CL.int2string_4ch(y2) + CL.int2string_4ch(x3) + CL.int2string_4ch(y3) + CL.int2string_4ch(x4) + CL.int2string_4ch(y4);//�Ӫ��A����
		code_out_ch = (char*)code_out.data();

		for (int i = 0; i < 5; i++) {
			mySerialPort.WriteData(code_out_ch, 34);//�ǰe�r�굹Arduino
		}
		std::cout << "PC Send :" << endl;
		std::cout << code_out_ch << endl;
		std::cout << "send points : " << i << endl;
		Sleep(200);

		cap.read(frame);
		if (frame.empty() == 1) {
			std::cout << "�Ϥ����s�b�A�нT�{�Ϥ�" << endl;// �ˬd�����Ӥ�
			//return 1;
		}
		else {
			std::cout << "test��ӡAOK!" << endl;
		}
		UINT BytesInQue = mySerialPort.GetBytesInCOM();// �T�{�w�İϦ��L���
		while (1) {
			BytesInQue = mySerialPort.GetBytesInCOM();// ���s�T�{�w�İϦ��L���
			mySerialPort.ReadChar(read_ch);// ���sŪ�����
			if (BytesInQue != 0 && read_ch == 'N') {
				std::cout << "������F : " << read_ch << endl;
				cap.read(frame);
				if (frame.empty() == 1) {
					std::cout << "�Ϥ����s�b�A�нT�{�Ϥ�" << endl;// �ˬd�����Ӥ�
				}
				else {
					std::cout << "�w���" << endl;
				}
				imshow("Frame", frame);
				waitKey(1);
				Im_Name = CL.NameCreate(4, i);
				Im_Filename_All = Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype;
				imwrite(Im_Filename_All, frame);
				std::cout << "�w�s�ϡA�� " << i << "�i" << endl;
				break;
			}
		}
	}
	// �����p�g
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);
	// ����e��
	cap.release();
	//��XCSV
	CL.Out_Data_Csv_2f(Csv_Savepath + Csv_Filename_Theory, Points_Laser_Theory, Points_Num_total);//�z�׭�
	std::cout << "Finsih!" << endl;
	return true;
}
bool Cam_Laser_Calibration_v2_BOLB() {
	// �s��Ū���Ϥ��A����ROI�ϰ�ABolb Dection�A�N���Ѩ쪺�I�i���x�s
	//�Ϥ��s�ɦ�m
	String Im_Savepath = FilePath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//CSV�ɮצs�ɦ�m
	String Csv_Savepath = FilePath + FileDate + "\\";
	String Csv_Filename_Dectection = FileDate + "_detected.csv";
	String Csv_Filename_Theory = FileDate + "_theory.csv";
	String Csv_Filename_Theory_v2 = FileDate + "_theory_v2.csv";

	// �ŧi�ܼ�
	Mat frame;
	Mat im_Crop; // ROI���Ϊ��Ϥ�
	Mat im_Gray;
	Mat im_HSV;
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	vector<Point2f> Point_Theory_v2;//�˴����᪺�i�βz���I
	vector<Point2f>	Points_Laser_Detect;
	int get_number = 0; // ��쪺�I�ƶq
	int lose_number = 0;// �򥢪��I�ƶq

	//ROI�Ѽ�
	Rect2d r;//r��Roi����
	//Rect2d r2;
	bool showCrosshair = false;
	bool fromCenter = false;
	int ROI_x, ROI_y, ROI_width, ROI_height;//ROI���U�����x�s

	// Bolb Dection �U���Ѽ�
	SimpleBlobDetector::Params params;

	// Blob Color // �]�w�C��
	//params.blobColor = 150;

	// Change thresholds	//�����������W�U��
	params.minThreshold = 80;
	params.maxThreshold = 120;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1;

	// Filter by Circularity //���
	params.filterByCircularity = false;
	params.minCircularity = 0.5;

	// Filter by Convexity	//�Y��
	params.filterByConvexity = false;
	params.minConvexity = 0.01;

	// Filter by Inertia	//�a�q��
	params.filterByInertia = false;
	params.minInertiaRatio = 0.3;

	// Storage for blobs
	vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	//�ǥ�Ū���z���I�ƶq�A�o��������Ӥ���
	CL.Read_Data_Csv_2f(Csv_Savepath + Csv_Filename_Theory, Point_Theory);

	// ����ܲĤ@�i�Ӥ��A���ROI�ϰ�
	Im_Name = "0000";// �_�l�Ӥ��s��
	frame = imread(Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype);
	r = cv::selectROI("�п��ROI�AESC ����", frame, showCrosshair, fromCenter);//r��Roi����
	ROI_x = r.x;
	ROI_y = r.y;
	ROI_width = r.width;
	ROI_height = r.height;
	std::cout << "ROI Select Succes!" << endl;
	std::cout << "ROI size is" << ROI_width << "," << ROI_height << endl;
	Rect2d r2(ROI_x, ROI_y, ROI_width, ROI_height);
	im_Crop = frame(r2);
	if (im_Crop.empty() == 1) {
		std::cout << "ROI���Υ���!" << endl;
		return 1;
	}
	for (int i = 0; i < Point_Theory.size(); i++) {
		Im_Name = CL.NameCreate(4, i);
		frame = imread(Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype);

		im_Crop = frame(r2);
		im_Crop.convertTo(im_Crop, -1, 1.2, -150);// plan a �վ���׻P�G��
		cvtColor(im_Crop, im_HSV, cv::COLOR_BGR2HSV);//��HSV���
		//std::cout << "imcrop to hsv finish" << endl;
		cvtColor(im_HSV, im_Gray, cv::COLOR_BGR2GRAY);//�Ϥ���Ƕ�
		//std::cout << "hsv to Gray finish" << endl;
		detector->detect(im_Gray, keypoints);//Bolb Dection
		//�P�_�O�_���I
		if (keypoints.size() > 1) {
			std::cout << "Bolb Dection ��" << i << "������h�� 1 �I" << endl;
			lose_number++;
		}
		else if (keypoints.size() < 1) {
			std::cout << "Bolb Dection ��" << i << "������֩� 1 �I" << endl;
			lose_number++;
		}
		else {
			std::cout << "Bolb Dection���b���� ��" << i << "�I ��[" <<
				keypoints[0].pt.x + ROI_x << "," << keypoints[0].pt.y + ROI_y << "]" << endl;//����ϼv���y��
			Point_Theory_v2.push_back(Point2f(Point_Theory[i].x, Point_Theory[i].y));
			Points_Laser_Detect.push_back(Point2f(keypoints[0].pt.x + ROI_x, keypoints[0].pt.y + ROI_y));//�N��쪺�I�x�s
			circle(frame, Point2f(keypoints[0].pt.x + ROI_x, keypoints[0].pt.y + ROI_y), 0.2, Scalar(0, 255, 0), 10);// �p�G���A�Nø��
			get_number++;
		}
		imshow("Result", frame);
		waitKey(3);
	}
	std::cout << "�@��� " << get_number << " �I�A ����� " << lose_number << " �I " << endl;
	for (int i = 0; i < get_number; i++) {
		circle(frame, Points_Laser_Detect[i], 1, Scalar(0, 0, 255), 5);
	}

	while (1) {
		imshow("Result", frame);
		waitKey(1);
		int waitkey = waitKey(5);
		if (waitkey == 27) {
			std::cout << "�����{��" << endl;
			break;
		}
	}
	Im_Name = FileDate + "_Dection_Result";
	Im_Filename_All = Im_Savepath + Im_Name + Im_Savetype;
	//Im_Filename_All = "D:\\59.��������s\\221016laser�ե��P�q�T��X����\\230208\\230208_dection_result.png";
	imwrite(Im_Filename_All, frame);
	CL.Out_Data_Csv_2f(Csv_Savepath + Csv_Filename_Dectection, Points_Laser_Detect, get_number);//�q�����G
	CL.Out_Data_Csv_2f(Csv_Savepath + Csv_Filename_Theory_v2, Point_Theory_v2, get_number);//��s�᪺�z���I
	std::cout << "�@��� " << get_number << " �I�A ����� " << lose_number << " �I " << endl;
	return true;
}
bool Cam_CAL_LSM() {
	//�ɮ׸��|
	String Csv_Path = FilePath + FileDate + "\\";
	String Csv_Header_Get = FileDate + "_detected.csv";
	String Csv_Header_Theory = FileDate + "_theory_v2.csv";
	String Csv_Header_Parameter_x = FileDate + "_x_Cal_Parameter.csv";// x��V���ե��Ѽ�
	String Csv_Header_Parameter_y = FileDate + "_y_Cal_Parameter.csv";// y��V���ե��Ѽ�
	//�ŧi�ܼ�
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	//Ū���ɮ�
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Get, Point_Get);
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Theory, Point_Theory);
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "��" << i << "�I�� " << Point_Get[i].x << "," << Point_Get[i].y << endl;
		std::cout << "��" << i << "�I�� " << Point_Theory[i].x << "," << Point_Theory[i].y << endl;
	}
	//LSM�Ѽƫŧi
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
	std::cout << "X A�� ��" << x_a << endl;
	std::cout << "X B�� ��" << x_b << endl;
	std::cout << "y A�� ��" << y_a << endl;
	std::cout << "y B�� ��" << y_b << endl;
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "�v�� x: " << x_get[i] << " �z��x : " << x_thy[i] << ", �ǰe���p�gx : " << (x_get[i] * x_a + x_b) << endl;
		std::cout << "�v�� y: " << y_get[i] << " �z��y : " << y_thy[i] << ", �ǰe���p�gy : " << (y_get[i] * y_a + y_b) << endl;
		std::cout << endl;
	}
	std::cout << "�����ʫY�� X R : " << x_r << endl;
	std::cout << "�����ʫY�� Y R : " << y_r << endl;

	//�ŧi�x�s�Ϊ��V�q�e��
	vector<Point2f> Parameter_LSM_x;
	vector<Point2f> Parameter_LSM_y;
	Parameter_LSM_x.push_back(Point2f(x_a, x_b));
	Parameter_LSM_y.push_back(Point2f(y_a, y_b));
	CL.Out_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_x, Parameter_LSM_x, Parameter_LSM_x.size());
	CL.Out_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_y, Parameter_LSM_y, Parameter_LSM_y.size());
	return true;
}
bool Cam_CAL_LSM_quadratic() {
	//�ɮ׸��|
	String Csv_Path = FilePath + FileDate + "\\";
	String Csv_Header_Get = FileDate + "_detected.csv";
	String Csv_Header_Theory = FileDate + "_theory_v2.csv";
	String Csv_Header_Parameter_x = FileDate + "_x_Cal_Parameter.csv";// x��V���ե��Ѽ�
	String Csv_Header_Parameter_y = FileDate + "_y_Cal_Parameter.csv";// y��V���ե��Ѽ�
	//�ŧi�ܼ�
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	//Ū���ɮ�
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Get, Point_Get);
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Theory, Point_Theory);
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "��" << i << "�I�� " << Point_Get[i].x << "," << Point_Get[i].y << endl;
		std::cout << "��" << i << "�I�� " << Point_Theory[i].x << "," << Point_Theory[i].y << endl;
	}
	//LSM�Ѽƫŧi
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
	std::cout << "X A�� ��" << x_a << endl;
	std::cout << "X B�� ��" << x_b << endl;
	std::cout << "X C�� ��" << x_c << endl;
	std::cout << "y A�� ��" << y_a << endl;
	std::cout << "y B�� ��" << y_b << endl;
	std::cout << "y C�� ��" << y_c << endl;
	for (int i = 0; i <= Point_Get.size() - 1; i++) {
		std::cout << "�v�� x: " << x_get[i] << " �z��x : " << x_thy[i] << ", �ǰe���p�gx : " << (x_get[i] * x_get[i] * x_a + x_get[i] * x_b + x_c) << endl;
		std::cout << "�v�� y: " << y_get[i] << " �z��y : " << y_thy[i] << ", �ǰe���p�gy : " << (y_get[i] * y_get[i] * y_a + y_get[i] * y_b + y_c) << endl;
		std::cout << endl;
	}


	return true;
}
bool Cam_Laser_Calibration_repeatability() {
	// 230610 : ���Ʃʴ��աAonline
	// �P�@�I�����h��
	// �ŧi�ܼ�
	Mat frame;// �۾��Ӥ��e��
	Mat frame_copy;
	Mat im_Crop;
	Mat im_Gray;
	Mat im_HSV;
	vector<Point2f> Point_Get;
	vector<Point2f> Point_Theory;
	vector<Point2f> Point_Theory_v2;//�˴����᪺�i�βz���I
	vector<Point2f>	Points_Laser_Detect;
	vector<vector<Point2f>> Point_Theory_v2_time;
	vector<vector<Point2f>>Points_Laser_Detect_time;

	//ROI�Ѽ�
	Rect2d roi;//Roi�e��
	bool showCrosshair = false;
	bool fromCenter = false;

	// �إ߸�Ƨ�
	string Folder_Path = FilePath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// �p�G�ɮפ��s�b
		command = "mkdir -p " + Folder_Path;// �Hdos�إ߸�Ƨ�
		system(command.c_str());

	}

	// XML�ɮ�
	String Xml_Savepath = Folder_Path + "\\";
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml(Xml_Savepath + Xml_Filename, FileStorage::WRITE);// �x�s�e��
	if (!Fs_Xml.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}

	//-----�p�g�Ѽ�-----//
	string code_out;// �����X��"�r��"
	char* code_out_ch;// �A���Φ�'�r��'�ǰe
	int Point_Numbers = 0;//�Ω�����`�I�ƪ��_�l��
	char read_ch;// Ū���T�{�X�Ϊ�
	string read_str;// �զX�����쪺���O
	int Lasertimes = 30;// �p�g��X�Ӵ`�����A�@�Ӵ`���O�@�㭱
	vector<Point2f>Points_Laser_Theory;// �z�׹p�g�I���e��(�h�`��)
	int x1 = 0;// �ǰe���I���e��
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	UINT BytesInQue;// �w�İϮe��

	// Bolb Dection �U���Ѽ�
	SimpleBlobDetector::Params params;
	// Change thresholds	//�����������W�U��
	params.minThreshold = 80;
	params.maxThreshold = 120;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1;

	// Filter by Circularity //���
	params.filterByCircularity = false;
	params.minCircularity = 0.5;

	// Filter by Convexity	//�Y��
	params.filterByConvexity = false;
	params.minConvexity = 0.01;

	// Filter by Inertia	//�a�q��
	params.filterByInertia = false;
	params.minInertiaRatio = 0.3;

	// Storage for blobs
	vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// �۾��Ѽ�
	VideoCapture cap;//���}SerialPort 0 ���۾�


	// �إ߹p�g�z���I
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}

	PortOpen();// ���}�p�g���誺port�f
	//mySerialPort.OpenListenThread();// ���}���p�q�T��ť
	Sleep(1 * 1000);
	cap.open(port_cam, apiID);// ���}�۾�
	if (!cap.isOpened()) {
		std::cout << "Can't open the camera, port number is " << port_cam << std::endl;
	}
	else {
		std::cout << "Camera is opened, port number is " << port_cam << std::endl;
		cap.set(cv::CAP_PROP_FRAME_WIDTH, Width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, Height);
	}

	Sleep(2 * 1000);

	// ���ROI
	cap.read(frame);// �۾����
	if (frame.empty() == 1) {
		std::cout << "Error while camera read frame!" << endl;// �ˬd�����Ӥ�
		return 1;
	}
	roi = cv::selectROI("�п��ROI�AESC ����", frame, showCrosshair, fromCenter);

	for (int j = 0; j < Lasertimes; j++) {
		for (int i = 0; i < Points_Num_total; i++) {
			//����p�g�I����w�I��
			x1 = CL.cal_0to2048((int)Points_Laser_Theory[i].x);
			y1 = CL.cal_0to2048((int)Points_Laser_Theory[i].y);
			code_out = "01" + CL.int2string_4ch(x1) + CL.int2string_4ch(y1) + CL.int2string_4ch(x2) + \
				CL.int2string_4ch(y2) + CL.int2string_4ch(x3) + CL.int2string_4ch(y3) + CL.int2string_4ch(x4) + CL.int2string_4ch(y4);//�Ӫ��A����
			code_out_ch = (char*)code_out.data();

			for (int i = 0; i < 5; i++) {
				mySerialPort.WriteData(code_out_ch, 34);//�ǰe�r�굹Arduino
			}
			Sleep(200);

			BytesInQue = mySerialPort.GetBytesInCOM();// �T�{�w�İϦ��L���
			while (1) {
				BytesInQue = mySerialPort.GetBytesInCOM();// ���s�T�{�w�İϦ��L���
				mySerialPort.ReadChar(read_ch);// ���sŪ�����
				if (BytesInQue != 0 && read_ch == 'N') {
					std::cout << "������F : " << read_ch << endl;
					while (1) {
						cap.read(frame);// �۾����
						frame_copy = frame.clone();
						if (frame.empty() == 1) {
							std::cout << "Error while camera read frame!" << endl;// �ˬd�����Ӥ�
							return 1;
						}

						// Blob detector
						im_Crop = frame(roi);
						im_Crop.convertTo(im_Crop, -1, 1.2, -150);// plan a �վ���׻P�G��
						cvtColor(im_Crop, im_HSV, cv::COLOR_BGR2HSV);//��HSV���
						//std::cout << "imcrop to hsv finish" << endl;
						cvtColor(im_HSV, im_Gray, cv::COLOR_BGR2GRAY);//�Ϥ���Ƕ�
						//std::cout << "hsv to Gray finish" << endl;
						detector->detect(im_Gray, keypoints);//Bolb Dection
						//�P�_�O�_���I

						if (keypoints.size() > 1) {
							std::cout << "Bolb Dection ��" << i << "������h�� 1 �I" << endl;
						}
						else if (keypoints.size() < 1) {
							std::cout << "Bolb Dection ��" << i << "������֩� 1 �I" << endl;
						}
						else {
							std::cout << "Bolb Dection���b���� ��" << i << "�I ��[" <<
								keypoints[0].pt.x + roi.x << "," << keypoints[0].pt.y + roi.y << "]" << endl;//����ϼv���y��
							Point_Theory_v2.push_back(Point2f(Points_Laser_Theory[i].x, Points_Laser_Theory[i].y));
							Points_Laser_Detect.push_back(Point2f(keypoints[0].pt.x + roi.x, keypoints[0].pt.y + roi.y));//�N��쪺�I�x�s
							circle(frame_copy, Point2f(keypoints[0].pt.x + roi.x, keypoints[0].pt.y + roi.y), 0.2, Scalar(0, 255, 0), 10);// �p�G���A�Nø��
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
	// ��X���
	Fs_Xml << "Point_Theory_v2_time" << Point_Theory_v2_time;
	Fs_Xml << "Points_Laser_Detect_time" << Points_Laser_Detect_time;

	// �����p�g
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);
	std::cout << "Finish!" << std::endl;

	return true;
}
bool Cam_Laser_repeatability_result() {
	// 230622 : Ū��data���R���Ʃ�
	// �ɮ׸��|
	string Folder_Path = FilePath + FileDate;
	// XML�ɮ�
	String Xml_Savepath = Folder_Path + "\\";
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml_read(Xml_Savepath + Xml_Filename, FileStorage::READ);// �x�s�e��
	if (!Fs_Xml_read.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}
	// �ŧi�ܼ�
	vector<vector<Point2f>> Point_Theory_v2_time;
	vector<vector<Point2f>>Points_Laser_Detect_time;

	// Ū�����
	Fs_Xml_read["Point_Theory_v2_time"] >> Point_Theory_v2_time;
	Fs_Xml_read["Points_Laser_Detect_time"] >> Points_Laser_Detect_time;

	// �������Ū�����
	/*for (int i = 0; i < Points_Laser_Detect_time.size(); i++) {
		for (int j = 0; j < Points_Laser_Detect_time[0].size(); j++) {

			cout << "time : "<<i<<", point : "<<j<<" result : "<<Points_Laser_Detect_time[i][j] << endl;

		}
	}*/
	// ���R���G
	// 1.������
	float x_point = 0; float y_point = 0;
	float x_average, y_average;
	float x_error = 0; float y_error = 0;
	float x_error_max = 0; float y_error_max = 0;
	float x_error_min = 1; float y_error_min = 1;
	float x_error_total = 0;
	float y_error_total = 0;
	float x_error_average = 0;
	float y_error_average = 0;

	vector<Point2f> point_dec_average;// ������
	vector<Point2f> point_dec_mean_error;// �����~�t
	vector<Point2f> point_dec_max_error;// �����~�t
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
		x_point = 0;// �Χ�����A�ܼ��k�s
		y_point = 0;
	}
	// ��ܥ����ȵ��G
	for (int i = 0; i < point_dec_average.size(); i++) {
		cout << "��" << i << "�I�A����" << point_dec_average[i] << endl;
	}
	// 2.�p��~�t
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
		/*cout << "��" << i << "�I�A����" << point_dec_average[i] << endl;*/
		x_error_total = 0;// �Χ�����A�ܼ��k�s
		y_error_total = 0;
		x_error_max = 0;
		y_error_max = 0;
	}

	// ��ܥ����~�t���G
	for (int i = 0; i < point_dec_mean_error.size(); i++) {
		cout << "��" << i << "�I�A�����~�t" << point_dec_mean_error[i] << endl;
	}
	// ��̤ܳj�~�t
	for (int i = 0; i < point_dec_mean_error.size(); i++) {
		cout << "��" << i << "�I�A�̤j�~�t" << point_dec_max_error[i] << endl;
	}
	// �p��������Ʃʻ~�t�A�n�����L�j���I

	// ��X��CSV�ɮ�
	String Filename_mean_error = FileDate + "_mean_error";
	String Filename_max_error = FileDate + "_max_error";
	String Filename_theory = FileDate + "_theory";
	String File_type = ".csv";

	CL.Out_Data_Csv_2f(Folder_Path + "\\" + Filename_mean_error + File_type, point_dec_mean_error, point_dec_mean_error.size());
	CL.Out_Data_Csv_2f(Folder_Path + "\\" + Filename_theory + File_type, Point_Theory_v2_time[0], Point_Theory_v2_time[0].size());// ��X�z���I�H�ѰѦ�
	return true;
}
int Feature_Match_Online_v5() {
	Mat img1;// img1 : �n���Ӥ�諸�Ӥ��Apattam
	Mat frame;// �۾���U�窺�v��
	Mat frame_copy;// �ƻs���v���A�Ω�ø��
	int num = 0;
	vector<Point2f> coner_point(4);// �v���y�ФW��Feature Match�|���I
	vector<Point2f> coner_point_Laser(4);// �ഫ��p�g�y�Ъ��|���I
	img1 = imread(FilePath + "221121_�S�x�ǰt\\pic008.jpg");
	String code_out;// ��X��X1 code���O
	char* code_out_ch;// ��X��X1 code���O char���A

	// Ū��CSV�ɮצU���ե���Ѽ�
	String Csv_Path = FilePath + FileDate + "\\";
	String Csv_Header_Parameter_x = FileDate + "_x_Cal_Parameter.csv";// x��V���ե��Ѽ�
	String Csv_Header_Parameter_y = FileDate + "_y_Cal_Parameter.csv";// y��V���ե��Ѽ�

	// �ե��Ѽ� �e��
	vector<Point2f>Parameter_x;// x ��V���ե��ѼơA .x�� a�A .y�� b
	vector<Point2f>Parameter_y;// y ��V���ե��ѼơA .x�� a�A .y�� b

	// Ū���ե��Ѽ�
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_x, Parameter_x);
	CL.Read_Data_Csv_2f(Csv_Path + Csv_Header_Parameter_y, Parameter_y);

	// ���}�U���]��
	PortOpen();// ���}�s���p�g���誺�s���f
	mySerialPort.OpenListenThread();
	VideoCapture cap;// ���}SerialPort 0 ���۾�
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
			std::cout << "�Ϥ����s�b�A�нT�{�Ϥ�" << endl;
			return -1;
		}

		if (!img1.data || !frame.data)
		{
			std::cout << "�Ϥ������I" << endl;
			return -1;
		}

		//imshow("Feature Match", frame);
		CL.Feature_ORB(0, 0, img1, frame, coner_point);//Feature Match�o��|���I
		frame_copy = frame.clone();
		for (int j = 0; j < 4; j++) {
			line(frame_copy, coner_point[j], coner_point[(j + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);// ����
			//std::cout << num << "     ��" << j << "�I" << coner_point[j];
		}
		for (int k = 0; k < 4; k++) {
			CL.LSM_Trans(coner_point[k], coner_point_Laser[k], Parameter_x[0].x, Parameter_x[0].y, Parameter_y[0].x, Parameter_y[0].y);
		}
		std::cout << endl;
		CL.Xcode_ctrl(coner_point_Laser, code_out);
		code_out_ch = (char*)code_out.data();// �ഫ���r��
		mySerialPort.WriteData(code_out_ch, 34);
		//std::cout << "�w�ǰe�r��" << code_out << endl;

		coner_point.clear();
		imshow("img1", img1);
		imshow("Frame result", frame_copy);
		namedWindow("Frame result", WINDOW_NORMAL);

		if (waitkey == 27) {
			std::cout << "�����{��" << endl;

			break;
		}
	}
	Sleep(1000);// ����1���A�A�e�X���O
	// �����p�g
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);
	// ����e��
	frame.release();
	cap.release();
	return 0;
}
int Cam_Laser_Calibration_stereo_v2() {
	// 230306�A���ե���ե�

	// ���إ߸�Ƨ�
	string Folder_Path = FilePath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// �p�G�ɮפ��s�b
		command = "mkdir -p " + Folder_Path;// �Hdos�إ߸�Ƨ�
		system(command.c_str());
	}

	//-----rs�Ѽ�-----//
	rs2::pipeline pipe; // rs�Ω󱵦��v�����e��
	rs2::config cfg;// rs�ۭq�Ѽ�
	float dist; // rs�x�s�`�ת��e��
	static Point p;// rs���I�����I�e��
	String Csv_Filename_Depth;
	String Csv_File_Path;
	// �ŧi�Ѽ�
	int w = 1280;
	int h = 720;
	int save_im_number = 0;// �Ω�p��Ӥ��ƶq

	// �إ�CSV�ɮ�
	String Csv_Savepath = FilePath + FileDate + "\\";
	ofstream Outfile;// Csv�ɮ׮e��

	// XML�ɮ�
	String Xml_Savepath = Csv_Savepath;
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml(Xml_Savepath + Xml_Filename, FileStorage::WRITE);// �x�s�e��
	if (!Fs_Xml.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}


	//-----�p�g�Ѽ�-----//
	string code_out;// �����X��"�r��"
	char* code_out_ch;// �A���Φ�'�r��'�ǰe
	int Point_Numbers = 0;//�Ω�����`�I�ƪ��_�l��
	char read_ch;// Ū���T�{�X�Ϊ�
	string read_str;// �զX�����쪺���O
	int Lasertimes = 3;// �p�g��X�Ӵ`�����A�@�Ӵ`���O�@�㭱
	vector<Point2f>Points_Laser_Theory;// �z�׹p�g�I���e��
	int x1 = 0;// �ǰe���I���e��
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	// �إ߹p�g�z���I
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}

	//-----�x�s�v����m-----//
	String Savepath = FilePath;
	String Im_Savepath = Savepath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;


	// �x�s�z���I��XML
	for (int k = 0; k < Lasertimes - 1; k++) {
		for (int i = 0; i <= Points_R_Num; i++) {
			for (int j = 0; j <= Points_C_Num; j++) {
				Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
			}
		}
	}
	Fs_Xml << "Point_Laser_Theory" << Points_Laser_Theory;
	// ��X�Ӥ��`�� 
	save_im_number = Lasertimes * (Points_R_Num + 1) * (Points_R_Num + 1);
	Fs_Xml << "save_im_number" << save_im_number;
	Fs_Xml.release();
	// �����p�g
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);

	return 0;
}
int Cam_Laser_Calibration_stereo_v3() {
	// 230712�A���ե���ե�

	// ���إ߸�Ƨ�
	string Folder_Path = FilePath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// �p�G�ɮפ��s�b
		command = "mkdir -p " + Folder_Path;// �Hdos�إ߸�Ƨ�
		system(command.c_str());
	}

	//-----rs�Ѽ�-----//
	rs2::pipeline pipe; // rs�Ω󱵦��v�����e��
	rs2::config cfg;// rs�ۭq�Ѽ�
	rs2::frameset frames; // ���ݤU�@�ռv��
	rs2::frame color_frame;// ���o�m��v��

	float dist; // rs�x�s�`�ת��e��
	static Point p;// rs���I�����I�e��
	String Csv_Filename_Depth;
	String Csv_File_Path;
	// �ŧi�Ѽ�
	int w = 1280;
	int h = 720;
	int save_im_number = 0;// �Ω�p��Ӥ��ƶq

	// �إ�CSV�ɮ�
	String Csv_Savepath = FilePath + FileDate + "\\";
	ofstream Outfile;// Csv�ɮ׮e��

	// XML�ɮ�
	String Xml_Savepath = Csv_Savepath;
	String Xml_Filename = FileDate + ".xml";
	FileStorage Fs_Xml(Xml_Savepath + Xml_Filename, FileStorage::WRITE);// �x�s�e��
	if (!Fs_Xml.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << Xml_Savepath << endl;
		std::cout << "File Name : " << Xml_Filename << endl;
		return false;
	}

	//Ū�����v���榡
	cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30); // BGR888�榡�m��v�� 30fps
	cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 30); // 16 bit�榡�Ƕ��`�׼v�� 30fps

	//-----�p�g�Ѽ�-----//
	string code_out;// �����X��"�r��"
	char* code_out_ch;// �A���Φ�'�r��'�ǰe
	int Point_Numbers = 0;//�Ω�����`�I�ƪ��_�l��
	char read_ch;// Ū���T�{�X�Ϊ�
	string read_str;// �զX�����쪺���O
	int Lasertimes = 3;// �p�g��X�Ӵ`�����A�@�Ӵ`���O�@�㭱
	vector<Point2f>Points_Laser_Theory;// �z�׹p�g�I���e��(�h�`��)
	int x1 = 0;// �ǰe���I���e��
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int x3 = 0;
	int y3 = 0;
	int x4 = 0;
	int y4 = 0;
	// �إ߹p�g�z���I
	for (int i = 0; i <= Points_R_Num; i++) {
		for (int j = 0; j <= Points_C_Num; j++) {
			Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
		}
	}
	PortOpen();// ���}�p�g���誺port�f

	Sleep(3 * 1000);

	//-----�x�s�v����m-----//
	String Savepath = FilePath;
	String Im_Savepath = Savepath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	// ��L�ܼ�
	rs2_intrinsics intr;
	int cycle;// �Ω���ΨC���ե��`��

	// �}�lŪ���v��
	pipe.start(cfg);


	int waitkey = waitKey(3);
	for (int i = 0; i < Lasertimes; i++) {
		for (int j = 0; j < Points_Num_total; j++) {
			//����p�g�I����w�I��
			x1 = CL.cal_0to2048((int)Points_Laser_Theory[j].x);
			y1 = CL.cal_0to2048((int)Points_Laser_Theory[j].y);
			code_out = "01" + CL.int2string_4ch(x1) + CL.int2string_4ch(y1) + CL.int2string_4ch(x2) + \
				CL.int2string_4ch(y2) + CL.int2string_4ch(x3) + CL.int2string_4ch(y3) + CL.int2string_4ch(x4) + CL.int2string_4ch(y4);//�Ӫ��A����
			code_out_ch = (char*)code_out.data();
			for (int k = 0; k < 5; k++) {
				mySerialPort.WriteData(code_out_ch, 34);//�ǰe�r�굹Arduino
			}
			std::cout << "PC Send :" << endl;
			std::cout << code_out_ch << endl;
			std::cout << "send points : " << j << endl;
			Sleep(200);
			UINT BytesInQue = mySerialPort.GetBytesInCOM();// �T�{�w�İϦ��L���
			while (1) {

				if (BytesInQue != 0) {
					mySerialPort.ReadChar(read_ch);// ���sŪ�����
					if (read_ch == 'N') {
						std::cout << "������F : " << read_ch << endl;
						// �}�lŪ���v��
						frames = pipe.wait_for_frames();// ���ݤU�@�ռv��
						color_frame = frames.get_color_frame();// ���o�m��v��
						rs2::depth_frame depth = frames.get_depth_frame();// ���o�`��

						// �ഫ��opencv�榡
						Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP); // �m��v��

						if (color_image.empty() == 1) {
							std::cout << "�Ϥ����s�b�A�нT�{�Ϥ�" << endl;// �ˬd�����Ӥ�
							return 1;
						}
						else {
							// �HOpenCV�禡����^����v��
							imshow("Color Image", color_image); // �m��v��

							// �x�s�Ӥ�
							waitKey(1);
							Im_Name = CL.NameCreate(4, save_im_number);
							Im_Filename_All = Im_Savepath + Im_Filename_Title + Im_Name + Im_Savetype;
							imwrite(Im_Filename_All, color_image);
							std::cout << "�x�s��" << save_im_number << "�i�Ӥ�" << endl;

							// �x�s�`�׸�T
							Csv_Filename_Depth = FileDate + "_depth" + CL.int2string_4ch(save_im_number) + ".csv";
							Csv_File_Path = Csv_Savepath + Csv_Filename_Depth;
							Outfile.open(Csv_File_Path, ios::out);

							// �C�C�񧹴��@�C�A�ѥ���k�A�A�ѤW��U
							for (int row = 0; row < w; row++) {
								for (int col = 0; col < h; col++) {
									p = Point(row, col);
									dist = depth.get_distance(p.x, p.y);
									//dist = 1;
									Outfile << dist << ",";
								}
								Outfile << endl;// ����
							}
							std::cout << "�x�s��" << save_im_number << "�� .csv�ɮ�" << endl;
							Outfile.close();
							save_im_number++;
							break;
						}

					}

				}
				else {
					cout << "Didn't read.." << endl;
					BytesInQue = mySerialPort.GetBytesInCOM();// ���s�T�{�w�İϦ��L���
				}
				Sleep(100);

			}
		}
		cout <<"������ "<<i<< " �Ӵ`���A�п�J���N�Ʀr�H�~��" << endl;
		cin >> cycle;
	}
	// ��X����
	intr = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Realsense����
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

	// �x�s�z���I��XML
	for (int k = 0; k < Lasertimes - 1; k++) {
		for (int i = 0; i <= Points_R_Num; i++) {
			for (int j = 0; j <= Points_C_Num; j++) {
				Points_Laser_Theory.push_back(Point2f(i * Points_Pro_Range / Points_R_Num, j * Points_Pro_Range / Points_C_Num));
			}
		}
	}
	Fs_Xml << "Point_Laser_Theory" << Points_Laser_Theory;
	// ��X�Ӥ��`��
	Fs_Xml << "save_im_number" << save_im_number;
	Fs_Xml.release();


	// �����p�g
	code_out = CL.End_code;
	code_out_ch = (char*)code_out.data();
	mySerialPort.WriteData(code_out_ch, 34);

	return 0;
}
int Cam_Laser_Calibration_stereo_Blob_v3() {
	/*
	* 230307�AŪ�������ɮסA���Ѽv�����p�g
	*�N����Ѫ���m�ι����`�׭p��X��
	*!!!�ݭn�p����I���u��y�СA�ӫD���������y�Ь�XY����
	*/
	// �s��Ū���Ϥ���m
	// ���q�i�H�s�����ROI�}�l
	//

	// �Ϥ����|
	String Im_Savepath = FilePath + FileDate + "\\";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	//-----�x�s������v����m-----//
	String Im_Savepath_Dection = Im_Savepath + "dection\\";
	String Im_Filename_Title_Dection = FileDate + "_image_dection";
	String Im_Name;
	String Im_Filename_All;

	// �إ߰������I����Ƨ�
	string Folder_Path = FilePath + FileDate + "\\dection";
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// �p�G�ɮפ��s�b
		command = "mkdir -p " + Folder_Path;// �Hdos�إ߸�Ƨ�
		system(command.c_str());
	}

	// XML�ɮ�
	String Data_savepath = Im_Savepath;
	String Filename = FileDate + ".xml";
	cout << Data_savepath + Filename << endl;

	FileStorage Fs_Xml_Read(Data_savepath + Filename, FileStorage::READ); // �x�s�e��
	
	if (!Fs_Xml_Read.isOpened()) {
		std::cout << "Fail to read file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}
	


	// csv�ɮ�
	String Csv_Savepath = Im_Savepath;
	String Csv_filename;

	// ROI�Ѽ�
	vector<vector<Point> > squares;// ��쪺�x�ήe��
	Rect2d r;// ROI�e��
	Mat Im_crop;
	Mat Im_gray;
	Mat Im_HSV;
	int thresh = 5, N = 10;// Canny�˴����ѼơA�S�ƥ����n�ʦn�F�A�аѦ�Opencv���d��
	double square_max_area = 0;
	int square_local = 0;


	// Bolb Dection �U���Ѽ�
	SimpleBlobDetector::Params params;
	params.filterByColor = true;
	params.blobColor = uchar(0);
	params.minThreshold = 85;// Change thresholds	//�����������W�U��
	params.maxThreshold = 120;
	params.filterByArea = true;// Filter by Area.
	params.minArea = 1;
	params.filterByCircularity = false;// Filter by Circularity //���
	params.minCircularity = 0.4;
	params.filterByConvexity = false;// Filter by Convexity	//�Y��
	params.minConvexity = 0.4;
	params.filterByInertia = false;// Filter by Inertia	//�a�q��
	params.minInertiaRatio = 0.3;

	// Storage for blobs
	vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// ��L�ܼ�
	Mat Im;// Ū�J���v��
	Mat Im_clone;// �ƻs���v��
	int save_im_number;// �Ӥ��`��
	int get_number = 0; // ��쪺�I�ƶq
	int lose_number = 0;// �򥢪��I�ƶq
	Point2f Point_source;// �b��ϤW���p�g�I��m�A�קK�~�ϥΨ�ROI����m
	vector<int>Im_get_number;// ��쪺�Ӥ����ǽs��
	vector<Point2f> Point_Theory;
	vector<Point2f> Point_Theory_Used;//�˴����᪺�i�βz���I�A�p�g�y��
	vector<Point2f>Points_Laser_Detect;// �˴������I�A����
	vector<Point3f>Points_Laser_Detect_RW;// �˴������I�A3D
	vector<double>Depth_vector;// �˴������I�������`��
	Mat_<double> Depth_mat;// �qcsv�ɮ�Ū���`�ׯx�}���e��

	//rs�ܼ�
	rs2_intrinsics intr;
	int intr_width;
	int intr_height;
	float intr_ppx;
	float intr_ppy;
	float intr_fx;
	float intr_fy;
	String intr_model;
	vector<float> intr_coeffs;

	// Ū�����
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

	Fs_Xml_Read.release();// Ū��������e��

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
	// �D�{��
	for (int Im_number = 0; Im_number < save_im_number; Im_number++) {
		Im = imread(Im_Savepath + Im_Filename_Title + CL.int2string_4ch(Im_number) + Im_Savetype);
		Im_clone = Im.clone();
		CL.findSquares(Im_clone, squares, N, thresh);
		CL.square_max(squares, square_max_area, square_local);
		if (CL.length_2P(squares[square_local][1], squares[square_local][0]) < 900 && CL.length_2P(squares[square_local][2], squares[square_local][1]) < 900) {
			r = boundingRect(squares[square_local]);
			for (int j = 0; j < 4; j++) {
				// ø�s�x�ο��ѵ��G
				//circle(im, squares[square_local][j], 1, Scalar(0, 255, 0), 5);
				//line(im, squares[square_local][j], squares[square_local][(j + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);
				//// roi�e�ϡA����
				//line(im, point2f(r.x, r.y), point2f(r.x + r.width, r.y), scalar(0, 255, 255), 2, 8, 0);
				//line(im, point2f(r.x + r.width, r.y), point2f(r.x + r.width, r.y + r.height), scalar(0, 255, 255), 2, 8, 0);
				//line(im, point2f(r.x + r.width, r.y + r.height), point2f(r.x, r.y + r.height), scalar(0, 255, 255), 2, 8, 0);
				//line(im, point2f(r.x, r.y + r.height), point2f(r.x, r.y), scalar(0, 255, 255), 2, 8, 0);
			}
		}

		// ���ե��NRGB�v�����ȡA�����G�����A���C�·t����
		Im_crop = Im_clone(r);
		convertScaleAbs(Im_crop, Im_crop, 1.2, -100);
		Mat element_1 = getStructuringElement(MORPH_RECT, Size(1, 1));
		Mat element_2 = getStructuringElement(MORPH_ELLIPSE, Size(1, 1));
		Mat element_3 = getStructuringElement(MORPH_RECT, Size(6, 6));
		Mat element_4 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

		erode(Im_crop, Im_crop, element_1);// �I�k	
		erode(Im_crop, Im_crop, element_2);// �I�k	
		dilate(Im_crop, Im_crop, element_3);// ����			
		dilate(Im_crop, Im_crop, element_4);// ����			
		cvtColor(Im_crop, Im_HSV, cv::COLOR_BGR2HSV);//��HSV���
		cvtColor(Im_HSV, Im_gray, cv::COLOR_BGR2GRAY);//�Ϥ���Ƕ�

		//---Bolb Dection---//
		detector->detect(Im_gray, keypoints);//Bolb Dection

		if (keypoints.size() == 1) {

			for (int i = 0; i < keypoints.size(); i++) {
				Point_source = Point2f(keypoints[i].pt.x + r.x, keypoints[i].pt.y + r.y);// ���ץ�����Ϫ���m
				circle(Im_clone, Point_source, 0.2, Scalar(0, 255, 0), 10);// �p�G���A�Nø��
			}
			get_number++;
			// �x�s���p�g�I���Ϥ�
			Im_Filename_All = Im_Savepath_Dection + Im_Filename_Title_Dection + "_" + CL.int2string_4ch(Im_number) + Im_Savetype;
			imwrite(Im_Filename_All, Im_clone);

			// �N����I����ƶi���x�s
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
		std::cout << "�@" << save_im_number << "�i�A�ثe��" << Im_number << "�i" << endl;
	}
	// �N��쪺�I�A�ഫ��3D�y��
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
	FileStorage Fs_Xml_Write(Data_savepath + Filename, FileStorage::WRITE); // �x�s�e��
	if (!Fs_Xml_Write.isOpened()) {
		std::cout << "Fail to write file! " << endl;
		std::cout << "File Path : " << Data_savepath << endl;
		std::cout << "File Name : " << Filename << endl;
		return false;
	}
	// ���мg�ª����
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

	// �s�W�����
	Fs_Xml_Write << "Points_Laser_Detect" << Points_Laser_Detect;
	Fs_Xml_Write << "Point_Theory_Used" << Point_Theory_Used;
	Fs_Xml_Write << "Depth_vector" << Depth_vector;
	Fs_Xml_Write << "Points_Laser_Detect_RW" << Points_Laser_Detect_RW;
	Fs_Xml_Write << "get_number" << get_number;

	std::cout << "�@�@�@" << save_im_number << "�I" << endl;
	std::cout << "�@���" << get_number << "�I" << endl;
	std::cout << "�@��" << lose_number << "�I" << endl;
	return 0;
}

/*-----��L�{��-----*/
int Feature_Match_most_pic() {
	vector<Mat> Pattern_pics;
	vector<String> Pattern_list;
	Mat frame;
	Mat frame_copy;
	int Pattern_number = 2;// �M��WŪ���X�i�A�H�e���ƨ�᭱
	//vector<Point2f>coner_point_list[];
	vector<vector<Point2f>> coner_point_list;
	vector<Point2f>coner_point;
	String window_name = "pattern";
	// Ū���Ӥ��M��A�w�p�Hxml�ɮ�Ū��
	Pattern_list.push_back(FilePath + "221121_�S�x�ǰt\\pic009.jpg");
	Pattern_list.push_back(FilePath + "221121_�S�x�ǰt\\pic010.jpg");

	if (Pattern_number > Pattern_list.size()) {
		std::cout << "ERROR!�]�wŪ���Ӥ��Ƥj��M��!" << std::endl;
		return 1;
	}

	// �N�M��W��Pattan�Ӥ���Ū���i��
	for (int i = 0; i < Pattern_number; i++) {
		Mat im_read = imread(Pattern_list[i]);
		if (im_read.empty() == 1) {
			std::cout << "ERROR when read Pattern!" << endl;
			return 1;
		}
		Pattern_pics.push_back(im_read);
	}

	// �}�l����
	VideoCapture cap;// ���}SerialPort 0 ���۾�
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
			std::cout << "ERROR�A�۾����!" << endl;
			return 1;
		}

		for (int i = 0; i < Pattern_number; i++) {
			imshow(window_name + CL.int2string_4ch(i), Pattern_pics[i]);
		}
		for (int i = 0; i < Pattern_number; i++) {
			CL.Feature_ORB(0, 0, Pattern_pics[i], frame, coner_point);//Feature Match�o��|���I
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
				line(frame_copy, coner_point_list[i][j], coner_point_list[i][(j + 1) % 4], Scalar(0, 0, 255), 2, 8, 0);//����	

				//line(frame_copy, coner_point_list[1][j], coner_point_list[1][(j + 1) % 4], Scalar(0, 255, 0), 2, 8, 0);//����	
			}
		}
		coner_point_list.clear();
		imshow("frame", frame_copy);
		cv::waitKey(3);
		if (waitkey == 27) {
			std::cout << "�����{��" << endl;
			break;
		}
	}
	return 0;
}
int ROI_square_offline() {
	// �s��Ū���Ϥ��A����ROI�ϰ�A�bROI�ϰ���ѯx��
	//�Ϥ��s�ɦ�m
	String Im_Savepath = "E:\\����\\43.�H����@\\���ɻP�v��_�����\\230408_ROI�ϰ�x�ο���_���ռv��\\out2";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//ROI�Ѽ�
	Rect2d r;//r��Roi����
	//Rect2d r2;
	bool showCrosshair = false;
	bool fromCenter = false;
	int ROI_x, ROI_y, ROI_width, ROI_height;//ROI���U�����x�s

	Mat Im;
	Mat Im_crop;
	vector<vector<Point> > squares;
	int thresh = 125, N = 10;
	double square_max_area = 2;
	int square_local = 0;
	float length_width, length_height;
	float length_width_min = 1;// �˴��x����ɤU��
	float length_width_max = 500;// �˴��x����ɤW��
	float length_height_min = 1;// �˴��x����ɤU��
	float length_height_max = 100;// �˴��x����ɤW��

	Im_Filename_All = Im_Savepath + "\\" + CL.int2string_4ch(0) + Im_Savetype;
	Im = imread(Im_Filename_All);
	if (Im.empty()) {
		cout << "error in read image" << endl;
		cout << "path : " << Im_Filename_All << endl;
		return 1;
	}
	r = cv::selectROI("�п��ROI�AESC ����", Im, showCrosshair, fromCenter);//r��Roi����
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
		if (squares.empty() == 0) {// �p�G���s�b�x�ΡA�~�p��
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
						line(Im, P1, P2, Scalar(0, 0, 255), 2, 8, 0);//����
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
	// �s��Ū���Ϥ��A����ROI�ϰ�A�bROI�ϰ���ѯx��
	//�Ϥ��s�ɦ�m
	String Im_Savepath = "E:\\����\\43.�H����@\\���ɻP�v��_�����\\230408_ROI�ϰ�x�ο���_���ռv��\\out2";
	String Im_Filename_Title = FileDate + "_image_";
	String Im_Savetype = ".png";
	String Im_Name;
	String Im_Filename_All;

	//ROI�Ѽ�
	Rect2d r;//r��Roi����
	//Rect2d r2;
	bool showCrosshair = false;
	bool fromCenter = false;
	int ROI_x, ROI_y, ROI_width, ROI_height;//ROI���U�����x�s

	Mat frame;
	Mat Im_crop;
	vector<vector<Point> > squares;
	int thresh = 125, N = 10;
	double square_max_area = 2;
	int square_local = 0;
	float length_width, length_height;
	float length_width_min = 1;// �˴��x����ɤU��
	float length_width_max = 500;// �˴��x����ɤW��
	float length_height_min = 1;// �˴��x����ɤU��
	float length_height_max = 100;// �˴��x����ɤW��

	VideoCapture cap;//���}SerialPort 0 ���۾�
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
		std::cout << "�Ϥ����s�b�A�нT�{�Ϥ�" << endl;
		//return 1;
	}
	r = cv::selectROI("�п��ROI�AESC ����", frame, showCrosshair, fromCenter);//r��Roi����
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
			std::cout << "�Ϥ����s�b�A�нT�{�Ϥ�" << endl;
			//return 1;
		}

		Im_crop = frame(r);
		CL.findSquares(Im_crop, squares, N, thresh);
		if (squares.empty() == 0) {// �p�G���s�b�x�ΡA�~�p��
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
						line(frame, P1, P2, Scalar(0, 0, 255), 2, 8, 0);//����
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

	// ���إ߸�Ƨ�
	string Folder_Path = Savepath + FileDate;
	string command;
	if (_access(Folder_Path.c_str(), 0) == -1) {// �p�G�ɮפ��s�b
		command = "mkdir -p " + Folder_Path;// �Hdos�إ߸�Ƨ�
		system(command.c_str());
	}

	// �]�w��XML�x�s�Ҧ�
	FileStorage fs(Data_savepath + Filename, FileStorage::WRITE);// �x�s�e��

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
	FileStorage fs(Data_savepath + Filename, FileStorage::READ);// �x�s�e��
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

	// XML �ɮ� Ū��
	String XML_File_Path = FilePath + FileDate+"\\";
	String XML_File_Name = FileDate + ".xml";
	FileStorage Fs_Xml_Read(XML_File_Path + XML_File_Name, FileStorage::READ); // �x�s�e��
	if (!Fs_Xml_Read.isOpened()) {
		std::cout << "Fail to open file! " << endl;
		std::cout << "File Path : " << XML_File_Path << endl;
		std::cout << "File Name : " << XML_File_Name << endl;
		return false;
	}

	// CSV �ɮ� �x�s
	String Csv_file_path = XML_File_Path;
	String Csv_file_name_theroy = FileDate + "_csv_theory" + ".csv";
	String Csv_file_name_detect = FileDate + "_csv_detect" + ".csv";
	String Csv_file_name_rw = FileDate + "_csv_rw" + ".csv";
	// Ū���e��
	vector<Point2f> Point_Theory_Used;//�˴����᪺�i�βz���I�A�p�g�y��
	vector<Point2f>Points_Laser_Detect;// �˴������I�A����
	vector<Point3f>Points_Laser_Detect_RW;// �˴������I�A3D
	int get_number;// �˴�������data�ƶq

	// Ū�����
	Fs_Xml_Read["Point_Theory_Used"] >> Point_Theory_Used;
	Fs_Xml_Read["Points_Laser_Detect"] >> Points_Laser_Detect;
	Fs_Xml_Read["Points_Laser_Detect_RW"] >> Points_Laser_Detect_RW;
	Fs_Xml_Read["get_number"] >> get_number;
	// ��X��csv
	CL.Out_Data_Csv_2f(Csv_file_path + Csv_file_name_theroy, Point_Theory_Used, get_number);
	CL.Out_Data_Csv_2f(Csv_file_path + Csv_file_name_detect, Points_Laser_Detect, get_number);
	CL.Out_Data_Csv_3f(Csv_file_path + Csv_file_name_rw, Points_Laser_Detect_RW, get_number);

	cout << "Out Finish!" << endl;

	// Ū���i��
	CL.Read_Data_Csv_2f(Csv_file_path + Csv_file_name_theroy, Point_Theory_Used);
	CL.Read_Data_Csv_2f(Csv_file_path + Csv_file_name_detect, Points_Laser_Detect);
	CL.Read_Data_Csv_3f(Csv_file_path + Csv_file_name_rw, Points_Laser_Detect_RW);

	cout << "Read Finish!" << endl;

	return true;
}