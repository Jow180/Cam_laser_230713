#include "CamLaser.h"
#include <iostream>


// ��l���ܼ�
string CamLaser::End_code = "0000000000000000000000000000000000";
/*---�\��Τp�{��---*/

void CamLaser::Out_Data_Csv(String path, vector<Point2f> Out_Point, int Point_number) {
	//path:�x�s�ɮ׸��|�A�]�t�ɮצW�٤�����
	//Out_Point:�x�s�����
	//Point_numbers:�x�s���I�ƶq

	//�إ�CSV�ɮ�
	ofstream Outfile;
	Outfile.open(path, ios::out);
	//�N��쪺�I��m�����_�Ӭ�CSV�ɮ�
	for (int i = 0; i <= Point_number - 1; i++) {
		Outfile << Out_Point[i].x << "," << Out_Point[i].y << endl;
		std::cout << "��" << i << "�I �x�s����" << endl;
	}
	Outfile.close();
	std::cout << "�x�s��m : " << path << endl;
}
void CamLaser::Read_Data_Csv(String path, vector<Point2f>& Out_Point) {
	// path : �ɮ׸��|
	// Out_Point : �s�I�Ϊ��e��
	ifstream Infile;
	Infile.open(path, ios::in);//���}CSV�ɮ�
	if (!Infile) {
		std::cout << "�ɮפ��s�b" << endl;
	}
	String Line;//�Ω�CSVŪ�X���C��r�ꪺ�e��
	while (getline(Infile, Line)) {
		//std::cout << Line << endl;
		std::vector<std::string> ret = split(Line, ',');
		Out_Point.push_back(Point2d(stold(ret[0]), stold(ret[1])));
	}
	/*
	for (int i = 0; i <= Out_Point.size() - 1; i++) {
		std::cout << "��" << i << "�I�� " << Out_Point[i].x << "," << Out_Point[i].y << endl;
	}
	*/
}
void CamLaser::Read_Csv2Mat(String path, Mat& Out_Mat) {
	// �Ѧ�From:https://answers.opencv.org/question/55210/reading-csv-file-in-opencv/
	// path : �ɮ׸��|
	// Out_Mat : ��X���x�}
	ifstream Infile;
	vector<vector<double>> Get_vector;
	Infile.open(path, ios::in);//���}CSV�ɮ�
	if (!Infile) {
		std::cout << "�ɮפ��s�b" << endl;
	}
	String Line;//�Ω�CSVŪ�X���C��r�ꪺ�e��
	while (getline(Infile, Line)) {
		// Now inside each line we need to seperate the cols
		vector<double> values;
		stringstream temp(Line);
		string single_value;
		while (getline(temp, single_value)) {
			// convert the string element to a integer value
			std::vector<std::string> ret = split(Line, ',');
			for (int i = 0; i < ret.size(); i++) {
				values.push_back(stold(ret[i]));
			}
		}

		// add the row to the complete data vector
		Get_vector.push_back(values);
	}

	// Now add all the data into a Mat element
	/*std::cout << (int)Get_vector.size() << endl;
	std::cout << (int)Get_vector[0].size() << endl;*/
	Mat_<double> vect = Mat::zeros((int)Get_vector.size(), (int)Get_vector[0].size(), CV_64FC1);
	// Loop over vectors and add the data
	for (int rows = 0; rows < (int)Get_vector.size(); rows++) {
		for (int cols = 0; cols < (int)Get_vector[0].size(); cols++) {
			vect.at<double>(rows, cols) = Get_vector[rows][cols];// Mat::at�A�Ω�ק��
		}
	}
	Out_Mat = vect.clone();
}
void CamLaser::Write_Csv2Mat(string path, Mat Input_Mat)
{
	// path : �ɮ׸��|
	// Input_Mat : ��J���x�}
	ofstream myfile;
	myfile.open(path.c_str());
	myfile << cv::format(Input_Mat, cv::Formatter::FMT_CSV) << std::endl;
	myfile.close();
}
void CamLaser::Xcode_ctrl(vector<Point2f>& Input_Point, String& Out_Xcode) {
	String X1 = "02";
	String Point1_X = int2string_4ch(cal_0to2048(int(Input_Point[0].x)));
	String Point1_Y = int2string_4ch(cal_0to2048(int(Input_Point[0].y)));
	String Point2_X = int2string_4ch(cal_0to2048(int(Input_Point[1].x)));
	String Point2_Y = int2string_4ch(cal_0to2048(int(Input_Point[1].y)));
	String Point3_X = int2string_4ch(cal_0to2048(int(Input_Point[2].x)));
	String Point3_Y = int2string_4ch(cal_0to2048(int(Input_Point[2].y)));
	String Point4_X = int2string_4ch(cal_0to2048(int(Input_Point[3].x)));
	String Point4_Y = int2string_4ch(cal_0to2048(int(Input_Point[3].y)));
	Out_Xcode = X1 + Point1_X + Point1_Y + Point2_X + Point2_Y + Point3_X + Point3_Y + Point4_X + Point4_Y;
}
String CamLaser::int2string_4ch(int input_num) {
	//�N��J������ഫ���r��A�åB�ɥ����ܤ�4�ӼƭȡAEX:25 -> 0025
	// intput_num : ��J���ƭ�
	// Out_String : ��X���r��
	String orign = to_string(input_num);
	stringstream Out_Sting_s;
	String Out_Sting;
	Out_Sting_s << setw(4) << setfill('0') << input_num;// �N�������ƭȳ����H0�ɻ�
	Out_Sting_s >> Out_Sting;
	return Out_Sting;
}
const vector<string> CamLaser::split(const string& str, const char& delimiter) {
	//Form:https://shengyu7697.github.io/cpp-string-split/
	vector<string> result;
	stringstream ss(str);
	string tok;

	while (getline(ss, tok, delimiter)) {
		result.push_back(tok);
	}
	return result;
}
int CamLaser::cal_0to2048(int input_num) {
	// �N��J���ƭȮե���0��2047�H���A�H�K�X��
	int out_num = input_num;
	if (input_num < 0) {
		out_num = 0;
	}
	else if (input_num >= 2048)
	{
		out_num = 2047;
	}
	return out_num;
}
String CamLaser::NameCreate(int ZerosNum, int Value) {
	//From�F�a�ɮ�
	//�Ω�إ��ɮצW�١A��K�b��Ƨ���ܮɬO������B����ɮ�Ū�����~�����I�A�p.001�B002�B003
	//--input
	//int ZerosNum : �X��� ex.4��� 0001 �B 5��� 00001
	//int Value : ��J���ƭ�
	//--output
	// string : �W�٦r��
	//
	//���wValue��Ƥ��|�W�LZerosNum
	vector<int> Calculation;
	int i = 0;
	string Name = "";
	if (Value != 0) {
		while (i < ZerosNum) {
			if ((Value / (pow(10, i))) < 1) {
				for (int j = 0; j < ZerosNum - i; j++) {
					Name = Name + "0";
				}
				Name = Name + to_string(Value);
				break;
			}
			i++;
		}
	}
	else {
		for (int j = 0; j < ZerosNum; j++) {
			Name = Name + "0";
		}
	}

	return Name;
}
void CamLaser::Max_P2f(vector<Point2f> input_vector, float out_x, float out_y) {
	// �Ω��Xvector�����̤j�γ̤p��
	int vector_size = input_vector.size();// �p���J���V�q�e������
	for (int i = 0; i < vector_size; i++) {
		out_x = max(input_vector[0].x, input_vector[i].x);
	}
	for (int i = 0; i < vector_size; i++) {
		out_y = max(input_vector[0].y, input_vector[i].y);
	}
}
float CamLaser::length_2P(Point2f p1, Point2f p2) {
	// �Ω�p����I�������Z��
	float x_leghth = fabs(p1.x - p2.x);
	float y_length = fabs(p1.y - p2.y);
	float length = fabs(sqrt(x_leghth * x_leghth + y_length * y_length));
	return length;
}
void CamLaser::square_max(vector<vector<Point>>square_input, double& square_max, int& square_max_local) {
	// 230219 : �Ω�p���J���x�Ϊ��̤j�ȤΥL�����ޭ�
	// square_input : ��J���x��
	// square_max :�@��^�䤤�̤j�x�έ��n
	// square_max_local : ��^�̤j���x�Ϊ����ޭ�
	//double square_max;
	int square_num = square_input.size();
	vector<double> area;
	vector<RotatedRect> square_rect;
	std::vector<std::vector<cv::Point> >::const_iterator  itc = square_input.begin();
	for (int i = 0; i < square_num; i++) {
		//area = contourArea(square_input[i]);

		area.push_back(contourArea(square_input[i]));// �p�⭱�n
		//std::cout << "�������n��" << area << endl;
		//square_max = fabs(MAX(square_max, area));
	}
	square_max = fabs(*max_element(area.begin(), area.end()));// �p��̤j���n
	square_max_local = max_element(area.begin(), area.end()) - area.begin();// �p��̤j���n���ޭ�
}
double CamLaser::angle(Point pt1, Point pt2, Point pt0) {
	double dx1 = (double)pt1.x - (double)pt0.x;
	double dy1 = (double)pt1.y - (double)pt0.y;
	double dx2 = (double)pt2.x - (double)pt0.x;
	double dy2 = (double)pt2.y - (double)pt0.y;
	return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
void CamLaser::findSquares(const Mat& image, vector<vector<Point> >& squares, int N, int thresh) {
	// Form : https://docs.opencv.org/3.4/db/d00/samples_2cpp_2squares_8cpp-example.html
	// �Ω���v�������x��
	// image : ��J���Ϥ�
	// square: ��X�x�Φ�m�A�H�G����ܡA�Ĥ@���׬��ĴX�ӯx�ΡA�ĤG���׬��x�Ϊ�4���I
	// N :���ժ��G�ȤƤ����X����
	// thresh : Canny��t�˴����֭�
	squares.clear();
	Mat pyr, timg, gray0(image.size(), CV_8U), gray;
	// down-scale and upscale the image to filter out the noise
	pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
	pyrUp(pyr, timg, image.size());
	vector<vector<Point> > contours;
	// find squares in every color plane of the image
	for (int c = 0; c < 3; c++)
	{
		int ch[] = { c, 0 };
		mixChannels(&timg, 1, &gray0, 1, ch, 1);
		// try several threshold levels
		for (int l = 0; l < N; l++)
		{
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if (l == 0)
			{
				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				Canny(gray0, gray, 500, thresh, 5);
				// dilate canny output to remove potential
				// holes between edge segments
				dilate(gray, gray, Mat(), Point(-1, -1));
			}
			else
			{
				// apply threshold if l!=0:
				//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				gray = gray0 >= (l + 1) * 255 / N;
			}
			// find contours and store them all as a list
			findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
			vector<Point> approx;
			// test each contour
			for (size_t i = 0; i < contours.size(); i++)
			{
				// approximate contour with accuracy proportional
				// to the contour perimeter
				approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				if (approx.size() == 4 &&
					fabs(contourArea(approx)) > 1000 &&
					isContourConvex(approx))
				{
					double maxCosine = 0;
					for (int j = 2; j < 5; j++)
					{
						// find the maximum cosine of the angle between joint edges
						int a = j % 4;
						int b = j - 2;
						int c = j - 1;					
						double cosine = fabs(angle(approx[a], approx[b], approx[c]));
						maxCosine = MAX(maxCosine, cosine);
					}
					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if (maxCosine < 0.3)
						squares.push_back(approx);
				}
			}
		}
	}
}
int CamLaser::GetFileNum(const std::string& inPath)// �p���Ƨ������`��
{
	int fileNum = 0;

	std::vector<std::string> pathVec;
	std::queue<std::string> q;
	q.push(inPath);

	while (!q.empty())
	{
		std::string item = q.front(); q.pop();

		std::string path = item + "\\*";
		struct _finddata_t fileinfo;
		auto handle = _findfirst(path.c_str(), &fileinfo);
		if (handle == -1) continue;

		while (!_findnext(handle, &fileinfo))
		{
			if (fileinfo.attrib & _A_SUBDIR)
			{
				if (strcmp(fileinfo.name, ".") == 0 || strcmp(fileinfo.name, "..") == 0)continue;
				q.push(item + "\\" + fileinfo.name);
			}
			else
			{
				fileNum++;
				pathVec.push_back(item + "\\" + fileinfo.name);
			}
		}
		_findclose(handle);
	}

	return fileNum;
}

/*---�p��ε{��---*/
void CamLaser::LineFitLeastSquares(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& r) {
	//LSM�̤p����k
	// data_x * a + b = data_y
	// data_x : �q����
	// data_y : �z�׭�
	// data_n : ��Ƽƶq
	// a : �@���Y��
	// b : �`��
	//From: https://blog.csdn.net/qq_36251561/article/details/88020558
	float A = 0.0;
	float B = 0.0;
	float C = 0.0;
	float D = 0.0;
	float E = 0.0;
	float F = 0.0;

	for (int i = 0; i < data_n; i++)
	{
		A += data_x[i] * data_x[i];
		B += data_x[i];
		C += data_x[i] * data_y[i];
		D += data_y[i];
	}

	// �p��ײva�M�I�Zb
	//float a, b, temp = 0;
	float temp = 0;
	if (temp = (data_n * A - B * B))// �P?������?0
	{
		a = (data_n * C - B * D) / temp;
		b = (A * D - B * C) / temp;
	}
	else
	{
		a = 1;
		b = 0;
	}

	// �p������ʫY��r�Ar>0�������Ar<0�t����
	float Xmean, Ymean;
	Xmean = B / data_n;
	Ymean = D / data_n;

	float tempSumXX = 0.0, tempSumYY = 0.0;
	for (int i = 0; i < data_n; i++)
	{
		tempSumXX += (data_x[i] - Xmean) * (data_x[i] - Xmean);
		tempSumYY += (data_y[i] - Ymean) * (data_y[i] - Ymean);
		E += (data_x[i] - Xmean) * (data_y[i] - Ymean);
	}
	F = sqrt(tempSumXX) * sqrt(tempSumYY);
	//float r;
	r = E / F;


}
void CamLaser::LineFitLeastSquares_quadratic(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& c) {
	//�@���G���A�̤p����k�A�h�������X
	// data_x * a + data_y * b + c = data_z
	// data_x : �q����x
	// data_y : �q����y
	// data_z : ��X��z
	// data_n : ��Ƽƶq
	// a : x���Y��
	// b : y���Y��
	// c : �`��
	// r1 : x & z �����ʫY��
	// r2 : y & Z �����ʫY��
	float x = 0.0;
	float y = 0.0;
	float x_power_2 = 0.0;
	float x_power_3 = 0.0;
	float x_power_4 = 0.0;
	float xy = 0.0;
	float x_power2_y = 0.0;
	// �����ʫY��
	float xz_e = 0.0;
	float yz_e = 0.0;
	float xz_f = 0.0;
	float yz_f = 0.0;
	for (int i = 0; i < data_n; i++)
	{
		x += data_x[i];
		y += data_y[i];
		x_power_2 += data_x[i] * data_x[i];
		x_power_3 += data_x[i] * data_x[i] * data_x[i];
		x_power_4 += data_x[i] * data_x[i] * data_x[i] * data_x[i];
		xy += data_x[i] * data_y[i];
		x_power2_y += data_x[i] * data_x[i] * data_y[i];
	}
	// �p��a,b,c�A�ϥΧJ�Ժ�
	float delta = 0;
	float delta_a = 0;
	float delta_b = 0;
	float delta_c = 0;
	delta = Determinant_tri(x_power_4, x_power_3, x_power_2, x_power_3, x_power_2, x, x_power_2, x, data_n);
	delta_a = Determinant_tri(x_power2_y, xy, y, x_power_3, x_power_2, x, x_power_2, x, data_n);
	delta_b = Determinant_tri(x_power_4, x_power_3, x_power_2, x_power2_y, xy, y, x_power_2, x, data_n);
	delta_c = Determinant_tri(x_power_4, x_power_3, x_power_2, x_power_3, x_power_2, x, x_power2_y, xy, y);

	if (delta == 0) {
		a = 1;
		b = 1;
		c = 1;
		cout << "�L��" << endl;
	}
	else {
		a = delta_a / delta;
		b = delta_b / delta;
		c = delta_c / delta;

	}
	// �G���Ť��������ʫY��
	// �p������ʫY��r�Ar>0�������Ar<0�t����
	/*float Xmean, Ymean, Zmean;
	Xmean = x / data_n;
	Ymean = y / data_n;
	Zmean = z / data_n;
	float tempSumXX = 0.0, tempSumYY = 0.0, tempSumZZ = 0.0;
	for (int i = 0; i < data_n; i++)
	{
		tempSumXX += (data_x[i] - Xmean) * (data_x[i] - Xmean);
		tempSumYY += (data_y[i] - Ymean) * (data_y[i] - Ymean);
		tempSumZZ += (data_z[i] - Zmean) * (data_z[i] - Zmean);
		xz_e += (data_x[i] - Xmean) * (data_z[i] - Zmean);
		yz_e += (data_y[i] - Ymean) * (data_z[i] - Zmean);
	}
	xz_f = sqrt(tempSumXX) * sqrt(tempSumZZ);
	yz_f = sqrt(tempSumYY) * sqrt(tempSumZZ);
	r1 = xz_e / xz_f;
	r2 = yz_e / yz_f;*/

}
float CamLaser::Determinant_tri(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3) {
	// ��C���A�T��
	// a1, a2, a3, b1, b2, b3, c1, c2, c3 :�@��C���Ѽ�
	// dst : ��X����C����
	// | a1 b1 c1 |
	// | a2 b2 c2 |
	// | a3 b3 c3 |
	float dst;
	dst = a1 * b2 * c3 + b1 * c2 * a3 + c1 * a2 * b3 - a3 * b2 * c1 - b3 * c2 * a1 - c3 * a2 * b1;
	return dst;
}
void CamLaser::LSM_Trans(Point2f& input_point, Point2f& out_point, double x_a, double x_b, double y_a, double y_b)
{
	out_point.x = input_point.x * x_a + x_b;
	out_point.y = input_point.y * y_a + y_b;
}
void CamLaser::LSM_Trans_quadratic(Point2f& input_point, Point2f& out_point, double x_a, double x_b, double x_c, double y_a, double y_b, double y_c)
{
	out_point.x = x_a * input_point.x * input_point.x + x_b * input_point.x + x_c;
	out_point.y = y_a * input_point.y * input_point.y + y_b * input_point.y + y_c;
}
void CamLaser::Feature_ORB(int, void*, Mat img1, Mat img2, vector<Point2f>& get_point4)
{	//img1��J���ҪO
	//img2��諸�Ϥ�
	//get_point4 ��X���Ϥ��I��m 4�I

	int Hession = 400;
	double t1 = getTickCount();
	//�S�x�I����
	Ptr<ORB> detector = ORB::create(2000);
	vector<KeyPoint> keypoints_obj;
	vector<KeyPoint> keypoints_scene;
	//�w�q�y�z�l
	Mat descriptor_obj, descriptor_scene;
	//�˴��íp�⦨�y�z�l
	detector->detectAndCompute(img1, Mat(), keypoints_obj, descriptor_obj);
	detector->detectAndCompute(img2, Mat(), keypoints_scene, descriptor_scene);

	double t2 = getTickCount();
	double t = (t2 - t1) * 1000 / getTickFrequency();
	//�S�x�ǰt
	FlannBasedMatcher fbmatcher(new flann::LshIndexParams(20, 10, 2));
	vector<DMatch> matches;
	//�N��쪺�y�z�l�i��ǰt�æs�Jmatches��
	fbmatcher.match(descriptor_obj, descriptor_scene, matches);

	double minDist = 1000;
	double maxDist = 0;
	//��X���u�y�z�l
	vector<DMatch> goodmatches;
	for (int i = 0; i < descriptor_obj.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < minDist)
		{
			minDist = dist;
		}
		if (dist > maxDist)
		{
			maxDist = dist;
		}
	}
	for (int i = 0; i < descriptor_obj.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < max(2 * minDist, 0.75))
		{
			goodmatches.push_back(matches[i]);
		}
	}
	Mat orbImg;

	drawMatches(img1, keypoints_obj, img2, keypoints_scene, goodmatches, orbImg,
		Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//----------�ؼЪ���ίx�μ��ѥX��------------//
	Mat H_copy;
	//std::cout << "goodmatches : " << goodmatches.size() << std::endl;
	if (goodmatches.size() > 15) {
		vector<Point2f> obj;
		vector<Point2f>scene;
		for (size_t i = 0; i < goodmatches.size(); i++)
		{
			obj.push_back(keypoints_obj[goodmatches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[goodmatches[i].trainIdx].pt);
		}
		vector<Point2f> obj_corner(4);
		vector<Point2f> scene_corner(4);
		//�ͦ��z���x�}
		Mat H = findHomography(obj, scene, RANSAC);

		if (H.empty() == 1) {// �T�{����x�}�s�b�~����A�p�G�S���A�Ϋe�@��H����
			H = Mat::ones(3, 3, CV_8U);
			//std::cout << "H:����x�} �� �ůx�}!�ץ�! H :" << endl;
			//std::cout << H << endl;
		}

		obj_corner[0] = Point(0, 0);
		obj_corner[1] = Point(img1.cols, 0);
		obj_corner[2] = Point(img1.cols, img1.rows);
		obj_corner[3] = Point(0, img1.rows);
		//�z���ܴ�
		perspectiveTransform(obj_corner, scene_corner, H);
		// �^�Ǳo�쪺�|���I
		if (scene_corner.empty() == 1) {
			std::cout << "���������I" << endl;
		}
		/*
		get_point4[0] = Point2f(scene_corner[0].x, scene_corner[0].y);
		get_point4[1] = Point2f(scene_corner[1].x, scene_corner[1].y);
		get_point4[2] = Point2f(scene_corner[2].x, scene_corner[2].y);
		get_point4[3] = Point2f(scene_corner[3].x, scene_corner[3].y);
		*/
		for (int i = 0; i < scene_corner.size(); i++) {
			//get_point4[i] = Point2f(scene_corner[i].x, scene_corner[i].y);//221214_�s���H�W!!!!!!
			get_point4.push_back(Point2f(scene_corner[i].x, scene_corner[i].y));
		}

		//obj.clear();
		//scene.clear();
	}
	else {
		for (int i = 0; i < 4; i++) {
			get_point4.push_back(Point2f(0, 0));
		}

	}
}
