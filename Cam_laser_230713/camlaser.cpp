#include "CamLaser.h"
#include <iostream>


// 初始化變數
string CamLaser::End_code = "0000000000000000000000000000000000";
/*---功能用小程式---*/

void CamLaser::Out_Data_Csv(String path, vector<Point2f> Out_Point, int Point_number) {
	//path:儲存檔案路徑，包含檔案名稱及類型
	//Out_Point:儲存的資料
	//Point_numbers:儲存的點數量

	//建立CSV檔案
	ofstream Outfile;
	Outfile.open(path, ios::out);
	//將找到的點位置紀錄起來為CSV檔案
	for (int i = 0; i <= Point_number - 1; i++) {
		Outfile << Out_Point[i].x << "," << Out_Point[i].y << endl;
		std::cout << "第" << i << "點 儲存完畢" << endl;
	}
	Outfile.close();
	std::cout << "儲存位置 : " << path << endl;
}
void CamLaser::Read_Data_Csv(String path, vector<Point2f>& Out_Point) {
	// path : 檔案路徑
	// Out_Point : 存點用的容器
	ifstream Infile;
	Infile.open(path, ios::in);//打開CSV檔案
	if (!Infile) {
		std::cout << "檔案不存在" << endl;
	}
	String Line;//用於CSV讀出的每行字串的容器
	while (getline(Infile, Line)) {
		//std::cout << Line << endl;
		std::vector<std::string> ret = split(Line, ',');
		Out_Point.push_back(Point2d(stold(ret[0]), stold(ret[1])));
	}
	/*
	for (int i = 0; i <= Out_Point.size() - 1; i++) {
		std::cout << "第" << i << "點為 " << Out_Point[i].x << "," << Out_Point[i].y << endl;
	}
	*/
}
void CamLaser::Read_Csv2Mat(String path, Mat& Out_Mat) {
	// 參考From:https://answers.opencv.org/question/55210/reading-csv-file-in-opencv/
	// path : 檔案路徑
	// Out_Mat : 輸出的矩陣
	ifstream Infile;
	vector<vector<double>> Get_vector;
	Infile.open(path, ios::in);//打開CSV檔案
	if (!Infile) {
		std::cout << "檔案不存在" << endl;
	}
	String Line;//用於CSV讀出的每行字串的容器
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
			vect.at<double>(rows, cols) = Get_vector[rows][cols];// Mat::at，用於修改值
		}
	}
	Out_Mat = vect.clone();
}
void CamLaser::Write_Csv2Mat(string path, Mat Input_Mat)
{
	// path : 檔案路徑
	// Input_Mat : 輸入的矩陣
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
	//將輸入的整數轉換為字串，並且補正為至少4個數值，EX:25 -> 0025
	// intput_num : 輸入的數值
	// Out_String : 輸出的字串
	String orign = to_string(input_num);
	stringstream Out_Sting_s;
	String Out_Sting;
	Out_Sting_s << setw(4) << setfill('0') << input_num;// 將不足的數值部分以0補齊
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
	// 將輸入的數值校正到0到2047以內，以免出錯
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
	//From東榮檔案
	//用於建立檔案名稱，方便在資料夾顯示時是整齊的且減少檔案讀取錯誤的風險，如.001、002、003
	//--input
	//int ZerosNum : 幾位數 ex.4位數 0001 、 5位數 00001
	//int Value : 輸入的數值
	//--output
	// string : 名稱字串
	//
	//假定Value位數不會超過ZerosNum
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
	// 用於找出vector中的最大及最小值
	int vector_size = input_vector.size();// 計算輸入的向量容器長度
	for (int i = 0; i < vector_size; i++) {
		out_x = max(input_vector[0].x, input_vector[i].x);
	}
	for (int i = 0; i < vector_size; i++) {
		out_y = max(input_vector[0].y, input_vector[i].y);
	}
}
float CamLaser::length_2P(Point2f p1, Point2f p2) {
	// 用於計算兩點之間的距離
	float x_leghth = fabs(p1.x - p2.x);
	float y_length = fabs(p1.y - p2.y);
	float length = fabs(sqrt(x_leghth * x_leghth + y_length * y_length));
	return length;
}
void CamLaser::square_max(vector<vector<Point>>square_input, double& square_max, int& square_max_local) {
	// 230219 : 用於計算輸入的矩形的最大值及他的索引值
	// square_input : 輸入的矩形
	// square_max :　返回其中最大矩形面積
	// square_max_local : 返回最大的矩形的索引值
	//double square_max;
	int square_num = square_input.size();
	vector<double> area;
	vector<RotatedRect> square_rect;
	std::vector<std::vector<cv::Point> >::const_iterator  itc = square_input.begin();
	for (int i = 0; i < square_num; i++) {
		//area = contourArea(square_input[i]);

		area.push_back(contourArea(square_input[i]));// 計算面積
		//std::cout << "輪廓面積為" << area << endl;
		//square_max = fabs(MAX(square_max, area));
	}
	square_max = fabs(*max_element(area.begin(), area.end()));// 計算最大面積
	square_max_local = max_element(area.begin(), area.end()) - area.begin();// 計算最大面積索引值
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
	// 用於找到影像中的矩形
	// image : 輸入的圖片
	// square: 輸出矩形位置，以二維表示，第一維度為第幾個矩形，第二維度為矩形的4個點
	// N :測試的二值化分為幾階級
	// thresh : Canny邊緣檢測的閥值
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
int CamLaser::GetFileNum(const std::string& inPath)// 計算資料夾內部總數
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

/*---計算用程式---*/
void CamLaser::LineFitLeastSquares(vector<float> data_x, vector<float> data_y, int data_n, float& a, float& b, float& r) {
	//LSM最小平方法
	// data_x * a + b = data_y
	// data_x : 量測值
	// data_y : 理論值
	// data_n : 資料數量
	// a : 一階係數
	// b : 常數
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

	// 計算斜率a和截距b
	//float a, b, temp = 0;
	float temp = 0;
	if (temp = (data_n * A - B * B))// 判?分母不?0
	{
		a = (data_n * C - B * D) / temp;
		b = (A * D - B * C) / temp;
	}
	else
	{
		a = 1;
		b = 0;
	}

	// 計算相關性係數r，r>0正相關，r<0負相關
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
	//一元二次，最小平方法，多項式擬合
	// data_x * a + data_y * b + c = data_z
	// data_x : 量測值x
	// data_y : 量測值y
	// data_z : 輸出值z
	// data_n : 資料數量
	// a : x項係數
	// b : y項係數
	// c : 常數
	// r1 : x & z 相關性係數
	// r2 : y & Z 相關性係數
	float x = 0.0;
	float y = 0.0;
	float x_power_2 = 0.0;
	float x_power_3 = 0.0;
	float x_power_4 = 0.0;
	float xy = 0.0;
	float x_power2_y = 0.0;
	// 相關性係數
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
	// 計算a,b,c，使用克拉瑪
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
		cout << "無解" << endl;
	}
	else {
		a = delta_a / delta;
		b = delta_b / delta;
		c = delta_c / delta;

	}
	// 二階級不能算相關性係數
	// 計算相關性係數r，r>0正相關，r<0負相關
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
	// 行列式，三階
	// a1, a2, a3, b1, b2, b3, c1, c2, c3 :　行列式參數
	// dst : 輸出的行列式值
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
{	//img1輸入的模板
	//img2比對的圖片
	//get_point4 輸出的圖片點位置 4點

	int Hession = 400;
	double t1 = getTickCount();
	//特徵點提取
	Ptr<ORB> detector = ORB::create(2000);
	vector<KeyPoint> keypoints_obj;
	vector<KeyPoint> keypoints_scene;
	//定義描述子
	Mat descriptor_obj, descriptor_scene;
	//檢測並計算成描述子
	detector->detectAndCompute(img1, Mat(), keypoints_obj, descriptor_obj);
	detector->detectAndCompute(img2, Mat(), keypoints_scene, descriptor_scene);

	double t2 = getTickCount();
	double t = (t2 - t1) * 1000 / getTickFrequency();
	//特徵匹配
	FlannBasedMatcher fbmatcher(new flann::LshIndexParams(20, 10, 2));
	vector<DMatch> matches;
	//將找到的描述子進行匹配並存入matches中
	fbmatcher.match(descriptor_obj, descriptor_scene, matches);

	double minDist = 1000;
	double maxDist = 0;
	//找出最優描述子
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

	//----------目標物體用矩形標識出來------------//
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
		//生成透視矩陣
		Mat H = findHomography(obj, scene, RANSAC);

		if (H.empty() == 1) {// 確認旋轉矩陣存在才執行，如果沒有，用前一個H頂替
			H = Mat::ones(3, 3, CV_8U);
			//std::cout << "H:旋轉矩陣 為 空矩陣!修正! H :" << endl;
			//std::cout << H << endl;
		}

		obj_corner[0] = Point(0, 0);
		obj_corner[1] = Point(img1.cols, 0);
		obj_corner[2] = Point(img1.cols, img1.rows);
		obj_corner[3] = Point(0, img1.rows);
		//透視變換
		perspectiveTransform(obj_corner, scene_corner, H);
		// 回傳得到的四個點
		if (scene_corner.empty() == 1) {
			std::cout << "未偵測到點" << endl;
		}
		/*
		get_point4[0] = Point2f(scene_corner[0].x, scene_corner[0].y);
		get_point4[1] = Point2f(scene_corner[1].x, scene_corner[1].y);
		get_point4[2] = Point2f(scene_corner[2].x, scene_corner[2].y);
		get_point4[3] = Point2f(scene_corner[3].x, scene_corner[3].y);
		*/
		for (int i = 0; i < scene_corner.size(); i++) {
			//get_point4[i] = Point2f(scene_corner[i].x, scene_corner[i].y);//221214_存取違規!!!!!!
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
