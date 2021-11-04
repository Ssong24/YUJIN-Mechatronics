// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.


#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdlib.h>
#include <stdio.h>

#include <iostream>  
#include <string> 
#include <math.h>
//#include "linefinder.h"
//#include "edgedetector.h"

using namespace cv;
using namespace rs2;
using namespace std;

float dist_array[1280][720] = { 0 };
void showThreeImage(colorizer color_map, pipeline pipe_color, pipeline pipe_depth);			// Show RGB, Infrared, Depth Image
void storeDepthImage(colorizer color_map, pipeline pipe_depth);			
void storeColorImage(pipeline pipe_depth);
void storeInfraredImage(colorizer color_map, pipeline pipe_depth);
void storeTwoImage(colorizer color_map, pipeline pipe_color, pipeline pipe_depth);
void markDepthImage(colorizer color_map, pipeline pipe_color, pipeline pipe_depth);
void setLabel(Mat& image, string str, vector<Point> contour);
void checkSquareRGB(colorizer color_map, pipeline pipe_color, pipeline pipe_depth);



void swapab(float* a, float* b) {
	auto temp = *a;
	*a = *b;
	*b = temp;
	//return;
}

int main(int argc, char *argv[]) try
{

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;

	// Color
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe_color;
	// Depth 
	rs2::pipeline pipe_depth;

	checkSquareRGB(color_map, pipe_color, pipe_depth);
	//storeColorImage(pipe_depth);
	//showThreeImage(color_map, pipe_color, pipe_depth);
	//storeInfraredImage(color_map, pipe_depth);
	//storeTwoImage(color_map, pipe_color, pipe_depth);
	//storeColorImage(pipe_color);
	//storeDepthImage(color_map, pipe_depth);
	//markDepthImage(color_map, pipe_color, pipe_depth);


	return EXIT_SUCCESS;
		
}

catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


void checkSquareRGB_Depth(colorizer color_map, pipeline pipe_color, pipeline pipe_depth)
{
	rs2::config cfg_color;
	cfg_color.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	pipe_color.start(cfg_color);

	const auto color_name = "color image";
	namedWindow(color_name, WINDOW_AUTOSIZE);

	while (waitKey(1) < 0) {
		const int width = 640;
		const int height = 480;

		rs2::frameset data_color = pipe_color.wait_for_frames();
		rs2::frame color = data_color.get_color_frame();

		Mat image2(Size(color.as<rs2::video_frame>().get_width(), color.as<rs2::video_frame>().get_height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

		Mat img_input;
		resize(image2, img_input, Size(width, height), 0, CV_INTER_LINEAR);
		imshow(color_name, img_input);

		Mat img_hsv;
		cvtColor(img_input, img_hsv, COLOR_BGR2HSV);

		Scalar lowerb(0, 85, 0);
		Scalar upperb(360, 255, 255);

		Mat img_dst, temp;
		inRange(img_hsv, lowerb, upperb, temp);

		img_dst = temp;   //부드럽게 처리
		blur(img_dst, img_dst, Size(3, 3));
		Canny(img_dst, img_dst, 0, 250 * 3, 3);


		//morph
		Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(40, 40));
		Mat morph;
		morphologyEx(img_dst, img_dst, CV_MOP_CLOSE, kernel);
		//const auto morph_name = "morphed image";
		//namedWindow(morph_name, WINDOW_AUTOSIZE);
		//imshow(morph_name, img_dst);


		//contour
		vector<vector<Point> > contours;
		findContours(img_dst, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

		//contour를 근사화한다.
		vector<Point2f> approx;
		Mat img_result;
		img_result = img_input.clone();

		//좌표들
		float points[6][2];

		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

			if (approx.size() == 4 && (fabs(contourArea(Mat(approx))) > 20000 && fabs(contourArea(Mat(approx))) < 10000000)) //면적이 일정크기 이상이어야 한다. 
			{

				int size = approx.size();

				//Contour를 근사화한 직선을 그린다.
				for (int k = 0; k < size; k++) {
					line(img_result, approx[k], approx[(k + 1) % size], Scalar(0, 255, 0), 3);
					points[k][0] = approx[k].x;
					points[k][1] = approx[k].y;
				}
				if (isContourConvex(Mat(approx)))
					setLabel(img_result, "rectangle", contours[i]); //사각형
				break;
			}

		}



		//4는 중심점
		points[4][0] = (points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4;
		points[4][1] = (points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4;

		// points[5]는 길이
		points[5][0] = (-(points[0][0] + points[1][0] - points[2][0] - points[3][0])) / 2;
		points[5][1] = (-points[0][1] + points[1][1] + points[2][1] - points[3][1]) / 2;


		for (int i = 0; i < 4; i++) {
			if ((points[i][0] - points[4][0]) < 0 && (points[i][1] - points[4][1]) < 0) {
				swapab(&(points[i][0]), &(points[3][0]));
				swapab(&(points[i][1]), &(points[3][1]));
				//break;
			}
			else if ((points[i][0] - points[4][0]) < 0 && (points[i][1] - points[4][1]) > 0) {
				swapab(&(points[i][0]), &(points[2][0]));
				swapab(&(points[i][1]), &(points[2][1]));
				//break;
			}
			else if ((points[i][0] - points[4][0]) > 0 && (points[i][1] - points[4][1]) < 0) {
				swapab(&(points[i][0]), &(points[0][0]));
				swapab(&(points[i][1]), &(points[0][1]));
				//break;
			}
			else if ((points[i][0] - points[4][0]) > 0 && (points[i][1] - points[4][1]) > 0) {
				swapab(&(points[i][0]), &(points[1][0]));
				swapab(&(points[i][1]), &(points[1][1]));
				//break;
			}
		}


		for (int i = 0; i < 6; i++) {
			if (i == 5)
				cout << "길이 : x =" << points[i][0] << ", y =" << points[i][1] << endl;
			else if (i == 4)
				cout << "중심 : (" << points[i][0] << ", " << points[i][1] << ")" << endl;
			else
				cout << "좌표 : (" << points[i][0] << ", " << points[i][1] << ")" << endl;

		}


		int dpt_max_X = 0;
		int dpt_max_Y = 0;
		int dpt_min_X = 1000;
		int dpt_min_Y  = 1000;
		
		dpt_max_X = max(points[0][0], points[1][0]);
		dpt_max_Y = max(points[1][1], points[2][1]);
		dpt_min_X = min(points[2][0], points[3][0]);
		dpt_min_Y = min(points[0][1], points[3][1]);
		



		int width2 = -1;
		int height2 = -1;
		if (width2 <= 0 || height2 <= 0) {
			cout << "width : ";
			cin >> width2;
			cout << "height : ";
			cin >> height2;
		}

		vector<vector<Point> > g_point(height2*width2, vector<Point>(0));
		//Rect rect[height2][width2];
		//vector<Point>* g_point = (vector<Point>*) malloc(sizeof(vector<Point>)*height2*width2);

		cout << sizeof(g_point) / sizeof(g_point[0]) << endl;

		int block_num = 0;

		for (int i = 0; i < height2; i++) {
			for (int j = 0; j < width2; j++) {
				//vector<Point> 자료형에 네 점을 넣고 boundingRect(vector<point> a)로 사각형을 만듦 -> Rect rect[][]에 저장
				if (points[3][1]<points[0][1]) {
					//왼위
					float x3 = points[3][0] + j * (points[0][0] - points[3][0]) / width2 + i * (points[2][0] - points[3][0]) / height2 + i * j * abs(points[0][0] - points[3][0] - (points[1][0] - points[2][0])) / (height2*width2);
					float y3 = points[3][1] + j * (points[0][1] - points[3][1]) / width2 + i * (points[2][1] - points[3][1]) / height2;
					g_point[block_num].push_back(Point(x3 + (points[0][0] - points[3][0]) / width2, y3 + (points[0][1] - points[3][1]) / width2));//[0]
					g_point[block_num].push_back(Point(x3 + (points[1][0] - points[0][0]) / height2 + (points[1][0] - points[3][0] + width2) / (width2), y3 + (points[2][1] - points[3][1]) / height2 + (points[1][1] - points[2][1]) / width2));//[1]
					g_point[block_num].push_back(Point(x3 + (points[2][0] - points[3][0]) / width2, y3 + (points[2][1] - points[3][1]) / height2));//[2]
					g_point[block_num].push_back(Point(x3 + (points[3][0] - points[3][0]) / width2, y3 + (points[3][1] - points[3][1]) / height2));//[3]            
				}
				else {
					//오위
					float x3 = points[3][0] + j * (points[0][0] - points[3][0]) / width2 + i * (points[2][0] - points[3][0]) / height2 + i * j * abs(points[0][0] - points[3][0] - (points[1][0] - points[2][0])) / (height2*width2);
					float y3 = points[3][1] + j * (points[0][1] - points[3][1]) / width2 + i * (points[2][1] - points[3][1]) / height2;
					g_point[block_num].push_back(Point(x3 + (points[0][0] - points[3][0]) / width2, y3 + (points[0][1] - points[3][1]) / width2));//[0]
					g_point[block_num].push_back(Point(x3 + (points[1][0] - points[3][0]) / width2, y3 + (points[2][1] - points[3][1]) / height2 + (points[1][1] - points[2][1]) / width2));//[1]
					g_point[block_num].push_back(Point(x3 + (points[2][0] - points[3][0]) / width2, y3 + (points[2][1] - points[3][1]) / height2));//[2]
					g_point[block_num].push_back(Point(x3 + (points[3][0] - points[3][0]) / width2, y3 + (points[3][1] - points[3][1]) / height2));//[3]
				}

				for (int k = 0; k < 4; k++) {
					line(img_result, g_point[block_num][k % 4], g_point[block_num][(k + 1) % 4], Scalar(0, 0, 255), 3);
				}
				string s1 = "s" + to_string(block_num);
				//setLabel_small(img_result, s1, g_point[block_num],0,0,255); //사각형
				block_num++;
			}
		}

		printf("Everything done\n");


		//imshow("dst", img_dst);
		//imshow("input", img_input);
		//imshow("result", img_result);



	}  // while 괄호

}


void checkSquareRGB(colorizer color_map, pipeline pipe_color, pipeline pipe_depth)
{
	rs2::config cfg_color;
	cfg_color.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	pipe_color.start(cfg_color);

	const auto color_name = "color image";
	namedWindow(color_name, WINDOW_AUTOSIZE);

	while (waitKey(1) < 0) {
		const int width = 640;
		const int height = 480;

		rs2::frameset data_color = pipe_color.wait_for_frames();
		rs2::frame color = data_color.get_color_frame();

		Mat image2(Size(color.as<rs2::video_frame>().get_width(), color.as<rs2::video_frame>().get_height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

		Mat img_input;
		resize(image2, img_input, Size(640, 480), 0, CV_INTER_LINEAR);
		imshow(color_name, img_input);

		Mat img_hsv;
		cvtColor(img_input, img_hsv, COLOR_BGR2HSV);


		Scalar lowerb(0, 85, 0);
		Scalar upperb(360, 255, 255);

		Mat img_dst, temp;
		inRange(img_hsv, lowerb, upperb, temp);

		img_dst = temp;   //부드럽게 처리
		blur(img_dst, img_dst, Size(3, 3));
		Canny(img_dst, img_dst, 0, 250 * 3, 3);


		//morph
		Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(40, 40));
		Mat morph;
		morphologyEx(img_dst, img_dst, CV_MOP_CLOSE, kernel);
		//const auto morph_name = "morphed image";
		//namedWindow(morph_name, WINDOW_AUTOSIZE);
		//imshow(morph_name, img_dst);


		//contour
		vector<vector<Point> > contours;
		findContours(img_dst, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

		//contour를 근사화한다.
		vector<Point2f> approx;
		Mat img_result;
		img_result = img_input.clone();

		//좌표들
		float points[6][2];

		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

			if (approx.size() == 4 && (fabs(contourArea(Mat(approx))) > 20000 && fabs(contourArea(Mat(approx))) < 10000000)) //면적이 일정크기 이상이어야 한다. 
			{

				int size = approx.size();

				//Contour를 근사화한 직선을 그린다.
				for (int k = 0; k < size; k++) {
					line(img_result, approx[k], approx[(k + 1) % size], Scalar(0, 255, 0), 3);
					points[k][0] = approx[k].x;
					points[k][1] = approx[k].y;
				}
				if (isContourConvex(Mat(approx)))
					setLabel(img_result, "rectangle", contours[i]); //사각형
				break;
			}

		}

		// 잘못된 값이 나올 때
		if (points[0][0] < 0)
			continue;
		
		//4는 중심점
		points[4][0] = (points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4;
		points[4][1] = (points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4;

		// points[5]는 길이
		points[5][0] = (-(points[0][0] + points[1][0] - points[2][0] - points[3][0])) / 2;
		points[5][1] = (-points[0][1] + points[1][1] + points[2][1] - points[3][1]) / 2;

		
		for (int i = 0; i < 4; i++) {
				if ((points[i][0] - points[4][0]) < 0 && (points[i][1] - points[4][1]) < 0) {
					swapab(&(points[i][0]), &(points[3][0]));
					swapab(&(points[i][1]), &(points[3][1]));
					//break;
				}
				else if ((points[i][0] - points[4][0]) < 0 && (points[i][1] - points[4][1]) > 0) {
					swapab(&(points[i][0]), &(points[2][0]));
					swapab(&(points[i][1]), &(points[2][1]));
					//break;
				}
				else if ((points[i][0] - points[4][0]) > 0 && (points[i][1] - points[4][1]) < 0) {
					swapab(&(points[i][0]), &(points[0][0]));
					swapab(&(points[i][1]), &(points[0][1]));
					//break;
				}
				else if ((points[i][0] - points[4][0]) > 0 && (points[i][1] - points[4][1]) > 0) {
					swapab(&(points[i][0]), &(points[1][0]));
					swapab(&(points[i][1]), &(points[1][1]));
					//break;
				}
		}
		

		for (int i = 0; i < 6; i++) {
			if (i == 5)
				cout << "길이 : x =" << points[i][0] << ", y =" << points[i][1] << endl;
			else if (i == 4)
				cout << "중심 : (" << points[i][0] << ", " << points[i][1] << ")" << endl;
			else
				cout << "좌표 : (" << points[i][0] << ", " << points[i][1] << ")" << endl;

		}




		float dpt_max_X = 0;
		float dpt_max_Y = 0;
		float dpt_min_X = 1000;
		float dpt_min_Y = 1000;

		dpt_max_X = max(points[0][0], points[1][0]);
		dpt_max_Y = max(points[1][1], points[2][1]);
		dpt_min_X = min(points[2][0], points[3][0]);
		dpt_min_Y = min(points[0][1], points[3][1]);

		printf("%.2f %.2f %.2f %.2f\n", dpt_max_X, dpt_max_Y, dpt_min_X, dpt_min_Y);



		int width2 = -1;
		int height2 = -1;
		if (width2 <= 0 || height2 <= 0) {
			cout << "width : ";
			cin >> width2;
			cout << "height : ";
			cin >> height2;
		}
		
		vector<vector<Point> > g_point(height2*width2, vector<Point>(0));
		//Rect rect[height2][width2];
		//vector<Point>* g_point = (vector<Point>*) malloc(sizeof(vector<Point>)*height2*width2);

		cout << sizeof(g_point) / sizeof(g_point[0]) << endl;

		int block_num = 0;

		for (int i = 0; i < height2; i++) {
			for (int j = 0; j < width2; j++) {
				//vector<Point> 자료형에 네 점을 넣고 boundingRect(vector<point> a)로 사각형을 만듦 -> Rect rect[][]에 저장
				if (points[3][1]<points[0][1]) {
					//왼위
					float x3 = points[3][0] + j * (points[0][0] - points[3][0]) / width2 + i * (points[2][0] - points[3][0]) / height2 + i * j * abs(points[0][0] - points[3][0] - (points[1][0] - points[2][0])) / (height2*width2);
					float y3 = points[3][1] + j * (points[0][1] - points[3][1]) / width2 + i * (points[2][1] - points[3][1]) / height2;
					g_point[block_num].push_back(Point(x3 + (points[0][0] - points[3][0]) / width2, y3 + (points[0][1] - points[3][1]) / width2));//[0]
					g_point[block_num].push_back(Point(x3 + (points[1][0] - points[0][0]) / height2 + (points[1][0] - points[3][0] + width2) / (width2), y3 + (points[2][1] - points[3][1]) / height2 + (points[1][1] - points[2][1]) / width2));//[1]
					g_point[block_num].push_back(Point(x3 + (points[2][0] - points[3][0]) / width2, y3 + (points[2][1] - points[3][1]) / height2));//[2]
					g_point[block_num].push_back(Point(x3 + (points[3][0] - points[3][0]) / width2, y3 + (points[3][1] - points[3][1]) / height2));//[3]            
				}
				else {
					//오위
					float x3 = points[3][0] + j * (points[0][0] - points[3][0]) / width2 + i * (points[2][0] - points[3][0]) / height2 + i * j * abs(points[0][0] - points[3][0] - (points[1][0] - points[2][0])) / (height2*width2);
					float y3 = points[3][1] + j * (points[0][1] - points[3][1]) / width2 + i * (points[2][1] - points[3][1]) / height2;
					g_point[block_num].push_back(Point(x3 + (points[0][0] - points[3][0]) / width2, y3 + (points[0][1] - points[3][1]) / width2));//[0]
					g_point[block_num].push_back(Point(x3 + (points[1][0] - points[3][0]) / width2, y3 + (points[2][1] - points[3][1]) / height2 + (points[1][1] - points[2][1]) / width2));//[1]
					g_point[block_num].push_back(Point(x3 + (points[2][0] - points[3][0]) / width2, y3 + (points[2][1] - points[3][1]) / height2));//[2]
					g_point[block_num].push_back(Point(x3 + (points[3][0] - points[3][0]) / width2, y3 + (points[3][1] - points[3][1]) / height2));//[3]
				}

				for (int k = 0; k < 4; k++) {
					line(img_result, g_point[block_num][k % 4], g_point[block_num][(k + 1) % 4], Scalar(0, 0, 255), 3);
				}
				string s1 = "s" + to_string(block_num);
				//setLabel_small(img_result, s1, g_point[block_num],0,0,255); //사각형
				block_num++;
			}
		}

		printf("Everything done\n");


		imshow("dst", img_dst);
		//imshow("input", img_input);
		imshow("result", img_result);

		

	}  // while 괄호

}




// Mark Depth Image and Get Depth Value
void markDepthImage(colorizer color_map, pipeline pipe_color, pipeline pipe_depth) {
	rs2::config cfg_color;
	cfg_color.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);

	//Instruct pipeline to start streaming with the requested configuration
	pipe_color.start(cfg_color);

	// Depth 
	pipe_depth.start();
	rs2::frameset data = pipe_depth.wait_for_frames();
	rs2::frame frame = data.get_depth_frame();
	depth_frame dpt_frame = frame.as<depth_frame>();
	rs2::frame depth = color_map(data.get_depth_frame());

	int dpt_width = dpt_frame.get_width();
	int dpt_height = dpt_frame.get_height();
	printf("dpt frame width: %d, height: %d\n", dpt_width, dpt_height);

	const auto origin_name = "test_depth_new";

	int width = 640;
	int height = 480;

	Mat image(Size(dpt_width, dpt_height), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

	cv::imshow(origin_name, image);



	/*
	FILE *fp = fopen("test_img/depth information.txt", "w");
	// drawMarker
	int markerSize = 5;
	int thickness = 1;
	int line_type = 8;
	
	for (int y = 260; y < 650; y++)  // 0 ~ dpt_height
	{
		for (int x = 620; x < 680; x++) // 0 ~ dpt_width
		{
			float pixel_distance_in_meters = dpt_frame.get_distance(x, y); //millimeter
			dist_array[x][y] = pixel_distance_in_meters;

			fprintf(fp, "(%d, %d) = %.3f\n", x, y, dist_array[x][y]);

			if (dist_array[x][y] < 1.536 && dist_array[x][y] > 1.500) {
				drawMarker(image, Point(x, y), Scalar(255, 255, 255), MARKER_SQUARE, markerSize, thickness, line_type);
			}


		}
	}

	imwrite("test_img/marked_img.jpg", image);
	fclose(fp);
	
	while (1) {
		cv::imshow("new Image", image);
		if (cvWaitKey(10) == 27)
			break;
	}
	*/
	// 필요한 정보만 출력
	/*
	for (int x = 0; x < dpt_width; x++)  // 0 ~ dpt_height -> 100 ~ 520
	{
		//if(x >390 && x < 926)
		//printf("(%d, 120) : %0.3f (m) \n ", x, dist_array[x][120]);

	}
	*/

	// drawMarker
	int markerSize = 5;
	int thickness = 1;
	int line_type = 8;


	// distance = 0인 부분만 검은색 / 나머지 흰색
	Mat grayscaled(Size(image.cols, image.rows), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
	for (int y = 0; y < grayscaled.rows; y++)  // 0 ~ dpt_height
	{
		for (int x = 0; x < grayscaled.cols; x++) // 0 ~ dpt_width
		{

			if (dist_array[x][y] < 0.48)
				drawMarker(grayscaled, Point(x, y), Scalar(255, 255, 255), MARKER_SQUARE, markerSize, thickness, line_type);
			else
				drawMarker(grayscaled, Point(x, y), Scalar(0, 0, 0), MARKER_SQUARE, markerSize, thickness, line_type);

		}
		imshow("new Image", image);
		//imwrite("test_img/Marked_Image_new1.jpg", image);

	}


	// 왼쪽 검은 Depth 이미지 처리
	for (int y = 0; y < grayscaled.rows; y++)  // 0 ~ dpt_height
	{
		for (int x = 0; x < 120; x++) // 0 ~ dpt_width
		{
			drawMarker(grayscaled, Point(x, y), Scalar(0, 0, 0), MARKER_SQUARE, markerSize, thickness, line_type);
		}
		//imshow("new Image", image);
		//imwrite("test_img/Marked_Image_new1.jpg", image);


		Mat image_gray, image_canny;
		cvtColor(grayscaled, image_gray, COLOR_BGR2GRAY);	// 채널3 -> 1 변경
		imwrite("test_img/1.grayscaled_mode.jpg", image_gray);
		// Blur & Edge
		blur(image_gray, image_canny, Size(3, 3));			// Reduce noise with a kernel 3x3
		imwrite("test_img/2.grayscaled_blur_mode2.jpg", image_canny);
		Canny(image_canny, image_canny, 0, 250 * 3, 3);
		imwrite("test_img/3.grayscaled_canny_mode3.jpg", image_canny);

		// morph
		Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(40, 40));
		Mat morph;
		morphologyEx(image_canny, image_canny, CV_MOP_CLOSE, kernel);
		imwrite("test_img/4.morph_mode.jpg", image_canny);

		// contour
		size_t idx, i;
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(image_canny, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);	// RETR_LIST : 가장 바깥쪽 윤곽만 추출, CHAIN_APPROX_SIMPLE: 선의 끝점 2개만 추출
		//findContours(image_canny, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat contours_img;
		cvtColor(image_canny, contours_img, CV_GRAY2BGR);
		for (idx = 0; idx < contours.size(); idx++) {
			RotatedRect rect = minAreaRect(contours[idx]);
			double areaRatio = abs(contourArea(contours[idx])) / (rect.size.width * rect.size.height);
			drawContours(contours_img, contours, idx, Scalar(0, 100, 100), 1);
		}

		imshow("contour image", contours_img);
		imwrite("test_img/5.contour_mode.jpg", contours_img);

		// 짧은 거리면 그어 주기
		vector<vector<Point>> min, max;


		// Contour 근사화
		vector<Point2f> approx;
		Mat img_result;
		img_result = contours_img.clone();


		//좌표들
		float points[6][2];

		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

			if (approx.size() == 4 && (fabs(contourArea(Mat(approx))) > 20000 && fabs(contourArea(Mat(approx))) < 100000)) //면적이 일정크기 이상이어야 한다. 
			{

				int size = approx.size();

				//Contour를 근사화한 직선을 그린다.
				for (int k = 0; k < size; k++) {
					line(img_result, approx[k], approx[(k + 1) % size], Scalar(0, 255, 0), 3);
					points[k][0] = approx[k].x;
					points[k][1] = approx[k].y;
				}
				if (isContourConvex(Mat(approx)))
					setLabel(img_result, "rectangle", contours[i]); //사각형
				//break;
			}

		}
		//4는 중심점
		points[4][0] = (points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4;
		points[4][1] = (points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4;

		points[5][0] = (points[0][0] + points[1][0] - points[2][0] - points[3][0]) / 2;
		points[5][1] = (-points[0][1] + points[1][1] + points[2][1] - points[3][1]) / 2;


		for (int i = 0; i < 6; i++) {
			if (i == 5)
				cout << "길이 : x =" << points[i][0] << ", y =" << points[i][1] << endl;
			else if (i == 4)
				cout << "중심 : (" << points[i][0] << ", " << points[i][1] << ")" << endl;
			else
				cout << "좌표 : (" << points[i][0] << ", " << points[i][1] << ")" << endl;

		}

		//imshow("canny", image_canny);
		//imshow("input", grayscaled);
		imshow("result", img_result);
		waitKey(0);


		// Gaussian Blur
		//Mat img_dst;
		//GaussianBlur(grayscaled, img_dst, Size(7, 7), 0);

		/*
		for (int y = 0; y < dpt_height; y++)  // 0 ~ dpt_height
		{
			for (int x = 130; x < 1200; x++) // 0 ~ dpt_width
			{

				if (dist_array[x][y] == 0)
					drawMarker(image, Point(x, y), Scalar(255, 255, 255), MARKER_SQUARE, markerSize, thickness, line_type);

			}
			//imshow("new Image", image);
			imwrite("test_img/Marked_Image_new100.jpg", image);
		}
		*/
		/* 일정 범위에서 drawMarker
		for (int y = 212; y < 405; y++)  // 0 ~ dpt_height
		{
			for (int x = 571; x < 853; x++) // 0 ~ dpt_width
			{

				if (dist_array[x][y] == 0)
					drawMarker(image, Point(x, y), Scalar(255, 255, 255), MARKER_SQUARE, markerSize, thickness, line_type);

			}

			imshow("new Image", image);
			imwrite("test_img/marked_Image_new.jpg", image);
		}
		*/
	}
}
//Contour 영역 내에 텍스트 쓰기 

void setLabel(Mat& image, string str, vector<Point> contour)
{
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.5;
	int thickness = 1;
	int baseline = 0;

	Size text = getTextSize(str, fontface, scale, thickness, &baseline);
	Rect r = boundingRect(contour);

	Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	rectangle(image, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(200, 200, 200), CV_FILLED);
	putText(image, str, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}


void storeTwoImage(colorizer color_map, pipeline pipe_color, pipeline pipe_depth) {

	rs2::config cfg_color;
	cfg_color.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	//Instruct pipeline to start streaming with the requested configuration
	pipe_color.start(cfg_color);


	const auto color_name = "color Image";
	namedWindow(color_name, WINDOW_AUTOSIZE);				// Create window of color image
	//namedWindow(infrared_name, WINDOW_AUTOSIZE);

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg_depth;
	//Add desired streams to configuration
	cfg_depth.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	cfg_depth.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	//Instruct pipeline to start streaming with the requested configuration
	pipe_depth.start(cfg_depth);

	const auto depth_name = "depth Image";
	namedWindow(depth_name, WINDOW_AUTOSIZE);				// Create windows of depth image


	while (waitKey(1) < 0)
	{


		const int width = 640;
		const int height = 480;

		// RGB color frame
		rs2::frameset data_color = pipe_color.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frame color = data_color.get_color_frame();


		// Query frame size (width and height)
		const int w_c = color.as<rs2::video_frame>().get_width();
		const int h_c = color.as<rs2::video_frame>().get_height();


		// Create OpenCV matrix of size (w,h) from the color data
		Mat image2(Size(w_c, h_c), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);


		Mat img_c;
		resize(image2, img_c, Size(width, height), 0, CV_INTER_LINEAR);

		// Depth frame
		rs2::frameset data_depth = pipe_depth.wait_for_frames();
		rs2::frame depth = color_map(data_depth.get_depth_frame());


		// Query frame size (width and height)
		const int w_d = depth.as<rs2::video_frame>().get_width();
		const int h_d = depth.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat image(Size(w_d, h_d), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
		resize(image, image, Size(width, height), 0, CV_INTER_LINEAR);

		// Update the window with new data
		//resizeWindow(depth_name, width, height);
		imshow(depth_name, image);

		// Update the window with new datac
		imshow(color_name, img_c);

		char new_win[14] = "capture d-img";
		char new_win2[14] = "capture c-img";

		if (cvWaitKey(10) == 'c') {

			cvNamedWindow(new_win, CV_WINDOW_AUTOSIZE);
			//cvNamedWindow(new_win2, CV_WINDOW_AUTOSIZE);
			//cvtColor(image, image, CV_BGR2GRAY);
			imshow(new_win, image);
			imshow(new_win, img_c);
			
			imwrite("test_img/DepthImage_ball_.jpg", image);
			imwrite("test_img/RGBImage_ball_.jpg", img_c);

		}
		else if (cvWaitKey(10) == 27)
			break;
	}
			
			
			
}

// Show RGB, Infrared, Depth Image
void showThreeImage(colorizer color_map, pipeline pipe_color, pipeline pipe_depth) {
	
	rs2::config cfg_color;
	cfg_color.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

	//Instruct pipeline to start streaming with the requested configuration
	pipe_color.start(cfg_color);

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg_depth;

	//Add desired streams to configuration
	cfg_depth.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	cfg_depth.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	//Instruct pipeline to start streaming with the requested configuration
	pipe_depth.start(cfg_depth);

	const auto depth_name = "depth Image";
	namedWindow(depth_name, WINDOW_AUTOSIZE);				// Create windows of depth image

	const auto color_name = "color Image";
	namedWindow(color_name, WINDOW_AUTOSIZE);				// Create window of color image

	const auto infrared_name = "infrared Image";
	namedWindow(infrared_name, WINDOW_AUTOSIZE);

	
	while (waitKey(1) < 0)
	{

		const int width = 640;
		const int height = 480;

		// RGB color frame
		rs2::frameset data_color = pipe_color.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frame color = data_color.get_color_frame();


		// Infrared frame
		rs2::frameset data_depth = pipe_depth.wait_for_frames();
		rs2::frame ir_frame = data_depth.first(RS2_STREAM_INFRARED);


		// Depth frame
		rs2::frame depth = color_map(data_depth.get_depth_frame());
		//rs2::frame depth = data_depth.get_depth_frame();

		// Query frame size (width and height)
		const int w_d = depth.as<rs2::video_frame>().get_width();
		const int h_d = depth.as<rs2::video_frame>().get_height();

		const int w_c = color.as<rs2::video_frame>().get_width();
		const int h_c = color.as<rs2::video_frame>().get_height();


		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat image(Size(w_d, h_d), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
		Mat image2(Size(w_c, h_c), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

		//	Infrared Image
		/*
		const int width = 640;
		const int height = 480;
		rs2::frameset data_depth = pipe_depth.wait_for_frames();
		rs2::frame ir_frame = data_depth.first(RS2_STREAM_INFRARED);
		*/
		Mat ir(Size(width, height), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);
		equalizeHist(ir, ir);
		applyColorMap(ir, ir, COLORMAP_JET);


		Mat img_d;
		Mat img_c;
		resize(image, img_d, Size(width, height), 0, CV_INTER_LINEAR);
		resize(image2, img_c, Size(width, height), 0, CV_INTER_LINEAR);

		
		imshow(depth_name, img_d);
		imshow(infrared_name, ir);
		imshow(color_name, img_c);
			

	}
}

// Infrared Image 저장
void storeInfraredImage(colorizer color_map, pipeline pipe_depth) {

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg_depth;
	//Add desired streams to configuration
	cfg_depth.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);		// YOU HAVE TO Adjust frame size (In this case 640x480)
	cfg_depth.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	//Instruct pipeline to start streaming with the requested configuration
	pipe_depth.start(cfg_depth);

	const auto Infrared_name = "Infrared Image";
	namedWindow(Infrared_name, WINDOW_AUTOSIZE);				
															
	
	while (waitKey(1) < 0)
	{
		const int width = 640;
		const int height = 480;
		
		// Infrared frame
		rs2::frameset data_depth = pipe_depth.wait_for_frames();
		rs2::frame ir_frame = data_depth.first(RS2_STREAM_INFRARED);

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat ir(Size(width, height), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);
		equalizeHist(ir, ir);
		applyColorMap(ir, ir, COLORMAP_JET);

		imshow(Infrared_name, ir);
		char new_win[20] = "Compare-Infrared";
		
		if (cvWaitKey(10) == 'c') {
			cvNamedWindow(new_win, CV_WINDOW_AUTOSIZE);
			imshow(new_win, ir);
			imwrite("test_img/Compare/Compare-Infrared.jpg", ir);
		}
		else if (cvWaitKey(10) == 27)
			break;
	}
	
}

// Depth Image 저장
void storeDepthImage(colorizer color_map, pipeline pipe_depth) {

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg_depth;
	//Add desired streams to configuration
	cfg_depth.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	cfg_depth.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	//Instruct pipeline to start streaming with the requested configuration
	pipe_depth.start(cfg_depth);

	const auto depth_name = "depth Image";
	namedWindow(depth_name, WINDOW_AUTOSIZE);				// Create windows of depth image

	while (waitKey(1) < 0)
	{
		// Depth frame
		rs2::frameset data_depth = pipe_depth.wait_for_frames();
		rs2::frame depth = color_map(data_depth.get_depth_frame());


		// Query frame size (width and height)
		const int w_d = depth.as<rs2::video_frame>().get_width();
		const int h_d = depth.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat image(Size(w_d, h_d), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);


		// Update the window with new data
		//resizeWindow(depth_name, width, height);
		imshow(depth_name, image);

		char new_win[12] = "capture img";

		if (cvWaitKey(10) == 'c') {

			cvNamedWindow(new_win, CV_WINDOW_AUTOSIZE);
			imshow(new_win, image);
			imwrite("test_img/test_depth_new.jpg", image);

		}
		else if (cvWaitKey(10) == 27)
			break;

	}
}

void storeColorImage(pipeline pipe_color) {

	rs2::config cfg_color;
	cfg_color.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	//Instruct pipeline to start streaming with the requested configuration
	pipe_color.start(cfg_color);


	const auto color_name = "color Image";
	namedWindow(color_name, WINDOW_AUTOSIZE);				// Create window of color image

	while (waitKey(1) < 0)
	{

		const int width = 640;
		const int height = 480;

		// RGB color frame
		rs2::frameset data_color = pipe_color.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frame color = data_color.get_color_frame();


		// Query frame size (width and height)
		const int w_c = color.as<rs2::video_frame>().get_width();
		const int h_c = color.as<rs2::video_frame>().get_height();


		// Create OpenCV matrix of size (w,h) from the color data
		Mat image2(Size(w_c, h_c), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);


		Mat img_c;
		resize(image2, img_c, Size(width, height), 0, CV_INTER_LINEAR);


		// Update the window with new data
		imshow(color_name, img_c);

		char new_win[12] = "capture img";
		int count = 0;


		//IplImage *Capture = new IplImage(img_c);
		//cvSaveImage("test.jpg", Capture);


		//imshow(infrared_name, image3);
		if (cvWaitKey(10) == 'c') {

			cvNamedWindow(new_win, CV_WINDOW_AUTOSIZE);
			imshow(new_win, img_c);
			imwrite("test_img/test1.jpg", img_c);

		}
		else if (cvWaitKey(10) == 27) break;

	}
	cvDestroyAllWindows();

}



