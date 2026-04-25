#pragma once
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
using namespace std;

#define YOLO_P6 false //�Ƿ�ʹ��P6ģ��
#define ORT_OLD_VISON 12  //ort1.12.0 ֮ǰ�İ汾Ϊ�ɰ汾API
struct OutputSeg {
	int id;             //������id
	float confidence;   //������Ŷ�
	cv::Rect box;       //���ο�
	cv::Mat boxMask;       //���ο���mask����ʡ�ڴ�ռ�ͼӿ��ٶ�
	float depth;   //���ĵ����
};
struct MaskParams {
	int segChannels = 32;
	int segWidth = 160;
	int segHeight = 160;
	int netWidth = 640;
	int netHeight = 640;
	float maskThreshold = 0.5;
	cv::Size srcImgShape;
	cv::Vec4d params;

};
bool CheckParams(int netHeight, int netWidth, const int* netStride, int strideSize);
void DrawPred(cv::Mat& img, std::vector<OutputSeg> result, std::vector<std::string> classNames, std::vector<cv::Scalar> color);
void LetterBox(const cv::Mat& image, cv::Mat& outImage,
	cv::Vec4d& params, //[ratio_x,ratio_y,dw,dh]
	const cv::Size& newShape = cv::Size(640, 640),
	bool autoShape = false,
	bool scaleFill = false,
	bool scaleUp = true,
	int stride = 32,
	const cv::Scalar& color = cv::Scalar(114, 114, 114));
void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams);
void GetMask2(const cv::Mat& maskProposals, const cv::Mat& maskProtos, OutputSeg& output, const MaskParams& maskParams);
void CalculateMaxApple(std:: string apple_r_txt);
//�Լ�������в�ժ˳��滮
vector<vector<cv::Rect>> PlanPickSequence(std::vector<OutputSeg>& result, std::vector<vector<OutputSeg>>& left_sequence, std::vector<vector<OutputSeg>>& right_sequence,cv::Mat& origin_image);
vector<cv::Rect> PlanHalfRegion(std::vector<OutputSeg>& region, std::vector<vector<OutputSeg>>& plan_re,cv::Mat& img,cv::Scalar scalar);
bool compareRectByTopLeft(const cv::Rect& rect1, const cv::Rect& rect2);
bool compareOutput(const OutputSeg& output1, const OutputSeg& output2);
bool compareOutput2(const OutputSeg& output1, const OutputSeg& output2);
bool compareRegionByX(const vector<OutputSeg>& region1, const vector<OutputSeg>& region2);
void get_left_right_box(cv::Mat& ori_img,vector<OutputSeg>& result,vector<OutputSeg>&left_box,vector<OutputSeg>&right_box);
//��Ŀ���ժƻ�����г���ȼ��
int Maturity_detection(cv::Mat& picking_apple);

