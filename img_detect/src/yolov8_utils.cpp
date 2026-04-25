#include "img_detect/yolov8_utils.h"
#include <string>
using namespace cv;
using namespace std;
bool CheckParams(int netHeight, int netWidth, const int* netStride, int strideSize) {
	if (netHeight % netStride[strideSize - 1] != 0 || netWidth % netStride[strideSize - 1] != 0)
	{
		cout << "Error:_netHeight and _netWidth must be multiple of max stride " << netStride[strideSize - 1] << "!" << endl;
		return false;
	}
	return true;
}

void LetterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params, const cv::Size& newShape,
	bool autoShape, bool scaleFill, bool scaleUp, int stride, const cv::Scalar& color)
{
	if (false) {
		int maxLen = MAX(image.rows, image.cols);
		outImage = Mat::zeros(Size(maxLen, maxLen), CV_8UC3);
		image.copyTo(outImage(Rect(0, 0, image.cols, image.rows)));
		params[0] = 1;
		params[1] = 1;
		params[3] = 0;
		params[2] = 0;
	}

	cv::Size shape = image.size();
	float r = std::min((float)newShape.height / (float)shape.height,
		(float)newShape.width / (float)shape.width);
	if (!scaleUp)
		r = std::min(r, 1.0f);

	float ratio[2]{ r, r };
	int new_un_pad[2] = { (int)std::round((float)shape.width * r),(int)std::round((float)shape.height * r) };

	auto dw = (float)(newShape.width - new_un_pad[0]);
	auto dh = (float)(newShape.height - new_un_pad[1]);

	if (autoShape)
	{
		dw = (float)((int)dw % stride);
		dh = (float)((int)dh % stride);
	}
	else if (scaleFill)
	{
		dw = 0.0f;
		dh = 0.0f;
		new_un_pad[0] = newShape.width;
		new_un_pad[1] = newShape.height;
		ratio[0] = (float)newShape.width / (float)shape.width;
		ratio[1] = (float)newShape.height / (float)shape.height;
	}

	dw /= 2.0f;
	dh /= 2.0f;

	if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1])
	{
		cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
	}
	else {
		outImage = image.clone();
	}

	int top = int(std::round(dh - 0.1f));
	int bottom = int(std::round(dh + 0.1f));
	int left = int(std::round(dw - 0.1f));
	int right = int(std::round(dw + 0.1f));
	params[0] = ratio[0];
	params[1] = ratio[1];
	params[2] = left;
	params[3] = top;
	cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos, std::vector<OutputSeg>& output, const MaskParams& maskParams) {
	//cout << maskProtos.size << endl;

	int seg_channels = maskParams.segChannels;
	int net_width = maskParams.netWidth;
	int seg_width = maskParams.segWidth;
	int net_height = maskParams.netHeight;
	int seg_height = maskParams.segHeight;
	float mask_threshold = maskParams.maskThreshold;
	Vec4f params = maskParams.params;
	Size src_img_shape = maskParams.srcImgShape;

	Mat protos = maskProtos.reshape(0, { seg_channels,seg_width * seg_height });

	Mat matmul_res = (maskProposals * protos).t();
	Mat masks = matmul_res.reshape(output.size(), { seg_width,seg_height });
	vector<Mat> maskChannels;
	split(masks, maskChannels);
	for (int i = 0; i < output.size(); ++i) {
		Mat dest, mask;
		//sigmoid
		cv::exp(-maskChannels[i], dest);
		dest = 1.0 / (1.0 + dest);

		Rect roi(int(params[2] / net_width * seg_width), int(params[3] / net_height * seg_height), int(seg_width - params[2] / 2), int(seg_height - params[3] / 2));
		dest = dest(roi);
		resize(dest, mask, src_img_shape, INTER_NEAREST);

		//crop
		Rect temp_rect = output[i].box;
		mask = mask(temp_rect) > mask_threshold;
		output[i].boxMask = mask;
	}
}

void GetMask2(const Mat& maskProposals, const Mat& mask_protos, OutputSeg& output, const MaskParams& maskParams) {
	int seg_channels = maskParams.segChannels;
	int net_width = maskParams.netWidth;
	int seg_width = maskParams.segWidth;
	int net_height = maskParams.netHeight;
	int seg_height = maskParams.segHeight;
	float mask_threshold = maskParams.maskThreshold;
	Vec4f params = maskParams.params;
	Size src_img_shape = maskParams.srcImgShape;

	Rect temp_rect = output.box;
	//crop from mask_protos
	int rang_x = floor((temp_rect.x * params[0] + params[2]) / net_width * seg_width);
	int rang_y = floor((temp_rect.y * params[1] + params[3]) / net_height * seg_height);
	int rang_w = ceil(((temp_rect.x + temp_rect.width) * params[0] + params[2]) / net_width * seg_width) - rang_x;
	int rang_h = ceil(((temp_rect.y + temp_rect.height) * params[1] + params[3]) / net_height * seg_height) - rang_y;

	//如果下面的 mask_protos(roi_rangs).clone()位置报错，说明你的output.box数据不对，或者矩形框就1个像素的，开启下面的注释部分防止报错。
	rang_w = MAX(rang_w, 1);
	rang_h = MAX(rang_h, 1);
	if (rang_x + rang_w > seg_width) {
		if (seg_width - rang_x > 0)
			rang_w = seg_width - rang_x;
		else
			rang_x -= 1;
	}
	if (rang_y + rang_h > seg_height) {
		if (seg_height - rang_y > 0)
			rang_h = seg_height - rang_y;
		else
			rang_y -= 1;
	}

	vector<Range> roi_rangs;
	roi_rangs.push_back(Range(0, 1));
	roi_rangs.push_back(Range::all());
	roi_rangs.push_back(Range(rang_y, rang_h + rang_y));
	roi_rangs.push_back(Range(rang_x, rang_w + rang_x));

	//crop
	Mat temp_mask_protos = mask_protos(roi_rangs).clone();
	Mat protos = temp_mask_protos.reshape(0, { seg_channels,rang_w * rang_h });
	Mat matmul_res = (maskProposals * protos).t();
	Mat masks_feature = matmul_res.reshape(1, { rang_h,rang_w });
	Mat dest, mask;

	//sigmoid
	cv::exp(-masks_feature, dest);
	dest = 1.0 / (1.0 + dest);

	int left = floor((net_width / seg_width * rang_x - params[2]) / params[0]);
	int top = floor((net_height / seg_height * rang_y - params[3]) / params[1]);
	int width = ceil(net_width / seg_width * rang_w / params[0]);
	int height = ceil(net_height / seg_height * rang_h / params[1]);

	resize(dest, mask, Size(width, height), INTER_NEAREST);
	mask = mask(temp_rect - Point(left, top)) > mask_threshold;
	output.boxMask = mask;

}

void CalculateMaxApple(string apple_r_txt)
{
	ifstream ifs;
	ifs.open(apple_r_txt, ios_base::in);
	string s;
	int max_r = 0;
	while (ifs >> s)
	{
		int num = stoi(s);
		if (num > max_r)
			max_r = num;
	}
	cout << "max apple radius=" << max_r << endl;
	ifs.close();
}
void get_left_right_box(cv::Mat& ori_img,vector<OutputSeg>& result,vector<OutputSeg>&left_box,vector<OutputSeg>&right_box)
{
	left_box.clear();
	right_box.clear();
	int w = ori_img.cols;
	int detect_nums = result.size();
	//分成左右区域
	for (int i = 0; i < detect_nums; i++)
	{
		if (result[i].box.x - result[i].box.width / 2 <= w / 2)
		{
			left_box.push_back(result[i]);
		}
		else
		{
			right_box.push_back(result[i]);
		}
	}
	sort(left_box.begin(),left_box.end(),compareOutput2);
	sort(right_box.begin(),right_box.end(),compareOutput2);
}
vector<vector<cv::Rect>> PlanPickSequence(std::vector<OutputSeg>& result, std::vector<vector<OutputSeg>>& left_sequence, std::vector<vector<OutputSeg>>& right_sequence,cv::Mat& origin_image)
{
	int w = origin_image.cols;
	int detect_nums = result.size();
	vector<OutputSeg> left_region;
	vector<OutputSeg> right_region;
	//分成左右区域
	for (int i = 0; i < detect_nums; i++)
	{
		if (result[i].box.x - result[i].box.width / 2 <= w / 2)
		{
			left_region.push_back(result[i]);
		}
		else
		{
			right_region.push_back(result[i]);
		}
	}
	//对左区域和有区域分别规划
	vector<vector<cv::Rect>> regions;
	vector<cv::Rect> left=PlanHalfRegion(left_region, left_sequence,origin_image,Scalar(0,255,0));
	vector<cv::Rect> right=PlanHalfRegion(right_region, right_sequence, origin_image,Scalar(0,0,255));
	return {left,right};
}

vector<cv::Rect> PlanHalfRegion(std::vector<OutputSeg>& region, std::vector<vector<OutputSeg>>& plan_re, cv::Mat& img, Scalar scalar)
{
	int apple_nums = region.size();
	sort(region.begin(), region.end(), compareOutput);
	std::vector<bool> visited(apple_nums, false);
	std::vector<cv::Rect> regionBox;
	for (int i = 0; i < apple_nums; i++)
	{
		if (visited[i]) continue;
		vector<OutputSeg> cur_apple_group;
		cur_apple_group.push_back(region[i]);
		visited[i] = true;
		int min_left_x = region[i].box.x;
		int min_left_y = region[i].box.y;
		int max_right_x = region[i].box.x + region[i].box.width;
		int max_right_y = region[i].box.y + region[i].box.height;
		for (int j = i+1; j < apple_nums; j++)   //划定子区
		{
			if (visited[j])
				continue;
			bool add_to_group = false;
			for (int k = 0; k < cur_apple_group.size(); k++)
			{
				cv::Point center1 = cur_apple_group[k].box.tl() + cur_apple_group[k].box.br();
				cv::Point center2 = region[j].box.tl() + region[j].box.br();
				center1.x /= 2;
				center1.y /= 2;
				center2.x /= 2;
				center2.y /= 2;
				int distance = cv::norm(center1 - center2);
				if (distance <= 150)    //在当前序列中存在与region[j]中心距小于150的苹果
				{
					add_to_group = true;
					break;
				}
			}
			if (add_to_group)
			{
				cur_apple_group.push_back(region[j]);
				//获取区域的位置
				min_left_x = min_left_x <= region[j].box.x ? min_left_x : region[j].box.x;
				min_left_y = min_left_y <= region[j].box.y ? min_left_y : region[j].box.y;
				max_right_x = max_right_x >= region[j].box.x + region[j].box.width ? max_right_x : region[j].box.x + region[j].box.width;
				max_right_y = max_right_y >= region[j].box.y + region[j].box.height ? max_right_y : region[j].box.y + region[j].box.height;
				visited[j] = true;
			}
		}
		Point region_tl;
		Point region_br;
		region_tl.x = min_left_x-20<0?0:min_left_x-20;
		region_tl.y = min_left_y-20<0?0:min_left_y;
		region_br.x = max_right_x+20<img.cols?max_right_x+20:img.cols;
		region_br.y = max_right_y+20<img.rows?max_right_y+20:img.rows;
		regionBox.push_back(cv::Rect(region_tl, region_br));
		sort(cur_apple_group.begin(), cur_apple_group.end(), compareOutput2);   //每个子区中的苹果按x方向从小到大排序,
																					//这样每个子区vector第一个苹果的检测框的的左上角点就可以用来对所有子区进行排序
		plan_re.push_back(cur_apple_group);	
	}
	//对子区进行排序  按从左到右排序
	sort(plan_re.begin(), plan_re.end(), compareRegionByX);      //依据子区中左角点x坐标从小到大进行排序
	
	//绘制采摘子区
	sort(regionBox.begin(), regionBox.end(),compareRectByTopLeft);
	for (int i = 0; i < regionBox.size(); i++)
	{
		cv::rectangle(img, regionBox[i], scalar, 3);
		string region_index = to_string(i);
		int baseline;
		Size label_size = getTextSize(region_index, FONT_HERSHEY_SIMPLEX, 1, 2, &baseline);
		cv::putText(img, region_index, Point(regionBox[i].x, regionBox[i].y - 5), FONT_HERSHEY_SIMPLEX, 3, scalar, 2);
	}
	return regionBox;
}

//从从左到右，上到下采摘
bool compareRectByTopLeft(const cv::Rect& rect1, const cv::Rect& rect2)
{
	if (rect1.tl().x < rect2.tl().x)
		return true;
	else if (rect1.tl().x == rect2.tl().x)
		return rect1.tl().y < rect2.tl().y;
	else
		return false;
}

bool compareOutput(const OutputSeg& output1, const OutputSeg& output2)
{
	if (output1.box.tl().y < output2.box.tl().y)
		return true;
	else if (output1.box.tl().y == output2.box.tl().y)
		return output1.box.tl().x < output2.box.tl().x;
	else
		return false;
}
bool compareOutput2(const OutputSeg& output1, const OutputSeg& output2)
{
	if (output1.box.tl().x < output2.box.tl().x)
		return true;
	else if (output1.box.tl().x == output2.box.tl().x)
		return output1.box.tl().y < output2.box.tl().y;
	else
		return false;
}

bool compareRegionByX(const vector<OutputSeg>& region1, const vector<OutputSeg>& region2)
{
	if (region1[0].box.tl().x < region2[0].box.tl().x)
		return true;
	else if (region1[0].box.tl().x == region2[0].box.tl().x)
		return region1[0].box.tl().y < region2[0].box.tl().y;
	return false;
}

int Maturity_detection(cv::Mat& picking_apple)
{
	cv::Mat apple_img;
	cv::Mat circle_mask(picking_apple.size(),CV_8UC1,Scalar(0));
	cv::circle(circle_mask, Point(picking_apple.cols / 2, picking_apple.rows / 2), min(picking_apple.cols / 2, picking_apple.rows / 2)-5, Scalar(255), -1);
	picking_apple.copyTo(apple_img, circle_mask);     //截取内接圆图像
	

	//在这里找到苹果的真实区域  参考LPX空间
	
	//cv::Mat lab_img;
	//cv::cvtColor(apple_img, lab_img, COLOR_BGR2Lab);
	//vector<cv::Mat> vec_lab;
	//cv::split(lab_img, vec_lab);
	//cv::Mat l = vec_lab[0];
	//cv::Mat a = vec_lab[1];
	//cv::Mat b = vec_lab[2];
	//cv::Mat a2 = a.clone();
	//for (int i=0;i<a.rows;i++)
	//{
	//	for (int j = 0; j < a.cols; j++)
	//	{
	//		int num=a.at<uchar>(i, j);
	//		if (num < 128)
	//		{
	//			a2.at<uchar>(i, j) = 0;
	//			apple_img.at<Vec3b>(i, j) = cv::Vec3b{0,0,0};   //截取可以看得到的苹果区域
	//		}
	//		else {
	//			a2.at<uchar>(i, j) = 255;
	//		} 
	//	}
	//}
	
	//将苹果的颜色信息与成熟度进行映射

	/*imshow("l", l);
	imshow("a", a);
	imshow("b", b);
	imshow("a2", a2);*/
	imshow("apple_img", apple_img);
	return 0;
}
	

void DrawPred(Mat& img, vector<OutputSeg> result, std::vector<std::string> classNames, vector<Scalar> color) {
	Mat mask = img.clone();
	//ofstream apple_r;
	//apple_r.open("F:/c++/yolov8_deploy_test2/yolov8_deploy_test2/apple_r.txt", ios_base::app);
	for (int i = 0; i < result.size(); i++) {
		int left, top;
		left = result[i].box.x;
		top = result[i].box.y;
		int color_num = i;
		//apple_r << result[i].box.width << endl;
		rectangle(img, result[i].box, color[result[i].id], 3, 8);
		string label = classNames[result[i].id] + ":" + to_string(result[i].confidence).substr(0,4);
		int baseLine;
		Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 0.5, &baseLine);
		top = max(top, labelSize.height);
		//rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
		//putText(img, label, Point(left-5, top-10), FONT_HERSHEY_SIMPLEX, 1, color[result[i].id], 2);
	}
	//apple_r.close();
	//imshow("1", img);
	//waitKey();
	//destroyAllWindows();

}



