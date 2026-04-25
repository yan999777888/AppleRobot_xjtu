#include "img_detect/yolov8_onnx.h"
using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace Ort;




bool Yolov8Onnx::ReadModel(const std::string& modelPath, bool isCuda, int cudaID, bool warmUp) {
	if (_batchSize < 1) _batchSize = 1;
	try
	{
		std::vector<std::string> available_providers = GetAvailableProviders();
		auto cuda_available = std::find(available_providers.begin(), available_providers.end(), "CUDAExecutionProvider");


		if (isCuda && (cuda_available == available_providers.end()))
		{
			std::cout << "Your ORT build without GPU. Change to CPU." << std::endl;
			std::cout << "************* Infer model on CPU! *************" << std::endl;
		}
		else if (isCuda && (cuda_available != available_providers.end()))
		{
			std::cout << "************* Infer model on GPU! *************" << std::endl;
		}
		else
		{
			std::cout << "************* Infer model on CPU! *************" << std::endl;
		}
		_OrtSessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);  //设置图优化级别为拓展级
#ifdef _WIN32   //windows系统下要使用wstring宽字符类型
		std::wstring model_path(modelPath.begin(), modelPath.end());
		_OrtSession = new Ort::Session(_OrtEnv, model_path.c_str(), _OrtSessionOptions);//会话环境、模型路径、会话选项
#else
		_OrtSession = new Ort::Session(_OrtEnv, modelPath.c_str(), _OrtSessionOptions);
#endif

		Ort::AllocatorWithDefaultOptions allocator;
		//init input
		_inputNodesNum = _OrtSession->GetInputCount();
#if ORT_API_VERSION < ORT_OLD_VISON
		_inputName = _OrtSession->GetInputName(0, allocator);
		_inputNodeNames.push_back(_inputName);
#else
		_inputName = std::move(_OrtSession->GetInputNameAllocated(0, allocator));  //获取索引为0 的输入对象的指针
		_inputNodeNames.push_back(_inputName.get());
#endif
		//cout << _inputNodeNames[0] << endl;
		Ort::TypeInfo inputTypeInfo = _OrtSession->GetInputTypeInfo(0);  //获取索引为0的输入节点类型信息
		auto input_tensor_info = inputTypeInfo.GetTensorTypeAndShapeInfo();  //获取输入节点的张量类型和形状信息
		_inputNodeDataType = input_tensor_info.GetElementType();    //获取输入节点的数据类型
		_inputTensorShape = input_tensor_info.GetShape();  //获取输入节点的形状信息

		if (_inputTensorShape[0] == -1) //检查输入节点的第一个维度是否为 -1。如果是 -1，表示该维度的形状是动态的，需要进行调整。
		{
			_isDynamicShape = true;
			_inputTensorShape[0] = _batchSize;

		}
		//输入节点的第三个和第四个维度是否为 -1。如果其中任一维度为 -1，表示该维度的形状是动态的，需要进行调整。同样，会将 _isDynamicShape 设置为 true，表示输入形状是动态的
		if (_inputTensorShape[2] == -1 || _inputTensorShape[3] == -1) {
			_isDynamicShape = true;
			_inputTensorShape[2] = _netHeight;
			_inputTensorShape[3] = _netWidth;
		}
		//init output
		_outputNodesNum = _OrtSession->GetOutputCount();
#if ORT_API_VERSION < ORT_OLD_VISON
		_output_name0 = _OrtSession->GetOutputName(0, allocator);
		_outputNodeNames.push_back(_output_name0);
#else
		_output_name0 = std::move(_OrtSession->GetOutputNameAllocated(0, allocator));
		_outputNodeNames.push_back(_output_name0.get());
#endif
		Ort::TypeInfo type_info_output0(nullptr);
		type_info_output0 = _OrtSession->GetOutputTypeInfo(0);  //output0

		auto tensor_info_output0 = type_info_output0.GetTensorTypeAndShapeInfo();
		_outputNodeDataType = tensor_info_output0.GetElementType();
		_outputTensorShape = tensor_info_output0.GetShape();

		if (isCuda && warmUp) {
			//draw run
			cout << "Start warming up" << endl;
			size_t input_tensor_length = VectorProduct(_inputTensorShape);
			float* temp = new float[input_tensor_length];
			std::vector<Ort::Value> input_tensors;
			std::vector<Ort::Value> output_tensors;
			input_tensors.push_back(Ort::Value::CreateTensor<float>(
				_OrtMemoryInfo, temp, input_tensor_length, _inputTensorShape.data(),
				_inputTensorShape.size()));
			for (int i = 0; i < 3; ++i) {
				output_tensors = _OrtSession->Run(Ort::RunOptions{ nullptr },
					_inputNodeNames.data(),
					input_tensors.data(),
					_inputNodeNames.size(),
					_outputNodeNames.data(),
					_outputNodeNames.size());
			}

			delete[]temp;
		}
		return true;
	}
	catch (const std::exception&) {
		return false;
	}

}

int Yolov8Onnx::Preprocessing(const std::vector<cv::Mat>& srcImgs, std::vector<cv::Mat>& outSrcImgs, std::vector<cv::Vec4d>& params) {
	outSrcImgs.clear();
	Size input_size = Size(_netWidth, _netHeight);
	for (int i = 0; i < srcImgs.size(); ++i) {
		Mat temp_img = srcImgs[i];
		Vec4d temp_param = { 1,1,0,0 }; //缩放的比例和边界偏移量
		if (temp_img.size() != input_size) {
			Mat borderImg;
			LetterBox(temp_img, borderImg, temp_param, input_size, false, false, true, 32); //将输入图像调整为指定大小，并在调整过程中保持恒定的纵横比
			//cout << borderImg.size() << endl;
			outSrcImgs.push_back(borderImg);
			params.push_back(temp_param);
		}
		else {
			outSrcImgs.push_back(temp_img);
			params.push_back(temp_param);
		}
	}

	int lack_num = _batchSize - srcImgs.size();
	if (lack_num > 0) {
		for (int i = 0; i < lack_num; ++i) {
			Mat temp_img = Mat::zeros(input_size, CV_8UC3);
			Vec4d temp_param = { 1,1,0,0 };
			outSrcImgs.push_back(temp_img);
			params.push_back(temp_param);
		}
	}
	return 0;

}
bool Yolov8Onnx::OnnxDetect(cv::Mat& srcImg, std::vector<OutputSeg>& output) {
	std::vector<cv::Mat> input_data = { srcImg };
	std::vector<std::vector<OutputSeg>> tenp_output;
	if (OnnxBatchDetect(input_data, tenp_output)) {
		output = tenp_output[0];
		return true;
	}
	else return false;
}
bool Yolov8Onnx::OnnxBatchDetect(std::vector<cv::Mat>& srcImgs, std::vector<std::vector<OutputSeg>>& output) {
	vector<Vec4d> params;
	vector<Mat> input_images;
	cv::Size input_size(_netWidth, _netHeight);
	//preprocessing 前处理
	auto pre_start_time = chrono::high_resolution_clock::now();
	Preprocessing(srcImgs, input_images, params);  //将图像比例调整为640*640
	cv::Mat blob = cv::dnn::blobFromImages(input_images, 1 / 255.0, input_size, Scalar(0, 0, 0), true, false);//数将一组输入图像转换为一个 blob（二进制大型对象）
	int64_t input_tensor_length = VectorProduct(_inputTensorShape);
	std::vector<Ort::Value> input_tensors;
	std::vector<Ort::Value> output_tensors;
	input_tensors.push_back(Ort::Value::CreateTensor<float>(_OrtMemoryInfo, (float*)blob.data, input_tensor_length, _inputTensorShape.data(), _inputTensorShape.size()));
	auto pre_end_time = chrono::high_resolution_clock::now();
	//cout << "preprocessing_time=" << chrono::duration_cast<chrono::milliseconds>(pre_end_time - pre_start_time).count() << " ms" << endl;
	//检测
	auto detect_start_time = chrono::high_resolution_clock::now();
	output_tensors = _OrtSession->Run(Ort::RunOptions{ nullptr },  //将图像变为张量信息输入获得张量类型的输出
		_inputNodeNames.data(),
		input_tensors.data(),
		_inputNodeNames.size(),
		_outputNodeNames.data(),
		_outputNodeNames.size()         
	);
	auto detect_end_time = chrono::high_resolution_clock::now();
	//cout << "detect_time=" << chrono::duration_cast<chrono::milliseconds>(detect_end_time - detect_start_time).count() << " ms" << endl;
	//post-process 后处理
	auto postprocessing_start_time = chrono::high_resolution_clock::now();
	int net_width = _className.size() + 4;
	float* all_data = output_tensors[0].GetTensorMutableData<float>();
	_outputTensorShape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
	int64_t one_output_length = VectorProduct(_outputTensorShape) / _outputTensorShape[0];
	for (int img_index = 0; img_index < srcImgs.size(); ++img_index) {
		Mat output0 = Mat(Size((int)_outputTensorShape[2], (int)_outputTensorShape[1]), CV_32F, all_data).t();  //[bs,5,8400]=>[bs,8400,5]
		all_data += one_output_length;
		float* pdata = (float*)output0.data;
		int rows = output0.rows;
		std::vector<int> class_ids;//结果id数组
		std::vector<float> confidences;//结果每个id对应置信度数组
		std::vector<cv::Rect> boxes;//每个id矩形框
		for (int r = 0; r < rows; ++r) {    //stride
			cv::Mat scores(1, _className.size(), CV_32F, pdata + 4);
			Point classIdPoint;
			double max_class_socre;
			minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);  //对所有类别的分数进行对比找出最高分数及其对应的类别
			max_class_socre = (float)max_class_socre;
			if (max_class_socre >= _classThreshold) { //如果大于类别置信度

				//rect [x,y,w,h]   params[0]、params[1]、params[2]、params[3]分别对应原图缩放为640，640时，X,Y对应的缩放比例和偏移量
				float x = (pdata[0] - params[img_index][2]) / params[img_index][0];  //x
				float y = (pdata[1] - params[img_index][3]) / params[img_index][1];  //y
				//float w = pdata[2] / params[img_index][0];  //w
				//float h = pdata[3] / params[img_index][1];  //h
				float w = pdata[2] / params[img_index][0]*0.9;  //w  缩放0.9
				float h = pdata[3] / params[img_index][1]*0.9;  //h	 缩放0.9
				//int left = MAX(int(x - 0.5 * w + 0.5), 0);
				int left = MAX(int(x - 0.5 * w + 0.5), 0);
				int top = MAX(int(y - 0.5 * h + 0.5), 0);
				//int top = MAX(int(y - 0.5 * h + 0.5), 0)+5;  //下移5个像素
				class_ids.push_back(classIdPoint.x);
				confidences.push_back(max_class_socre);
				boxes.push_back(Rect(left, top, int(w + 0.5), int(h + 0.5)));
			}
			pdata += net_width;//下一行
		}

		vector<int> nms_result;
		//对识别的所有检测框进行非极大值抑制，对同一个对象多个检测框的情况，只选取置信度最高的那个框
		cv::dnn::NMSBoxes(boxes, confidences, _classThreshold, _nmsThreshold, nms_result);   
		std::vector<vector<float>> temp_mask_proposals;
		cv::Rect holeImgRect(0, 0, srcImgs[img_index].cols, srcImgs[img_index].rows);
		std::vector<OutputSeg> temp_output;
		for (int i = 0; i < nms_result.size(); ++i) {
			int idx = nms_result[i];
			OutputSeg result;
			result.id = class_ids[idx];
			result.confidence = confidences[idx];
			result.box = boxes[idx] & holeImgRect;
			temp_output.push_back(result);
		}
		output.push_back(temp_output);
		auto postprocessing_end_time = chrono::high_resolution_clock::now();
		//cout << "postprecessing_time=" << chrono::duration_cast<chrono::milliseconds>(postprocessing_end_time - postprocessing_start_time).count()<<" ms" << endl;
		auto sum_detect_time = chrono::duration_cast<chrono::milliseconds>(postprocessing_end_time - pre_start_time).count();
		//cout << "model_detect_sum_time=" << sum_detect_time << " ms" << endl;
		//cout << "FPS=" << 1000.0 / (double)(sum_detect_time) << endl;
	}

	if (output.size())
		return true;
	else
		return false;

}

//检测一张图片
bool Yolov8Onnx:: DetectOneImg(string img_path,vector<OutputSeg>& result)
{
	cv::Mat img=cv::imread(img_path);
	if(img.empty())
	{
		cout<<"加载图片失败！"<<endl;
		return false;
	}
	cv::Scalar color{255,0,0};
	if (OnnxDetect(img, result))
    {
        DrawPred(img, result, _className, {color});
    }
    else {
		cout << "Detect Failed!" << endl;
		return false;
	}
	 imshow("img", img);
    cv::waitKey(0);
	return true;

}

//多张图片连续检测
void Yolov8Onnx:: DetectImgs(string dir_path,int start_index,int images_num)
{
	Mat image1;
	cv::Scalar color{255,0,0};
	for (int i = start_index; i <= images_num; i++)
	{
		int zero_nums = 6 - to_string(i + start_index).size();
		string image_name = string(zero_nums, '0') + to_string(i + start_index);
		string image_path = dir_path + image_name + ".jpg";
		image1 = imread(image_path);
		vector<OutputSeg> result;
		if (OnnxDetect(image1, result)) {
			DrawPred(image1, result, _className, {color});
		}
		else {
			cout << "Detect Failed!" << endl;
		}
		imshow("img", image1);
		waitKey(20);
		}
}