#pragma once
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <chrono>
#include "yolov8_utils.h"
class Yolov8Onnx {
public:
	//构造函数初始化_OrtMemoryInfo对象，使用内存类型为CPU输出类型
	Yolov8Onnx() :_OrtMemoryInfo(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtDeviceAllocator, OrtMemType::OrtMemTypeCPUOutput)) {};
	~Yolov8Onnx() {};// delete _OrtMemoryInfo;


public:
	/** \brief Read onnx-model
	* \param[in] modelPath:onnx-model path
	* \param[in] isCuda:if true,use Ort-GPU,else run it on cpu.
	* \param[in] cudaID:if isCuda==true,run Ort-GPU on cudaID.
	* \param[in] warmUp:if isCuda==true,warm up GPU-model.
	*/
	bool ReadModel(const std::string& modelPath, bool isCuda = true, int cudaID = 0, bool warmUp = true);

	/** \brief  detect.
	* \param[in] srcImg:a 3-channels image.
	* \param[out] output:detection results of input image.
	*/
	bool OnnxDetect(cv::Mat& srcImg, std::vector<OutputSeg>& output);
	/** \brief  detect,batch size= _batchSize
	* \param[in] srcImg:A batch of images.
	* \param[out] output:detection results of input images.
	*/
	bool OnnxBatchDetect(std::vector<cv::Mat>& srcImg, std::vector<std::vector<OutputSeg>>& output);

	//单张图片检测函数
	bool DetectOneImg(string img_path,vector<OutputSeg>& result);
	//多张图片连续检测
	void DetectImgs(string dir_path,int start_index,int images_num);

private:

	template <typename T>
	T VectorProduct(const std::vector<T>& v)
	{
		return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
	};
	int Preprocessing(const std::vector<cv::Mat>& SrcImgs, std::vector<cv::Mat>& OutSrcImgs, std::vector<cv::Vec4d>& params);   
	//const float _netAnchors[3][6] = { { 10,13, 16,30, 33,23 },{ 30,61, 62,45, 59,119 },{ 116,90, 156,198, 373,326 } };
	const int _netWidth = 640;   //ONNX-net-input-width
	const int _netHeight = 640;  //ONNX-net-input-height

	int _batchSize = 1;  //if multi-batch,set this
	bool _isDynamicShape = false;//onnx support dynamic shape


	float _classThreshold = 0.5;  //置信度
	float _nmsThreshold = 0.2;


	//ONNXRUNTIME	
	Ort::Env _OrtEnv = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_ERROR, "Yolov8-Detect");   //创建环境对象，对错误日志进行记录
	Ort::SessionOptions _OrtSessionOptions = Ort::SessionOptions();  //创建默认会话选项对象
	Ort::Session* _OrtSession = nullptr;            //会话对象
	Ort::MemoryInfo _OrtMemoryInfo;						//内存信息对象
#if ORT_API_VERSION < ORT_OLD_VISON
	char* _inputName, * _output_name0;
#else
	std::shared_ptr<char> _inputName, _output_name0;
#endif

	std::vector<char*> _inputNodeNames; //输入节点名
	std::vector<char*> _outputNodeNames;//输出节点名

	size_t _inputNodesNum = 0;        //输入节点数
	size_t _outputNodesNum = 0;       //输出节点数

	ONNXTensorElementDataType _inputNodeDataType; //数据类型
	ONNXTensorElementDataType _outputNodeDataType;
	std::vector<int64_t> _inputTensorShape; //输入张量shape

	std::vector<int64_t> _outputTensorShape;

public:
	std::vector<std::string> _className = {
		//"0","1","2","3"
		"0"
		//"0","1"
	};
};