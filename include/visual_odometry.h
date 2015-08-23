#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_

#include <vector>
#include <opencv2/opencv.hpp>

#include "pinhole_camera.h"

class VisualOdometry
{
public:
	//目前所在帧，通过限制条件更好的确定第一帧和第二帧图像用于确定初始位置
	enum FrameStage {
		STAGE_FIRST_FRAME,//第一帧
		STAGE_SECOND_FRAME,//第二帧
		STAGE_DEFAULT_FRAME//默认帧
	};

	VisualOdometry(PinholeCamera* cam);
	virtual ~VisualOdometry();

	/// 提供一个图像
	void addImage(const cv::Mat& img, int frame_id);

	/// 获取当前帧的旋转
	cv::Mat getCurrentR() { return cur_R_; }
	/// 获得当前帧的平移
	cv::Mat getCurrentT() { return cur_t_; }

protected:
	/// 处理第一帧
	virtual bool processFirstFrame();
	/// 处理第二帧
	virtual bool processSecondFrame();
	/// 处理完开始的两个帧后处理所有帧
	virtual bool processFrame(int frame_id);
	/// 计算绝对尺度
	double getAbsoluteScale(int frame_id);
	/// 特征检测
	void featureDetection(cv::Mat image, std::vector<cv::Point2f> &px_vec);
	/// 特征跟踪
	void featureTracking(cv::Mat image_ref, cv::Mat image_cur,
		std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities);

protected:
	FrameStage frame_stage_;                 //!< 当前为第几帧
	PinholeCamera *cam_;                     //!< 相机
	cv::Mat new_frame_;                      //!< 当前帧
	cv::Mat last_frame_;                     //!< 最近处理帧

	cv::Mat cur_R_;//!< 计算出当前的姿态，后期将图像和姿态进行封装为帧
	cv::Mat cur_t_;//!< 当前的平移

	std::vector<cv::Point2f> px_ref_;      //!< 在参考帧中用于跟踪的特征点
	std::vector<cv::Point2f> px_cur_;      //!< 在当前帧中跟踪的特征点
	std::vector<double> disparities_;      //!< 第一帧与第二帧对应光流跟踪的特征之间的像素差值

	double focal_;//!<相机焦距
	cv::Point2d pp_; //!<相机中心点

};

#endif // VISUAL_ODOMETRY_H_
