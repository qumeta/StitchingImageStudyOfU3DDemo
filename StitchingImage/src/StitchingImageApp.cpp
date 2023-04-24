#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>

#include "Poco/AutoPtr.h"
#include "Poco/Util/Application.h"
#include "Poco/Util/Option.h"
#include "Poco/Util/OptionSet.h"
#include "Poco/Util/HelpFormatter.h"
#include "Poco/Util/AbstractConfiguration.h"

using Poco::Util::Application;
using Poco::Util::Option;
using Poco::Util::OptionSet;
using Poco::Util::HelpFormatter;
using Poco::Util::AbstractConfiguration;
using Poco::Util::OptionCallback;
using Poco::AutoPtr;

#include <opencv2/opencv.hpp>
#include <opencv2/core/quaternion.hpp>

using namespace cv;

class StitchingImageApp : public Application
{

public:
	StitchingImageApp() : _helpRequested(false)
	{
	}

protected:
	void initialize(Application& self)
	{
		loadConfiguration(); // load default configuration files, if present
		Application::initialize(self);
		// add your own initialization code here
	}

	void uninitialize()
	{
		// add your own uninitialization code here
		Application::uninitialize();
	}

	void reinitialize(Application& self)
	{
		Application::reinitialize(self);
		// add your own reinitialization code here
	}

	void defineOptions(OptionSet& options)
	{
		Application::defineOptions(options);

		options.addOption(
			Option("help", "h", "display help information on command line arguments")
			.required(false)
			.repeatable(false)
			.callback(OptionCallback<StitchingImageApp>(this, &StitchingImageApp::handleHelp)));

		options.addOption(
			Option("Path", "p", "set fairway image path")
			.required(true)
			.repeatable(false)
			.argument("Path")
			.callback(OptionCallback<StitchingImageApp>(this, &StitchingImageApp::handleFairwayPath)));

		options.addOption(
			Option("Max", "m", "process image num")
			.required(false)
			.repeatable(false)
			.argument("Max")
			.callback(OptionCallback<StitchingImageApp>(this, &StitchingImageApp::handleMax)));

		options.addOption(
			Option("Orthographic", "o", "export orthographic image")
			.required(false)
			.repeatable(false)
			.argument("Orthographic")
			.callback(OptionCallback<StitchingImageApp>(this, &StitchingImageApp::handleOrthographic)));

		options.addOption(
			Option("Dummy", "d", "set fairway image path")
			.required(false)
			.repeatable(false)
			.argument("Dummy")
			.callback(OptionCallback<StitchingImageApp>(this, &StitchingImageApp::handleDummyPath)));
	}

	void handleHelp(const std::string& name, const std::string& value)
	{
		_helpRequested = true;
		displayHelp();
		stopOptionsProcessing();
	}

	void handleFairwayPath(const std::string& name, const std::string& value)
	{
		config().setString("StitchingImage.Path", value);
	}

	void handleDummyPath(const std::string& name, const std::string& value)
	{
		config().setBool("StitchingImage.Dummy", value == "true");
	}

	void handleMax(const std::string& name, const std::string& value)
	{
		config().setInt("StitchingImage.Max", atoi(value.c_str()));
	}

	
	void handleOrthographic(const std::string& name, const std::string& value)
	{
		config().setBool("StitchingImage.Orthographic", value == "true");
	}

	
	void displayHelp()
	{
		HelpFormatter helpFormatter(options());
		helpFormatter.setCommand(commandName());
		helpFormatter.setUsage("OPTIONS");
		helpFormatter.setHeader("合成照片");
		helpFormatter.format(std::cout);
	}

	void defineProperty(const std::string& def)
	{
		std::string name;
		std::string value;
		std::string::size_type pos = def.find('=');
		if (pos != std::string::npos)
		{
			name.assign(def, 0, pos);
			value.assign(def, pos + 1, def.length() - pos);
		}
		else name = def;
		config().setString(name, value);
	}

	int main(const ArgVec& args)
	{
		if (!_helpRequested)
		{
			logger().information("Command line:");
			std::ostringstream ostr;
			for (ArgVec::const_iterator it = argv().begin(); it != argv().end(); ++it)
			{
				ostr << *it << ' ';
			}
			logger().information(ostr.str());
			logger().information("Arguments to main():");
			for (ArgVec::const_iterator it = args.begin(); it != args.end(); ++it)
			{
				logger().information(*it);
			}
			logger().information("Application properties:");

			// StitchingImage.FairwayPath
			auto fairwayPath = std::filesystem::u8path(config().getString("StitchingImage.Path")).generic_string();

			// 查找所有风机图片(fan blade fairway)
			std::vector<std::string> imgs;
			for (const auto& file : std::filesystem::directory_iterator(fairwayPath))
			{
				// 湖北恩施板桥风电场_T20_1_LE_1.JPG
				auto str = file.path().extension().generic_string();
				transform(str.begin(), str.end(), str.begin(), ::tolower);

				auto name = file.path().filename().generic_string();
				auto no = name.substr(name.find_last_of('_') + 1, name.find_first_of('.') - name.find_last_of('_') - 1);
				auto notNum = false;

				for (auto c : no)
				{
					if (!std::isdigit(c))
					{
						notNum = true;
						break;
					}
				}

				if (notNum)
					continue;

				if (str == ".png" || str == ".jpg") 
				{
					imgs.push_back(file.path().generic_string());
				}
			}

			std::sort(imgs.begin(), imgs.end(), path_lesser);

			basicPanoramaStitching(fairwayPath, imgs);
		}
		return Application::EXIT_OK;
	}

	// 按照文件名字的数字顺序升序排序（3.JPG 24.JPG , 3 < 24）
	bool static path_lesser(std::filesystem::path path1, std::filesystem::path path2)
	{
		auto name1 = path1.filename().generic_string();
		auto name2 = path2.filename().generic_string();

		auto elem1 = std::atoi(name1.substr(name1.find_last_of('_') + 1, name1.find_first_of('.') - name1.find_last_of('_') - 1).c_str());
		auto elem2 = std::atoi(name2.substr(name2.find_last_of('_') + 1, name2.find_first_of('.') - name2.find_last_of('_') - 1).c_str());


		return elem1 < elem2;
	}

	void convertRTFromU3D(Vec3d cR, Vec3d cT, Mat& R, Mat& tvec)
	{
		Matx33d u3d2cv = Matx33d(1, 0, 0, 0, -1, 0, 0, 0, 1);
		//Matx33d u3d2cv = Matx33d(1, 0, 0, 0, 1, 0, 0, 0, 1);

		Vec3d cR_Rad = cR * (CV_PI / 180);

		// 必须分开算才能和U3D匹配（ZXY旋转顺序）
		//Mat R_T = Mat(Quatd::createFromEulerAngles(cR, QuatEnum::EulerAnglesType::EXT_YXZ).toRotMat3x3()).t();

		Mat R_X = Mat(Quatd::createFromXRot(cR_Rad[0]).toRotMat3x3());
		Mat R_Y = Mat(Quatd::createFromYRot(cR_Rad[1]).toRotMat3x3());
		Mat R_Z = Mat(Quatd::createFromZRot(cR_Rad[2]).toRotMat3x3());

		Mat R_T = (R_Y * R_X * R_Z).t();
		//Mat R_T = (R_X * R_Y * R_Z).t();

		R = u3d2cv * R_T;
		tvec = u3d2cv * Mat(-1 * R_T * cT);
	}

	void computeC2MC1(const Mat& R1, const Mat& tvec1, const Mat& R2, const Mat& tvec2,
		Mat& R_1to2, Mat& tvec_1to2)
	{
		//c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
		R_1to2 = R2 * R1.t();
		tvec_1to2 = R2 * (-R1.t() * tvec1) + tvec2;
	}
	
	Mat computeHomography(const Mat& R_1to2, const Mat& tvec_1to2, const double d_inv, const Mat& normal)
	{
		Mat homography = R_1to2 + d_inv * tvec_1to2 * normal.t();
		return homography;
	}

	// https://blog.csdn.net/Kalenee/article/details/80659489
	Point3f getWorldPoints(Mat R, Mat T, Mat K, Point2f inPoints)
	{
		double zConst = 0;

		//获取图像坐标
		cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
		imagePoint.at<double>(0, 0) = inPoints.x;
		imagePoint.at<double>(1, 0) = inPoints.y;

		//计算比例参数S
		Mat leftSideMat = R.inv() * K.inv() * imagePoint;
		Mat rightSideMat = R.inv() * T;
		auto s = zConst + rightSideMat.at<double>(2, 0);
		s /= leftSideMat.at<double>(2, 0);

		//计算世界坐标
		Mat wcPoint = R.inv() * (s * K.inv() * imagePoint - T);
		Point3f worldPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));
		return worldPoint;
	}

	void getHomography(Mat K, Vec3d C1R, Vec3d C1T, Vec3d C1N, double distance, Vec3d C2R, Vec3d C2T, int w, int h, Mat& offsetH, int& xmin, int& ymin, int& xmax, int& ymax, int& xoffset, int& yoffset)
	{
		Mat R1, R2, tvec1, tvec2;
		convertRTFromU3D(C1R, C1T, R1, tvec1);
		convertRTFromU3D(C2R, C2T, R2, tvec2);

		Mat rvec1, rvec2;
		Rodrigues(R1, rvec1);
		Rodrigues(R2, rvec2);

		Mat R_1to2, t_1to2;
		computeC2MC1(R1, tvec1, R2, tvec2, R_1to2, t_1to2);
		Mat rvec_1to2;
		Rodrigues(R_1to2, rvec_1to2);

		//std::cout << rvec_1to2 << t_1to2 << std::endl;

		// 计算距离(相机面法向量,相机必须在Z轴水平面之上U3D坐标系)
		//Mat normal = (Mat_<double>(3, 1) << 0, 0, 1);
		Mat normal = Mat(C1N);
		Mat normal1 = R1 * normal;

		Mat origin(3, 1, CV_64F, Scalar(0));
		Mat origin1 = R1 * origin + tvec1;

		// TODO == C1T[2] (z)
		//double d_inv1 = cv_abs(1.0 / normal1.dot(origin1));
		double inv1 = normal1.dot(origin1) - distance;// +(index > 0 ? 5 : 0);
		double d_inv1 = 1.0 / inv1;
		//d_inv1 = 0; 

		//std::cout << "inv1 " << inv1 << " ==z " << C1T[2] << std::endl;

		//! [compute-homography-from-camera-displacement]
		Mat homography_euclidean = computeHomography(R_1to2, t_1to2, d_inv1, normal1);
		Mat homography = K * homography_euclidean * K.inv();

		homography /= homography.at<double>(2, 2);
		//! [compute-homography-from-camera-displacement]

		//std::cout << homography << std::endl;

		std::vector<Point2f> icorners(4);
		icorners[0] = Point2f(0, 0);
		icorners[1] = Point2f((float)(w - 0), 0);
		icorners[2] = Point2f((float)(w - 0), (float)(h - 0));
		icorners[3] = Point2f(0, (float)(h - 0));
		std::vector<Point2f> corners;
		perspectiveTransform(icorners, corners, homography);

		float xminF = float(INT32_MAX);// 0;
		float yminF = float(INT32_MAX);// 0;
		float xmaxF = float(INT32_MIN);// w;
		float ymaxF = float(INT32_MIN);// h;

		for (auto i = 0; i < 4; i++) {
			//std::cout << "corners.x " << corners[i].x << " corners.y " << corners[i].y << std::endl;

			xminF = std::min(xminF, corners[i].x);
			yminF = std::min(yminF, corners[i].y);

			xmaxF = std::max(xmaxF, corners[i].x);
			ymaxF = std::max(ymaxF, corners[i].y);
		}

		//xminF = std::min(xminF, corners[0].x);
		//yminF = std::min(yminF, corners[0].y);

		//xmaxF = std::max(xmaxF, corners[2].x);
		//ymaxF = std::max(ymaxF, corners[2].y);

		xmin = int(xminF); //int(xminF - 0.5);
		ymin = int(yminF); //int(yminF - 0.5);
		xmax = int(xmaxF); //int(xmaxF + 0.5);
		ymax = int(ymaxF); //int(ymaxF + 0.5);

		xoffset = xmin;	// corners[0].x;
		yoffset = ymin; // corners[0].y;
		std::cout << "xmin " << xmin << " ymin " << ymin << " xmax " << xmax << " ymax " << ymax << std::endl;

		// translate
		Mat Ht = Mat::eye(3, 3, CV_64F);
		//Ht.at<double>(0, 2) = -xmin;
		//Ht.at<double>(1, 2) = -ymin;
		Ht.at<double>(0, 2) = -xoffset;
		Ht.at<double>(1, 2) = -yoffset;

		offsetH = Ht * homography;
	}

	void basicPanoramaStitching(const std::string& imagePath, const std::vector<std::string>& imgFiles)
	{
		auto imageOutPath = std::filesystem::path(imagePath);
		imageOutPath /= std::filesystem::path("out");
		std::filesystem::directory_entry entry(imageOutPath);
		if (!entry.exists())
			std::filesystem::create_directories(imageOutPath.parent_path());

		Mat imgFirst = imread(imgFiles[0], IMREAD_UNCHANGED);
		//auto cnMos = std::vector<Mat>();
		auto cnMosPosition = std::vector<Vec3d>();
		auto cnMosRotation = std::vector<Vec3d>();
		auto cnMosNormal = std::vector<Vec3d>();
		auto cnMosDistance = std::vector<double>();
		auto cMosRotation_camera = std::vector<Vec3d>();

		//! [load-config]
		FileStorage fs(imagePath + "/u3d.yml", FileStorage::READ);
		//Mat cMoSrc;
		Mat cameraMatrixK;
		Vec3d cMoSrcPosition, cMoRotation, cMoNormal, cMo_rotation_camera;
		double cMoDistance;

		int num;
		int w, h;
		double fovH, fovV, fov, focalLength, sensorSizeX, sensorSizeY;

		fs["num"] >> num;
		fs["w"] >> w;
		fs["h"] >> h;

		auto maxNum = config().getInt("StitchingImage.Max", num);
		if (num > maxNum && maxNum > 1)
			num = maxNum;

		// scale
		auto scale = config().getInt("StitchingImage.Scale", 1);
		w /= scale;
		h /= scale;

		fs["focalLength"] >> focalLength;
		fs["sensorSizeX"] >> sensorSizeX;
		fs["sensorSizeY"] >> sensorSizeY;


		fs["fovH"] >> fovH;
		fs["fovV"] >> fovV;
		fs["fov"] >> fov;
		fs["cameraMatrix"] >> cameraMatrixK;
		
		//K = Matrix(
		//((s_u, skew, u_0),
		//(0, s_v, v_0),
		//(0, 0, 1)))

		cameraMatrixK /= scale;

		cameraMatrixK.at<double>(2, 2) = 1;

		//fs["cMo"] >> cMoSrc;
		fs["cMo_position"] >> cMoSrcPosition;
		fs["cMo_rotation"] >> cMoRotation;
		fs["cMo_normal"] >> cMoNormal;
		fs["cMo_distance"] >> cMoDistance;
		fs["cMo_rotation_camera"] >> cMo_rotation_camera;

		for (auto i = 1; i <= imgFiles.size(); i++) {
			Mat cMo;
			char key[24];

			Vec3d pv;
			snprintf(key, sizeof(key), "c%dMo_position", i);
			fs[key] >> pv;
			cnMosPosition.push_back(pv);

			Vec3d rv;
			snprintf(key, sizeof(key), "c%dMo_rotation", i);
			fs[key] >> rv;

			cnMosRotation.push_back(rv);

			snprintf(key, sizeof(key), "c%dMo_rotation_camera", i);
			fs[key] >> rv;
			cMosRotation_camera.push_back(rv);

			Vec3d nv;
			snprintf(key, sizeof(key), "c%dMo_normal", i);
			fs[key] >> nv;
			cnMosNormal.push_back(nv);

			double distance;
			snprintf(key, sizeof(key), "c%dMo_distance", i);
			fs[key] >> distance;
			cnMosDistance.push_back(distance);
		}

		fs.release();

		auto sumCols = 0;
		auto sumRows = 0;

		auto imgPerspectives = std::vector<Mat>();
		// 图片位置
		auto imgPts = std::vector<Point2i>();
		// 图片尺寸
		int imgMinX = INT32_MAX;
		int imgMinY = INT32_MAX;
		int imgMaxX = INT32_MIN;
		int imgMaxY = INT32_MIN;

		for (auto i = 0; i < num; i++) {
			std::cout << "perspective " << (i + 1) << "/" << (num) << std::endl;

			Mat img = imread(samples::findFile(imgFiles[i]), IMREAD_UNCHANGED);
			// add_alpha_channel
			if (img.channels() == 3)
			{
				std::vector<Mat> rgbLayer;
				split(img, rgbLayer);         // seperate channels

				Mat alpha = cv::Mat::ones(img.rows, img.cols, CV_8UC1) * 255;
				Mat dst = cv::Mat(img.rows, img.cols, CV_8UC4);

				//添加透明度通道
				rgbLayer.push_back(alpha);
				merge(rgbLayer, img);			// glue together again
			}

			Vec3d pv = cnMosPosition[i];
			Vec3d rv = cnMosRotation[i];
			Vec3d nv = cnMosNormal[i];
			double distance = cnMosDistance[i];

			//! [Perspective]
			Mat offsetH;
			int xmin, ymin, xmax, ymax;
			int xoffset, yoffset;
			getHomography(cameraMatrixK, rv, pv, nv, distance, cMoRotation, cMoSrcPosition, w, h, offsetH,
				xmin, ymin, xmax, ymax, 
				xoffset, yoffset);

			//imgPts.push_back(Point2i(xmin, ymin));
			imgPts.push_back(Point2i(xoffset, yoffset));

			imgMinX = std::min(xmin, imgMinX);
			imgMinY = std::min(ymin, imgMinY);

			imgMaxX = std::max(xmax, imgMaxX);
			imgMaxY = std::max(ymax, imgMaxY);

			Mat img_perspective;
			warpPerspective(img, img_perspective, offsetH, Size(xmax - xmin, ymax - ymin));

#ifdef _WIN32
			//imshow("img_perspective", img_perspective);
			//waitKey();
#endif

			char key[256];
			snprintf(key, sizeof(key), "%s/out/img-perspective%d.png", imagePath.c_str(), i + 1);
			imwrite(key, img_perspective);
			//! [Perspective]

			//imgPerspectives.push_back(img_perspective);

			// 使用warpPerspective透视变化，根据旋转角度得到旋转矩阵
			auto orthographic = config().getBool("StitchingImage.Orthographic", false);
			if (false && orthographic) {
				//if (orthographic) {
				Mat H;
				int xmin, ymin, xmax, ymax, xoffset, yoffset;
				w = img.cols;
				h = img.rows;

				Vec3d rvC = cMosRotation_camera[i];

				getHomography(cameraMatrixK, rv, Vec3d::zeros(),
					nv, distance,
					rvC,
					Vec3d::zeros(), w, h, H,
					xmin, ymin, xmax, ymax,
					xoffset, yoffset);
				Mat img_perspective_end;
				warpPerspective(img, img_perspective_end, H, Size(xmax - xmin, ymax - ymin));

				snprintf(key, sizeof(key), "%s/out/img-perspective%d-orthographic.png", imagePath.c_str(), i + 1);
				imwrite(key, img_perspective_end);
			}
		}
		
		std::cout << "imgMinX " << imgMinX << " imgMinY " << imgMinY << " imgMaxX " << imgMaxX << " imgMaxY " << imgMaxY << std::endl;

		//return;

		//! [stitch]
		//Mat img_stitch(imgMaxY - imgMinY, imgMaxX - imgMinX, imgFirst.type()); // TODO 最大尺寸
		Mat img_stitch(imgMaxY - imgMinY, imgMaxX - imgMinX, CV_8UC4); // TODO 最大尺寸
		//! [stitch]

		// dummy size
		// rows(1920) cols(1080)
		auto maxW = 1920;
		auto maxH = 1080;
		auto dsize = cv::Size(maxH, maxW);

		auto aspect = (float)img_stitch.cols / (float)img_stitch.rows;
		if (aspect > (float)maxH / (float)maxW) // 更宽
			dsize = cv::Size(maxW, (int)(maxW / aspect));
		else
			dsize = cv::Size((int)(maxH * aspect), maxH);
		// dummy size

		Mat distCoeffs;
		double squareSize = 0.025;
		Mat rvec1, tvec1;
		//drawFrameAxes(img_stitch, cameraMatrixK, distCoeffs, rvec1, tvec1, 2 * squareSize);

		// 整个画布上偏移计算
		auto offsetX = 0;
		auto offsetY = 0;
		auto startOffsetX = -imgMinX;
		auto startOffsetY = -imgMinY;

		for (auto i = 0; i < num; i++) {
			std::cout << "stitch " << (i + 1) << "/" << (num) << std::endl;

			//auto imgPerspective = imgPerspectives[i];
			char key[256];
			snprintf(key, sizeof(key), "%s/out/img-perspective%d.png", imagePath.c_str(), i + 1);
			auto imgPerspective = imread(samples::findFile(key), IMREAD_UNCHANGED);

			auto offsetXFact = imgPts[i].x + startOffsetX;
			auto offsetYFact = imgPts[i].y + startOffsetY;

			Mat mask;
			std::vector<Mat> rgbLayer;
			split(imgPerspective, rgbLayer);

			if (imgPerspective.channels() == 4)
			{
				//split(imgPerspective, rgbLayer);         // seperate channels
				//Mat cs[3] = { rgbLayer[0],rgbLayer[1],rgbLayer[2] };
				//merge(cs, 3, imgPerspective);  // glue together again
				mask = rgbLayer[3];       // png's alpha channel used as mask
			}

			auto slice = img_stitch(Rect(offsetXFact, offsetYFact, imgPerspective.cols, imgPerspective.rows));
			imgPerspective.copyTo(slice, mask);

			auto dummyFlag = config().getBool("StitchingImage.Dummy", false);
			if (dummyFlag) {
				auto dstMat = cv::Mat(dsize, CV_32S);
				cv::resize(img_stitch, dstMat, dsize);

#ifdef _WIN32
				//imshow("Stitch", img_stitch);
				imshow("Stitch", dstMat);
				waitKey(i == num - 1 ? 5 : 0);
#endif
			}
		}

		// 使用warpPerspective透视变化，根据旋转角度得到旋转矩阵
		auto orthographic = config().getBool("StitchingImage.Orthographic", false);
		if (orthographic) {
			Mat H;
			int xmin, ymin, xmax, ymax, xoffset, yoffset;
			// 趋近于最大，则缩放
			auto dstMat = img_stitch;
			if (img_stitch.cols * 2 >= SHRT_MAX || img_stitch.rows * 2 >= SHRT_MAX) {
				dsize = cv::Size(img_stitch.cols / 4, img_stitch.rows / 4);
				dstMat = cv::Mat(dsize, img_stitch.type());
				cv::resize(img_stitch, dstMat, dsize);
			}

			w = dstMat.cols;
			h = dstMat.rows;
			
			get_calibration_matrix_K_for_u3d(cameraMatrixK, w, h, focalLength, sensorSizeX);

			getHomography(cameraMatrixK, cMoRotation, Vec3d::zeros(),
				cMoNormal, cMoDistance,
				cMo_rotation_camera,
				Vec3d::zeros(), w, h, H,
				xmin, ymin, xmax, ymax,
				xoffset, yoffset);

			if (xmax - xmin >= SHRT_MAX || ymax - ymin >= SHRT_MAX) {
				std::cout << "Too MaxSize" << std::endl;
			}

			Mat img_stitch_orthographic;
			warpPerspective(dstMat, img_stitch_orthographic, H, Size(xmax - xmin, ymax - ymin));
			imwrite(imagePath + "/out/img-all-orthographic.png", img_stitch_orthographic);

			auto dummyFlag = config().getBool("StitchingImage.Dummy", false);
			if (dummyFlag) {

				auto aspect = (float)img_stitch_orthographic.cols / (float)img_stitch_orthographic.rows;
				if (aspect > (float)maxH / (float)maxW) // 更宽
					dsize = cv::Size(maxW, (int)(maxW / aspect));
				else
					dsize = cv::Size((int)(maxH * aspect), maxH);

				auto dstMat = cv::Mat(dsize, CV_32S);
				cv::resize(img_stitch_orthographic, dstMat, dsize);

#ifdef _WIN32
				imshow("Stitch", dstMat);
				waitKey();

#endif
			}
		}

		auto dummyFlag = config().getBool("StitchingImage.Dummy", false);
		if (dummyFlag) {
			//cv::resize(img_stitch, img_stitch, dsize);
		}

		imwrite(imagePath + "/out/img-all.png", img_stitch);
#ifdef _WIN32
		//imshow("Stitch", img_stitch);
		waitKey();
#endif
		std::cout << "OK." << std::endl;

	}

	void get_calibration_matrix_K_for_u3d(Mat& K, double w, double h, double focalLength, double sensor_sizeX)
	{
		//var K = Matrix4x4d.Identity;

		// 50
		auto f_in_mm = focalLength;
		auto resolution_x_in_px = w;
		auto resolution_y_in_px = h;

		//// https://docs.unity3d.com/Manual/PhysicalCameras.html
		auto sensor_size_in_mm = sensor_sizeX;
		auto view_fac_in_px = resolution_x_in_px;

		//if (camera.GateFit == Camera.GateFitMode.Horizontal)
		//{
		//	sensor_size_in_mm = camera.SensorSizeX;
		//	view_fac_in_px = resolution_x_in_px;
		//}
		//else if (camera.GateFit == Camera.GateFitMode.Vertical)
		//{
		//	sensor_size_in_mm = camera.SensorSizeY;
		//	view_fac_in_px = resolution_y_in_px;
		//}

		auto pixel_aspect_ratio = 1.f;

		auto pixel_size_mm_per_px = sensor_size_in_mm / f_in_mm / view_fac_in_px;
		auto s_u = 1 / pixel_size_mm_per_px;
		auto s_v = 1 / pixel_size_mm_per_px / pixel_aspect_ratio;

		auto u_0 = resolution_x_in_px / 2;
		auto v_0 = resolution_y_in_px / 2;
		auto skew = 0.f; // only use rectangular pixels

		//K.M11 = s_u;
		//K.M12 = skew;
		//K.M13 = u_0;

		//K.M22 = s_v;
		//K.M23 = v_0;

		//K.M33 = 1f;

		////K = Matrix(
		////((s_u, skew, u_0),
		////(0, s_v, v_0),
		////(0, 0, 1)))

		K.at<double>(0, 0) = s_u;
		K.at<double>(0, 1) = skew;
		K.at<double>(0, 2) = u_0;

		K.at<double>(1, 0) = 0;
		K.at<double>(1, 1) = s_v;
		K.at<double>(1, 2) = v_0;

		K.at<double>(2, 0) = 0;
		K.at<double>(2, 1) = 0;
		K.at<double>(2, 2) = 1;
	}
private:
	bool _helpRequested;
};

POCO_APP_MAIN(StitchingImageApp)