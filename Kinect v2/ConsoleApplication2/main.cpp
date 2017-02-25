#include <iostream>
#include <sstream>
#include <windows.h>

#include "NtKinect.h"

using namespace std;

void doJob() {
	NtKinect kinect;
	//cv::Mat image;
	int x[6], y[6];
	float z[6] = { 1, 1, 1, 1, 1, 1 };
	bool c = false, a = false, b = false;//aは固定する時の状態(固定の時true)
										 //bはチョキ出す時の状態(出す時true)
										 //cは球体の状態(移動する時true)
	cv::Scalar colors[] = {
		cv::Scalar(255,0,0),  // HandState_Unknown
		cv::Scalar(0,255,0),  // HandState_NotTracked
		cv::Scalar(255,255,0), // HandState_Open
		cv::Scalar(255,0,255), // HandState_Closed
		cv::Scalar(0,255,255),  // HandState_Lasso
	};
	kinect.setRGB();
	//image = cv::Mat(kinect.rgbImage.cols, kinect.rgbImage.rows, CV_8UC3);
	while (1) {
		kinect.setRGB();
		kinect.setSkeleton();
		for (int i = 0; i < kinect.skeleton.size(); i++) {
			auto person = kinect.skeleton[i];
			for (int j = 0; j < person.size(); j++) {
				Joint joint = person[j];
				if (joint.TrackingState == TrackingState_NotTracked) continue;
				ColorSpacePoint cp;
				kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position, &cp);
				//cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X - 5, (int)cp.Y - 5, 10, 10), cv::Scalar(0, 0, 255), 2);
				if (/*j == JointType_HandLeft ||*/ j == JointType_HandRight) {
					pair<int, int> handState = kinect.handState(i, j == JointType_HandLeft);
					if (!c && !b && handState.first == HandState_Lasso) { a = !a; b = true; }
					else if (b && handState.first != HandState_Lasso)b = false;
					if(a)
						cv::circle(kinect.rgbImage, cv::Point(x[i], y[i]), 100 * (1 / z[i]), colors[handState.first], -1);
					else
						switch (handState.first) {
						case HandState_Closed:
							cv::circle(kinect.rgbImage, cv::Point((int)cp.X, (int)cp.Y), 100 * (1 / joint.Position.Z), colors[handState.first], -1);
							x[i] = (int)cp.X;
							y[i] = (int)cp.Y;
							z[i] = joint.Position.Z;
							c = true;
							break;
						case HandState_Open:
							cv::circle(kinect.rgbImage, cv::Point(x[i], y[i]), 100 * (1 / z[i]), colors[handState.first], -1);
							c = false;
							break;
						default:
							if (c) {
								cv::circle(kinect.rgbImage, cv::Point((int)cp.X, (int)cp.Y), 100 * (1 / joint.Position.Z), colors[handState.first], -1);
								x[i] = (int)cp.X;
								y[i] = (int)cp.Y;
								z[i] = joint.Position.Z;
							}
							else {
								cv::circle(kinect.rgbImage, cv::Point(x[i], y[i]), 100 * (1 / z[i]), colors[handState.first], -1);
							}
							break;
						}
				}
			}
		}
		//cv::imshow("rgb2", image);
		cv::resize(kinect.rgbImage, kinect.rgbImage, cv::Size(kinect.rgbImage.cols / 2, kinect.rgbImage.rows / 2), 0, 0);
		cv::imshow("rgb", kinect.rgbImage);
		auto key = cv::waitKey(1);
		if (key == 'q') break;
	}
	cv::destroyAllWindows();
}

int main(int argc, char** argv) {
	try {
		doJob();
	}
	catch (exception &ex) {
		cout << ex.what() << endl;
		string s;
		cin >> s;
	}
	return 0;
}
