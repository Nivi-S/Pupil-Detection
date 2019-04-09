//////////////////////////
// 
// This project is the main driver which incorporates all the libraries written previously to control following components: 
//		MLX90615 thermometer
//		Maxim MAX30102 sensor Heart Rate Sensor
//		Lp55231 RGB LEDs
//		CFAF128128B1-0145T SPI Display
//
// Previous update in this version is creating multi threaded processes for following process that work simultaneously:
//		countdown on display
//		strobing led with camera
//		capturing camera frames
//		taking sensor measurements (temp, ir and respiratory)
// 
// Recent Update: getting rid of I2C interference error by creatinf a bool var I2C_BUSY
// (better implementation would be creating a mutex lock I2CBUSY )
//
// Build using sudo g++    ////-lwiringPi -I PATH main.cpp lp55231.cpp -o test
//////////////////////////

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "lp55231.h"
#include "mlx90615.h"
#include "cfaf128128b1.h"
#include "max30102.h"
#include "hr_spo2_algorithm.h"
#include "tlv320aic3110.h"
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <thread>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>



#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/background_segm.hpp>

#include <iostream>
#include <fstream>
#include <iostream> // for standard I/O
//#include <dirent.h>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <time.h>
#include <sys/timeb.h>


using namespace cv;
using namespace std;


//global vars:
double tempfinal;
float spo2final;
int hrfinalLB;
int hrfinalHB;
Mat src, src_gray;
Mat dst, detected_edges;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratior = 3;
int kernel_size = 3;

static bool I2C_BUSY = false;


void blinkYellow() {
	Lp55231 testLED;
	testLED.Begin();
	//testLED.Enable();
	int i = 0;
	int del; 
	while (i<5) {
		testLED.Enable();
		cout << "blinking YellowLED" << endl;
		testLED.SetDriveCurrent(2, 255);
		testLED.SetDriveCurrent(3, 255);
		testLED.SetDriveCurrent(7, 255);
		testLED.SetChannelPWM(2, 200);		//G   //yellow
		testLED.SetChannelPWM(3, 0);		//B
		testLED.SetChannelPWM(7, 255);		//R
		del = millis();
		while ((del + 500) > millis());
		//delay(500);
		testLED.SetDriveCurrent(2, 0);
		testLED.SetDriveCurrent(3, 0);
		testLED.SetDriveCurrent(7, 0);
		del = millis();
		while ((del + 500) > millis());
		//delay(500);
		i++;
	}
}

void GreenLEDon() {
	Lp55231 testLED;
	testLED.Begin();
	testLED.Enable();
	cout << "Test LED" << endl;
	cout << "TestLED green" << endl;
	testLED.SetChannelPWM(2, 255);	//G	//green
	testLED.SetChannelPWM(3, 0);
	testLED.SetChannelPWM(7, 0);
}

void readSensor() {
	wiringPiSetup();
	bool flag = true;
	Mlx90615 mlx90615;
	flag = maxim_max30102_init();
	mlx90615.init();
	if (flag) cout << "Sensor init success" << endl;
	else cout << "Sensor init failed" << endl;
	int32_t hr;
	float spo2;
	bool proximity;
	int j = 0;
	while (j<5) {
		cout << " Object temp: " << mlx90615.getObjectTemp(&I2C_BUSY) << endl;
		read_hr_spo2(&hr, &spo2, &proximity);// &I2C_BUSY);
		cout << "HR: " << hr << ", SpO2: " << spo2 << endl;
		j++;
	}
	//cout << "sensor measurement complete" << endl;
	//read_hr_spo2(&hr, &spo2final, &proximity);// &I2C_BUSY);
	//taking last measurement only
	spo2final = spo2;
	hrfinalHB = hr;
	tempfinal = mlx90615.getObjectTemp(&I2C_BUSY);
	
}

void readTemp() {
	// enable MLX90615 thermometer
	Mlx90615 mlx90615;
	cout << "MLX90615 Init: " << mlx90615.init() << endl;
	int k = 0;
	while (k<10) {
		//cout << "Ambient: " << mlx90615.getAmbTemp() << ", Object: " << mlx90615.getObjectTemp() << endl;
		cout << " Object temp: " << mlx90615.getObjectTemp(&I2C_BUSY) << endl;
		k++;
	}
	//tempfinal = mlx90615.getObjectTemp();
}

void Audio() {
	CodecBegin();
	CodecInit();
	int i = 0;
	//while (i < 1) {
		//CodecBeep();
		//delay(1000);
		//SystemBeep("music_tone.sh");
		SystemRecord("audio_test", 10);
		//delay(1000);
		//i++;
	//}
}

int ImgProc(Mat frame1, int count) {
	src = frame1;
	if (!src.data)
	{
		return -1;
	}
	/// Create a matrix of the same type and size as src (for dst)
	dst.create(src.size(), src.type());
	/// Convert the image to grayscale
	cvtColor(src, src_gray, CV_BGR2GRAY);
	/// Create a window
	namedWindow("edge", CV_WINDOW_AUTOSIZE);
	/// Create a Trackbar for user to enter threshold
	//createTrackbar("Min Threshold:", "edge", &lowThreshold, max_lowThreshold, CannyThreshold);
	/// Show the image
	/// Reduce noise with a kernel 3x3
	blur(src_gray, detected_edges, Size(3, 3));
	/// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratior, kernel_size);
	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);
	src.copyTo(dst, detected_edges);
	imshow("edge", dst);
	char x;
	x = waitKey(20);
	char name[10];
	sprintf(name, "test %d .png", count);
	if ((imwrite(name, dst)) == false) cout << "couldn't save image\n";
	return 0;

}

void countdown(int n) {
	int timer;
	Fill_LCD(0x00, 0x00, 0x00);
	int whitepulse = millis();	
	while (n > 0) {
		char meas[5];
		timer = millis();
		sprintf(meas, "%d  ", n);
		Display_Bitmap("countback.bmp");
		//Fill_LCD(0x00, 0x00, 0x00);
		//LCD_Circle(60, 60, 40, 0xEB, 0x87, 0x30);
		DrawString(40, 30, meas, 255, 128, 0, 5);
		while ((timer + 1000) > millis());
		n--;
	}
}

void Camera1() {
	cout<<"Starting RaspiVid 1 Capture"<<endl;
//	system("v4l2-ctl -d /dev/video1 -c rotate=270");
	system("raspivid -cs 1 -t 15000 -w 640 -h 480 -fps 60 -awb off -awbg '1.0,1.0' -ex night -o video1.h264");
	system("MP4Box -add video1.h264 video_out1.mp4 -fps 60");
	cout<<"Finished Video1"<<endl;

}

void Camera0() {	
	unsigned int total_time = millis();
	unsigned int frames=0;
//	system("v4l2-ctl -d /dev/video0 -c rotate=270");
	cout<<"Starting RaspiVid 0 Capture"<<endl;
//	system("raspivid -cs 0 -t 18000 -w 640 -h 480 -ex off -ss 10000 -ISO 800 -awb off -awbg '1.0,1.0' -fps 60 -rot 270 -o video0.h264");
	system("raspivid -cs 0 -t 18000 -w 640 -h 480 -ex off -ss 10000 -fli 60hz -awb off -awbg '1.0,1.0' -ag 16 -fps 60 -rot 270 -o video0.h264");
	system("MP4Box -add video0.h264 video_out.mp4 -fps 60");
	cout<<"Finished Video0"<<endl;
//Forget OpenCV, Record with GPU

}

/*void* strobe_led(void* state) {
	cout << "initiating strobe_led" << endl;
	wiringPiSetup();
//	pinMode(41, OUTPUT);
//	int t_frame = 10000;
//	int t_pause =  6600;
	Lp55231 led;
	led.Begin();
	led.Enable();
	led.SetDriveCurrent(0, 102);
	led.SetDriveCurrent(4, 102);

	int end_time = millis()+18000;
	volatile int strobe_end_time;
	volatile int strobe_start_time;
	led.SetChannelPWM(0,0);
	led.SetChannelPWM(4,0);
	while(digitalRead(4));
	while(millis()<end_time){
		while(!digitalRead(4)); //Wait for falling edge
		strobe_start_time = micros() + 8100; //8.1 mS
		strobe_end_time = strobe_start_time+2300;
		//digitalWrite(41,LOW);
		while(micros()<strobe_start_time);
	//	delayMicroseconds(7900);
		led.SetChannelPWM(0,255);
		led.SetChannelPWM(4,255);
		while(micros()<strobe_end_time);
	//	delayMicroseconds(2300);
		led.SetChannelPWM(0,0);
		led.SetChannelPWM(4,0);
		//digitalWrite(41,HIGH);
	}
	led.SetChannelPWM(0,0);
	led.SetChannelPWM(4,0);

//	while(digitalRead(4)); //Wait for first pulse
	int a_time  = micros();
	bool led_state = true;
//	led.SetChannelPWM(0, 255);
//	led.SetChannelPWM(4, 255);
//	while (true) {
		if(millis()>(init_time+18000)) break;
		if(led_state && (micros()>(t_frame+a_time))){
		led.SetChannelPWM(0, 0);
		led.SetChannelPWM(4, 0);
		led_state = false;
		a_time = micros();
		}
		if(!led_state && (micros()>(t_pause+a_time))){
		led.SetChannelPWM(0, 255);
		led.SetChannelPWM(4, 255);
		led_state = true;
		a_time = micros();
		}


//		if (digitalRead(4)) {
//			led.SetChannelPWM(0, 255);
//			led.SetChannelPWM(4, 255);
//		}
//		else {
//			led.SetChannelPWM(0, 0);
///			led.SetChannelPWM(4, 0);
//		}
//	}
}*/

void* audio(void* null) {
	time_t auds = time(0);
	char* aud = ctime(&auds);
	cout << "initiating audio at " << aud << endl;
	Audio();
}

void* hr_sensor(void* null) {
	//sensors (temp, ir and respiratory) 
	time_t irs = time(0);
	char* ir = ctime(&irs);
	cout << "initiating IR sensor at " << ir << endl;
	readSensor();
}

void* white_pulse(void* null) {
	int whitepulse = millis();
		Lp55231 testLEDwhite(0x33);
		testLEDwhite.Begin();

	while ((whitepulse + 6000) > millis()); 

		testLEDwhite.Enable();
		testLEDwhite.SetDriveCurrent(2, 255);
		testLEDwhite.SetDriveCurrent(3, 255);
		testLEDwhite.SetDriveCurrent(7, 255);
		testLEDwhite.SetChannelPWM(2, 25);		//G   //white
		testLEDwhite.SetChannelPWM(3, 25);		//B
		testLEDwhite.SetChannelPWM(7, 75);
		whitepulse = millis();
		while ((whitepulse + 165) > millis());
		testLEDwhite.SetDriveCurrent(2, 0);
		testLEDwhite.SetDriveCurrent(3, 0);
		testLEDwhite.SetDriveCurrent(7, 0);
		testLEDwhite.Disable();
}

/*static volatile int strobe_end_time;
void strobe_led_ISR(void){
	digitalWrite(41,LOW);
	strobe_end_time = micros() + 11900;  //11.9mS
	while(micros()<strobe_end_time);
	digitalWrite(41,HIGH);
	//NOTE: LED MUST BE MANUALLY TURNED OFF AT THE END OF TEST!!
}*/

// Part one: splitting video
void split() {
	//Playing video from file :
	cap = VideoCapture('video_out.mp4')

	int currentFrame = 1;
	while (currentFrame<600) {
		//Capture frame - by - frame
		Mat frame = read(cap);
		//ret, frame = cap.read()
		//Saves image of the current frame in jpg file
		char name[20];
		sprintf(name, "frame %d .png",currentFrame); 
		//cout << "creating " + name << endl;
		//print('Creating...' + name)
		imwrite(name, frame);

		// To stop duplicate images
		currentFrame++;
	}
	waitKey(100);
}

//=========================================================
// Part two: Processing and saving individual frames
void imgprocess() {
	int frame_counter0 = 10;
	while (frame_counter0 < 600) {
		//frame_name = "frame{}.jpg".format(frame_counter)
		char frame_name[20];
		sprintf(frame_name, "frame %d .png", frame_counter0);
		//= "frame" + frame_counter0 + ".png";
		Mat image = imread(frame_name, IMREAD_COLOR);
		image(Rect(Point(90, 450), Point(90, 450))));
		//frame = image[90:450, 90 : 450]
		Mat gray;
		cvtColor(image, gray, COLOR_BGR2GRAY);
		Mat thresholded, black, white;
		threshold(gray, thresholded, 0, 250, THRESH_BINARY_INV);
		threshold(gray, black, 0, 0, THRESH_BINARY_INV);
		threshold(gray, white, 0, 0, THRESH_BINARY_INV);
		//retval, thresholded = cv2.threshold(gray, 0, 250, cv2.THRESH_BINARY_INV)
		//rretval, black = cv2.threshold(gray, 0, 0, cv2.THRESH_BINARY_INV)
		//rretval, white = cv2.threshold(gray, 250, 250, cv2.THRESH_BINARY_INV)
		Mat edges;
		Canny(thresholded, rdges, 200, 500);
		//edges = cv2.Canny(thresholded, 200, 500)

		//convert grayscale to color image
		Mat color;
		cvtColor(gray, color, COLOR_GRAY2RGB);

		vector<Vec3f> circles;
		HoughCircles(edges, circles, HOUGH_GRADIENT, 1, 500, 50, 12 60, 140);
		for (size_t i = 0; i < circles.size(); i++)
		{
			Vec3i c = circles[i];
			Point center = Point(c[0], c[1]);
			// circle center
			circle(color, center, 2, Scalar(0, 0, 255), 3, LINE_AA);
			// circle outline
			int radius = c[2];
			circle(color, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
			circle(black, center, 120, Scalar(255, 255, 255), -1, cv2.LINE_AA);
			circle(white, center, 150, Scalar(0, 0, 0), -1, cv2.LINE_AA);
		}


		bitwise_and(thresholded, black, thresholded);
		int Sum = 0;
		int xsum = 0;
		int ysum = 0;
		int xo, yo;
		//rows, cols = thresholded.shape
			for (int i = 0, i <= 360; i++) {
				for (int j = 0; j <= 360; j++) {
					pixel = thresholded.at(i, j);
					//print(pixel)
						if (pixel > 0) {
							Sum += 1;
							ysum = i + ysum;
							xsum = j + xsum;
							xsum = xsum + Sum;
							ysum = ysum + Sum;
							xo = xsum / Sum;
							yo = ysum / Sum;
							R = math.sqrt(Sum / 3.14);
							//print(R);
						}
				}
			}
		xo += 90;
		yo += 90;
		circle(image, (xo, yo), R, (0, 255, 0), 3);
		circle(image, (xo, yo), 2, (0, 255, 0), 3);

		char out_name[10];
		sprintf(out_name, "image_frame_ %d .png", frame_counter0);
		//= "image_frame_" + frame_counter0 + ".png";
		imwrite(out_name, image);
		frame_counter0 += 1;
	}
	waitKey(100);
}

//=============================
//Part three: combining frames to video
//ap = argparse.ArgumentParser()
//ap.add_argument("-o", "--output", required = False, default = 'fin_output.mp4', help = "output video file")
//args = vars(ap.parse_args())

void combineframe() {
	string output = "fin_output.mp4";

	int frame_counter = 10;
	schar frame_namei[10];
	sprintf(frame_namei, "image_frame_ %d .png", frame_counter);
	Mat frame1 = imread(frame_namei);
	//height, width, channels = frame1.shape

	// Define the codec and create VideoWriter object
	int fourcc = VideoWriter_fourcc(*'mp4v');//Be sure to use lower case
	out = VideoWriter(output, fourcc, 20.0, (640, 480));

	while (frame_counter <600) {
		char frame_namec[10];
		sprintf(frame_namec, "image_frame_ %d .png", frame_counter);
		//string frame_namec = "image_frame_" + frame_counter + ".png";
		Mat frame2 = imread(frame_namec);
		write(frame2); //Write out frame to video
					   //imshow('video', frame);
		frame_counter += 1;
	}
	waitKey(100);
	cout << "The output video is " + output << endl;
}


int main(void)
{
	wiringPiSetup();
//	wiringPiISR(4, INT_EDGE_FALLING, strobe_led_ISR);
//	pinMode(41,OUTPUT);
	// enable MLX90615 thermometer
	Mlx90615 mlx90615;
	cout << "MLX90615 Init: " << mlx90615.init() << endl;

	int run = 0;
	while (true)
	{
		//if ON switch is triggerred
		//if Errors : L1
		//Display Error msg
		//RESETON: 
		//else:
		//device turs ON (Upon Restart here)

		//if device ON :
		//Standby mode: Yellow LED turns ON Computer turns ON Pulse Meter turns ON IR sensor turns ON Display turns ON
		//if Errors: goto L1
		//goto RESETON;

		//else:
		//else {
		//Display welcome screen
		RESETON:
		destroyAllWindows();
		cout << "LCD init():" << LCD_init() << endl;
		cout << "Welcome Screen" << endl;
		//}
		
		//if pulse meter && IR no signal detected within 15s 
		//break; goto L2
		cout << "waiting for signal" << endl;
		delay(500);
		//while ((mlx90615.getObjectTemp() < 30)) 
		if (mlx90615.getObjectTemp(&I2C_BUSY) < 30) {
			cout << "no temp reading" << endl;
			blinkYellow();
			Display_Bitmap("logoblack.bmp");
			//break;
			goto RESETON;
		}

		//else:Blink YellowLED //device read for measurement 
		if (mlx90615.getObjectTemp(&I2C_BUSY) > 30)
			cout << "temp reading received" << endl;

		//if no signal: dsplay adjust device goto L2 
		/*if ((mlx90615.getObjectTemp() == NULL))
		cout << "no temp reading" << endl;SS
		goto ADJUST;*/

		//else: Camera turns ON Display turns ON LEDs turn ON Audio turns ON
		cout << "Turning ON camera, Display, LED and Audio " << endl;
		//Draw a white circle
		//LCD_Circle(64, 64, 20, 0xFF, 0xFF, 0xFF);


		//if Camera, LED, Display, Audio not turned on correcty
		//break; goto L1
		//goto RESETON;

		//else:
		//if eye alignment && lighting is not good 
		//break; goto L2
		//if eyes closed detected
		//blink RED or sound alarm
		//else 
		//greenLED ON 
		//device ready for measurement
		//else    green circle
		//LCD_Circle(21, 64, 20, 0x00, 0xFF, 0x00);
		//delay(5000);

		char start[10] = {'S','t','a','r','t', ' '};
		Display_Bitmap("countback.bmp");
		DrawString(64, 64, start, 255, 128, 0, 2);
		//Begin Measurement:

		//do 30 sec countdown
		//thread 1 - countdown display
		//pthread_t countdown_thread;
		//pthread_create(&countdown_thread, NULL, countdown, NULL);


		//Simultaneously work camera data, temperature, HR and SPO2 sensors, and audio
		//timestamp using linux time

		//thread 2 - strobing led with camera
//		pthread_t strobe_led_thread;
//		pthread_create(&strobe_led_thread, NULL, strobe_led, NULL);
		//ir led on all time
		Lp55231 strobeLED;
		strobeLED.Begin();
		strobeLED.Enable();
		strobeLED.SetDriveCurrent(4,51);
		strobeLED.SetDriveCurrent(0,51);
		strobeLED.SetChannelPWM(0,255);
		strobeLED.SetChannelPWM(4,255);

		//thread 3 - camera capture frames
		thread camera1_frames_thread(Camera0);
		thread camera0_frames_thread(Camera1); //Second Thread
		//thread_create(&camera_frames_thread, NULL, camera_frames, NULL);

		//thread 4 - audio
		pthread_t audio_thread;
		pthread_create(&audio_thread, NULL, audio, NULL);

		//thread 5 - heart rate sensor
		pthread_t hr_sensor_thread;
		pthread_create(&hr_sensor_thread, NULL, hr_sensor, NULL);
		   
		pthread_t white_pulse_thread;
		pthread_create(&white_pulse_thread, NULL, white_pulse, NULL);  

		//countdown
		countdown(15);

		//time_t temps = time(0);
		//char* temp = ctime(&temps);
		//cout << "initiating temp sensor at " << temp << endl;
		//readTemp();
		strobeLED.SetChannelPWM(0,0);
		strobeLED.SetChannelPWM(4,0);
		strobeLED.Disable();
		void *status;
		//join threads
		camera1_frames_thread.join();
		camera0_frames_thread.join();  //Second Camera Thread
		pthread_join(hr_sensor_thread, &status);
		pthread_join(audio_thread, &status);
		destroyAllWindows();

		//Tests Done, Results Obtained
		Fill_LCD(0x00, 0x00, 0x00);
		cout << "Measurement Complete" << endl;
		char comp[10] = { 'D', 'o','n','e', ' ' };
		DrawString(64, 64, comp, 255, 128, 0, 2);
		int wait = millis();
		while ((wait + 500) > millis());

		Fill_LCD(0xEB, 0x87, 0x30);
		cout << "Processing" << endl;
		char comp1[11] = { 'P','r','o','c','e','s','s','i','n','g', ' ' };
		DrawString(10, 60, comp1, 255, 255, 255, 2);
		int wait1 = millis();
		while ((wait1 + 1000) > millis());

		//Display results on Screen
		Display_Bitmap("newicons.bmp");//24bit 128x128

		char tempreading[7];
		char heartreading[3];
		char sporeading[5];
		sprintf(tempreading, " %.3f", tempfinal);
		DrawString(30, 85, tempreading, 255, 128, 0, 2);
		char errmsg[4] = { 'E','R','R',' ' };
		if (hrfinalHB < 0) DrawString(50, 48, errmsg, 255, 128, 0, 2);
		if (hrfinalHB > 0){
			sprintf(heartreading, "%d  ", hrfinalHB);
			DrawString(50, 48, heartreading, 255, 128, 0, 2);
		}

		if (spo2final < 0) DrawString(50, 15, errmsg, 255, 128, 0, 2);
		if (spo2final > 0){
			sprintf(sporeading, "%.3f", spo2final);
			DrawString(50, 15, sporeading, 255, 128, 0, 2);
		}

		cout << "split video" << endl;
		split();
		cout << "img process" << endl;
		imgprocess();
		cout << "combine frames" << endl;
		combineframe();

		Display_Bitmap("good.bmp");
		//delay(5000);

		//ADJUST:
		//Draw a red circle
		//LCD_Circle(107, 64, 20, 0xFF, 0x00, 0x00);
		//cout << "Adjust Device" << endl;
		//run++;

		//waitKey();
		delay(5000);
		//goto L1;

	}
	return 0;
}
