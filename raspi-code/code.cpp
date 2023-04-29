#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>
// #include <Rpi.GPIO>

using namespace std;
using namespace cv;
using namespace raspicam;

// Image processing variables
Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate;
Mat ROILane;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result, distance_stop;

RaspiCam_Cv Camera;

stringstream ss;


vector<int> histrogramLane;

Point2f Source[] = {Point2f(30,200), Point2f(335,200), Point2f(0,230), Point2f(360,230)};
Point2f Destination[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};

//Machine Learning variables
CascadeClassifier Stop_Cascade;
Mat frame_Stop, RoI_Stop, gray_Stop;
vector<Rect> Stop;

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
{
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,75 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,80 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,40 ) );
    Camera.set ( CAP_PROP_GAMMA,  ( "-g",argc,argv ,40 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,50));

}

void Capture()
{
    Camera.grab();
    Camera.retrieve(frame);
    cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
    cvtColor(frame, frame, COLOR_BGR2RGB);
}

void Perspective()
{
    line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
    line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
    line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
    line(frame,Source[2], Source[0], Scalar(0,0,255), 2);
	
    Matrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame, framePers, Matrix, Size(400,240));
}

void Threshold()
{
    int x = 140;
    cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
    inRange(frameGray, x, 255, frameThresh);			// 200, 255 originally
    Canny(frameGray,frameEdge, 900, 900, 3, false);
    add(frameThresh, frameEdge, frameFinal);
    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
    cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
	
}

void Histrogram()
{
    histrogramLane.resize(400);
    histrogramLane.clear();
    
    for(int i=0; i<400; i++)       //frame.size().width = 400
    {
	ROILane = frameFinalDuplicate(Rect(i,140,1,100));
	divide(255, ROILane, ROILane);
	histrogramLane.push_back((int)(sum(ROILane)[0])); 
    }
}

void LaneFinder() //Finds the lane and lane mid
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr); 
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin() +250, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
    
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2); 
}

void LaneCenter() // right gives -ve //left +ve
{
    laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    frameCenter = 184;
    
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);

    Result = laneCenter-frameCenter;
}

//void Stop_detection() {
    //if (!Stop_Cascade.load("/home/pi/Desktop/Stop_cascade.xml")) {
	//printf("Unable to open stop cascade file.");
    //}
    //RoI_Stop = frame_Stop(Rect(0,0,400,240));
    //cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    //equalizeHist(gray_Stop, gray_Stop);
    //Stop_Cascade.detectMultiScale(gray_Stop, Stop);
    
    //for(int i =0; i<Stop.size(); i++) {
	//Point P1(Stop[i].x, Stop[i].y);
	//Point P2(Stop[i].x + Stop[i].width, Stop[i].x + Stop[i].height);
	
	//rectangle(RoI_Stop, P1, P2, Scalar(0,0,255), 2);
	//putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255, 255), 2);
    //}
//}

void Stop_detection(){
    if(!Stop_Cascade.load("/home/pi/Desktop/Stop_cascade.xml"))
    { 
        cout<<"unable to open the cascade fil"<<endl;
    }
    RoI_Stop = frame_Stop(Rect(200,0,200,240));
    cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(gray_Stop, gray_Stop);
    Stop_Cascade.detectMultiScale(gray_Stop,Stop);
    for(int i=0;i<  Stop.size() ;i++)

    {
        Point P1(Stop[i].x, Stop[i].y);
        Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);

        rectangle(RoI_Stop, P1, P2, Scalar(192,192,192), 1);
        putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_COMPLEX_SMALL, 0.6, Scalar(192,192,192), 1);

    distance_stop = (-0.9523)*(P2.x-P1.x)+107.2317;
    ss.str(" ");
    ss.clear();
    //ss<<"Distance  = "<<P2.x-P1.x<<"(Pixels)";
    ss<<"Distance = "<<distance_stop<<"cm";

    putText(RoI_Stop,ss.str(),Point2f(1,230), 0,0.55 ,Scalar(192,192,192),1.6);

    }
}

int main(int argc,char **argv)
{
     wiringPiSetup();
     pinMode(21, OUTPUT);
     pinMode(22, OUTPUT);
     pinMode(23, OUTPUT);
     pinMode(24, OUTPUT);
     Setup(argc, argv, Camera);                                                                                                                                                                                                                                                                                                                                                                               
     cout<<"Connecting to camera"<<endl;
     if (!Camera.open())
     {
	cout<<"Failed to Connect"<<endl;
     }
     
     cout<<"Camera Id = "<<Camera.getId()<<endl;
     
     while(1)
    {
	auto start = std::chrono::system_clock::now();

    Capture();//to get the vid
    Perspective();//set the perspective view
    Threshold();//get out the white lines
    Histrogram();//detecting the white lines
    LaneFinder();//
    LaneCenter();
    Stop_detection();
    
    if (Result == 0)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 0
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Forward"<<endl;
    }
    
    else if (Result >0 && Result <10)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 1
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right1"<<endl;
    }
    
    else if (Result >=10 && Result <20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 2
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right2"<<endl;
    }
    
    else if (Result >20) 
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 3
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right3"<<endl;
    }
    
    else if (Result <0 && Result >-10)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 4
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left1"<<endl;
    }
    
    else if (Result <=-10 && Result >-20)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 5
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left2"<<endl;
    }
    
    else if (Result <-20)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 6
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left3"<<endl;
    }
    else if (Result == -59) 
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 7
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Stop"<<endl;
    }
    
    ss.str(" ");
    ss.clear();
    ss<<"Result = "<<Result;
    putText(frame, ss.str(), Point2f(1,40), 0,1, Scalar(0,0,255), 2);
    
    namedWindow("orignal", WINDOW_KEEPRATIO);
    moveWindow("orignal", 0, 100);
    resizeWindow("orignal", 300, 250);
    imshow("orignal", frame);
    
    namedWindow("Perspective", WINDOW_KEEPRATIO);
    moveWindow("Perspective", 300, 100);
    resizeWindow("Perspective", 300, 250);
    imshow("Perspective", framePers);
    
    namedWindow("Final", WINDOW_KEEPRATIO);
    moveWindow("Final", 600, 100);
    resizeWindow("Final", 300 , 250);
    imshow("Final", frameFinal);
    
    namedWindow("Stop Sign", WINDOW_KEEPRATIO);
    moveWindow("Stop Sign", 900, 100);
    resizeWindow("Stop Sign", 300, 250);
    imshow("Stop Sign", RoI_Stop);
    
    waitKey(1);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;
    
    }
return 0;     
}
