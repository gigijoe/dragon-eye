// Standard include files
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudaobjdetect.hpp>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>

using namespace cv;
using namespace std;

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

extern "C" {
#include "jetsonGPIO/jetsonGPIO.h"
}
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/*
*
*/

static int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tcgetattr\n", errno);
            return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            printf ("error %d from tcsetattr\n", errno);
            return -1;
    }
    return 0;
}

static void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tggetattr\n", errno);
            return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes\n", errno);
}

#define BASE_B

static void base_toggle(int fd, uint64_t frameNo) 
{
    static uint8_t serNo = 0x3f;
    static uint64_t lastFrameNo = 0;
    uint8_t data[1];

    if(!fd)
        return;

    if(frameNo - lastFrameNo > 6) { /* Not been triggled for over 6 frames is the same triggle. Keep serNo steady. */
        if(++serNo > 0x3f)
            serNo = 0;
    }
    lastFrameNo = frameNo;

#ifdef BASE_A
    data[0] = (serNo & 0x3f);
    printf("BASE_A[%d]\r\n", serNo);
#endif    
#ifdef BASE_B
    data[0] = (serNo & 0x3f) | 0x40;
    printf("BASE_B[%d]\r\n", serNo);
#endif
    write(fd, data, 1);
}

/*
*
*/

static inline bool ContoursSort(vector<cv::Point> contour1, vector<cv::Point> contour2)  
{  
    //return (contour1.size() > contour2.size()); /* Outline length */
    return (cv::contourArea(contour1) > cv::contourArea(contour2)); /* Area */
}  

inline void writeText( Mat & mat, const string text )
{
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 1;
    int thickness = 1;  
    Point textOrg( 10, 10 );
    putText( mat, text, textOrg, fontFace, fontScale, Scalar(0, 255, 0), thickness, 8 );
}

class Target
{
protected:
    double m_arcLength;
    unsigned long m_frameTick;

public:
    Target(Rect & roi, unsigned long frameTick) : m_arcLength(0), m_frameTick(frameTick) {
        m_rects.push_back(roi);
        m_points.push_back(roi.tl());
        m_frameTick = frameTick;
    }

    vector< Rect > m_rects;
    vector< Point > m_points;
    Point m_velocity;

    void Update(Rect & roi, unsigned long frameTick) {
        if(m_rects.size() > 0)
            m_arcLength += norm(roi.tl() - m_rects.back().tl());
        if(m_points.size() > 1) {
            m_velocity.x = (roi.tl().x - m_rects.back().tl().x);
            m_velocity.y = (roi.tl().y - m_rects.back().tl().y);
        }
        m_rects.push_back(roi);
        m_points.push_back(roi.tl());
        m_frameTick = frameTick;
    }

    inline double ArcLength() { return m_arcLength; }
    inline unsigned long FrameTick() { return m_frameTick; }
    inline Rect & LatestRect() { return m_rects.back(); }
    inline Point & LatestPoint() { return m_points.back(); }
};

#define MAX_NUM_FRAME_MISSING_TARGET 30

class Tracker
{
private:
    unsigned long m_frameTick;

public:
    Tracker() : m_frameTick(0) {}

    list< Target > m_targets;

    void Update(vector< Rect > & roiRect) {
        for(list< Target >::iterator t=m_targets.begin();t!=m_targets.end();) {
            int i;
            for(i=0; i<roiRect.size(); i++) {
                Rect r = t->m_rects.back();
                if((r & roiRect[i]).area() > 0) /* Target tracked ... */
                    break;                
                r.x += t->m_velocity.x;
                r.y += t->m_velocity.y;
                if((r & roiRect[i]).area() > 0) /* Target tracked with velocity ... */
                    break;
            }
            if(i == roiRect.size()) { /* Target missing ... */
                if(m_frameTick - t->FrameTick() > MAX_NUM_FRAME_MISSING_TARGET) { /* Target still missing for over 6 frames */
#ifdef DEBUG            
Point p = t->m_points.back();
printf("lost target : %d, %d\n", p.x, p.y);
#endif
                    t = m_targets.erase(t); /* Remove tracing target */
                    continue;
                }
            }
            t++;
        }

        for(int i=0; i<roiRect.size(); i++) {
            list< Target >::iterator t;
            for(t=m_targets.begin();t!=m_targets.end();) {
                Rect r = t->m_rects.back();
                if((r & roiRect[i]).area() > 0) { /* Next step tracked ... */
                    t->Update(roiRect[i], m_frameTick);
                    break;
                }
                r.x += t->m_velocity.x;
                r.y += t->m_velocity.y;
                if((r & roiRect[i]).area() > 0) { /* Next step tracked with velocity ... */
                    t->Update(roiRect[i], m_frameTick);
                    break;
                }
                t++;
            }
            if(t == m_targets.end()) { /* New target */
                m_targets.push_back(Target(roiRect[i], m_frameTick));
#ifdef DEBUG            
printf("new target : %d, %d\n", roiRect[i].tl().x, roiRect[i].tl().y);
#endif
            }
        }
        m_frameTick++;
    }

    Target *PrimaryTarget() {
        double trackingArcLength = 0;
        list< Target >::iterator it = m_targets.end();
        for(list< Target >::iterator t=m_targets.begin();t!=m_targets.end();t++) {
            if(t->ArcLength() > trackingArcLength) {
                trackingArcLength = t->ArcLength();
                it = t;
#ifdef DEBUG            
Point p = t->m_points.back();
printf("primary target : %d, %d\n", p.x, p.y);
#endif
            }
        }
        if(it != m_targets.end()) {
            return &(*it);
        }
        return 0;
    }
};

#define JETSON_NANO

#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 720
#define CAMERA_FPS 60

#define MIN_TARGET_WIDTH 16
#define MIN_TARGET_HEIGHT 16
#define MAX_TARGET_WIDTH 320
#define MAX_TARGET_HEIGHT 240

//#define VIDEO_INPUT_FILE "../video/84598.t.mp4"
//#define VIDEO_INPUT_FILE "../video/84599.t.mp4"
//#define VIDEO_INPUT_FILE "../video/84600.t.mp4"
//#define VIDEO_INPUT_FILE "../video/580284764.mp4"
//#define VIDEO_INPUT_FILE "../video/5609.t.mp4"
//#define VIDEO_INPUT_FILE "../video/580285079.mp4"
//#define VIDEO_INPUT_FILE "../video/5610.t.mp4"
//#define VIDEO_INPUT_FILE "../video/580378201.mp4"

//#define VIDEO_OUTPUT_SCREEN

#define VIDEO_OUTPUT_DIR "/home/gigijoe/Videos"
#define VIDEO_OUTPUT_FILE_NAME "result"

#if defined(VIDEO_OUTPUT_FILE_NAME) || defined(VIDEO_OUTPUT_SCREEN)
#define VIDEO_OUTPUT_FRAME
//#define VIDEO_OUTPUT_CAP_FRAME
#endif

#define VIDEO_FRAME_DROP 30

//#define ENABLE_MULTI_TARGET
#define F3F
#ifdef JETSON_NANO
#define F3F_TTY_BASE
#endif

#define MAX_NUM_CONTOURS 16
#define MAX_NUM_TARGET 4

static bool bShutdown = false;
static bool bPause = true;

void sig_handler(int signo)
{
    if(signo == SIGINT) {
        printf("SIGINT\n");
        bShutdown = true;
    }
}

class FrameQueue
{
public:
    struct cancelled {};

public:
    FrameQueue() : cancelled_(false) {}

    void push(Mat const & image);
    Mat pop();

    void cancel();
    bool isCancelled() { return cancelled_; }
    void reset();

private:
    std::queue<cv::Mat> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool cancelled_;
};

void FrameQueue::cancel()
{
    std::unique_lock<std::mutex> mlock(mutex_);
    cancelled_ = true;
    cond_.notify_all();
}

void FrameQueue::push(cv::Mat const & image)
{
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(image);
    while(queue_.size() > 16) /* Maximum Q size is 16 ... */
        queue_.pop();
    cond_.notify_one();
}

Mat FrameQueue::pop()
{
    std::unique_lock<std::mutex> mlock(mutex_);

    while (queue_.empty()) {
        if (cancelled_) {
            throw cancelled();
        }
        cond_.wait(mlock);
        if (cancelled_) {
            throw cancelled();
        }
    }
    Mat image(queue_.front());
    queue_.pop();
    return image;
}

void FrameQueue::reset()
{
    cancelled_ = false;
}

/*
*
*/

#if defined(VIDEO_OUTPUT_FILE_NAME) || defined(VIDEO_OUTPUT_SCREEN)
FrameQueue videoWriterQueue;

void VideoWriterThread(int width, int height)
{    
    Size videoSize = Size((int)width,(int)height);
    char gstStr[320];
#ifdef VIDEO_OUTPUT_FILE_NAME
    VideoWriter outFile;
    char filePath[64];
    int videoOutoutIndex = 0;
    while(videoOutoutIndex < 1000) {
#ifdef JETSON_NANO
        snprintf(filePath, 64, "%s/%s%03d.mkv", VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, videoOutoutIndex);
#else
        snprintf(filePath, 64, "%s/%s%03d.mp4", VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, videoOutoutIndex);
#endif
        FILE *fp = fopen(filePath, "rb");
        if(fp) { /* file exist ... */
            fclose(fp);
            videoOutoutIndex++;
        } else
            break; /* File doesn't exist. OK */
    }
#ifdef JETSON_NANO
    /* Countclockwise rote 90 degree - nvvidconv flip-method=1 */
    snprintf(gstStr, 320, "appsrc ! video/x-raw, format=(string)BGR ! \
                   videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
                   nvvidconv flip-method=1 ! omxh265enc preset-level=3 ! matroskamux ! filesink location=%s/%s%03d.mkv ", 
        30, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, videoOutoutIndex);
    outFile.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), 30, videoSize);
    cout << "Vodeo output " << gstStr << endl;
#else
    outFile.open(filePath, VideoWriter::fourcc('X', '2', '6', '4'), 30, videoSize);
    cout << "Vodeo output " << filePath << endl;
#endif
#endif //VIDEO_OUTPUT_FILE_NAME
#ifdef VIDEO_OUTPUT_SCREEN
#ifdef JETSON_NANO
    snprintf(gstStr, 320, "appsrc ! video/x-raw, format=(string)BGR ! \
                   videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
                   nvvidconv ! video/x-raw(memory:NVMM) ! \
                   nvoverlaysink sync=false -e ", 30);
    VideoWriter outScreen;
    outScreen.open(gstStr, VideoWriter::fourcc('I', '4', '2', '0'), 30, Size(CAMERA_WIDTH, CAMERA_HEIGHT));
#else
#endif //JETSON_NANO
#endif //VIDEO_OUTPUT_SCREEN
    videoWriterQueue.reset();
    try {
        while(1) {
            Mat frame = videoWriterQueue.pop();
#ifdef VIDEO_OUTPUT_FILE_NAME
            outFile.write(frame);
#endif
#ifdef VIDEO_OUTPUT_SCREEN
#ifdef JETSON_NANO
            outScreen.write(frame);
#else
#endif //JETSON_NANO
#endif
        }
    } catch (FrameQueue::cancelled & /*e*/) {
        // Nothing more to process, we're done
        std::cout << "FrameQueue " << " cancelled, worker finished." << std::endl;
#ifdef VIDEO_OUTPUT_FILE_NAME
        outFile.release();
#endif
#ifdef VIDEO_OUTPUT_SCREEN
#ifdef JETSON_NANO
        outScreen.release();
#else
#endif //JETSON_NANO
#endif
    }    
}

#endif

int main(int argc, char**argv)
{
#ifdef JETSON_NANO
    jetsonNanoGPIONumber greenLED = gpio16;     // Ouput
    jetsonNanoGPIONumber redLED = gpio17;     // Ouput

    jetsonNanoGPIONumber pushButton = gpio18;

    /* 
    * Do enable GPIO by /etc/profile.d/export-gpio.sh 
    */

    gpioExport(redLED);
    gpioExport(greenLED);
    gpioExport(pushButton);

    gpioSetDirection(redLED, outputPin); /* Red LED on while detection */
    gpioSetValue(redLED, off);

    gpioSetDirection(greenLED, outputPin); /* Flash during frames */
    gpioSetValue(greenLED, on);

    gpioSetDirection(pushButton, inputPin); /* Pause / Restart */
#endif
#ifdef F3F_TTY_BASE
    const char *ttyName = "/dev/ttyTHS1";

    int ttyFd = open (ttyName, O_RDWR | O_NOCTTY | O_SYNC);
    if (ttyFd) {
        set_interface_attribs (ttyFd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (ttyFd, 0);                // set no blocking
    } else
        printf ("error %d opening %s: %s\n", errno, ttyName, strerror (errno));
#endif

    if(signal(SIGINT, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGINT\n");

    Mat frame, capFrame;
    cuda::GpuMat gpuFrame;

    cuda::printShortCudaDeviceInfo(cuda::getDevice());
    std::cout << cv::getBuildInformation() << std::endl;

#ifdef JETSON_NANO
    static char gstStr[320];
#endif
#ifdef VIDEO_INPUT_FILE
#ifdef JETSON_NANO
    snprintf(gstStr, 320, "filesrc location=%s ! qtdemux name=demux demux.video_0 ! queue ! "
        "h264parse ! omxh264dec ! videoconvert ! appsink ", VIDEO_INPUT_FILE);
    VideoCapture cap(gstStr, cv::CAP_GSTREAMER);
#else
    VideoCapture cap(VIDEO_INPUT_FILE, cv::CAP_FFMPEG);
#endif
#else
    int index = 0;    
    if(argc > 1)
        index = atoi(argv[1]);
#ifdef JETSON_NANO
    /* export GST_DEBUG=2 to show debug message */
    snprintf(gstStr, 320, "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
        nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true -e ", 
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
    VideoCapture cap(gstStr, cv::CAP_GSTREAMER);
#else
    VideoCapture cap(index);
#endif

#ifdef JETSON_NANO
        cout << "Video input " << gstStr << endl;
#else
        cout << "Video input (" << static_cast<int32_t>(cap.get(CAP_PROP_FRAME_WIDTH)) << "x" << static_cast<int32_t>(cap.get(CAP_PROP_FRAME_HEIGHT))
            << ") at " << cap.get(CAP_PROP_FPS) << " FPS." << endl;
#endif

#endif
    if(!cap.isOpened()) {
        cout << "Could not open video" << endl;
        return 1;
    }

#ifdef VIDEO_INPUT_FILE
#else
#ifdef JETSON_NANO
    cout << "Video input " << gstStr << endl;
#else
    cap.set(CAP_PROP_FOURCC ,VideoWriter::fourcc('M', 'J', 'P', 'G') );
    cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
    cap.set(CAP_PROP_FPS, 30.0);

    cout << "Video input (" << static_cast<int32_t>(cap.get(CAP_PROP_FRAME_WIDTH)) << "x" << static_cast<int32_t>(cap.get(CAP_PROP_FRAME_HEIGHT))
        << ") at " << cap.get(CAP_PROP_FPS) << " FPS." << endl;
#endif
    cout << "Drop first " << VIDEO_FRAME_DROP << " for camera stable ..." << endl;
    for(int i=0;i<VIDEO_FRAME_DROP;i++) {
        if(!cap.read(capFrame))
            printf("Error read camera frame ...\n");
    }
#endif
#if defined(VIDEO_OUTPUT_FILE_NAME) || defined(VIDEO_OUTPUT_SCREEN)
    thread outThread;
#endif
    //Ptr<BackgroundSubtractor> bsModel = createBackgroundSubtractorKNN();
    //Ptr<BackgroundSubtractor> bsModel = createBackgroundSubtractorMOG2();
    Ptr<cuda::BackgroundSubtractorMOG2> bsModel = cuda::createBackgroundSubtractorMOG2();

    bool doUpdateModel = true;
    bool doSmoothMask = true;

    Mat foregroundMask, background;
#ifdef VIDEO_OUTPUT_FRAME
    Mat outFrame;
#endif
    cuda::GpuMat gpuForegroundMask;
    Ptr<cuda::Filter> gaussianFilter;
    Ptr<cuda::Filter> erodeFilter;
    Ptr<cuda::Filter> erodeFilter2;

    Tracker tracker;
    Target *primaryTarget = 0;

#ifdef F3F
    int cx, cy;
    while(1) {
        if(cap.read(capFrame))
            break;
        if(bShutdown)
            return 0;
    }
    cx = capFrame.cols - 1;
    cy = (capFrame.rows / 2) - 1;
#endif

    high_resolution_clock::time_point t1(high_resolution_clock::now());

#ifdef JETSON_NANO
    uint64_t frameCount = 0;
    unsigned int vPushButton = 0;
#endif
    while(cap.read(capFrame)) {
#ifdef JETSON_NANO
        unsigned int gv;
        gpioGetValue(pushButton, &gv);

        if(gv == 1 && vPushButton == 0) { /* Raising edge */
            bPause = !bPause;
#if defined(VIDEO_OUTPUT_FILE_NAME) || defined(VIDEO_OUTPUT_SCREEN)
            if(bPause) {
                cout << "Paused ..." << endl;
                videoWriterQueue.cancel();
                outThread.join();
            } else {/* Restart, record new video */
                cout << "Restart ..." << endl;
                outThread = thread(&VideoWriterThread, capFrame.cols, capFrame.rows);
            }
#endif
        }
        vPushButton = gv;

        if(bPause) {
            if(bShutdown)
                break;
            gpioSetValue(redLED, off);
            gpioSetValue(greenLED, on);
            continue;
        }

        if(frameCount++ % 2 == 0)
            gpioSetValue(greenLED, on);
        else
            gpioSetValue(greenLED, off);
#endif
        cvtColor(capFrame, frame, COLOR_BGR2GRAY);
#ifdef VIDEO_OUTPUT_FRAME
        capFrame.copyTo(outFrame);
#ifdef F3F
        //line(outFrame, Point(cx, 0), Point(cx, cy), Scalar(0, 255, 0), 1);
        line(outFrame, Point(0, cy), Point(cx, cy), Scalar(0, 255, 0), 1);
#endif
#endif
        int erosion_size = 6;   
        Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                          cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                          cv::Point(erosion_size, erosion_size) );
#if 0 /* Very poor performance ... Running by CPU is 10 times quick */
        gpuFrame.upload(frame);
        if(erodeFilter.empty()) 
            erodeFilter = cuda::createMorphologyFilter(MORPH_ERODE, gpuFrame.type(), element);
        erodeFilter->apply(gpuFrame, gpuFrame);
#else
        erode(frame, frame, element);
        gpuFrame.upload(frame);	
#endif    
        bsModel->apply(gpuFrame, gpuForegroundMask, doUpdateModel ? -1 : 0);

        if(gaussianFilter.empty())
            gaussianFilter = cuda::createGaussianFilter(gpuForegroundMask.type(), gpuForegroundMask.type(), Size(5, 5), 3.5);

        if(doSmoothMask) {
            gaussianFilter->apply(gpuForegroundMask, gpuForegroundMask);
            cuda::threshold(gpuForegroundMask, gpuForegroundMask, 40.0, 255.0, THRESH_BINARY);
            
			/* Erode & Dilate */
            int erosion_size = 6;   
            Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                          cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                          cv::Point(erosion_size, erosion_size) );
#if 0
        if(erodeFilter2.empty()) 
            erodeFilter2 = cuda::createMorphologyFilter(MORPH_ERODE, gpuForegroundMask.type(), element);
        erodeFilter2->apply(gpuForegroundMask, gpuForegroundMask);
        gpuForegroundMask.download(foregroundMask);
#else
            gpuForegroundMask.download(foregroundMask);
            erode(foregroundMask, foregroundMask, element);
#endif
        } else
            gpuForegroundMask.download(foregroundMask);

		vector< vector<Point> > contours;
    	vector< Vec4i > hierarchy;
    	findContours(foregroundMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        sort(contours.begin(), contours.end(), ContoursSort); /* Contours sort by area, controus[0] is largest */

        vector<Rect> boundRect( contours.size() );
        vector<Rect> roiRect;

        RNG rng(12345);
#if 0
        if(contours.size() > MAX_NUM_CONTOURS ) { /* Too many objects */
            printf("Video unstable ...\n"); 
        }
#endif
    	for(int i=0; i<contours.size(); i++) {
    		approxPolyDP( Mat(contours[i]), contours[i], 3, true );
       		boundRect[i] = boundingRect( Mat(contours[i]) );
       		//drawContours(contoursImg, contours, i, color, 2, 8, hierarchy);
    		if(boundRect[i].width > MIN_TARGET_WIDTH && 
                boundRect[i].height > MIN_TARGET_HEIGHT &&
    			boundRect[i].width <= MAX_TARGET_WIDTH && 
                boundRect[i].height <= MAX_TARGET_HEIGHT) {
#if 1
                if(primaryTarget && contours.size() > MAX_NUM_CONTOURS ) { /* Too many objects */
                    /* if primary target exist, try to track it ONLY and ignore others ... */
                    if((primaryTarget->LatestRect() & boundRect[i]).area() > 0) /* Primagy target tracked ... */
                        roiRect.push_back(boundRect[i]);
                } else
#endif
                    roiRect.push_back(boundRect[i]);
#ifdef ENABLE_MULTI_TARGET                
#ifdef VIDEO_OUTPUT_FRAME
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            rectangle( outFrame, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
#endif
#endif
    		}
            if(roiRect.size() >= MAX_NUM_TARGET) /* Deal top 5 only */
                break;
    	}
        tracker.Update(roiRect);
#ifdef JETSON_NANO
        gpioSetValue(redLED, off);
#endif
        primaryTarget = tracker.PrimaryTarget();
        if(primaryTarget) {
#ifdef VIDEO_OUTPUT_FRAME
            Rect r = primaryTarget->m_rects.back();
            rectangle( outFrame, r.tl(), r.br(), Scalar( 255, 0, 0 ), 2, 8, 0 );

            if(primaryTarget->m_points.size() > 1) { /* Minimum 2 points ... */
                for(int i=0;i<primaryTarget->m_points.size()-1;i++) {
                    line(outFrame, primaryTarget->m_points[i], primaryTarget->m_points[i+1], Scalar(0, 255, 255), 1);
                }
            }
#endif
#ifdef F3F
/*
#ifdef VIDEO_OUTPUT_FRAME
            char str[32];
            snprintf(str, 32, "Velocity : [%d, %d]", primaryTarget->m_velocity.x, primaryTarget->m_velocity.y);
            writeText(outFrame, string(str));
#endif
*/
            if(primaryTarget->ArcLength() > 16) {
                if((primaryTarget->m_points[0].y > cy && primaryTarget->LatestPoint().y <= cy) ||
                    (primaryTarget->m_points[0].y < cy && primaryTarget->LatestPoint().y >= cy)) {
#ifdef VIDEO_OUTPUT_FRAME
//                    line(outFrame, Point(cx, 0), Point(cx, cy), Scalar(0, 0, 255), 3);
                    line(outFrame, Point(0, cy), Point(cx, cy), Scalar(0, 0, 255), 3);
#endif                
#ifdef F3F_TTY_BASE
        		    base_toggle(ttyFd, frameCount);
#endif
#ifdef JETSON_NANO
                    gpioSetValue(redLED, on);
#endif
                }
            }
#endif  /* F3F */            
        }
        //imshow("foreground mask", foregroundMask);
/*
        bsModel->getBackgroundImage(background);
        if (!background.empty())
            imshow("mean background image", background );
*/
#ifdef VIDEO_OUTPUT_SCREEN
        int k = waitKey(1);
        if(k == 27) {
            break;
        } else if(k == 'p') {
            while(waitKey(1) != 'p') {
                if(bShutdown)
                    break;
            }
        }
#ifdef JETSON_NANO
#else        
        imshow("Out Frame",outFrame);
#endif
#endif
#if defined(VIDEO_OUTPUT_FILE_NAME) || defined(VIDEO_OUTPUT_SCREEN)
#ifdef VIDEO_OUTPUT_CAP_FRAME
        videoWriterQueue.push(capFrame.clone());
#else
        videoWriterQueue.push(outFrame.clone());
#endif
#endif
        if(bShutdown)
            break;

        high_resolution_clock::time_point t2(high_resolution_clock::now());
        double dt_us(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));
        std::cout << "FPS : " << fixed  <<  setprecision(2) << (1000000.0 / dt_us) << std::endl;

        t1 = high_resolution_clock::now();
    }
#ifdef JETSON_NANO
    gpioSetValue(greenLED, off);
    gpioSetValue(redLED, off);
#endif
#if defined(VIDEO_OUTPUT_FILE_NAME) || defined(VIDEO_OUTPUT_SCREEN)
    if(bPause == false) {
        videoWriterQueue.cancel();
        outThread.join();
    }
#endif

#ifdef F3F_TTY_BASE
    if(ttyFd)
        close(ttyFd);
#endif
    std::cout << "Finished ..." << endl;

    return 0;     
}
