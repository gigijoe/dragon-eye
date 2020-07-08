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

//#define CAMERA_1080P

#ifdef CAMERA_1080P
    #define CAMERA_WIDTH 1920
    #define CAMERA_HEIGHT 1080
    #define CAMERA_FPS 30
    #define MIN_TARGET_WIDTH 12
    #define MIN_TARGET_HEIGHT 12
    #define MAX_TARGET_WIDTH 480
    #define MAX_TARGET_HEIGHT 480
#else
    #define CAMERA_WIDTH 1280
    #define CAMERA_HEIGHT 720
    #define CAMERA_FPS 60
    #define MIN_TARGET_WIDTH 8
    #define MIN_TARGET_HEIGHT 8
    #define MAX_TARGET_WIDTH 320
    #define MAX_TARGET_HEIGHT 320
#endif

#define MAX_NUM_TARGET 3                /* Maximum targets to tracing */
#define MAX_NUM_TRIGGER 4               /* Maximum number of RF trigger after detection of cross line */
#define MAX_NUM_FRAME_MISSING_TARGET 10 /* Maximum number of frames to keep tracing lost target */

#define MIN_COURSE_LENGTH 120           /* Minimum course length of RF trigger after detection of cross line */
#define MIN_TARGET_TRACKED_COUNT 3      /* Minimum target tracked count of RF trigger after detection of cross line */

#define VIDEO_OUTPUT_DIR "/home/gigijoe/Videos"
#define VIDEO_OUTPUT_FILE_NAME "sB"

typedef enum { BASE_A, BASE_B, BASE_TIMER, BASE_ANEMOMETER } BaseType;

static BaseType baseType = BASE_A;

static bool bVideoOutputScreen = false;
static bool bVideoOutputFile = false;
static bool bVideoOutputResult = false;
static bool bShutdown = false;
static bool bPause = true;

static uint64_t frameCount = 0;

void sig_handler(int signo)
{
    if(signo == SIGINT) {
        printf("SIGINT\n");
        bShutdown = true;
    }
}

/*
*
*/

static int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if(tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tcgetattr\n", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    cfmakeraw(&tty); /* RAW mode */

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    //tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout
    tty.c_cc[VTIME] = 0;            // Non block read

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf ("error %d from tcsetattr\n", errno);
        return -1;
    }
    return 0;
}

static void base_trigger(int fd, bool newTrigger) 
{
    static uint8_t serNo = 0x3f;
    uint8_t data[1];

    if(!fd)
        return;

    if(newTrigger) { /* It's NEW trigger */
        if(++serNo > 0x3f)
            serNo = 0;
    }

    if(baseType == BASE_A) {
        data[0] = (serNo & 0x3f);
        printf("BASE_A[%d]\r\n", serNo);
        write(fd, data, 1);
    } else if(baseType == BASE_B) {
        data[0] = (serNo & 0x3f) | 0x40;
        printf("BASE_B[%d]\r\n", serNo);
        write(fd, data, 1);
    }
}

/*
*
*/

static inline bool ContoursSort(vector<cv::Point> contour1, vector<cv::Point> contour2)  
{  
    //return (contour1.size() > contour2.size()); /* Outline length */
    return (cv::contourArea(contour1) > cv::contourArea(contour2)); /* Area */
}  

/*
*
*/

class Target
{
protected:
    double m_courseLength;
    unsigned long m_frameTick;
    uint8_t m_triggerCount;

public:
    Target(Rect & roi, unsigned long frameTick) : m_courseLength(0), m_frameTick(frameTick), m_triggerCount(0) {
        m_rects.push_back(roi);
        m_points.push_back(roi.tl());
        m_frameTick = frameTick;
    }

    vector< Rect > m_rects;
    vector< Point > m_points;
    Point m_velocity;

    void Update(Rect & roi, unsigned long frameTick) {
        if(m_rects.size() > 0)
            m_courseLength += norm(roi.tl() - m_rects.back().tl());

        if(m_points.size() == 1) {
            m_velocity.x = (roi.tl().x - m_rects.back().tl().x);
            m_velocity.y = (roi.tl().y - m_rects.back().tl().y);
        } else if(m_points.size() > 1) {
            m_velocity.x = (m_velocity.x + (roi.tl().x - m_rects.back().tl().x)) / 2;
            m_velocity.y = (m_velocity.y + (roi.tl().y - m_rects.back().tl().y)) / 2;
        }
        m_rects.push_back(roi);
        m_points.push_back(roi.tl());
        m_frameTick = frameTick;
    }
#if 0
    void Reset() {
        Rect r = m_rects.back();
        Point p = r.tl();
        m_rects.clear();
        m_points.clear();
        m_rects.push_back(r);
        m_points.push_back(p);
        m_triggerCount = 0;
        //m_frameTick =
        m_courseLength = 0; /* TODO : Do clear this ? */
    }
#endif
    inline double CourseLength() { return m_courseLength; }
    inline unsigned long FrameTick() { return m_frameTick; }
    inline Rect & LatestRect() { return m_rects.back(); }
    inline Point & LatestPoint() { return m_points.back(); }
    inline void Trigger() { m_triggerCount++; }
    inline uint8_t TriggerCount() { return m_triggerCount; }
    //inline Point & Velocity() { return m_velocity; }
    inline size_t TrackedCount() { return m_rects.size(); }
};

static inline bool TargetSort(Target & a, Target & b)
{
    return a.LatestRect().area() > b.LatestRect().area();
}

/*
*
*/

class Tracker
{
private:
    unsigned long m_frameTick;
    list< Target > m_targets;
    Target *m_primaryTarget;

public:
    Tracker() : m_frameTick(0), m_primaryTarget(0) {}

    void Update(vector< Rect > & roiRect) {
        const int euclidean_distance = 120;

        if(m_primaryTarget) {
            Target *t = m_primaryTarget;
            int i;
            for(i=0; i<roiRect.size(); i++) {
                Rect r = t->m_rects.back();
                if((r & roiRect[i]).area() > 0) /* Target tracked by region overlap  */
                    break;                

                unsigned long f = m_frameTick - t->FrameTick();
                r.x += t->m_velocity.x * f;
                r.y += t->m_velocity.y * f;
                if((r & roiRect[i]).area() > 0) /* Target tracked by velocity ... */
                    break;

                r = t->m_rects.back();
                if(cv::norm(r.tl()-roiRect[i].tl()) < euclidean_distance) /* Target tracked by Euclidean distance ... */
                    break;
            }
            if(i != roiRect.size()) { /* Primary Target tracked */
                t->Update(roiRect[i], m_frameTick);
                return;
            }
        }

        for(list< Target >::iterator t=m_targets.begin();t!=m_targets.end();) { /* Try to find lost targets */
            int i;
            for(i=0; i<roiRect.size(); i++) {
                Rect r = t->m_rects.back();
                if((r & roiRect[i]).area() > 0) /* Target tracked ... */
                    break;                

                unsigned long f = m_frameTick - t->FrameTick();
                r.x += t->m_velocity.x * f;
                r.y += t->m_velocity.y * f;
                if((r & roiRect[i]).area() > 0) /* Target tracked with velocity ... */
                    break;

                r = t->m_rects.back();
                if(cv::norm(r.tl()-roiRect[i].tl()) < euclidean_distance) /* Target tracked with Euclidean distance ... */
                    break;
            }
            if(i == roiRect.size()) { /* Target missing ... */
                if(m_frameTick - t->FrameTick() > MAX_NUM_FRAME_MISSING_TARGET) { /* Target still missing for over X frames */
#ifdef DEBUG            
                    Point p = t->m_points.back();
                    printf("lost target : %d, %d\n", p.x, p.y);
#endif
                    if(&(*t) == m_primaryTarget)
                        m_primaryTarget = 0;

                    t = m_targets.erase(t); /* Remove tracing target */
                    continue;
                }
            }
            t++;
        }

        for(int i=0; i<roiRect.size(); i++) { /* Try to find NEW target for tracking ... */
            list< Target >::iterator t;
            for(t=m_targets.begin();t!=m_targets.end();t++) {
                Rect r = t->m_rects.back();
                if((r & roiRect[i]).area() > 0) /* Next step tracked ... */
                    break;

                unsigned long f = m_frameTick - t->FrameTick();
                r.x += t->m_velocity.x * f;
                r.y += t->m_velocity.y * f;
                if((r & roiRect[i]).area() > 0) /* Next step tracked with velocity ... */
                    break;

                r = t->m_rects.back();
                if(cv::norm(r.tl()-roiRect[i].tl()) < euclidean_distance) /* Target tracked with Euclidean distance ... */                    
                    break;
            }

            if(t == m_targets.end()) { /* New target */
                m_targets.push_back(Target(roiRect[i], m_frameTick));
#ifdef DEBUG            
                printf("new target : %d, %d\n", roiRect[i].tl().x, roiRect[i].tl().y);
#endif
            } else
                t->Update(roiRect[i], m_frameTick);
        }
        m_frameTick++;

        if(m_targets.size() > 1)
            m_targets.sort(TargetSort);

        if(m_targets.size() > 0) {
            if(m_primaryTarget == 0)
                m_primaryTarget = &m_targets.front();
        }
    }

    Target *PrimaryTarget() {
        if(m_targets.size() == 0)
            return 0;

        //m_targets.sort(TargetSort);
        return m_primaryTarget;
    }
};

/*
*
*/

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
    //static uint32_t delayCount = 0;
    while(queue_.size() >= 3) { /* Prevent memory overflow ... */
        //usleep(10000000); /* Wait for 10 ms */
        //if(++delayCount > 3) {
        //    delayCount = 0;
            return; /* Drop frame */
        //}
    }

    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(image);
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

static FrameQueue videoWriterQueue;

void VideoWriterThread(int width, int height)
{    
    Size videoSize = Size((int)width,(int)height);
    char gstStr[320];

    VideoWriter outFile;
    VideoWriter outScreen;
    char filePath[64];
    int videoOutoutIndex = 0;
    while(videoOutoutIndex < 1000) {
        snprintf(filePath, 64, "%s/%s%c%03d.mkv", VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
        FILE *fp = fopen(filePath, "rb");
        if(fp) { /* file exist ... */
            fclose(fp);
            videoOutoutIndex++;
        } else
            break; /* File doesn't exist. OK */
    }

    if(bVideoOutputFile) {
    /* Countclockwise rote 90 degree - nvvidconv flip-method=1 */
        snprintf(gstStr, 320, "appsrc ! video/x-raw, format=(string)BGR ! \
                   videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
                   nvvidconv flip-method=1 ! omxh265enc preset-level=3 bitrate=8000000 ! matroskamux ! filesink location=%s/%s%c%03d.mkv ", 
            30, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
        outFile.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), 30, videoSize);
        cout << "Vodeo output " << gstStr << endl;
    }

    if(bVideoOutputScreen) {
        snprintf(gstStr, 320, "appsrc ! video/x-raw, format=(string)BGR ! \
                       videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
                       nvvidconv ! video/x-raw(memory:NVMM) ! \
                       nvoverlaysink sync=false -e ", 30);
        outScreen.open(gstStr, VideoWriter::fourcc('I', '4', '2', '0'), 30, Size(CAMERA_WIDTH, CAMERA_HEIGHT));
    }

    videoWriterQueue.reset();

    try {
        while(1) {
            Mat frame = videoWriterQueue.pop();
            if(bVideoOutputFile)
                outFile.write(frame);
            if(bVideoOutputScreen)
                outScreen.write(frame);
        }
    } catch (FrameQueue::cancelled & /*e*/) {
        // Nothing more to process, we're done
        std::cout << "FrameQueue " << " cancelled, worker finished." << std::endl;
        if(bVideoOutputFile)
            outFile.release();
        if(bVideoOutputScreen)
            outScreen.release();
    }    
}

static jetsonNanoGPIONumber redLED = gpio16; // Ouput
static jetsonNanoGPIONumber greenLED = gpio17; // Ouput
static jetsonNanoGPIONumber blueLED = gpio50; // Ouput
static jetsonNanoGPIONumber relayControl = gpio51; // Ouput

static jetsonNanoGPIONumber videoOutputScreenSwitch = gpio19; // Input
static jetsonNanoGPIONumber videoOutputFileSwitch = gpio20; // Input

static jetsonNanoGPIONumber baseSwitch = gpio12; /* Input */
static jetsonNanoGPIONumber videoOutputResultSwitch = gpio13; /* Input */
static jetsonNanoGPIONumber pushButton = gpio18; // Input

static thread outThread;

class RF_Base {
private:
    int m_ttyFd;
    int m_videoWidth, m_videoHeight;
    unsigned int vPushButton;

    void OnPushButton() {
        if(frameCount < 10) { /* Button debunce */
            usleep(10000);
            return;
        }
        bPause = !bPause;
        frameCount = 0;

        if(bPause) {
            cout << endl;
            cout << "### Object tracking stoped !!!" << endl;
            cout << endl;
            if(bVideoOutputScreen || bVideoOutputFile) {
                videoWriterQueue.cancel();
                outThread.join();
            }
        } else {/* Restart, record new video */
            UpdateDipSwitch();

            cout << endl;
            cout << "### Object tracking started !!!" << endl;
            cout << endl;

            if(bVideoOutputScreen || bVideoOutputFile)
                outThread = thread(&VideoWriterThread, m_videoWidth, m_videoHeight);
        }
    }

public:
    RF_Base() : m_ttyFd(0), m_videoWidth(0), m_videoHeight(0), vPushButton(1) {
    }

    void InitGPIO() {
        /* 
        * Do enable GPIO by /etc/profile.d/export-gpio.sh 
        */

        /* Output */
        gpioExport(redLED);
        gpioExport(greenLED);
        gpioExport(blueLED);
        gpioExport(relayControl);

        gpioSetDirection(redLED, outputPin); /* While object detected */
        gpioSetValue(redLED, off);

        gpioSetDirection(greenLED, outputPin); /* Flash during frames */
        gpioSetValue(greenLED, on);

        gpioSetDirection(blueLED, outputPin); /* Flash during file save */
        gpioSetValue(blueLED, off);

        gpioSetDirection(relayControl, outputPin); /* */
        gpioSetValue(relayControl, off);

        /* Input */
        gpioExport(videoOutputScreenSwitch);
        gpioExport(videoOutputFileSwitch);

        gpioExport(baseSwitch);
        gpioExport(videoOutputResultSwitch);
        gpioExport(pushButton);

        gpioSetDirection(videoOutputScreenSwitch, inputPin); /* Base A / B */
        gpioSetDirection(videoOutputFileSwitch, inputPin); /* Video output on / off */

        gpioSetDirection(baseSwitch, inputPin);
        gpioSetDirection(videoOutputResultSwitch, inputPin);
        gpioSetDirection(pushButton, inputPin); /* Pause / Restart */

    #if 0
        gpioSetEdge(pushButton, "rising");
        int gfd = gpioOpen(pushButton);
        if(gfd) {
            char v;
            struct pollfd p;
            p.fd = fd;
            poll(&p, 1, -1); /* Discard first IRQ */
            read(fd, &v, 1);
            while(1) {
                poll(&p, 1, -1);
                if((p.revents & POLLPRI) == POLLPRI) {
                    lseek(fd, 0, SEEK_SET);
                    read(fd, &v, 1);
                    // printf("Interrup GPIO value : %c\n", v);
                }
            }
            gpioCloae(gfd);
        }
    #endif
    }

    int Open() {
        const char *ttyName = "/dev/ttyTHS1";

        m_ttyFd = open (ttyName, O_RDWR | O_NOCTTY | O_SYNC);
        if (m_ttyFd)
            set_interface_attribs (m_ttyFd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
        else
            printf ("error %d opening %s: %s\n", errno, ttyName, strerror (errno));

        return m_ttyFd;
    }

    void Close() {
        if(m_ttyFd)
            close(m_ttyFd);
    }

    void Toggle(bool newTrigger) {
        static uint8_t serNo = 0x3f;
        uint8_t data[1];

        if(!m_ttyFd)
            return;

        if(newTrigger) { /* It's NEW trigger */
            if(++serNo > 0x3f)
                serNo = 0;
        }

        if(baseType == BASE_A) {
            data[0] = (serNo & 0x3f);
            printf("BASE_A[%d]\r\n", serNo);
            write(m_ttyFd, data, 1);
        } else if(baseType == BASE_B) {
            data[0] = (serNo & 0x3f) | 0x40;
            printf("BASE_B[%d]\r\n", serNo);
            write(m_ttyFd, data, 1);
        }        
    }

    void UpdateDipSwitch() {
        unsigned int gv;
        gpioGetValue(baseSwitch, &gv);

        if(gv == 0) 
            baseType = BASE_A;
        else
            baseType = BASE_B;

        cout << endl;
        printf("### BASE %c\n", gv ? 'B' : 'A');

        gpioGetValue(videoOutputScreenSwitch, &gv);
        if(gv == 0) /* pull low */
            bVideoOutputScreen = true;
        else                    
            bVideoOutputScreen = false;

        printf("### Video output screen : %s\n", bVideoOutputScreen ? "Enable" : "Disable");

        gpioGetValue(videoOutputFileSwitch, &gv);
        if(gv == 0)
            bVideoOutputFile = true;
        else
            bVideoOutputFile = false;

        printf("### Video output file : %s\n", bVideoOutputFile ? "Enable" : "Disable");

        gpioGetValue(videoOutputResultSwitch, &gv);
        if(gv == 0)
            bVideoOutputResult = true;
        else
            bVideoOutputResult = false;                

        printf("### Video output result : %s\n", bVideoOutputResult ? "Enable" : "Disable");
    } 

    void Handler() {        
        frameCount++;

        unsigned int gv;
        gpioGetValue(pushButton, &gv);
        if(gv == 0 && vPushButton == 1) /* Raising edge */
            OnPushButton();
        vPushButton = gv;

        //printf("gv = %d, vPushButton = %d\n", gv, vPushButton);

        /**/

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(m_ttyFd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if(select(m_ttyFd+1, &rfds, NULL, NULL, &tv) > 0) {
            uint8_t data[1];
            int r = read(m_ttyFd, data, 1); /* Receive trigger from f3f timer */
            if(r == 1) {
                if(baseType == BASE_A) {
                    if((data[0] & 0xc0) == 0x80) {
                        uint8_t v = data[0] & 0x3f;
                        if(v == 0x00) { /* BaseA Off - 10xx xxx0 */
                            if(bPause == false) {
                                bPause = true;
                                frameCount = 0;
                            }
                        } else if(v == 0x01) { /* BaseA On - 10xx xxx1 */
                            if(bPause == true) {
                                bPause = false;
                                frameCount = 0;
                            }
                        }
                    }
                } else if(baseType == BASE_B) {
                    if((data[0] & 0xc0) == 0xc0) {
                        uint8_t v = data[0] & 0x3f;
                        if(v == 0x00) { /* BaseB Off - 11xx xxx0 */
                            if(bPause == false) {
                                bPause = true;
                                frameCount = 0;
                            }
                        } else if(v == 0x01) { /* BaseB On - 11xx xxx1 */
                            if(bPause == true) {
                                bPause = false;
                                frameCount = 0;
                            }
                        }
                    }
                }
            }
        }
    }

    void RedLed(pinValues onOff) {
        gpioSetValue(redLED, onOff);
    }

    void GreenLed(pinValues onOff) {
        gpioSetValue(greenLED, onOff);
    }

    void BlueLed(pinValues onOff) {
        gpioSetValue(blueLED, onOff);
    }

    void Relay(pinValues onOff) {
        gpioSetValue(relayControl, onOff);
    }

    void UpdateVideoSize(int width, int height) {
        m_videoWidth = width;
        m_videoHeight = height;
    }

};

static RF_Base rfBase; 

int main(int argc, char**argv)
{
    cuda::printShortCudaDeviceInfo(cuda::getDevice());
    std::cout << cv::getBuildInformation() << std::endl;

    rfBase.InitGPIO();
    rfBase.Open();
    rfBase.UpdateDipSwitch();

    if(signal(SIGINT, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGINT\n");

    static char gstStr[512];

/* Reference : nvarguscamerasrc.txt */

/* export GST_DEBUG=2 to show debug message */
    snprintf(gstStr, 512, "nvarguscamerasrc wbmode=0 tnr-mode=2 tnr-strength=1 ee-mode=1 ee-strength=0 gainrange=\"1 16\" ispdigitalgainrange=\"1 8\" exposuretimerange=\"5000000 20000000\" exposurecompensation=2 ! \
        video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
        nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true -e ", 
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
/*
    snprintf(gstStr, 512, "v4l2src device=/dev/video1 ! \
        video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
        nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true -e ", 
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
*/
    VideoCapture cap(gstStr, cv::CAP_GSTREAMER);

    cout << "Video input : " << gstStr << endl;

    if(!cap.isOpened()) {
        cout << "Could not open video" << endl;
        return 1;
    }

    //Ptr<BackgroundSubtractor> bsModel = createBackgroundSubtractorKNN();
    //Ptr<BackgroundSubtractor> bsModel = createBackgroundSubtractorMOG2();
    Ptr<cuda::BackgroundSubtractorMOG2> bsModel = cuda::createBackgroundSubtractorMOG2(90, 16, false); /* background history count, varThreshold, shadow detection */

    bool doUpdateModel = true;
    bool doSmoothMask = true;

    Mat foregroundMask, background;
    Mat outFrame;

    cuda::GpuMat gpuForegroundMask;
    Ptr<cuda::Filter> gaussianFilter;
    Ptr<cuda::Filter> erodeFilter;
    Ptr<cuda::Filter> erodeFilter2;

    Tracker tracker;
    Target *primaryTarget = 0;

    Mat frame, capFrame;
    cuda::GpuMat gpuFrame;

    while(1) {
        if(cap.read(capFrame))
            break;
        else
            usleep(10000);
        if(bShutdown)
            return 0;
    }

    int cx = capFrame.cols - 1;
    int cy = (capFrame.rows / 2) - 1;

    rfBase.UpdateVideoSize(capFrame.cols, capFrame.rows);

    cout << endl;
    cout << "### Press button to start object tracking !!!" << endl;
    cout << endl;

    double fps = 0;

    high_resolution_clock::time_point t1(high_resolution_clock::now());

    while(cap.read(capFrame)) {
        unsigned int gv;

        if(bShutdown)
            break;

        rfBase.Handler();

        if(bPause) {
            rfBase.RedLed(off);
            rfBase.Relay(off);
            rfBase.GreenLed(on);
            if(bVideoOutputFile)
                rfBase.BlueLed(on);

            usleep(10000); /* Wait 10ms */
            continue;
        }

        if(frameCount % 2 == 0) {
            rfBase.GreenLed(on); /* Flash during frames */
            if(bVideoOutputFile)
                rfBase.BlueLed(on);
        } else {
            rfBase.GreenLed(off); /* Flash during frames */
            if(bVideoOutputFile)
                rfBase.BlueLed(off);
        }

        cvtColor(capFrame, frame, COLOR_BGR2GRAY);
        if(bVideoOutputResult) {
            if(bVideoOutputScreen || bVideoOutputFile) {
                capFrame.copyTo(outFrame);
                line(outFrame, Point(0, cy), Point(cx, cy), Scalar(0, 255, 0), 1);
            }
        }

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
            /* Disable threadhold while low background noise */
            /* 10.0 may be good senstitive */
            //cuda::threshold(gpuForegroundMask, gpuForegroundMask, 10.0, 255.0, THRESH_BINARY);
            /* 40.0 with lower senstitive */
            //cuda::threshold(gpuForegroundMask, gpuForegroundMask, 40.0, 255.0, THRESH_BINARY);
            
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
//    	findContours(foregroundMask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        findContours(foregroundMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        sort(contours.begin(), contours.end(), ContoursSort); /* Contours sort by area, controus[0] is largest */

        vector<Rect> boundRect( contours.size() );
        vector<Rect> roiRect;

        RNG rng(12345);

    	for(int i=0; i<contours.size(); i++) {
    		approxPolyDP( Mat(contours[i]), contours[i], 3, true );
       		boundRect[i] = boundingRect( Mat(contours[i]) );
       		//drawContours(contoursImg, contours, i, color, 2, 8, hierarchy);
    		if(boundRect[i].width > MIN_TARGET_WIDTH && 
                boundRect[i].height > MIN_TARGET_HEIGHT &&
    			boundRect[i].width <= MAX_TARGET_WIDTH && 
                boundRect[i].height <= MAX_TARGET_HEIGHT) {

                    roiRect.push_back(boundRect[i]);
                    if(bVideoOutputResult) {
                        if(bVideoOutputScreen || bVideoOutputFile) {
                            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                            rectangle( outFrame, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
                        }
                    }
    		}
            if(roiRect.size() >= MAX_NUM_TARGET) /* Deal top 5 only */
                break;
    	}

        tracker.Update(roiRect);

        rfBase.RedLed(off);
        rfBase.Relay(off);

        primaryTarget = tracker.PrimaryTarget();
        if(primaryTarget) {
            if(bVideoOutputResult) {
                if(bVideoOutputScreen || bVideoOutputFile) {
                    Rect r = primaryTarget->m_rects.back();
                    rectangle( outFrame, r.tl(), r.br(), Scalar( 255, 0, 0 ), 2, 8, 0 );
                    if(primaryTarget->m_points.size() > 1) { /* Minimum 2 points ... */
                        for(int i=0;i<primaryTarget->m_points.size()-1;i++) {
                            line(outFrame, primaryTarget->m_points[i], primaryTarget->m_points[i+1], Scalar(0, 255, 255), 1);
                        }
                    }
                }
            }
            if(primaryTarget->CourseLength() > MIN_COURSE_LENGTH && 
                    primaryTarget->TrackedCount() > MIN_TARGET_TRACKED_COUNT) {
                if((primaryTarget->m_points[0].y > cy && primaryTarget->LatestPoint().y <= cy) ||
                    (primaryTarget->m_points[0].y < cy && primaryTarget->LatestPoint().y >= cy)) {

                    if(primaryTarget->TriggerCount() < MAX_NUM_TRIGGER) { /* Triggle 3 times maximum  */
                        if(bVideoOutputResult) {
                            if(bVideoOutputScreen || bVideoOutputFile)
                                line(outFrame, Point(0, cy), Point(cx, cy), Scalar(0, 0, 255), 3);
                        }
                        rfBase.Toggle(primaryTarget->TriggerCount() == 0);
                        rfBase.RedLed(on);
                        rfBase.Relay(on);

                        primaryTarget->Trigger();
                    }
                }
            }
        }
/*
        bsModel->getBackgroundImage(background);
        if (!background.empty())
            imshow("mean background image", background );
*/
        if(bVideoOutputScreen || bVideoOutputFile) {
            if(bVideoOutputResult) {
                char str[32];
                snprintf(str, 32, "FPS : %.2lf", fps);
                int fontFace = FONT_HERSHEY_SIMPLEX;
                const double fontScale = 1;
                const int thicknessScale = 1;  
                Point textOrg(40, 40);
                putText(outFrame, string(str), textOrg, fontFace, fontScale, Scalar(0, 255, 0), thicknessScale, cv::LINE_8);
                videoWriterQueue.push(outFrame.clone());
            } else {
                videoWriterQueue.push(capFrame.clone());
            }
        }

        high_resolution_clock::time_point t2(high_resolution_clock::now());
        double dt_us(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));
        //std::cout << "FPS : " << fixed  <<  setprecision(2) << (1000000.0 / dt_us) << std::endl;
        fps = (1000000.0 / dt_us);
        std::cout << "FPS : " << fixed  << setprecision(2) <<  fps << std::endl;

        t1 = high_resolution_clock::now();
    }

    rfBase.GreenLed(off); /* Flash during frames */
    rfBase.BlueLed(off); /* Flash during file save */
    rfBase.RedLed(off); /* While object detected */
    rfBase.Relay(off);

    if(bVideoOutputScreen || bVideoOutputFile) {
        if(bPause == false) {
            videoWriterQueue.cancel();
            outThread.join();
        }
    }

    rfBase.Close();

    std::cout << "Finished ..." << endl;

    return 0;     
}
