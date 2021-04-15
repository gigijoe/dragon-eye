// Standard include files
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/route.h>
#include <sys/ioctl.h>
#include <linux/if.h>

using namespace cv;
using namespace std;

#include <chrono>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <list>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <regex>

extern "C" {
#include "jetsonGPIO/jetsonGPIO.h"

#include "gstreamer-1.0/gst/gst.h"
#include "gstreamer-1.0/gst/gstmessage.h"
#include "gstreamer-1.0/gst/rtsp-server/rtsp-server.h"

#include "glib-2.0/glib.h"
#include <gstreamer-1.0/gst/app/app.h>
}

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;

//using std::chrono::system_clock;
using std::chrono::seconds;

#ifndef DEBUG
//#define DEBUG
#endif

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
    #define CAMERA_FPS 30
    #define MIN_TARGET_WIDTH 8
    #define MIN_TARGET_HEIGHT 6
    #define MAX_TARGET_WIDTH 320
    #define MAX_TARGET_HEIGHT 320
#endif

#define MAX_NUM_TARGET               9      /* Maximum targets to tracing */
#define MAX_NUM_TRIGGER              6      /* Maximum number of RF trigger after detection of cross line */
#define MAX_NUM_FRAME_MISSING_TARGET 3      /* Maximum number of frames to keep tracing lost target */

#define MIN_COURSE_LENGTH            30     /* Minimum course length of RF trigger after detection of cross line */
#define MIN_TARGET_TRACKED_COUNT     3      /* Minimum target tracked count of RF trigger after detection of cross line */

#define VIDEO_OUTPUT_DIR             "/home/gigijoe/Videos" /* $(HOME)/Videos/ */
#define VIDEO_OUTPUT_FILE_NAME       "base"
#define VIDEO_FILE_OUTPUT_DURATION   90     /* Video file duration 90 secends */
#define VIDEO_OUTPUT_MAX_FILES       400    /* Needs about 30G bytes disk space */

#define STR_SIZE                     1024
#define CONFIG_FILE_DIR              "/etc/dragon-eye"

typedef enum { BASE_UNKNOWN, BASE_A, BASE_B, BASE_TIMER, BASE_ANEMOMETER } BaseType_t;

/*
*
*/

static bool bShutdown = false;
static bool bStopped = true;

typedef enum { EvtStop, EvtStart } EvtType_t;
static queue<EvtType_t> evtQueue;

static char *ipv4_address(const char *dev) {
    static char host[NI_MAXHOST];
    struct ifaddrs *ifaddr, *ifa;
    int family, s;

    if(dev == 0 || strlen(dev) == 0)
        return 0;

    if(getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        return 0;
    }

    for(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if(ifa->ifa_addr == NULL)
            continue;  

        s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if((strcmp(ifa->ifa_name, dev) == 0) && (ifa->ifa_addr->sa_family == AF_INET)) {
            if (s != 0) {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                return 0;
            }
            //printf("\tInterface : <%s>\n", ifa->ifa_name );
            //printf("\t  Address : <%s>\n", host);
            break;
        }
    }
    freeifaddrs(ifaddr);

    return host;
}

static int add_route(const char *_target, const char *_netmask, const char *_dev) {
    struct rtentry route;  /* route item struct */
    char target[128] = {0};
    char netmask[128] = {0};
    char dev[16] = {0};

    struct sockaddr_in *addr;

    int skfd;

    /* clear route struct by 0 */
    memset((char *)&route, 0x00, sizeof(route));

    /* default target is net (host)*/
    route.rt_flags = RTF_UP ;

    strcpy(target, _target);
    addr = (struct sockaddr_in*) &route.rt_dst;
    addr->sin_family = AF_INET;
    addr->sin_addr.s_addr = inet_addr(target);

    strcpy(netmask, _netmask);
    addr = (struct sockaddr_in*) &route.rt_genmask;
    addr->sin_family = AF_INET;
    addr->sin_addr.s_addr = inet_addr(netmask);

    strcpy(dev, _dev);
    route.rt_dev = dev;

    /* create a socket */
    skfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(skfd < 0) {
        perror("socket");
        return -1;
    }
    
    if(ioctl(skfd, SIOCADDRT, &route) < 0) {
        perror("SIOCADDRT");
        close(skfd);
        return -1;
    }
    (void) close(skfd);

    return 0;
}

/*
*
*/

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%H:%M:%S %m/%d", &tstruct);
    return buf;
}

/*
*
*/

inline void writeText( Mat & mat, const string text, const Point textOrg)
{
   int fontFace = FONT_HERSHEY_SIMPLEX;
   double fontScale = 1;
   int thickness = 1;  
   //Point textOrg( 10, 40 );
   putText( mat, text, textOrg, fontFace, fontScale, Scalar(0, 0, 0), thickness, cv::LINE_8 );
}

/*
*
*/

class Target
{
protected:
    uint32_t m_id;
    double m_arcLength;
    double m_absLength;
    unsigned long m_lastFrameTick;
    uint8_t m_triggerCount;
    uint16_t m_bugTriggerCount;

    vector< unsigned long > m_frameTicks;
    vector< Rect > m_rects;
    vector< Point > m_vectors;
    double m_maxVector, m_minVector;
    Point m_velocity;
    Point m_acceleration;
    int m_averageArea;
    double m_normVelocity;
    double m_angleOfTurn;

    static uint32_t s_id;

    const Point & Center(Rect & r) {
        return std::move(Point(r.tl().x + (r.width / 2), r.tl().y + (r.height / 2)));
    }

    double CosineAngle(const Point & v1, const Point & v2) {
        /* A.B = |A||B|cos() */
        /* cos() = A.B / |A||B| */
        return v1.dot(v2) / (norm(v1) * norm(v2));
    }

    double CosineAngle(const Point & p1, const Point & p2, const Point & p3) {
        Point v1, v2;
        v1.x = p1.x - p2.x;
        v1.y = p1.y - p2.y;
        v2.x = p2.x - p3.x;
        v2.y = p2.y - p3.y;

        /* A.B = |A||B|cos() */
        /* cos() = A.B / |A||B| */
        return v1.dot(v2) / (norm(v1) * norm(v2));
    }

public:
    Target(Rect & roi, unsigned long frameTick) : m_arcLength(0), m_lastFrameTick(frameTick), m_triggerCount(0), m_bugTriggerCount(0), 
            m_maxVector(0), m_minVector(0), m_averageArea(0), m_normVelocity(0), m_angleOfTurn(0) {
        m_id = s_id++;
        m_rects.push_back(roi);
        m_lastFrameTick = frameTick;
        m_frameTicks.push_back(frameTick);
    }

    void Reset() {
        Rect r = m_rects.back();
        m_rects.clear();
        m_rects.push_back(r);
        m_frameTicks.clear();
        m_frameTicks.push_back(m_lastFrameTick);
        m_triggerCount = 0;
        m_bugTriggerCount = 0;
        m_maxVector = 0;
        m_minVector = 0;
        m_averageArea = 0;
        m_normVelocity = 0;
        m_angleOfTurn = 0;
        //m_arcLength = 0; /* TODO : Do clear this ? */
        //m_absLength = 0;
        //m_lastFrameTick =
    }

    void Update(Rect & roi, unsigned long frameTick) {
        if(frameTick <= m_lastFrameTick) /* Reverse tick ??? Illegal !!! */
            return;

        int itick = frameTick - m_lastFrameTick;

        if(m_rects.size() > 0) { /* We have 1 point now and will have 2 */
            Point p = (roi.tl() - m_rects.back().tl()) / itick;
            double v = norm(p) / itick;

            m_arcLength += v;
            m_vectors.push_back(p);

            m_absLength = norm(roi.tl() - m_rects[0].tl());

            if(m_rects.size() == 1) {
                m_maxVector = v;
                m_minVector = v;
            } else if(v > m_maxVector)
                m_maxVector = v;
            else if(v < m_minVector)
                m_minVector = v;

            m_averageArea = (m_averageArea + roi.area()) / 2;
        } else {
            m_averageArea = roi.area();
        }

        if(m_rects.size() == 1) { /* We have 2 point now */
            m_velocity.x = (roi.tl().x - m_rects.back().tl().x) / itick;
            m_velocity.y = (roi.tl().y - m_rects.back().tl().y) / itick;
        } else if(m_rects.size() > 1) { /* We have at latest 3 point now */
            m_velocity.x = (m_velocity.x + (roi.tl().x - m_rects.back().tl().x) / itick) / 2;
            m_velocity.y = (m_velocity.y + (roi.tl().y - m_rects.back().tl().y) / itick) / 2;

            size_t n = m_vectors.size() - 1;
            size_t n_1 = n - 1;

            m_acceleration.x = (m_acceleration.x + (m_vectors[n].x - m_vectors[n_1].x)) / 2;
            m_acceleration.y = (m_acceleration.y + (m_vectors[n].y - m_vectors[n_1].y)) / 2;

            double v = CosineAngle(m_vectors[n], m_vectors[n_1]);
            double radian;
            if(v <= -1.0f)
                radian = M_PI;
            else if(v >= 1.0f)
                radian = 0;
            else
                radian = acos(v);

            /* 
            * r = (v1.x * v2.y) - (v2.x * v1.y) 
            * If r > 0 v2 is located on the left side of v1. 
            * if r == 0 v2 and v1 on the same line.
            * If r < 0 v2 is located on the right side of v2.
            */ 
            v = (m_vectors[n].x * m_vectors[n_1].y) - (m_vectors[n_1].x * m_vectors[n].y);
            if(v < 0)
                radian *= -1.0;

            m_angleOfTurn += (radian * 180 / M_PI);
        }

        m_normVelocity = norm(m_velocity);

        m_rects.push_back(roi);
        m_lastFrameTick = frameTick;
        m_frameTicks.push_back(frameTick);
#if 0
        if(m_triggerCount >= MAX_NUM_TRIGGER)
            Reset();
#endif
    }

    void Update(Target & t) {
        for(size_t i=0;i<t.m_rects.size();i++) {
            Update(t.m_rects[i], t.m_frameTicks[i]);
        }
    }

    int DotProduct(const Point & p) {
        size_t i = m_rects.size();
        if(i < 2)
            return 0;
        i--;
        Point v[2];
        v[0].x = p.x - m_rects[i].tl().x;
        v[0].y = p.y - m_rects[i].tl().y;
        v[1].x = m_rects[i].tl().x - m_rects[i-1].tl().x;
        v[1].y = m_rects[i].tl().y - m_rects[i-1].tl().y;
        int dp = v[0].x * v[1].x + v[0].y * v[1].y;
        return dp;
    }

    double CosineAngle(const Point & p) {
        size_t i = m_rects.size();
        if(i < 2)
            return 0;
        i--;
        Point v1, v2;
        v1.x = p.x - m_rects[i].tl().x;
        v1.y = p.y - m_rects[i].tl().y;
        v2.x = m_rects[i].tl().x - m_rects[i-1].tl().x;
        v2.y = m_rects[i].tl().y - m_rects[i-1].tl().y;

        /* A.B = |A||B|cos() */
        /* cos() = A.B / |A||B| */
        return v1.dot(v2) / (norm(v1) * norm(v2));
    }

    void Draw(Mat & outFrame, bool drawAll = false) {
        Rect r = m_rects.back();
        rectangle( outFrame, r.tl(), r.br(), Scalar( 255, 0, 0 ), 2, 8, 0 );

        //RNG rng(12345);
        if(m_rects.size() > 1) { /* Minimum 2 points ... */
            for(int i=0;i<m_rects.size()-1;i++) {
                //Point p0 = m_rects[i].tl();
                //Point p1 = m_rects[i+1].tl();
                Point p0 = Center(m_rects[i]);
                Point p1 = Center(m_rects[i+1]);                
                line(outFrame, p0, p1, Scalar(0, 0, 255), 1);
                //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                //line(outFrame, p0, p1, color, 1);
                //Point v = p1 - p0;
                //printf("[%d,%d]\n", v.x, v.y);
                if(drawAll)
                    //rectangle( outFrame, m_rects[i].tl(), m_rects[i].br(), Scalar( 196, 0, 0 ), 2, 8, 0 );
                    rectangle( outFrame, m_rects[i].tl(), m_rects[i].br(), Scalar( (m_rects[0].x + m_rects[0].y) % 255, 0, 0 ), 1, 8, 0 );
            }
        }        
    }

    void Info() {
#ifdef DEBUG
        printf("\033[0;31m"); /* Red */
        printf("\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =\n");
        printf("[%u] Target :\n\tsamples = %lu, area = %d, arc length = %.1f, abs length = %.1f, velocity = %.1f\n", 
            m_id, m_rects.size(), m_averageArea, m_arcLength, m_absLength, m_normVelocity);
        printf("\nVectors : length\n");      
        for(auto v : m_vectors) {
            printf("%.1f\t", norm(v));
        }
        printf("\n");
        printf("maximum = %.1f, minimum = %.1f\n", m_maxVector, m_minVector);      
        printf("\nTrajectory : [ Tick ] (x, y) area\n");
        /*
        for(auto p : m_rects) {
            printf("(%4d,%4d)<%5d>\t", p.tl().x, p.tl().y, p.area());
        }
        */
        for(size_t i=0;i<m_rects.size();i++) {
            printf("[%lu](%4d,%4d) %d\t", m_frameTicks[i] - m_frameTicks[0], m_rects[i].tl().x, m_rects[i].tl().y, m_rects[i].area());
        }

        printf("\nAngle of turn :\n"); 
        if(m_vectors.size() > 1)
        for(size_t i=0;i<m_vectors.size()-1;i++) {
            double v = CosineAngle(m_vectors[i], m_vectors[i+1]);
            double radian;
            if(v <= -1.0f)
                radian = M_PI;
            else if(v >= 1.0f)
                radian = 0;
            else
                radian = acos(v);

            /* 
            * r = (v1.x * v2.y) - (v2.x * v1.y) 
            * If r > 0 v2 is located on the left side of v1. 
            * if r == 0 v2 and v1 on the same line.
            * If r < 0 v2 is located on the right side of v2.
            */ 
            v = (m_vectors[i+1].x * m_vectors[1].y) - (m_vectors[1].x * m_vectors[i+1].y);
            if(v < 0)
                radian *= -1.0;

            printf("<%f ", (radian * 180 / M_PI));
        }
        printf("\n= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =\n");
        printf("\033[0m"); /* Default color */
#endif
    }

    inline double VectorDistortion() {
        return m_minVector > 0 ? (m_maxVector / m_minVector) : 0;
    }

    inline double ArcLength() { return m_arcLength; }
    inline double AbsLength() { return m_absLength; }

    inline unsigned long FrameTick() { return m_lastFrameTick; }
    inline const Rect & LastRect() { return m_rects.back(); }

    inline const Point & BeginCenterPoint() { return Center(m_rects[0]); }
    inline const Point & EndCenterPoint() { return Center(m_rects.back()); }

    inline const Point & CurrentCenterPoint() { return Center(m_rects.back()); }
    const Point & PreviousCenterPoint() {
        if(m_rects.size() < 2)
            return std::move(Point(0, 0));

        auto it = m_rects.rbegin();
        return Center(*(++it)); 
    }
    
    bool Trigger(bool enableBugTrigger = false) {
        bool r = false;
        if(m_vectors.size() <= 8 &&
            VectorDistortion() >= 40) { /* 最大位移向量值與最小位移向量值的比例 */
#ifdef DEBUG
            printf("\033[0;31m"); /* Red */
            printf("Velocity distortion %f !!!\n", VectorDistortion());
            printf("\033[0m"); /* Default color */
#endif
        } else if(enableBugTrigger) { 
            if((m_averageArea < 144 && m_normVelocity > 25) || /* 12 x 12 */
                    (m_averageArea < 256 && m_normVelocity > 40) || /* 16 x 16 */
                    (m_averageArea < 324 && m_normVelocity > 50) || /* 18 x 18 */
                    (m_averageArea < 400 && m_normVelocity > 75) || /* 20 x 20 */
                    (m_averageArea < 576 && m_normVelocity > 100) || /* 24 x 24 */
                    (m_averageArea < 900 && m_normVelocity > 125) /* 30 x 30 */
                ) {
#ifdef DEBUG
                printf("\033[0;31m"); /* Red */
                printf("Bug detected !!! average area = %d, velocity = %f\n", m_averageArea, m_normVelocity);
                printf("\033[0m"); /* Default color */
#endif
                m_bugTriggerCount++;
            } else {
                if(m_bugTriggerCount > 0) {
#ifdef DEBUG
                    printf("\033[0;31m"); /* Red */
                    printf("False trigger due to bug trigger count is %u\n", m_bugTriggerCount);
                    printf("\033[0m"); /* Default color */
#endif
                    if(m_bugTriggerCount <= 1) /* To avoid false bug detection */
                        m_bugTriggerCount--;
                } else {
                    m_triggerCount++;
                    r = true;
                }
            }
        } else {
            m_triggerCount++;
            r = true;
        }
        if(r) {
#ifdef DEBUG
            printf("\033[0;31m"); /* Red */
            printf("[%u] T R I G G E R (%d)\n", m_id, m_triggerCount);
            printf("\033[0m"); /* Default color */
#endif
        }

        Info();

        return r;
    }

    inline uint8_t TriggerCount() { return m_triggerCount; }

    inline int AverageArea() { return m_averageArea; }
    inline size_t TrackedCount() { return m_rects.size(); }

    friend class Tracker;
};

uint32_t Target::s_id = 0;

static inline bool TargetSortByArea(Target & a, Target & b)
{
    return a.LastRect().area() > b.LastRect().area();
}

static Rect MergeRect(Rect & r1, Rect & r2) {
    Rect r;
    r.x = min(r1.x, r2.x);
    r.y = min(r1.y, r2.y);
    r.width = r2.x + r2.width - r1.x;
    r.height = max(r1.y + r1.height, r2.y + r2.height) - r.y;
    return r;
}

/*
*
*/

class Tracker
{
private:
    int m_width, m_height;
    unsigned long m_lastFrameTick;
    list< Target > m_targets;
    Rect m_newTargetRestrictionRect;
    list< list< Rect > > m_newTargetsHistory;

public:
    Tracker(int width, int height) : m_width(width), m_height(height), m_lastFrameTick(0) {}

    void NewTargetRestriction(const Rect & r) {
        m_newTargetRestrictionRect = r;
    }

    Rect NewTargetRestriction() const {   return m_newTargetRestrictionRect; }

    void Update(list< Rect > & roiRect, bool enableFakeTargetDetection = false) {
        ++m_lastFrameTick;
        for(list< Target >::iterator t=m_targets.begin();t!=m_targets.end();) { /* Try to find lost targets */
            list<Rect>::iterator rr;
            Rect r1 = t->m_rects.back();
            Rect r2 = r1;
            int f = m_lastFrameTick - t->FrameTick();

            r2.x += (t->m_velocity.x + t->m_acceleration.x) * f;
            r2.y += (t->m_velocity.y + t->m_acceleration.y) * f;

            double n0 = 0;
            if(t->m_vectors.size() > 0) {
                Point v = t->m_velocity + t->m_acceleration;
                n0 = cv::norm(v);
            }
            
            for(rr=roiRect.begin();rr!=roiRect.end();++rr) {

                if(r1.area() > (rr->area() * 32) ||
                    rr->area() > (r1.area() * 32)) /* Object and target area difference */
                    continue;

                double n1 = cv::norm(rr->tl() - r1.tl()); /* Distance between object and target */
#if 1
                if(n1 > 320)
                    continue; /* Too far */

                if(t->m_vectors.size() > 1) {
                    double n = cv::norm(t->m_vectors.back());
                    //if((n1 > (n * 10) || n > (n1 * 10)))
                    if(n1 > (n * f * 2))
                        continue; /* Too far */
                }
#endif
                if((r1 & *rr).area() > 0) { /* Target tracked ... */
                    //if(t->DotProduct(rr->tl()) >= 0) /* Two vector less than 90 degree */
                        break;                
                }

                if(t->m_vectors.size() > 0) {
                    if((r2 & *rr).area() > 0) { /* Target tracked with velocity ... */
                        //if(t->DotProduct(rr->tl()) >= 0) /* Two vector less than 90 degree */
                            break;
                    }
                }

                if(t->m_vectors.size() == 0) { /* new target with zero velocity */
                    if(rr->x < (m_width / 5)) {
                        if(n1 < (rr->width + rr->height)) /* Target tracked with Euclidean distance ... */
                            break;
                    } else {
                        if(n1 < (rr->width + rr->height) * 3) /* Target tracked with Euclidean distance ... */
                            break;
                    }
                } else if(n1 < (n0 * f)) { /* Target tracked with velocity and Euclidean distance ... */
                    if(rr->x < (m_width / 5)) {
                        double a = t->CosineAngle(rr->tl());
                        if(a > 0.9659) /* cos(PI/12) */
                            break;
                    } else {
                        //double a = t->CosineAngle(rr->tl());
                        //if(a > 0.8587) /* cos(PI/6) */
                        //    break;
                    }
                }
            }
 
            if(rr == roiRect.end() && 
                t->m_vectors.size() > 0) { /* Target missing ... */
                for(rr=roiRect.begin();rr!=roiRect.end();++rr) { /* */

                    if(r1.area() > (rr->area() * 32) ||
                        rr->area() > (r1.area() * 32)) /* Object and target area difference */
                        continue;

                    double n1 = cv::norm(rr->tl() - r1.tl()); /* Distance between object and target */
#if 1
                    if(n1 > 320)
                        continue; /* Too far */

                    double n = cv::norm(t->m_vectors.back());
                    //if((n1 > (n * 10) || n > (n1 * 10)))
                    if(n1 > (n * f * 2))
                        continue; /* Too far */
#endif
                    double a = t->CosineAngle(rr->tl());
                    double n2 = cv::norm(rr->tl() - r2.tl());
#if 0
                    if(a > 0.5 && 
                        n2 < (n0 * 1)) { /* cos(PI/3) */
                        break;
                    }               

                    if(a > 0.8587 && 
                        n2 < (n0 * 2)) { /* cos(PI/6) */
                        break;
                    }
#endif
                    /* This number has been tested by various video. Don't touch it !!! */
                    if(a > 0.5 && 
                        n2 < (n0 * f)) { /* cos(PI/3) */
                        break;
                    }
                }
            }

            if(rr == roiRect.end()) { /* Target missing ... */
                uint32_t compensation = (t->TrackedCount() / 10); /* Tracking more frames with more sample */
                if(compensation > 4)
                    compensation = 4;
                if((m_lastFrameTick - t->FrameTick() > MAX_NUM_FRAME_MISSING_TARGET + compensation) || /* Target still missing for over X frames */
                        t->m_vectors.size() == 0) { /* new target with zero velocity */
#ifdef DEBUG
                    Point p = t->m_rects.back().tl();
                    printf("\033[0;35m"); /* Puple */
                    printf("<%u> Lost target : (%d, %d), samples : %lu\n", t->m_id, p.x, p.y, t->m_rects.size());
                    printf("\033[0m"); /* Default color */
#endif
                    t = m_targets.erase(t); /* Remove tracing target */
                    continue;
                } else {
#ifdef DEBUG
                    Point p = t->m_rects.back().tl();
                    printf("<%u> Search target : (%d, %d) -> [%d, %d]\n", t->m_id, p.x, p.y, 
                        (t->m_velocity.x + t->m_acceleration.x) * f, (t->m_velocity.y + t->m_acceleration.y) * f);
#endif
                    for(list< Target >::iterator tt=m_targets.begin();tt!=m_targets.end();++tt) {
                        if(tt->m_id == t->m_id)
                            continue;
                        if((t->m_rects.back() & tt->m_rects.front()).area() > 0) { /**/
                            t->Update(*tt);
#ifdef DEBUG
                            printf("\033[0;33m"); /* Yellow */
                            printf("Merge targets : <%d> -->> <%d>\n", tt->m_id, t->m_id);
                            printf("\033[0m"); /* Default color */
#endif
                            m_targets.erase(tt);
                            break;
                        }
                    }                   
                }
            } else { /* Target tracked ... */
#ifdef DEBUG
                Point p = t->m_rects.back().tl();
                printf("\033[0;32m"); /* Green */
                printf("<%u> Target tracked : [%lu](%d, %d) -> (%d, %d)[%d, %d]\n", t->m_id, m_lastFrameTick, p.x, p.y, 
                    rr->x, rr->y, rr->x - t->m_rects.back().x, rr->y - t->m_rects.back().y);
                printf("\033[0m"); /* Default color */
#endif
                t->Update(*rr, m_lastFrameTick);
                roiRect.erase(rr);
            }
            ++t;
        }

        list< Rect > newTargetList;

        for(list<Rect>::iterator rr=roiRect.begin();rr!=roiRect.end();++rr) { /* New targets registration */
            if(!m_newTargetRestrictionRect.empty()) {
                if((m_newTargetRestrictionRect & *rr).area() > 0)
                    continue;
            }

            uint32_t overlap_count = 0;
            for(auto & l : m_newTargetsHistory) {
                for(auto & r : l) {
                    if(rr->x >= (m_width / 5))
                        continue;
                    if((r & *rr).area() > 0) { /* new target overlap previous new target */
                        ++overlap_count;
                    }
                }
            }
            if(rr->x < (m_width / 5)) {
                if(overlap_count > 0) {
                    rr->x -= rr->width;
                    rr->y -= rr->height;
                    rr->width = rr->width << 1;
                    rr->height = rr->height << 1;
                }
                newTargetList.push_back(*rr);
            }

            if(enableFakeTargetDetection && 
                overlap_count >= 2) {
#ifdef DEBUG
                printf("[X] Fake target : (%u)\n", overlap_count);
#endif
            } else {
                m_targets.push_back(Target(*rr, m_lastFrameTick));
#ifdef DEBUG
                printf("\033[0;32m"); /* Green */
                printf("<%u> New target : [%lu](%d, %d)\n", m_targets.back().m_id, m_lastFrameTick, rr->tl().x, rr->tl().y);
                printf("\033[0m"); /* Default color */
#endif
            }
        }

        m_newTargetsHistory.push_back(newTargetList);
        if(m_newTargetsHistory.size() >= 90) {
            m_newTargetsHistory.pop_front();
        }

        if(m_targets.size() > 1)
            m_targets.sort(TargetSortByArea);
    }

    inline list< Target > & TargetList() { return m_targets; }
    inline list< list< Rect > > & NewTargetHistory() { return m_newTargetsHistory; }
};

static Tracker tracker(CAMERA_WIDTH, CAMERA_HEIGHT);

/*
*
*/

class FrameQueue
{
public:
    struct cancelled {};

public:
    FrameQueue() : isCancelled(false), refCnt(0) {}

    void push(Mat const & image);
    void pop();
    const Mat & front();

    void cancel();
    void reset();

private:
    std::queue<cv::Mat> matQueue;
    std::mutex matMutex;
    std::condition_variable condEvent;
    bool isCancelled;

    std::atomic_long refCnt;
};

void FrameQueue::cancel()
{
    std::unique_lock<std::mutex> mlock(matMutex);
    isCancelled = true;
    condEvent.notify_all();
}

void FrameQueue::push(cv::Mat const & image)
{
    while(matQueue.size() >= 3) /* Prevent memory overflow ... */
        return; /* Drop frame */

    std::unique_lock<std::mutex> mlock(matMutex);
    matQueue.push(image);
    condEvent.notify_all();
}

void FrameQueue::pop()
{
    std::unique_lock<std::mutex> mlock(matMutex);

    while(matQueue.empty()) {
        if (isCancelled) {
            throw cancelled();
        }
        condEvent.wait(mlock);
        if (isCancelled) {
            throw cancelled();
        }
    }

    if(--refCnt == 0)
        matQueue.pop();
}

const Mat & FrameQueue::front()
{
    std::unique_lock<std::mutex> mlock(matMutex);

    while (matQueue.empty()) {
        if (isCancelled) {
            throw cancelled();
        }
        condEvent.wait(mlock);
        if (isCancelled) {
            throw cancelled();
        }
    }

    ++refCnt;

    return matQueue.front();
}

void FrameQueue::reset()
{
    isCancelled = false;
}

/*
*
*/

static FrameQueue videoOutputQueue;

/*
*
*/

typedef struct {
    int numberFrames;
    GstClockTime timestamp;
} RtspServerContext;

/* called when we need to give data to appsrc */
static void
need_data (GstElement * appsrc, guint unused, RtspServerContext *ctx)
{
    GstBuffer *buffer;
    uint64_t size=CAMERA_WIDTH * CAMERA_HEIGHT * 4; // Image size * deth of BGRx;
    GstFlowReturn ret;
    buffer = gst_buffer_new_allocate (NULL, size, NULL);
    GstMapInfo map;
    gint8 *raw;

    gst_buffer_map (buffer, &map, GST_MAP_WRITE); // make buffer writable
    raw = (gint8 *)map.data;

//cout << __PRETTY_FUNCTION__ << ":" << __LINE__ << endl;
    const Mat & lastFrame = videoOutputQueue.front();

    for (int i=0;i<CAMERA_HEIGHT;i++) {
        const Vec3b* ptr = lastFrame.ptr<Vec3b>(i);
        for (int j = 0; j<CAMERA_WIDTH; j++) {
            uint64_t offset = ((i*CAMERA_WIDTH)+j)*4;
            raw[offset] = ptr[j][0];
            raw[offset+1] = ptr[j][1];
            raw[offset+2] = ptr[j][2];
            raw[offset+3] = 127;
        }
    }

    videoOutputQueue.pop();

    gst_buffer_unmap (buffer, &map);

    /* increment the timestamp every 1/FPS second */
    GST_BUFFER_PTS (buffer) = ctx->timestamp;
    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 30);
    ctx->timestamp += GST_BUFFER_DURATION (buffer);

    g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref (buffer);
}

/* called when a new media pipeline is constructed. CAMERA_WIDTHe can query the
 * pipeline and configure our appsrc */
static void
media_configure (GstRTSPMediaFactory * factory, GstRTSPMedia * media, gpointer user_data)
{
    // should be incremented once on each frame for timestamping

    GstElement *element, *appsrc;
    RtspServerContext *ctx;

    /* get the element used for providing the streams of the media */
    element = gst_rtsp_media_get_element (media);

    /* get our appsrc, we named it 'mysrc' with the name property */
    appsrc = gst_bin_get_by_name_recurse_up (GST_BIN (element), "mysrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg (G_OBJECT (appsrc), "format", "time");
    /* configure the caps of the video */
    g_object_set (G_OBJECT (appsrc), "caps",
    gst_caps_new_simple ("video/x-raw",
                "format", G_TYPE_STRING, "BGRx",
                "width", G_TYPE_INT, CAMERA_WIDTH,
                "height", G_TYPE_INT, CAMERA_HEIGHT,
                "framerate", GST_TYPE_FRACTION, 30, 1, NULL), NULL);

    ctx = g_new0 (RtspServerContext, 1);
    ctx->timestamp = 0;
    ctx->numberFrames = 0;

    /* make sure ther datais freed when the media is gone */
    g_object_set_data_full (G_OBJECT (media), "my-extra-data", ctx, (GDestroyNotify) g_free);

    /* install the callback that will be called when a buffer is needed */
    g_signal_connect (appsrc, "need-data", (GCallback) need_data, ctx);
    gst_object_unref (appsrc);
    gst_object_unref (element);
}

#include <glib-object.h>
#include <glib-unix.h>

// This callback will be inovked when this process receives SIGHUP.
gboolean HangupSignalCallback(gpointer data) {
  cout << "Received a signal to terminate the daemon";
  GMainLoop* loop = reinterpret_cast<GMainLoop*>(data);
  g_main_loop_quit(loop);
  // This function can return false to remove this signal handler as we are
  // quitting the main loop anyway.

  return false;
}

int gst_rtsp_server_task()
{
    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    char *args[] = {
        (char*)"gst-rtsp-server",
        NULL
    };
    int argv = 0;
    gst_init (&argv, (char ***)&args);

    loop = g_main_loop_new (NULL, FALSE);

    // Set up a signal handler for handling SIGHUP.
    g_unix_signal_add(SIGHUP, HangupSignalCallback, loop);
    
    /* create a server instance */
    server = gst_rtsp_server_new ();
    /* get the mount points for this server, every server has a default object
    * that be used to map uri mount points to media factories */
    mounts = gst_rtsp_server_get_mount_points (server);

    /* make a media factory for a test stream. The default media factory can use
    * gst-launch syntax to create pipelines.
    * any launch line works as long as it contains elements named pay%d. Each
    * element with pay%d names will be a stream */
    factory = gst_rtsp_media_factory_new ();
    gst_rtsp_media_factory_set_launch (factory,
//          "( appsrc name=mysrc is-live=true ! videoconvert ! omxh265enc ! rtph265pay mtu=1400 name=pay0 pt=96 )");
        "appsrc name=mysrc is-live=true ! videoconvert ! \
nvvidconv flip-method=1 ! omxh265enc control-rate=2 bitrate=1000000 ! rtph265pay mtu=1400 name=pay0 pt=96 )");

    gst_rtsp_media_factory_set_shared (factory, TRUE);
    /* notify when our media is ready, This is called whenever someone asks for
    * the media and a new pipeline with our appsrc is created */
    g_signal_connect (factory, "media-configure", (GCallback) media_configure, NULL);

    /* attach the test factory to the /test url */
    gst_rtsp_mount_points_add_factory (mounts, "/test", factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref (mounts);

    /* attach the server to the default maincontext */
    gst_rtsp_server_attach (server, NULL);

    /* start serving */
    g_print ("stream ready at rtsp://127.0.0.1:8554/test\n");
    g_main_loop_run (loop);

    return 0;
}

static thread rtspServerThread;

/*
*
*/

void VideoOutputTask(BaseType_t baseType, bool isVideoOutputScreen, bool isVideoOutputFile, 
    bool isVideoOutputRTP, const char *rtpRemoteHost, uint16_t rtpRemotePort, 
    bool isVideoOutputHLS, bool isVideoOutputRTSP, int width, int height)
{    
    Size videoSize = Size((int)width,(int)height);
    char gstStr[STR_SIZE];

    VideoWriter outFile;
    VideoWriter outScreen;
    VideoWriter outRTP;
    VideoWriter outHLS;
    VideoWriter outRTSP;

    int videoOutoutIndex = 0;

    if(isVideoOutputFile) {       
        char filePath[64];
        while(videoOutoutIndex < VIDEO_OUTPUT_MAX_FILES) {
            snprintf(filePath, 64, "%s/%s%c%03d.mkv", VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
            FILE *fp = fopen(filePath, "rb");
            if(fp) { /* file exist ... */
                fclose(fp);
                ++videoOutoutIndex;
            } else
                break; /* File doesn't exist. OK */
        }
        if(videoOutoutIndex == VIDEO_OUTPUT_MAX_FILES)
            videoOutoutIndex = 0; /* Loop */
        /* Countclockwise rote 90 degree - nvvidconv flip-method=1 */
        snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=1 ! omxh265enc preset-level=3 bitrate=8000000 ! matroskamux ! filesink location=%s/%s%c%03d.mkv ", 
            30, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#if 0 /* Always start from index 0 */
        /* 90 secs duration, maximum 100 files */
        snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
                   videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
                   nvvidconv flip-method=1 ! omxh265enc preset-level=3 bitrate=8000000 ! \
                   splitmuxsink muxer=matroskamux sink=filesink location=%s/%s%c%%03d.mkv max-size-time=90000000000 max-files=100 async-finalize=true async-handling=true ", 
            30, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B');
#endif
#if 0 /* NOT work, due to tee */
        snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=1 ! omxh265enc control-rate=2 bitrate=4000000 ! \
tee name=t \
t. ! queue ! matroskamux ! filesink location=%s/%s%c%03d.mkv  \
t. ! queue ! video/x-h265, stream-format=byte-stream ! h265parse ! rtph265pay mtu=1400 ! \
udpsink host=224.1.1.1 port=5000 auto-multicast=true sync=false async=false ",
            30, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#endif
        outFile.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), 30, videoSize);
        cout << endl;
        cout << gstStr << endl;
        cout << endl;
        cout << "*** Start record video ***" << endl;
    }

    if(isVideoOutputScreen) {
        snprintf(gstStr, STR_SIZE, "appsrc is-live=true ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv ! video/x-raw(memory:NVMM) ! \
nvoverlaysink sync=false ", 30);
        outScreen.open(gstStr, VideoWriter::fourcc('I', '4', '2', '0'), 30, Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        cout << endl;
        cout << gstStr << endl;
        cout << endl;
        cout << "*** Start display video ***" << endl;
    }

    if(isVideoOutputRTP && rtpRemoteHost) {
#undef MULTICAST_RTP
#ifdef MULTICAST_RTP /* Multicast RTP does NOT work with wifi due to low speed ... */
        snprintf(gstStr, STR_SIZE, "appsrc is-live=true ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=1 ! omxh265enc control-rate=2 bitrate=4000000 ! video/x-h265, stream-format=byte-stream ! \
h265parse ! rtph265pay mtu=1400 ! udpsink host=224.1.1.1 port=%u auto-multicast=true sync=false async=false ",
            30, rtpRemotePort);
#else
        snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=1 ! omxh265enc control-rate=2 bitrate=4000000 ! video/x-h265, stream-format=byte-stream ! \
h265parse ! rtph265pay mtu=1400 config-interval=10 pt=96 ! udpsink host=%s port=%u sync=false async=false ",
            30, rtpRemoteHost, rtpRemotePort);
#endif
        outRTP.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), 30, Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        cout << endl;
        cout << gstStr << endl;
        cout << endl;
        cout << "*** Start RTP video ***" << endl;        
    }

    if(isVideoOutputHLS) {
        snprintf(gstStr, STR_SIZE, "appsrc is-live=true ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=1 ! omxh264enc control-rate=2 bitrate=4000000 ! h264parse ! mpegtsmux ! \
hlssink playlist-location=/tmp/playlist.m3u8 location=/tmp/segment%%05d.ts target-duration=1 max-files=10 ", 30);
        outHLS.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), 30, Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        cout << endl;
        cout << gstStr << endl;
        cout << endl;
        cout << "*** Start HLS video ***" << endl;        
    }

    if(isVideoOutputRTSP) {
        rtspServerThread = thread(&gst_rtsp_server_task);
        cout << endl;
        cout << "*** Start RTSP video ***" << endl;        
    }

    videoOutputQueue.reset();

    steady_clock::time_point t1 = steady_clock::now();

    try {
        while(1) {
            //Mat frame = videoOutputQueue.pop();
            const Mat & frame = videoOutputQueue.front();

            if(isVideoOutputFile) {
                outFile.write(frame);

                steady_clock::time_point t2 = steady_clock::now();
                double secs(static_cast<double>(duration_cast<seconds>(t2 - t1).count()));

                if(secs >= VIDEO_FILE_OUTPUT_DURATION) { /* Reach duration limit, stop record video */
                    cout << endl;
                    cout << "*** Stop record video ***" << endl;
                    outFile.release();

                    if(videoOutoutIndex < VIDEO_OUTPUT_MAX_FILES)
                        ++videoOutoutIndex;
                    else
                        videoOutoutIndex = 0; /* loop */

                    snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=1 ! omxh265enc preset-level=3 bitrate=8000000 ! matroskamux ! filesink location=%s/%s%c%03d.mkv ", 
                        30, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);

                    outFile.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), 30, videoSize);
                    cout << endl;
                    cout << gstStr << endl;
                    cout << endl;
                    cout << "*** Start record video ***" << endl;

                    t1 = steady_clock::now();
                }
            }

            if(isVideoOutputScreen)
                outScreen.write(frame);
            if(isVideoOutputRTP)
                outRTP.write(frame);
            if(isVideoOutputHLS)
                outHLS.write(frame);
            //if(isVideoOutputRTSP)
            //    outRTSP.write(frame);

            videoOutputQueue.pop();
        }
    } catch (FrameQueue::cancelled & /*e*/) {
        // Nothing more to process, we're done
        if(isVideoOutputFile) {
            cout << endl;
            cout << "*** Stop record video ***" << endl;
            outFile.release();
        }
        if(isVideoOutputScreen) {
            cout << endl;
            cout << "*** Stop display video ***" << endl;
            outScreen.release();
        }
        if(isVideoOutputRTP) {
            cout << endl;
            cout << "*** Stop RTP video ***" << endl;
            outRTP.release();
        }
        if(isVideoOutputHLS) {
            cout << endl;
            cout << "*** Stop HLS video ***" << endl;
            outHLS.release();
        }
        if(isVideoOutputRTSP) {
            cout << endl;
            cout << "*** Stop RTSP video ***" << endl;
            //outRTSP.release();
            kill(getpid(), SIGHUP);
            if(rtspServerThread.joinable())
                rtspServerThread.join();
        }
    }    
}

/*
*
*/

static bool IsValidateIpAddress(const string & ipAddress)
{
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ipAddress.c_str(), &(sa.sin_addr));
    return result != 0;
}

static void ParseConfigString(string & line, vector<pair<string, string> > & cfg)
{
    auto delimiterPos = line.find("=");
    auto name = line.substr(0, delimiterPos);
    auto value = line.substr(delimiterPos + 1);

    name = std::regex_replace(name, std::regex(" +$"), ""); /* Remove tail space */
    value = std::regex_replace(value, std::regex("^ +"), ""); /* Remove leading space */

    //cfg.insert(pair<string, string>(name, value));
    cfg.push_back(make_pair(name, value));
}

static size_t ParseConfigFile(char *file, vector<pair<string, string> > & cfg)
{
    ifstream input(file);
    if(input.is_open()) {
        for(string line; getline( input, line ); ) {
            //line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
            line = std::regex_replace(line, std::regex("^ +| +$|( ) +"), "$1"); /* Strip leading & tail space */
            if(line[0] == '#' || line.empty())
                    continue;
            ParseConfigString(line, cfg);
        }
        input.close();
    }
    return cfg.size();
}

static size_t ParseConfigStream(istringstream & iss, vector<pair<string, string> > & cfg)
{
    string line;
    while(getline(iss, line)) {
        line = std::regex_replace(line, std::regex("^ +| +$|( ) +"), "$1"); /* Strip leading & tail space */
        if(line[0] == '#' || line.empty())
            continue;
        ParseConfigString(line, cfg);
    }
    return cfg.size();
}

/*
*
*/

class Camera {
private:
    VideoCapture cap;
    char gstStr[STR_SIZE];

public:
    int sensor_id = 0;
    int wbmode = 0;
    int tnr_mode = 1;
    float tnr_strength = -1;
    int ee_mode = 1;
    float ee_strength = -1;
    string gainrange; /* Default null */
    string ispdigitalgainrange; /* Default null */
    string exposuretimerange; /* Default null */
    float exposurecompensation = 0;
    int exposurethreshold = 5;
//    int width, height;

    Camera() : sensor_id(0), wbmode(0), tnr_mode(1), tnr_strength(-1), ee_mode(1), ee_strength(-1), 
        gainrange("1 16"), ispdigitalgainrange("1 8"), exposuretimerange("5000000 10000000"),
        exposurecompensation(0), exposurethreshold(5) {
    }

    bool Open() {
        if(cap.isOpened())
            return true;
/* Reference : nvarguscamerasrc.txt */
/* export GST_DEBUG=2 to show debug message */
#if 0
    snprintf(gstStr, STR_SIZE, "nvarguscamerasrc sensor-id=0 wbmode=0 tnr-mode=2 tnr-strength=1 ee-mode=1 ee-strength=0 gainrange=\"1 16\" ispdigitalgainrange=\"1 8\" exposuretimerange=\"5000000 10000000\" exposurecompensation=0 ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
        CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
#else
        snprintf(gstStr, STR_SIZE, "nvarguscamerasrc sensor-id=%d wbmode=%d tnr-mode=%d tnr-strength=%f ee-mode=%d ee-strength=%f gainrange=%s ispdigitalgainrange=%s exposuretimerange=%s exposurecompensation=%f ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
            sensor_id, wbmode, tnr_mode, tnr_strength, ee_mode, ee_strength, gainrange.c_str(), ispdigitalgainrange.c_str(), exposuretimerange.c_str(), exposurecompensation,
            CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
#endif
/*
        snprintf(gstStr, STR_SIZE, "v4l2src device=/dev/video1 ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
            CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
*/
        cout << endl;
        cout << gstStr << endl;
        cout << endl;

        if(!cap.open(gstStr, cv::CAP_GSTREAMER)) {
            cout << endl;
            cout << "!!! Could not open video" << endl;
            return false;
        }

        return true;
    }

    void Close() {
        if(cap.isOpened())
            cap.release();
    }

    inline bool Read(OutputArray & a) { 
        bool r = cap.read(a);
        //cout << "timestamp : " << cap.get(CAP_PROP_POS_MSEC) << endl;
        return r;
    }

    void ApplyConfig(vector<pair<string, string> > & cfg)
    {
        cout << endl;
        cout << "### Camera config" << endl; 
        vector<pair<string, string> >::iterator it;
        for (it=cfg.begin(); it!=cfg.end(); it++) {
            cout << it->first << " = " << it->second << endl;
            if(it->first == "sensor-id")
                sensor_id = stoi(it->second);
            else if(it->first == "wbmode")
                wbmode = stoi(it->second);
            else if(it->first == "tnr-mode")
                tnr_mode = stoi(it->second);
            else if(it->first == "tnr-strength")
                tnr_strength = stof(it->second);
            else if(it->first == "ee-mode")
                ee_mode = stoi(it->second);
            else if(it->first == "ee-strength")
                ee_strength = stof(it->second);
            else if(it->first == "gainrange")
                gainrange = it->second;
            else if(it->first == "ispdigitalgainrange")
                ispdigitalgainrange = it->second;
            else if(it->first == "exposuretimerange")
                exposuretimerange = it->second;
            else if(it->first == "exposurecompensation")
                exposurecompensation = stof(it->second);
            else if(it->first == "exposurethreshold")
                exposurethreshold = stoi(it->second);
        }
        cout << endl;
    }

    void LoadConfig() {
        char str[STR_SIZE];
        snprintf(str, STR_SIZE, "%s/camera.config", CONFIG_FILE_DIR);
        vector<pair<string, string> > cfg;
        ParseConfigFile(str, cfg);
        ApplyConfig(cfg);
    }

    void SaveConfig(string s) {
        char fn[STR_SIZE];
        snprintf(fn, STR_SIZE, "%s/camera.config", CONFIG_FILE_DIR);
        ofstream out(fn);
        if(out.is_open()) {
            out << s;
            out.close();
        }
    }
#if 0
    bool UpdateExposure() {
        bool isCameraOpened = cap.isOpened(); /* Used to keep camera open / close status */

        const char *exposureTimeRange[5] = {
            "\"5000000 10000000\"",
            "\"3000000 8000000\"",
            "\"1000000 5000000\"",
            "\"500000 2000000\"",
            "\"250000 1000000\""
        };

        if(exposurethreshold >= 255)
            return true;

        if(isCameraOpened)
            cap.release();

        vector<pair<string, float> > exposure_brightness;

        for(int i=0;i<5;i++) {
            snprintf(gstStr, STR_SIZE, "nvarguscamerasrc sensor-id=%d wbmode=%d tnr-mode=%d tnr-strength=%d ee-mode=%d ee-strength=%d gainrange=%s ispdigitalgainrange=%s exposuretimerange=%s exposurecompensation=%d ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
                sensor_id, wbmode, tnr_mode, tnr_strength, ee_mode, ee_strength, gainrange.c_str(), ispdigitalgainrange.c_str(), exposureTimeRange[i], exposurecompensation,
                CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);

            cout << endl;
            cout << gstStr << endl;
            cout << endl;

            if(!cap.open(gstStr, cv::CAP_GSTREAMER)) {
                cout << endl;
                cout << "!!! Could not open video" << endl;
                return false;
            }

            Mat capFrame;
            int meanCount = 30;
            while(meanCount-- > 0) { /* Drop first 30 frames */
                cap.read(capFrame);
            }

            meanCount = 3;
            float meanValue = 0;
            while(meanCount-- > 0) {
                cap.read(capFrame);
                Mat grayFrame;
                cvtColor(capFrame, grayFrame, COLOR_BGR2GRAY);
                Scalar v = mean(grayFrame);
                meanValue += v.val[0];
            }
            meanValue /= 3;
            
            if(isCameraOpened == false)
                cap.release();

            //cout << "Exposure time range - " << exposureTimeRange[i] << " : Brightness - " << meanValue << endl;
            exposure_brightness.push_back(make_pair(exposureTimeRange[i], meanValue));
        }

        vector<pair<string, float> >::iterator it;
        for (it=exposure_brightness.begin(); it!=exposure_brightness.end(); it++) {
            cout << "Exposure time range - " << it->first << " : Brightness - " << it->second << endl;
        }

        for (it=exposure_brightness.begin(); it!=exposure_brightness.end(); it++) {
            if(it->second <= exposurethreshold) {
                exposuretimerange = it->first;
                cout << endl;
                cout << "### Set exposure time range - " << it->first <<  endl;
                break;
            }
        }

        return true;
    }
#else
    void UpdateExposure() {
        const char *exposureTimeRange[6] = {
            "\"125000 500000\"",
            "\"250000 1000000\"",
            "\"500000 2000000\"",
            "\"1000000 5000000\"",
            "\"3000000 8000000\"",
            "\"5000000 10000000\"",
        };

        if(exposurethreshold >= 0 && exposurethreshold <= 5) {
            exposuretimerange = exposureTimeRange[exposurethreshold];
        }
    }
#endif
};

static Camera camera; 

/*
*
*/

#define WLAN_STA    "wlan0"
#define WLAN_AP     "wlan9"

class F3xBase {
private:
    int m_ttyFd, m_udpSocket, m_apMulticastSocket, m_staMulticastSocket;
    BaseType_t m_baseType;

    jetsonNanoGPIONumber m_redLED, m_greenLED, m_blueLED, m_relay;
    jetsonNanoGPIONumber m_pushButton;

    bool m_isVideoOutputScreen;
    bool m_isVideoOutputFile;
    bool m_isVideoOutputRTP;
    bool m_isVideoOutputHLS;
    bool m_isVideoOutputRTSP;
    bool m_isVideoOutputResult;

    string m_udpRemoteHost;
    uint16_t m_udpRemotePort;

    uint16_t m_udpLocalPort;

    string m_rtpRemoteHost;
    uint16_t m_rtpRemotePort;

    uint8_t m_mog2_threshold; /* 0 ~ 64 / Most senstive is 0 / Default 16 */
    bool m_isNewTargetRestriction;
    bool m_isFakeTargetDetection;
    bool m_isBugTrigger;

    thread m_udpServerThread;
    bool m_bUdpServerRun;

    unsigned int m_srcIp;
    unsigned short m_srcPort;

    thread m_multicastSenderThread;
    bool m_bMulticastSenderRun;

    std::queue<uint8_t> m_triggerQueue;
    thread m_triggerThread;

    int SetupTTY(int fd, int speed, int parity) {
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

    void TriggerTtyTask() {
        while(m_ttyFd) {
            if(m_triggerQueue.size() > 0) {
                auto data = m_triggerQueue.front();
                m_triggerQueue.pop();
                write(m_ttyFd, &data, sizeof(data));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

public:
    F3xBase() : m_ttyFd(0), m_udpSocket(0), m_apMulticastSocket(0), m_staMulticastSocket(0), m_baseType(BASE_A),
        m_redLED(gpio16), m_greenLED(gpio17), m_blueLED(gpio50), m_relay(gpio51), m_pushButton(gpio18),
        m_isVideoOutputScreen(false), 
        m_isVideoOutputFile(false), 
        m_isVideoOutputRTP(false), 
        m_isVideoOutputHLS(false),
        m_isVideoOutputRTSP(false),
        m_isVideoOutputResult(false),
        m_udpRemotePort(4999), m_udpLocalPort(4999), m_rtpRemotePort(5000),
        m_mog2_threshold(16),
        m_isNewTargetRestriction(false),
        m_isFakeTargetDetection(false),
        m_isBugTrigger(false),
        m_bUdpServerRun(false),
        m_srcIp(0), m_srcPort(0),
        m_bMulticastSenderRun(false) {
    }

    ~F3xBase() {
        if(m_udpServerThread.joinable())
            m_udpServerThread.detach();
    }

    void SetupGPIO() {
        /* 
        * Do enable GPIO by /etc/profile.d/export-gpio.sh 
        */

        /* Output */
        gpioExport(m_redLED);
        gpioExport(m_greenLED);
        gpioExport(m_blueLED);
        gpioExport(m_relay);

        gpioSetDirection(m_redLED, outputPin); /* Flash during object detected */
        gpioSetValue(m_redLED, off);

        gpioSetDirection(m_greenLED, outputPin); /* Flash during frames */
        gpioSetValue(m_greenLED, on);

        gpioSetDirection(m_blueLED, outputPin); /* Flash during file save */
        gpioSetValue(m_blueLED, off);

        gpioSetDirection(m_relay, outputPin); /* */
        gpioSetValue(m_relay, off);

        /* Input */
        gpioExport(m_pushButton);
        gpioSetDirection(m_pushButton, inputPin); /* Pause / Restart */

    #if 0
        gpioSetEdge(m_pushButton, "rising");
        int gfd = gpioOpen(m_pushButton);
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

    int OpenTty() { /* Try ttyUSB0 first then ttyTHS1 */
        const char *ttyUSB0 = "/dev/ttyUSB0";
        const char *ttyTHS1 = "/dev/ttyTHS1";

        m_ttyFd = open(ttyUSB0, O_RDWR | O_NOCTTY | O_SYNC);
        if(m_ttyFd > 0) {
            SetupTTY(m_ttyFd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
            printf("Open %s successful ...\n", ttyUSB0);
        } else {
            printf("Error %d opening %s: %s\n", errno, ttyUSB0, strerror (errno));
            m_ttyFd = open (ttyTHS1, O_RDWR | O_NOCTTY | O_SYNC);
            if(m_ttyFd > 0) {
                SetupTTY(m_ttyFd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
                printf("Open %s successful ...\n", ttyTHS1);
            } else
                printf("Error %d opening %s: %s\n", errno, ttyTHS1, strerror (errno));
        }

        m_triggerThread = thread(&F3xBase::TriggerTtyTask, this);

        return m_ttyFd;
    }

    int ReadTty(uint8_t *data, size_t size) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(m_ttyFd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        if(data == 0 || size == 0)
            return 0;
        if(select(m_ttyFd+1, &rfds, NULL, NULL, &tv) > 0) {
            int r = read(m_ttyFd, data, size); /* Receive trigger from f3f timer */
            if(r == size)
                return 1;
        }
        return 0;
    }

    void TriggerTty(bool newTrigger) {
        static uint8_t serNo = 0x3f;
        uint8_t data;

        if(!m_ttyFd)
            return;

        if(newTrigger) { /* It's NEW trigger */
            if(++serNo > 0x3f)
                serNo = 0;
        }

        if(m_baseType == BASE_A) {
            data = (serNo & 0x3f);
            printf("TriggerTty : BASE_A[%d]\r\n", serNo);
        } else if(m_baseType == BASE_B) {
            data = (serNo & 0x3f) | 0x40;
            printf("TriggerTty : BASE_B[%d]\r\n", serNo);
        }        
#if 0
        write(m_ttyFd, &data, 1);
#else
        m_triggerQueue.push(data);
#endif
    }

    void CloseTty() {
        if(m_ttyFd)
            close(m_ttyFd);
        m_ttyFd = 0;

        if(m_triggerThread.joinable())
            m_triggerThread.join();
    }

    int OpenUdpSocket() {
        int sockfd;
        struct sockaddr_in addr; 

        if(m_udpRemoteHost.empty()) {
            printf("Invalid UDP host !!!\n");
            return 0;
        }

        sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(sockfd < 0) {
            printf("Error open UDP socket !!!\n");
            return 0;
        }

        memset((char*) &(addr),0, sizeof((addr)));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(m_udpLocalPort);

        if(bind(sockfd,(struct sockaddr*)&addr, sizeof(addr)) != 0) {
            switch(errno) {
            case 0:
                printf("Could not bind socket\n");
            case EADDRINUSE:
                printf("Port %u for receiving UDP is in use\n", m_udpLocalPort);
                break;
            case EADDRNOTAVAIL:
                printf("Cannot assign requested address\n");
                break;
            default:
                printf("Could not bind UDP receive port : Error %s\n", strerror(errno));
                break;            
            }
            return 0;
        }

        m_udpSocket = sockfd;

        printf("Open UDP socket successful ...\n");

        return m_udpSocket;
    }

    size_t ReadUdpSocket(uint8_t *data, size_t size, unsigned int timeoutMs) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(m_udpSocket, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = timeoutMs * 1000;

        if(data == 0 || size == 0)
            return 0;

        if(select(m_udpSocket+1, &rfds, NULL, NULL, &tv) <= 0)
            return 0; /* timeout */ 

        if(FD_ISSET(m_udpSocket, &rfds) <= 0)
            return 0;
        
        struct sockaddr_in from;
        int fromLen = sizeof(from);
        size_t originalSize = size;
        memset(data, 0, size);
#if 1
        int r = recvfrom(m_udpSocket,
               data,
               originalSize,
               0,
               (struct sockaddr *)&from,
               (socklen_t*)&fromLen);
#else
        int r = recv(m_udpSocket, data, originalSize, 0);
#endif
        if(r > 0) {
            m_srcPort = ntohs(from.sin_port);
            m_srcIp = ntohl(from.sin_addr.s_addr);

            return r;
        }
        else if(r == -1) {
            switch(errno) {
               case ENOTSOCK:
                  printf("Error fd not a socket\n");
                  break;
               case ECONNRESET:
                  printf("Error connection reset - host not reachable\n");
                  break;              
               default:
                  printf("Socket Error : %d\n", errno);
                  break;
            }
        }
        
        return 0;
    }

    size_t WriteUdpSocket(const uint8_t *data, size_t size) {
        if(!m_udpSocket)
            return 0;

        struct sockaddr_in to;
        int toLen = sizeof(to);
        memset(&to, 0, toLen);
        
        struct hostent *server;
        server = gethostbyname(m_udpRemoteHost.c_str());

        to.sin_family = AF_INET;
        to.sin_port = htons(m_udpRemotePort);
        to.sin_addr = *((struct in_addr *)server->h_addr);
        
        //printf("Write to %s:%u ...\n", m_udpRemoteHost.c_str(), m_udpRemotePort);
        
        int s = sendto(m_udpSocket, data, size, 0,(struct sockaddr*)&to, toLen);

        return s;
    }

    size_t WriteSourceUdpSocket(const uint8_t *data, size_t size) {
        if(!m_udpSocket || m_srcIp == 0 || m_srcPort == 0)
            return 0;

        struct sockaddr_in to;
        int toLen = sizeof(to);
        memset(&to, 0, toLen);
        
        to.sin_family = AF_INET;
        to.sin_port = htons(m_srcPort);
        to.sin_addr.s_addr = htonl(m_srcIp);
        
        //printf("Write to %s:%u ...\n", m_udpRemoteHost.c_str(), m_udpRemotePort);
        
        int s = sendto(m_udpSocket, data, size, 0,(struct sockaddr*)&to, toLen);

        return s;
    }
#if 0
    void TriggerUdpSocket(bool newTrigger) {
        static uint8_t serNo = 0x3f;
        uint8_t data[1];

        if(!m_udpSocket)
            return;

        if(newTrigger) { /* It's NEW trigger */
            if(++serNo > 0x3f)
                serNo = 0;
        }
        if(m_baseType == BASE_A) {
            data[0] = (serNo & 0x3f);
            printf("BASE_A[%d]\r\n", serNo);            
        } else if(m_baseType == BASE_B) {
            data[0] = (serNo & 0x3f) | 0x40;
            printf("BASE_B[%d]\r\n", serNo);
        }        
        WriteUdpSocket(data, 1);
    }
#else
    void TriggerUdpSocket(bool newTrigger) {
        static uint8_t serNo = 0x3f;
        if(newTrigger) { /* It's NEW trigger */
            if(++serNo > 0x3f)
                serNo = 0;
        }
        //const char trigger[] = "#Trigger";

        string trigger("#Trigger:");
        trigger.append(std::to_string(serNo));
        WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(trigger.c_str()), trigger.size());
    }
#endif
    void CloseUdpSocket() {
        if(m_udpSocket)
            close(m_udpSocket);
        m_udpSocket = 0;
    }

    void UdpServerTask()
    {
        m_bUdpServerRun = true;
        OpenUdpSocket();
        while(m_bUdpServerRun) {
            if(bShutdown)
                break;
            uint8_t data[1024];
            size_t r = ReadUdpSocket(data, 1024, 20);
            if(r > 0) {
                string s = reinterpret_cast<char *>(data);
                istringstream iss(s);
                vector<pair<string, string> > cfg;
                string line;
                if(!getline(iss, line))
                    continue;
                line = std::regex_replace(line, std::regex("^ +| +$|( ) +"), "$1"); /* Strip leading & tail space */

                const char ack[] = "#Ack";
                const char started[] = "#Started";
                const char stopped[] = "#Stopped";

                if(line == "#SystemSettings") {
                    if(ParseConfigStream(iss, cfg) > 0) {
                        WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ack), strlen(ack));
                        ApplySystemConfig(cfg);
                        SaveSystemConfig(s.substr(16)); /* Strip cmd ahead */
                    } else { /* Request for system config */
                        char fn[STR_SIZE];
                        snprintf(fn, STR_SIZE, "%s/system.config", CONFIG_FILE_DIR);
                        FILE *fp = fopen(fn, "rb");
                        if(fp) {
                            fseek(fp, 0L, SEEK_END);
                            size_t sz = ftell(fp);
                            if(sz > 0) {
                                fseek(fp, 0L, SEEK_SET);
                                size_t cmd_size = strlen("#SystemSettings\n");
                                uint8_t *buf = (uint8_t *)malloc(cmd_size + sz);
                                strcpy((char *)buf, "#SystemSettings\n");
                                size_t r = fread(buf + cmd_size, 1, sz, fp);
                                if(r)
                                    WriteSourceUdpSocket(buf, cmd_size + sz);
                                free(buf);
                            }
                            fclose(fp);
                        }
                    }
                } else if(line == "#CameraSettings") {
                    if(ParseConfigStream(iss, cfg) > 0) {
                        WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ack), strlen(ack));
                        camera.ApplyConfig(cfg);
                        camera.SaveConfig(s.substr(16)); /* Strip cmd ahead */
                    }  else { /* Request for system config */
                        char fn[STR_SIZE];
                        snprintf(fn, STR_SIZE, "%s/camera.config", CONFIG_FILE_DIR);
                        FILE *fp = fopen(fn, "rb");
                        if(fp) {
                            fseek(fp, 0L, SEEK_END);
                            size_t sz = ftell(fp);
                            if(sz > 0) {
                                fseek(fp, 0L, SEEK_SET);
                                size_t cmd_size = strlen("#CameraSettings\n");
                                uint8_t *buf = (uint8_t *)malloc(cmd_size + sz);
                                strcpy((char *)buf, "#CameraSettings\n");
                                size_t r = fread(buf + cmd_size, 1, sz, fp);
                                if(r)
                                    WriteSourceUdpSocket(buf, cmd_size + sz);
                                free(buf);
                            }
                            fclose(fp);
                        }
                    }
                } else if(line == "#Start") {                    
                    WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(started), strlen(started));
                    evtQueue.push(EvtStart);
                } else if(line == "#Stop") {
                    WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(stopped), strlen(stopped));
                    evtQueue.push(EvtStop);
                } else if(line == "#Status") {
                    if(bStopped) /* Stopped */
                        WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(stopped), strlen(stopped));
                    else
                        WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(started), strlen(started));
                }
            }
        }
        CloseUdpSocket();
    }

    void StartUdpServer() {
        m_udpServerThread = thread(&F3xBase::UdpServerTask, this);
    }

    void StopUdpServer() {
        if(m_udpServerThread.joinable()) {
            m_bUdpServerRun = false;
            m_udpServerThread.join();
        }
    }

    int OpenMulticastSocket(const char *group, uint16_t port, const char *ifname) {
        printf("Open multicast socket %s : group %s:%u\n", ifname, group, port);

        if(group == 0 || strlen(group) == 0)
            return 0;
        if(ifname == 0 || strlen(ifname) == 0)
            return 0;

        char *ip = ipv4_address(ifname);
        if(ip == 0)
            return 0;

        int sockfd;

        sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP); /* Dummy protocol for TCP.  */
        if(sockfd < 0) {
            printf("Error open socket !!!\n");
            return 0;
        }

        struct sockaddr_in addr; 

        memset((char*) &(addr),0, sizeof((addr)));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if(bind(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0) {
            printf("Error bind socket !!!\n");
            close(sockfd);
            return 0;
        }

        int flag = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&flag, sizeof(flag));
        //setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (char*)&flag, sizeof(flag));
        unsigned char loop = 0;
        setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_LOOP, (const char*)&loop, sizeof(loop));
        unsigned char ttl = 255;
        setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_TTL, (const char*)&ttl, sizeof(ttl));

        struct ip_mreq mreq;
        // use setsockopt() to request that the kernel join a multicast group
        mreq.imr_multiaddr.s_addr = inet_addr(group);
        //mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        mreq.imr_interface.s_addr = inet_addr(ip); /* wlan9 ip address */

        setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_IF, (const char*)&mreq.imr_interface.s_addr, sizeof(struct sockaddr_in));
        setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char*)&mreq, sizeof(mreq));

        return sockfd; 
    }

    size_t WriteMulticastSocket(int sockfd, const char *group, uint16_t port, const char *ifname, const uint8_t *data, size_t size) {
        if(!sockfd)
            return 0;
        if(group == 0 || strlen(group) == 0)
            return 0;

        struct sockaddr_in addr; 

        memset((char*) &(addr),0, sizeof((addr)));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(group);
        addr.sin_port = htons(port);

        int ret = 0;
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        ifr.ifr_addr.sa_family = AF_INET;
        strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
        if(ioctl(sockfd, SIOCGIFFLAGS, &ifr) >= 0 &&
            (ifr.ifr_flags &IFF_UP)) {
            if(setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifname, strlen(ifname)) == -1) {
                printf("setsocketopt %s SO_BINDTODEVICE fail !!!\n", ifname);
            } else {
                int r = sendto(sockfd, data, size, 0,(struct sockaddr*)&addr, sizeof(addr));
                if(r < 0)
                    perror(ifname);
                else
                    ret = r;
            }
        }
    }

    void TriggerMulticastSocket(bool newTrigger) {
        static uint8_t serNo = 0x3f;
        if(newTrigger) { /* It's NEW trigger */
            if(++serNo > 0x3f)
                serNo = 0;
        }

        string raw;
        switch(m_baseType) {
            case BASE_A: raw.append("TRIGGER_A");
                printf("TriggerMulticastSocket : BASE_A[%d]\r\n", serNo);
                break;
            case BASE_B: raw.append("TRIGGER_B");
                printf("TriggerMulticastSocket : BASE_B[%d]\r\n", serNo);
                break;
            default:
                break;
        }
        raw.push_back(':');
        raw.append(std::to_string(serNo));

        if(m_apMulticastSocket)
            WriteMulticastSocket(m_apMulticastSocket, "224.0.0.2", 9002, WLAN_AP, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
        if(m_staMulticastSocket)
            WriteMulticastSocket(m_staMulticastSocket, "224.0.0.3", 9003, WLAN_STA, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
    }

    void MulticastSenderTask() {
        m_bMulticastSenderRun = true;

        m_apMulticastSocket = OpenMulticastSocket("224.0.0.2", 9002, WLAN_AP);
        m_staMulticastSocket = OpenMulticastSocket("224.0.0.3", 9003, WLAN_STA);

        while(m_bMulticastSenderRun) {
            if(bShutdown)
                break;

            char *ip = ipv4_address(WLAN_AP);
            if(ip && m_apMulticastSocket) {
                string raw("BASE_X");
                switch(m_baseType) {
                    case BASE_A: raw = "BASE_A";
                        break;
                    case BASE_B: raw = "BASE_B";
                        break;
                    default:
                        break;
                }
                raw.push_back(':');
                raw.append(ip);
                
                WriteMulticastSocket(m_apMulticastSocket, "224.0.0.2", 9002, WLAN_AP, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
            }

            ip = ipv4_address(WLAN_STA);
            if(ip && m_staMulticastSocket) {
                string raw("BASE_X");
                switch(m_baseType) {
                    case BASE_A: raw = "BASE_A";
                        break;
                    case BASE_B: raw = "BASE_B";
                        break;
                    default:
                        break;
                }
                raw.push_back(':');
                raw.append(ip);
                
                WriteMulticastSocket(m_staMulticastSocket, "224.0.0.3", 9003, WLAN_STA, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
            }

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        if(m_apMulticastSocket) {
            close(m_apMulticastSocket);
            m_apMulticastSocket = 0;
        }
        if(m_staMulticastSocket) {
            close(m_staMulticastSocket);
            m_staMulticastSocket = 0;
        }
    }

    void StartMulticastSender() {
        m_multicastSenderThread = thread(&F3xBase::MulticastSenderTask, this);
    }

    void StopMulticastSender() {
        if(m_multicastSenderThread.joinable()) {
            m_bMulticastSenderRun = false;
            m_multicastSenderThread.join();
        }
    }

    void ApplySystemConfig(vector<pair<string, string> > & cfg)
    {
        cout << endl;
        cout << "### System config" << endl; 
        vector<pair<string, string> >::iterator it;
        for (it=cfg.begin(); it!=cfg.end(); it++) {
            cout << it->first << " = " << it->second << endl;
            
            if(it->first == "base.type") {
                if(it->second == "A")
                    m_baseType = BASE_A;
                else if(it->second == "B")
                    m_baseType = BASE_B;
                else
                    m_baseType = BASE_UNKNOWN;
            } else if(it->first == "base.new.target.restriction") { /* triggle then reset target */
                if(it->second == "yes" || it->second == "1")
                    m_isNewTargetRestriction = true;
                else
                    m_isNewTargetRestriction = false;
            } else if(it->first == "base.fake.target.detection") { /* fake target detection */
                if(it->second == "yes" || it->second == "1")
                    m_isFakeTargetDetection = true;
                else
                    m_isFakeTargetDetection = false;                
            } else if(it->first == "base.bug.trigger") { /* bug trigger */
                if(it->second == "yes" || it->second == "1")
                    m_isBugTrigger = true;
                else
                    m_isBugTrigger = false;                
            } else if(it->first == "base.mog2.threshold") { /* Backgroung subtractor MOG2 threshold */
                string & s = it->second;
                if(::all_of(s.begin(), s.end(), ::isdigit)) {
                    int r = stoi(s);
                    if(r >= 0 && r <=64)
                        m_mog2_threshold = (r & 0xff);
                    else
                        cout << "Out of range " << it->first << "=" << s << endl;
                } else
                    cout << "Invalid " << it->first << "=" << s << endl;
            } else if(it->first == "base.udp.remote.host") {
                if(IsValidateIpAddress(it->second))
                    m_udpRemoteHost = it->second;
            } else if(it->first == "base.udp.remote.port") {
                string & s = it->second;
                if(::all_of(s.begin(), s.end(), ::isdigit))
                    m_udpRemotePort = stoi(s);
                else
                    cout << "Invalid " << it->first << "=" << s << endl;
            } else if(it->first == "base.rtp.remote.host") {
                if(IsValidateIpAddress(it->second))
                    m_rtpRemoteHost = it->second;
            } else if(it->first == "base.rtp.remote.port") {
                string & s = it->second;
                if(::all_of(s.begin(), s.end(), ::isdigit))
                    m_rtpRemotePort = stoi(s);
                else
                    cout << "Invalid " << it->first << "=" << s << endl;
            } else if(it->first == "video.output.screen") {
                if(it->second == "yes" || it->second == "1")
                    m_isVideoOutputScreen = true;
                else
                    m_isVideoOutputScreen = false;
            } else if(it->first == "video.output.file") {
                if(it->second == "yes" || it->second == "1")
                    m_isVideoOutputFile = true;
                else
                    m_isVideoOutputFile = false;
            } else if(it->first == "video.output.rtp") {
                if(it->second == "yes" || it->second == "1")
                    m_isVideoOutputRTP = true;
                else
                    m_isVideoOutputRTP = false;
            } else if(it->first == "video.output.hls") {
                if(it->second == "yes" || it->second == "1")
                    m_isVideoOutputHLS = true;
                else
                    m_isVideoOutputHLS = false;
            } else if(it->first == "video.output.rtsp") {
                if(it->second == "yes" || it->second == "1")
                    m_isVideoOutputRTSP = true;
                else
                    m_isVideoOutputRTSP = false;
            } else if(it->first == "video.output.result") {
                if(it->second == "yes" || it->second == "1")
                    m_isVideoOutputResult = true;
                else
                    m_isVideoOutputResult = false;
            }
        }
    }

    void LoadSystemConfig() {
        char fn[STR_SIZE];
        snprintf(fn, STR_SIZE, "%s/system.config", CONFIG_FILE_DIR);
        vector<pair<string, string> > cfg;
        if(ParseConfigFile(fn, cfg) > 0)
            ApplySystemConfig(cfg);
    }

    void SaveSystemConfig(string s) {
        char fn[STR_SIZE];
        snprintf(fn, STR_SIZE, "%s/system.config", CONFIG_FILE_DIR);
        ofstream out;
        out.open(fn);
        if(out.is_open()) {
            out << s;
            out.close();
        }
    }

    inline BaseType_t BaseType() const {
        return m_baseType;
    }

    inline bool IsVideoOutputScreen() const {
        return m_isVideoOutputScreen;
    }

    inline bool IsVideoOutputFile() const {
        return m_isVideoOutputFile;
    }

    inline bool IsVideoOutputRTP() const {
        return m_isVideoOutputRTP;
    }

    const char *RtpRemoteHost() {
        if(m_rtpRemoteHost.empty())
            return 0;
        return m_rtpRemoteHost.c_str();
    }

    inline uint16_t RtpRemotePort() {
        return m_rtpRemotePort;
    }

    inline bool IsVideoOutputHLS() const {
        return m_isVideoOutputHLS;
    }

    inline bool IsVideoOutputRTSP() const {
        return m_isVideoOutputRTSP;
    }

    const char *UdpRemoteHost() {
        if(m_udpRemoteHost.empty())
            return 0;
        return m_udpRemoteHost.c_str();
    }

    inline uint16_t UdpRemotePort() {
        return m_udpRemotePort;
    }

    inline bool IsVideoOutput() const {
        return (m_isVideoOutputScreen || m_isVideoOutputFile || m_isVideoOutputRTP || m_isVideoOutputHLS || m_isVideoOutputRTSP);
    }

    inline bool IsVideoOutputResult() const {
        return m_isVideoOutputResult;
    }

    inline uint8_t Mog2Threshold() const {
        return m_mog2_threshold;
    }

    inline bool IsNewTargetRestriction() const {
        return m_isNewTargetRestriction;
    }

    inline bool IsFakeTargetDetection() const {
        return m_isFakeTargetDetection;
    }

    inline bool IsBugTrigger() const {
        return m_isBugTrigger;
    }

    void RedLed(pinValues onOff) {
        gpioSetValue(m_redLED, onOff);
    }

    void GreenLed(pinValues onOff) {
        gpioSetValue(m_greenLED, onOff);
    }

    void BlueLed(pinValues onOff) {
        gpioSetValue(m_blueLED, onOff);
    }

    void Relay(pinValues onOff) {
if(onOff)
    printf("Relay %s\n", onOff ? "on" : "off");        
        gpioSetValue(m_relay, onOff);
    }

    unsigned int GetPushButton() {
        unsigned int gv;
        gpioGetValue(m_pushButton, &gv);
        return gv;
    }

    static void Start();
    static void Stop();
};

static F3xBase f3xBase; 

/*
*
*/

static Ptr<cuda::BackgroundSubtractorMOG2> bsModel;
#if 0
static Ptr<cuda::Filter> erodeFilter;
static Ptr<cuda::Filter> dilateFilter;
#else
static Mat elementErode;
static Mat elementDilate;
#endif

static thread videoOutputThread;

void F3xBase::Start()
{
    if(f3xBase.IsNewTargetRestriction())
        //tracker.NewTargetRestriction(Rect(cy - 200, cx - 200, 400, 200));
        tracker.NewTargetRestriction(Rect(0, 180, 180, 360));
    else
        tracker.NewTargetRestriction(Rect());
#if 0
    bsModel->setVarThreshold(fb.Mog2Threshold());
#else
    /* background history count, varThreshold, shadow detection */
    //Ptr<cuda::BackgroundSubtractorMOG2> bsModel = cuda::createBackgroundSubtractorMOG2(30, f3xBase.Mog2Threshold(), false); /* To anit grass wave ... */ 
    bsModel = cuda::createBackgroundSubtractorMOG2(30, f3xBase.Mog2Threshold(), false); /* To anit grass wave ... */ 
    //cout << bsModel->getVarInit() << " / " << bsModel->getVarMax() << " / " << bsModel->getVarMax() << endl;
    /* Default variance of each gaussian component 15 / 75 / 75 */ 
    bsModel->setVarInit(15);
    bsModel->setVarMax(20);
    bsModel->setVarMin(4);    
#endif
    camera.UpdateExposure();

    cout << endl;
    cout << "*** Object tracking started ***" << endl;

    camera.Open();

    Mat frame;
    for(int i=0;i<30;i++) /* Read out unstable frames ... */
        camera.Read(frame);

    if(f3xBase.IsVideoOutput()) {
        F3xBase & fb = f3xBase;
        videoOutputThread = thread(&VideoOutputTask, fb.BaseType(), fb.IsVideoOutputScreen(), fb.IsVideoOutputFile(), 
            fb.IsVideoOutputRTP(), fb.RtpRemoteHost(), fb.RtpRemotePort(), 
            fb.IsVideoOutputHLS(), fb.IsVideoOutputRTSP(),CAMERA_WIDTH, CAMERA_HEIGHT);
    }

    f3xBase.GreenLed(off);

    bStopped = false;
}

void F3xBase::Stop()
{
    f3xBase.GreenLed(on); /* On while pause */

    cout << endl;
    cout << "*** Object tracking stoped ***" << endl;
    
    if(f3xBase.IsVideoOutput()) {
        videoOutputQueue.cancel();
        if(videoOutputThread.joinable())
            videoOutputThread.join();
    }

    camera.Close();

    bStopped = true;
}

static void contour_moving_object(Mat & frame, Mat & foregroundFrame, list<Rect> & roiRect)
{
    uint32_t num_target = 0;

    vector< vector<Point> > contours;
    vector< Vec4i > hierarchy;
//  findContours(foregroundFrame, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    findContours(foregroundFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
#if 0
    sort(contours.begin(), contours.end(), [](vector<cv::Point> contour1, vector<cv::Point> contour2) {  
            //return (contour1.size() > contour2.size()); /* Outline length */
            return (cv::contourArea(contour1) > cv::contourArea(contour2)); /* Area */
        }); /* Contours sort by area, controus[0] is largest */
#endif
    vector<Rect> boundRect( contours.size() );
    
    for(int i=0; i<contours.size(); i++) {
        //approxPolyDP( Mat(contours[i]), contours[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours[i]) );
    }

    sort(boundRect.begin(), boundRect.end(), [](const Rect & r1, const Rect & r2) {  
            //return (contour1.size() > contour2.size()); /* Outline length */
            return (r1.area() > r2.area()); /* Area */
        }); /* Rects sort by area, boundRect[0] is largest */

    for(int i=0; i<boundRect.size(); i++) {
        //approxPolyDP( Mat(contours[i]), contours[i], 3, true );
        //boundRect[i] = boundingRect( Mat(contours[i]) );
        //drawContours(contoursImg, contours, i, color, 2, 8, hierarchy);
        if(boundRect[i].width > MAX_TARGET_WIDTH &&
            boundRect[i].height > MAX_TARGET_HEIGHT)
            continue; /* Extremely large object */

        if(boundRect[i].width < MIN_TARGET_WIDTH && 
            boundRect[i].height < MIN_TARGET_HEIGHT)
            break; /* Rest are small objects, ignore them */

#if 1 /* Anti cloud ... */
        double minVal; 
        double maxVal; 
        Point minLoc; 
        Point maxLoc;

        Mat roiFrame = frame(boundRect[i]);
        minMaxLoc(roiFrame, &minVal, &maxVal, &minLoc, &maxLoc ); 
            /* If difference of max and min value of ROI rect is too small then it could be noise such as cloud or sea */
        if((maxVal - minVal) < 24)
            continue; /* Too small, drop it. */
#endif
#if 1
        if(roiFrame.cols > roiFrame.rows && (roiFrame.cols >> 4) > roiFrame.rows)
            continue; /* Ignore thin object */
#endif                        
        roiRect.push_back(boundRect[i]);
        if(++num_target >= MAX_NUM_TARGET)
            break;
    }    
}

static void extract_moving_object(Mat & frame, list<Rect> & roiRect)
{
    Mat foregroundFrame;
    cuda::GpuMat gpuFrame;
    cuda::GpuMat gpuForegroundFrame;

    gpuFrame.upload(frame); 
    // pass the frame to background bsGrayModel
    bsModel->apply(gpuFrame, gpuForegroundFrame, 0.05);
    //cuda::threshold(gpuForegroundFrame, gpuForegroundFrame, 10.0, 255.0, THRESH_BINARY);
#if 0 /* Run with GPU */
    cuda::GpuMat gpuErodeFrame;
    cuda::GpuMat gpuDilateFrame;
    erodeFilter->apply(gpuForegroundFrame, gpuErodeFrame);
    dilateFilter->apply(gpuErodeFrame, gpuDilateFrame);
    gpuDilateFrame.download(foregroundFrame);
#else /* Run with CPU */
    gpuForegroundFrame.download(foregroundFrame);
    morphologyEx(foregroundFrame, foregroundFrame, MORPH_ERODE, elementErode);
    morphologyEx(foregroundFrame, foregroundFrame, MORPH_DILATE, elementDilate);
#endif
    contour_moving_object(frame, foregroundFrame, roiRect);
}

/*
*
*/

void sig_handler(int signo)
{
    if(signo == SIGINT) {
        printf("SIGINT\n");
        kill(getpid(), SIGHUP); /* To stop RTSP server */
        bShutdown = true;
    } 
}

/*
*
*/

#define PID_FILE "/var/run/dragon-eye.pid"

int main(int argc, char**argv)
{
    if(signal(SIGINT, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGINT\n");

    ofstream pf(PID_FILE); 
    if(pf) {
        pf << getpid();
        pf.close();
    } else
        cout << "Error open " << PID_FILE << endl;

    cuda::printShortCudaDeviceInfo(cuda::getDevice());
    std::cout << cv::getBuildInformation() << std::endl;

    f3xBase.LoadSystemConfig();
    f3xBase.SetupGPIO();
    f3xBase.OpenTty();
    f3xBase.StartUdpServer();
    f3xBase.StartMulticastSender();

    camera.LoadConfig();
    camera.UpdateExposure();

    int cx = CAMERA_WIDTH - 1;
    int cy = (CAMERA_HEIGHT / 2) - 1;
#if 0
    erodeFilter = cuda::createMorphologyFilter(MORPH_ERODE, CV_8UC1, elementErode);
    dilateFilter = cuda::createMorphologyFilter(MORPH_DILATE, CV_8UC1, elementDilate);
#else
    int erosion_size = 1;   
    elementErode = cv::getStructuringElement(cv::MORPH_RECT,
                    cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                    cv::Point(-1, -1) ); /* Default anchor point */

    int dilate_size = 2;   
    elementDilate = cv::getStructuringElement(cv::MORPH_RECT,
                    cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), 
                    cv::Point(-1, -1) ); /* Default anchor point */
#endif
#if 0
    /* background history count, varThreshold, shadow detection */
    //Ptr<cuda::BackgroundSubtractorMOG2> bsModel = cuda::createBackgroundSubtractorMOG2(30, f3xBase.Mog2Threshold(), false); /* To anit grass wave ... */ 
    bsModel = cuda::createBackgroundSubtractorMOG2(30, f3xBase.Mog2Threshold(), false); /* To anit grass wave ... */ 
    //cout << bsModel->getVarInit() << " / " << bsModel->getVarMax() << " / " << bsModel->getVarMax() << endl;
    /* Default variance of each gaussian component 15 / 75 / 75 */ 
    bsModel->setVarInit(15);
    bsModel->setVarMax(20);
    bsModel->setVarMin(4);
#endif
    cout << endl;
    cout << "### Press button to start object tracking !!!" << endl;

    double fps = 0;
    steady_clock::time_point t1(steady_clock::now());
    uint64_t loopCount = 0;

    if(bStopped == false)
        F3xBase::Start();

    steady_clock::time_point lastTriggerTime(steady_clock::now());

    while(1) {
        if(bShutdown)
            break;

        if(evtQueue.size() > 0) {
            EvtType_t t = evtQueue.front();
            evtQueue.pop();
            switch(t) {
                case EvtStop: 
                    if(bStopped == false) 
                        F3xBase::Stop();
                    break;
                case EvtStart:
                    if(bStopped) 
                        F3xBase::Start();
                    break;
                default:
                    break;
            }
        }

        static unsigned int vPushButton = 1;
        unsigned int gv = f3xBase.GetPushButton();
        if(gv == 0 && vPushButton == 1) { /* Raising edge */
            if(loopCount >= 10) { /* Button debunce */
                if(bStopped)
                    F3xBase::Start();
                else
                    F3xBase::Stop();
                loopCount = 0;
            }
        }
        vPushButton = gv;
     
        loopCount++; /* Increase loop count */

        if(bStopped) {
            f3xBase.RedLed(off);
            f3xBase.Relay(off);
            f3xBase.GreenLed(on); /* On while pause */
            f3xBase.BlueLed(f3xBase.IsVideoOutputFile() ? on : off);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if(loopCount % 2 == 0) {
            f3xBase.GreenLed(on); /* Flash during frames */
            if(f3xBase.IsVideoOutputFile())
                f3xBase.BlueLed(on);
        } else {
            f3xBase.GreenLed(off); /* Flash during frames */
            if(f3xBase.IsVideoOutputFile())
                f3xBase.BlueLed(off);
        }

        Mat capFrame;
        camera.Read(capFrame);

        steady_clock::time_point t3(steady_clock::now());

        Mat outFrame;
        if(f3xBase.IsVideoOutputResult()) {
            if(f3xBase.IsVideoOutput()) {
                capFrame.copyTo(outFrame);
                line(outFrame, Point(0, cy), Point(cx, cy), Scalar(0, 255, 0), 1);
            }
        }

        /* Gray color space for whole region */
        Mat grayFrame;
#if 0
        cuda::GpuMat gpuCap, gpuGray;
        gpuCap.upload(capFrame);
        cuda::cvtColor(gpuCap, gpuGray, COLOR_BGR2GRAY);
        gpuGray.download(grayFrame);
#else
        cvtColor(capFrame, grayFrame, COLOR_BGR2GRAY);
#endif
        list<Rect> roiRect;

        extract_moving_object(grayFrame, roiRect);

        if(f3xBase.IsVideoOutputResult()) {
            if(f3xBase.IsVideoOutput()) {
                for(list<Rect>::iterator rr=roiRect.begin();rr!=roiRect.end();++rr)
                    rectangle( outFrame, rr->tl(), rr->br(), Scalar(0, 255, 0), 2, 8, 0 );
                if(f3xBase.IsNewTargetRestriction()) {
                    Rect nr = tracker.NewTargetRestriction();
                    rectangle( outFrame, nr.tl(), nr.br(), Scalar(127, 0, 127), 2, 8, 0 );
                    writeText( outFrame, "New Target Restriction Area", Point(0, 180));
                }
                writeText( outFrame, currentDateTime(), Point(240, 40));
                line(outFrame, Point((CAMERA_WIDTH / 5), 0), Point((CAMERA_WIDTH / 5), CAMERA_HEIGHT), Scalar(127, 127, 0), 1);
            }
        }

        f3xBase.RedLed(off);
        f3xBase.Relay(off);

        tracker.Update(roiRect, f3xBase.IsFakeTargetDetection());

        list< Target > & targets = tracker.TargetList();

        if(f3xBase.IsVideoOutput()) {
            if(f3xBase.IsVideoOutputResult()) {
                list< list< Rect > > & newTargetHistory = tracker.NewTargetHistory();
                for(auto & it : newTargetHistory) {
                    for(auto & r : it) {
                        rectangle( outFrame, r.tl(), r.br(), Scalar(127, 127, 0), 2, 8, 0 );
                    }
                }
            }
        }

        bool doTrigger = false;

        for(list< Target >::iterator t=targets.begin();t!=targets.end();++t) {
            if(f3xBase.IsVideoOutputResult()) { 
                if(f3xBase.IsVideoOutput()) {
                    t->Draw(outFrame, true); /* Draw target */
                }
            }

            if(t->TriggerCount() > 0 && t->TriggerCount() < MAX_NUM_TRIGGER)
                doTrigger = true;

            if(t->ArcLength() > MIN_COURSE_LENGTH && 
                t->AbsLength() > MIN_COURSE_LENGTH && 
                t->TrackedCount() > MIN_TARGET_TRACKED_COUNT) {
                if((t->BeginCenterPoint().y > cy && t->EndCenterPoint().y <= cy) ||
                        (t->BeginCenterPoint().y < cy && t->EndCenterPoint().y >= cy)||
                        (t->PreviousCenterPoint().y > cy && t->CurrentCenterPoint().y <= cy) ||
                        (t->PreviousCenterPoint().y < cy && t->CurrentCenterPoint().y >= cy)) {
                    bool tgr = t->Trigger(f3xBase.IsBugTrigger());
                    if(doTrigger == false)
                        doTrigger = tgr;
                }
            }
        }

        if(doTrigger) { /* t->TriggerCount() > 0 */
            if(f3xBase.IsVideoOutputResult()) {
                if(f3xBase.IsVideoOutput())
                    line(outFrame, Point(0, cy), Point(cx, cy), Scalar(0, 0, 255), 3);
            }

            bool isNewTrigger = false;
            if(duration_cast<milliseconds>(steady_clock::now() - lastTriggerTime).count() > 500) { /* new trigger */
                isNewTrigger = true;
                lastTriggerTime = steady_clock::now();
            }

            f3xBase.TriggerTty(isNewTrigger);
            //f3xBase.TriggerUdpSocket(t->TriggerCount() == 0);
            f3xBase.TriggerMulticastSocket(isNewTrigger);
            f3xBase.RedLed(on);

            if(isNewTrigger)
                f3xBase.Relay(on);
        }         

        if(f3xBase.IsVideoOutput()) {
            if(f3xBase.IsVideoOutputResult()) {
                char str[32];
                snprintf(str, 32, "FPS : %.2lf", fps);
                writeText(outFrame, string(str), Point( 10, 40 ));
                videoOutputQueue.push(outFrame.clone());
            } else
                videoOutputQueue.push(capFrame.clone());
        }

        if(loopCount > 0 && loopCount % 30 == 0) {
            steady_clock::time_point t2(steady_clock::now());
            double dt_us(static_cast<double>(duration_cast<microseconds>(t2 - t1).count()));

            fps = 30 * (1000000.0 / dt_us);

            std::cout << "FPS : " << fixed  << setprecision(2) <<  fps << " / " << duration_cast<milliseconds>(t2 - t3).count() << " ms" << std::endl;

            t1 = steady_clock::now();
        }
    }

    f3xBase.GreenLed(off); /* Flash during frames */
    f3xBase.BlueLed(off); /* Flash during file save */
    f3xBase.RedLed(off); /* While object detected */
    f3xBase.Relay(off);

    if(f3xBase.IsVideoOutput()) {
        if(bStopped == false) {
            videoOutputQueue.cancel();
            videoOutputThread.join();
        }
    }

    f3xBase.CloseTty();
    f3xBase.CloseUdpSocket();

    camera.Close();

    cout << endl;
    cout << "Finished ..." << endl;

    return 0;     
}
