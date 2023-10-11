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
#include <dirent.h>

#include <curl/curl.h>

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

//using std::chrono::system_clock;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::seconds;

#ifndef DEBUG
//#define DEBUG
#endif

#ifdef DEBUG
#define dprintf(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define dprintf(...) do{ } while ( false )
#endif

#define VERSION "v0.1.9"

//#define CAMERA_1080P

#ifdef CAMERA_1080P
	#define CAMERA_WIDTH 1080
	#define CAMERA_HEIGHT 1920
	#define CAMERA_FPS 30
	#define MIN_TARGET_WIDTH 9
	#define MIN_TARGET_HEIGHT 12
	#define MAX_TARGET_WIDTH 480
	#define MAX_TARGET_HEIGHT 480
#else
	#define CAMERA_WIDTH 720
	#define CAMERA_HEIGHT 1280
	#define CAMERA_FPS 30
	#define MIN_TARGET_WIDTH 6
	#define MIN_TARGET_HEIGHT 8
	#define MAX_TARGET_WIDTH 320
	#define MAX_TARGET_HEIGHT 320
#endif

#define MAX_TARGET_TRACKING_DISTANCE    360

#define MAX_NUM_TARGET               	9      /* Maximum targets to tracing */
#define MAX_NUM_TRIGGER              	6      /* Maximum number of RF trigger after detection of cross line */
#define MAX_NUM_FRAME_MISSING_TARGET 	3      /* Maximum number of frames to keep tracing lost target */

#define MIN_COURSE_LENGTH            	16     /* Minimum course length of RF trigger after detection of cross line */
#define MIN_TARGET_TRACKED_COUNT     	3      /* Minimum target tracked count of RF trigger after detection of cross line */

#define VIDEO_OUTPUT_FPS             	30
#define VIDEO_OUTPUT_DIR             	"/opt/Videos"
#define VIDEO_OUTPUT_FILE_NAME       	"base"
#define VIDEO_FILE_OUTPUT_DURATION   	90     /* Video file duration 90 secends */
#define VIDEO_OUTPUT_MAX_FILES       	400    /* Needs about 30G bytes disk space */
//#define VIDEO_OMXH265ENC

#define STR_SIZE                     	1024
#define CONFIG_FILE_DIR              	"/etc/dragon-eye"

#define HORIZON_RATIO                	8 / 10

typedef enum { JETSON_NANO, JETSON_XAVIER_NX } JetsonDevice_t;

typedef enum { BASE_UNKNOWN, BASE_A, BASE_B, BASE_TIMER, BASE_ANEMOMETER } BaseType_t;

/*
*
*/

static bool IsValidateIpAddress(const string & ipAddress)
{
	struct sockaddr_in sa;
	int result = inet_pton(AF_INET, ipAddress.c_str(), &(sa.sin_addr));
	return result != 0;
}

static bool bShutdown = false;
static bool bStopped = true;
static string s_errorString;
static int s_fps = 0;

typedef enum { EvtStop, EvtStart } EvtType_t;
static queue<EvtType_t> evtQueue;

static std::mutex ipMutex;

static const char *ipv4_address(const char *dev, string & result) {
	std::unique_lock<std::mutex> mlock(ipMutex);

	char host[NI_MAXHOST];
	struct ifaddrs *ifaddr, *ifa;
	int family, s;

	if(dev == 0 || strlen(dev) == 0)
		return 0;

	if(getifaddrs(&ifaddr) == -1) {
		perror("getifaddrs");
		return 0;
	}

	memset(host, 0, NI_MAXHOST);

	bool found = false;
	for(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		if(ifa->ifa_addr == NULL)
			continue;  

		if(ifa->ifa_addr->sa_family != AF_INET)
			continue;

		s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

		if((strcmp(ifa->ifa_name, dev) == 0)) {
			if (s != 0) {
				printf("getnameinfo() failed: %s\n", gai_strerror(s));
				return 0;
			}
			dprintf("\tipv4 <%s> %s\n", ifa->ifa_name, host);
			found = true;
			break;
		}
	}
	freeifaddrs(ifaddr);

	if(found == false)
		return 0;

	if(strlen(host) < 7) /* X.X.X.X */
		return 0;

	if(IsValidateIpAddress(host) == false)
		return 0;

	result = host;

	return result.c_str();
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

static void writeText(Mat & mat, const string & text, Point textOrg)
{
   //int fontFace = FONT_HERSHEY_SIMPLEX; 
   int fontFace = FONT_HERSHEY_DUPLEX;
   double fontScale = 1;
   int thickness = 2;  
   putText(mat, text, textOrg, fontFace, fontScale, Scalar(0, 0, 0), thickness, cv::LINE_8);
}

/*
*
*/

static inline Point Center(Rect & r) {
	return Point(r.tl().x + (r.width / 2), r.tl().y + (r.height / 2));
}

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
		m_averageArea = roi.area();
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
#if 1
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

    double CosineAngleTl(const Point & p) {
        size_t i = m_rects.size();
        if(i < 2)
            return 0;
        --i;
        Point v1, v2;
        v1.x = p.x - m_rects[i].tl().x;
        v1.y = p.y - m_rects[i].tl().y;
        v2.x = m_rects[i].tl().x - m_rects[i-1].tl().x;
        v2.y = m_rects[i].tl().y - m_rects[i-1].tl().y;

        /* A.B = |A||B|cos() */
        /* cos() = A.B / |A||B| */
        return v1.dot(v2) / (norm(v1) * norm(v2));
    }

    double CosineAngleBr(const Point & p) {
        size_t i = m_rects.size();
        if(i < 2)
            return 0;
        --i;
        Point v1, v2;
        v1.x = p.x - m_rects[i].br().x;
        v1.y = p.y - m_rects[i].br().y;
        v2.x = m_rects[i].br().x - m_rects[i-1].br().x;
        v2.y = m_rects[i].br().y - m_rects[i-1].br().y;

        /* A.B = |A||B|cos() */
        /* cos() = A.B / |A||B| */
        return v1.dot(v2) / (norm(v1) * norm(v2));
    }

    double CosineAngleCt(const Point & p) {
        size_t i = m_rects.size();
        if(i < 2)
            return 0;
        --i;
        Point v1, v2;
        Point ct0 = Center(m_rects[i]);
        Point ct1 = Center(m_rects[i-1]);
        v1.x = p.x - ct0.x;
        v1.y = p.y - ct0.y;
        v2.x = ct0.x - ct1.x;
        v2.y = ct0.y - ct1.y;

        /* A.B = |A||B|cos() */
        /* cos() = A.B / |A||B| */
        return v1.dot(v2) / (norm(v1) * norm(v2));
    }

	void Draw(Mat & outFrame, bool drawAll = false) {
		RNG rng(m_rects.front().area());
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		Rect r = m_rects.back();
		//rectangle( outFrame, r.tl(), r.br(), Scalar( 255, 0, 0 ), 2, 8, 0 );
		rectangle( outFrame, r.tl(), r.br(), color, 1, 8, 0 );

		if(m_rects.size() > 1) { /* Minimum 2 points ... */
			for(int i=0;i<m_rects.size()-1;i++) {
				//line(outFrame, p0, p1, Scalar(0, 0, 255), 1);
				line(outFrame, Center(m_rects[i]), Center(m_rects[i+1]), color, 1);
				if(drawAll)
					//rectangle( outFrame, m_rects[i].tl(), m_rects[i].br(), Scalar( 196, 0, 0 ), 2, 8, 0 );
					//rectangle( outFrame, m_rects[i].tl(), m_rects[i].br(), Scalar( (m_rects[0].x + m_rects[0].y) % 255, 0, 0 ), 1, 8, 0 );
					rectangle( outFrame, m_rects[i].tl(), m_rects[i].br(), color, 1, 8, 0 );
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

	inline const Point BeginCenterPoint() { return Center(m_rects[0]); }
	inline const Point EndCenterPoint() { return Center(m_rects.back()); }

	inline const Point CurrentCenterPoint() { return Center(m_rects.back()); }
	const Point PreviousCenterPoint() {
		if(m_rects.size() < 2)
			return std::move(Point(0, 0));

		auto it = m_rects.rbegin();
		return Center(*(++it)); 
	}
	
	bool Trigger(bool enableBugTrigger = false) {
		bool r = false;
		if(m_vectors.size() <= 8 &&
			VectorDistortion() >= 40) { /* 最大位移向量值與最小位移向量值的比例 */
			dprintf("\033[0;31m"); /* Red */
			dprintf("Velocity distortion %f !!!\n", VectorDistortion());
			dprintf("\033[0m"); /* Default color */
		} else if(enableBugTrigger) { 
			if((m_averageArea < 144 && m_normVelocity > 30) || /* 12 x 12 */
					(m_averageArea < 256 && m_normVelocity > 40) || /* 16 x 16 */
					(m_averageArea < 324 && m_normVelocity > 50) || /* 18 x 18 */
					(m_averageArea < 400 && m_normVelocity > 75) || /* 20 x 20 */
					(m_averageArea < 576 && m_normVelocity > 100) || /* 24 x 24 */
					(m_averageArea < 900 && m_normVelocity > 125) /* 30 x 30 */
				) {
				dprintf("\033[0;31m"); /* Red */
				dprintf("Bug detected !!! average area = %d, velocity = %f\n", m_averageArea, m_normVelocity);
				dprintf("\033[0m"); /* Default color */
				m_bugTriggerCount++;
			} else {
				if(m_bugTriggerCount > 0) {
					dprintf("\033[0;31m"); /* Red */
					dprintf("False trigger due to bug trigger count is %u\n", m_bugTriggerCount);
					dprintf("\033[0m"); /* Default color */
					if(m_bugTriggerCount <= 3) /* To avoid false bug detection */
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
			dprintf("\033[0;31m"); /* Red */
			dprintf("[%u] T R I G G E R (%d)\n", m_id, m_triggerCount);
			dprintf("\033[0m"); /* Default color */
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

static inline bool TargetSortByTrackedCount(Target & a, Target & b)
{
    return a.TrackedCount() > b.TrackedCount();
}

static Rect MergeRect(Rect & r1, Rect & r2) {
    Rect r;
    r.x = min(r1.x, r2.x);
    r.y = min(r1.y, r2.y);
    r.width = max(r1.x + r1.width, r2.x + r2.width) - r.x;
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
	int m_horizonHeight;

    size_t MaxTrackedCountOfTargets() {
        size_t maxCount = 0;
        for(list< Target >::iterator t=m_targets.begin();t!=m_targets.end();t++) {
            if(t->TrackedCount() > maxCount)
                maxCount = t->TrackedCount();
        }
        return maxCount;
    }

public:
	Tracker() : m_width(CAMERA_WIDTH), m_height(CAMERA_HEIGHT), m_lastFrameTick(0), m_horizonHeight(CAMERA_HEIGHT * HORIZON_RATIO) {}
	Tracker(int width, int height) : Tracker() {
		m_width = width;
		m_height = height;
		m_horizonHeight = height * HORIZON_RATIO;
	}

	void UpdateHorizonRatio(uint16_t horizonRatio) {
		switch(horizonRatio) {
			case 20: m_horizonHeight = m_height * 8 / 10;
				break;
			case 30: m_horizonHeight = m_height * 7 / 10;
				break;
		}
	}

	void Initialisize(int width, int height) {
		m_width = width;
		m_height = height;
	}

	int HorizonHeight() const { return m_horizonHeight; }

	void NewTargetRestriction(const Rect & r) {
		m_newTargetRestrictionRect = r;
	}

	Rect NewTargetRestrictionRect() const {   return m_newTargetRestrictionRect; }

	void Update(list< Rect > & roiRect, bool enableFakeTargetDetection = false) {
		++m_lastFrameTick;
		for(list< Target >::iterator t=m_targets.begin();t!=m_targets.end();) { /* Try to find lost targets */
			list<Rect>::iterator rr;
			Rect r1 = t->m_rects.back();
			Rect r2 = r1;
			int f = m_lastFrameTick - t->FrameTick();
			if(t->m_vectors.size() > 0) {
				Point v = t->m_vectors.back();
				for(int i=0;i<f;i++) {
					v.x += t->m_acceleration.x;
					v.y += t->m_acceleration.y;
					r2.x += v.x;
					r2.y += v.y;
				}
			}

			if(t->m_triggerCount > 0 &&
					(r2.x < 0 || r2.x > m_width)) {
				dprintf("\033[0;35m"); /* Puple */
				dprintf("<%u> Out of range target : (%d, %d), samples : %lu\n", t->m_id, t->m_rects.back().tl().x, t->m_rects.back().tl().y, t->m_rects.size());
				dprintf("\033[0m"); /* Default color */
				t = m_targets.erase(t); /* Remove tracing target */
				continue;
			}

			double n0 = 0;
			if(t->m_vectors.size() > 0) {
				n0 = cv::norm(Center(r2) - Center(r1)); /* Moving distance of predict target */
			}
			
			for(rr=roiRect.begin();rr!=roiRect.end();++rr) {

				if(r1.area() > (rr->area() * 32) ||
					rr->area() > (r1.area() * 32)) /* Object and target area difference */
					continue;

                Point rrct = Center(*rr);
                double n1 = cv::norm(rrct - Center(r1)); /* Distance between object and target */

				if(n1 > MAX_TARGET_TRACKING_DISTANCE)
					continue; /* Too far */
#if 0
				if(t->m_vectors.size() > 1) {
					double n = cv::norm(t->m_vectors.back());
					if(n1 > (n0 * 3) / 2)
						continue; /* Too far */
				}
#endif
				if((r1 & *rr).area() > 0) { /* Target tracked ... */
					//if(t->DotProduct(rr->tl()) >= 0) /* Two vector less than 90 degree */
						break;                
				}

				if(t->m_vectors.size() > 0) {
					Rect r = r1;
					bool tracked = false;
					Point v = t->m_vectors.back();
					for(int i=0;i<f;i++) {
						r.x += v.x;
						r.y += v.y;
						if((r & *rr).area() > 0) { /* Target tracked with velocity ... */
							tracked = true;
							break;
						}
					}
					if(tracked)
						break;
				}

				if(t->m_vectors.size() > 0) {
					Rect r = r1;
					bool tracked = false;
					Point v = t->m_vectors.back();
					for(int i=0;i<f;i++) {
						v.x += t->m_acceleration.x;
						v.y += t->m_acceleration.y;
						r.x += v.x;
						r.y += v.y;
						if((r & *rr).area() > 0) { /* Target tracked with velocity ... */
							tracked = true;
							break;
						}
					}
					if(tracked)
						break;
				}

				if(t->m_vectors.size() == 0) { /* new target with zero velocity */
					if(rr->y >= m_horizonHeight) {
						if(n1 < (rr->width + rr->height)) /* Target tracked with Euclidean distance ... */
							break;
					} else {
						if(n1 < (rr->width + rr->height) * 2) /* Target tracked with Euclidean distance ... */
							break;
					}
				} else if(n1 < (n0 * 3) / 2) { /* Target tracked with velocity and Euclidean distance ... */
					if(rr->y >= m_horizonHeight) {
						double a = t->CosineAngleCt(rrct);
						if(a > 0.9659) /* cos(PI/12) */
							break;
					} else {
						//double a = t->CosineAngleTl(rr->tl());
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

					Point rrct = Center(*rr);
					double n1 = cv::norm(rrct - Center(r1)); /* Distance between object and target */

					if(n1 > (MAX_TARGET_TRACKING_DISTANCE/ 2))
						continue; /* Too far */

                    double a = t->CosineAngleCt(rrct);
                    double n2 = cv::norm(rrct - Center(r2));

                    if(a > 0.5 && /* cos(PI/3) */
                        n2 < (n0 * 3) / 2) { 
                        break;
                    }

                    if(a > 0.8587 && /* cos(PI/6) */
                        n1 < (t->m_normVelocity * f * 2)) {
                        break;
                    }
				}
			}

			if(rr == roiRect.end()) { /* Target missing ... */
				bool isTargetLost = false;
				if(t->m_vectors.size() > 1) {
					uint32_t compensation = (t->TrackedCount() / 6); /* Tracking more frames with more sample */
					if(compensation > 5)
						compensation = 5;
					if(f > (MAX_NUM_FRAME_MISSING_TARGET + compensation)) /* Target still missing for over X frames */
						isTargetLost = true;
				} else { /* new target with zero velocity */
					if(f > MAX_NUM_FRAME_MISSING_TARGET) 
						isTargetLost = true;
				}
				if(isTargetLost) {
					dprintf("\033[0;35m"); /* Puple */
					dprintf("<%u> Lost target : (%d, %d), samples : %lu\n", t->m_id, t->m_rects.back().tl().x, t->m_rects.back().tl().y, t->m_rects.size());
					dprintf("\033[0m"); /* Default color */
					t = m_targets.erase(t); /* Remove tracing target */
					continue;
				} else {
#if 0                    
					Point p = t->m_rects.back().tl();
					dprintf("<%u> Search target : (%d, %d) -> [%d, %d]\n", t->m_id, p.x, p.y, 
						(t->m_velocity.x + t->m_acceleration.x) * f, (t->m_velocity.y + t->m_acceleration.y) * f);
					for(list< Target >::iterator tt=m_targets.begin();tt!=m_targets.end();++tt) {
						if(tt->m_id == t->m_id)
							continue;
						if((t->m_rects.back() & tt->m_rects.front()).area() > 0) { /**/
							t->Update(*tt);

							dprintf("\033[0;33m"); /* Yellow */
							dprintf("Merge targets : <%d> -->> <%d>\n", tt->m_id, t->m_id);
							dprintf("\033[0m"); /* Default color */

							m_targets.erase(tt);
							break;
						}
					}
#endif                                       
				}
			} else { /* Target tracked ... */
				Point p = t->m_rects.back().tl();
				dprintf("\033[0;32m"); /* Green */
				dprintf("<%u> Target tracked : [%lu](%d, %d) -> (%d, %d)[%d, %d]\n", t->m_id, m_lastFrameTick, p.x, p.y, 
					rr->x, rr->y, rr->x - t->m_rects.back().x, rr->y - t->m_rects.back().y);
				dprintf("\033[0m"); /* Default color */
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
					if(rr->y < m_horizonHeight)
						continue;
					if((r & *rr).area() > 0) { /* new target overlap previous new target */
						++overlap_count;
					}
				}
			}
			if(rr->y >= m_horizonHeight) {
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
				dprintf("[X] Fake target : (%u)\n", overlap_count);
			} else {
				m_targets.push_back(Target(*rr, m_lastFrameTick));
				dprintf("\033[0;32m"); /* Green */
				dprintf("<%u> New target : [%lu](%d, %d)\n", m_targets.back().m_id, m_lastFrameTick, rr->tl().x, rr->tl().y);
				dprintf("\033[0m"); /* Default color */
			}
		}

		m_newTargetsHistory.push_back(newTargetList);
		if(m_newTargetsHistory.size() >= 90) {
			m_newTargetsHistory.pop_front();
		}

		if(m_targets.size() > 1) {
			if(MaxTrackedCountOfTargets() > 6)
				m_targets.sort(TargetSortByTrackedCount);
			else
				m_targets.sort(TargetSortByArea);
		}
	}

	inline list< Target > & TargetList() { return m_targets; }
	inline list< list< Rect > > & NewTargetHistory() { return m_newTargetsHistory; }
};

static Tracker tracker;

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
	size_t size() { return matQueue.size(); }
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
	std::unique_lock<std::mutex> mlock(matMutex);

	while(matQueue.size() >= 15) { /* Prevent memory overflow ... */
		printf("Video Frame droped !!!\n");
		if(refCnt == 0)
			matQueue.pop();
		else
			return;
	}

	matQueue.push(image);
	condEvent.notify_all();
}

void FrameQueue::pop()
{
	std::unique_lock<std::mutex> mlock(matMutex);

	if(matQueue.empty())
		return;

	if(refCnt > 0)
		--refCnt;

	if(refCnt == 0)
		matQueue.pop();
}

const Mat & FrameQueue::front()
{
	std::unique_lock<std::mutex> mlock(matMutex);

	while (matQueue.empty() && !isCancelled) {
		condEvent.wait(mlock);
	}

	if(isCancelled) {
		throw cancelled();
	}

	++refCnt;

	return matQueue.front();
}

void FrameQueue::reset()
{
	std::unique_lock<std::mutex> mlock(matMutex);

	matQueue = std::queue<cv::Mat>();
	isCancelled = false;
}

/*
*
*/

static FrameQueue videoOutputQueue;
static FrameQueue videoRtspQueue;

/*
*
*/

typedef struct {
	int width;
	int height;
	int fps;
} VideoProperties;

typedef struct {
	int numberFrames;
	GstClockTime timestamp;
	VideoProperties videoProperties;
	GstBuffer *buffer;
} RtspServerContext;

/* called when we need to give data to appsrc */
static void
need_data (GstElement * appsrc, guint unused, RtspServerContext *ctx)
{
	GstBuffer *buffer;
//	uint64_t size = ctx->videoProperties.width * ctx->videoProperties.height * 4; // Image size * deth of BGRx;
	uint64_t size = ctx->videoProperties.width * ctx->videoProperties.height * 3; // Image size * deth of BGR;
	GstFlowReturn ret;

	if(ctx->buffer == 0) {
		ctx->buffer = gst_buffer_new_allocate (NULL, size, NULL);
	}
	buffer = ctx->buffer;

	//buffer = gst_buffer_new_allocate (NULL, size, NULL);
	GstMapInfo map;
	gint8 *raw;

	gst_buffer_map (buffer, &map, GST_MAP_WRITE); // make buffer writable
	raw = (gint8 *)map.data;

	const Mat & lastFrame = videoRtspQueue.front();
/*
#if 0
	for (int i=0;i<ctx->videoProperties.height;i++) {
		const Vec3b* ptr = lastFrame.ptr<Vec3b>(i);
		for (int j = 0; j<ctx->videoProperties.width; j++) {
			uint64_t offset = ((i*ctx->videoProperties.width)+j)*4;
			raw[offset] = ptr[j][0];
			raw[offset+1] = ptr[j][1];
			raw[offset+2] = ptr[j][2];
			raw[offset+3] = 127;
		}
	}
#else
	for (int i=0;i<ctx->videoProperties.height;i++) {
		const Vec3b* ptr = lastFrame.ptr<Vec3b>(i);
		uint64_t offset = (i*ctx->videoProperties.width) * 4;
		for (int j=0; j<ctx->videoProperties.width; j++) {
			raw[offset++] = ptr[j][0];
			raw[offset++] = ptr[j][1];
			raw[offset++] = ptr[j][2];
			raw[offset++] = 127;
		}
	}
#endif
*/
	memcpy(raw, lastFrame.data, size);
	videoRtspQueue.pop();

	gst_buffer_unmap (buffer, &map);

	/* increment the timestamp every 1/FPS second */
	GST_BUFFER_PTS (buffer) = ctx->timestamp;
	GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, VIDEO_OUTPUT_FPS);
	ctx->timestamp += GST_BUFFER_DURATION (buffer);

	g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
	//gst_buffer_unref (buffer);
}

static void free_ctx(gpointer mem) {
//printf("%s:%d\n", __PRETTY_FUNCTION__, __LINE__);
	RtspServerContext *ctx = (RtspServerContext *)mem;
	if(ctx->buffer)
		gst_buffer_unref(ctx->buffer);
	g_free(ctx);
}

/* called when a new media pipeline is constructed. CAMERA_WIDTHe can query the
 * pipeline and configure our appsrc */
static void
media_configure (GstRTSPMediaFactory * factory, GstRTSPMedia * media, gpointer user_data)
{
	// should be incremented once on each frame for timestamping
	GstElement *element, *appsrc;
	RtspServerContext *ctx;
	VideoProperties *vp = (VideoProperties *)user_data;

	/* get the element used for providing the streams of the media */
	element = gst_rtsp_media_get_element (media);

	/* get our appsrc, we named it 'mysrc' with the name property */
	appsrc = gst_bin_get_by_name_recurse_up (GST_BIN (element), "mysrc");

	/* this instructs appsrc that we will be dealing with timed buffer */
	gst_util_set_object_arg (G_OBJECT (appsrc), "format", "time");
	/* configure the caps of the video */
	g_object_set (G_OBJECT (appsrc), "caps",
	gst_caps_new_simple ("video/x-raw",
				"format", G_TYPE_STRING, "BGR",
				"width", G_TYPE_INT, vp->width,
				"height", G_TYPE_INT, vp->height,
				"framerate", GST_TYPE_FRACTION, vp->fps, 1, NULL), NULL);

	ctx = g_new0 (RtspServerContext, 1);
	ctx->timestamp = 0;
	ctx->numberFrames = 0;
	ctx->videoProperties.width = vp->width;
	ctx->videoProperties.height = vp->height;
	ctx->videoProperties.fps = vp->fps;
	ctx->buffer = 0;

	/* make sure ther datais freed when the media is gone */
	g_object_set_data_full (G_OBJECT (media), "my-extra-data", ctx, (GDestroyNotify) free_ctx);

	/* install the callback that will be called when a buffer is needed */
	g_signal_connect (appsrc, "need-data", (GCallback) need_data, ctx);
	gst_object_unref (appsrc);
	gst_object_unref (element);
}

#include <glib-object.h>
#include <glib-unix.h>

// This callback will be inovked when this process receives SIGUSR1.
gboolean HangupSignalCallback(gpointer data) {
  //out << "Received a signal to terminate the daemon";
  GMainLoop* loop = reinterpret_cast<GMainLoop*>(data);
  g_main_loop_quit(loop);
  // This function can return false to remove this signal handler as we are
  // quitting the main loop anyway.

  return false;
}

/*
* https://forums.developer.nvidia.com/t/rtsp-server-cleanup-from-thread-does-not-stop-client-apps-video/107137/15
*/

std::atomic<uint32_t> g_clientCount(0);

GstRTSPFilterResult clientFilter(GstRTSPServer* server, GstRTSPClient* client, gpointer user) {
	return GST_RTSP_FILTER_REMOVE;
}

void closeClients(GstRTSPServer* server)
{
	printf("RTSP Server - closing clients [%u]\n", g_clientCount.load());
	if(g_clientCount.load() > 0)
		gst_rtsp_server_client_filter(server, clientFilter, nullptr);
}

void clientClosed(GstRTSPClient* client, gpointer user)
{
	printf("RTSP Server - clients closed [%u]\n", --g_clientCount);
}

void clientConnected(GstRTSPServer* server, GstRTSPClient* client, gpointer user)
{
	g_signal_connect(client, "closed", reinterpret_cast<GCallback>(clientClosed), nullptr);
	printf("RTSP Server - client connected [%u]\n", ++g_clientCount);
}

static VideoProperties s_videoProperties;
static GstRTSPServer *s_server = 0;

int gst_rtsp_server_task(int width, int height, int fps)
{
	GMainLoop *loop;
	//GstRTSPServer *server;
	GstRTSPMountPoints *mounts;
	GstRTSPMediaFactory *factory;

	s_videoProperties.width = width;
	s_videoProperties.height = height;
	s_videoProperties.fps = fps;

	char *args[] = {
		(char*)"gst-rtsp-server",
		NULL
	};
	int argv = 0;
	gst_init (&argv, (char ***)&args);

	loop = g_main_loop_new (NULL, FALSE);

	// Set up a signal handler for handling SIGUSR1.
	g_unix_signal_add(SIGUSR1, HangupSignalCallback, loop);
	
	/* create a server instance */
	s_server = gst_rtsp_server_new ();
	/* get the mount points for this server, every server has a default object
	* that be used to map uri mount points to media factories */
	mounts = gst_rtsp_server_get_mount_points (s_server);

	/* make a media factory for a test stream. The default media factory can use
	* gst-launch syntax to create pipelines.
	* any launch line works as long as it contains elements named pay%d. Each
	* element with pay%d names will be a stream */
	factory = gst_rtsp_media_factory_new ();
#ifdef VIDEO_OMXH265ENC
	gst_rtsp_media_factory_set_launch (factory,
		"appsrc name=mysrc is-live=true ! video/x-raw, format=(string)BGR ! videoconvert ! \
omxh265enc insert-sps-pps=1 ! rtph265pay mtu=1400 name=pay0 pt=96 )");
#else
/*
	gst_rtsp_media_factory_set_launch (factory,
		"appsrc name=mysrc is-live=true ! video/x-raw, format=(string)BGR ! videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420 ! \
nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! rtph265pay mtu=1400 name=pay0 pt=96 )");
*/
/*

https://forums.developer.nvidia.com/t/pulsing-at-iframe-interval-using-nvv4l2h265enc/194172/4

https://forums.developer.nvidia.com/t/random-blockiness-in-the-picture-rtsp-server-client-jetson-tx2/174896

https://forums.developer.nvidia.com/t/h264-vs-h265/115260/3

https://forums.developer.nvidia.com/t/nvv4l2h264enc-latency-and-preset-level/184221/15

*/
/*
preset-level        : HW preset level for encoder
                        flags: readable, writable, changeable only in NULL or READY state
                        Enum "GstV4L2VideoEncHwPreset" Default: 1, "UltraFastPreset"
                           (0): DisablePreset    - Disable HW-Preset
                           (1): UltraFastPreset  - UltraFastPreset for high perf
                           (2): FastPreset       - FastPreset
                           (3): MediumPreset     - MediumPreset
                           (4): SlowPreset       - SlowPreset

poc-type            : Set Picture Order Count type value
                        flags: readable, writable, changeable only in NULL or READY state
                        Unsigned Integer. Range: 0 - 2 Default: 0
maxperf-enable      : Enable or Disable Max Performance mode
                        flags: readable, writable, changeable only in NULL or READY state
                        Boolean. Default: false                        
*/	
	gst_rtsp_media_factory_set_launch (factory,
		"appsrc name=mysrc is-live=true ! video/x-raw, format=(string)BGR ! videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420 ! \
nvv4l2h265enc bitrate=8000000 control-rate=1 vbv-size=360000 preset-level=1 poc-type=2 maxperf-enable=true insert-vui=true insert-sps-pps=1 ! rtph265pay mtu=1400 name=pay0 pt=96 )");

//nvv4l2h265enc maxperf-enable=1 iframeinterval=10 bitrate=8000000 ! rtph265pay mtu=1400 name=pay0 pt=96 )");
#endif
	gst_rtsp_media_factory_set_eos_shutdown(factory, TRUE);
	gst_rtsp_media_factory_set_shared (factory, TRUE);
	/* notify when our media is ready, This is called whenever someone asks for
	* the media and a new pipeline with our appsrc is created */
	g_signal_connect (factory, "media-configure", (GCallback) media_configure, &s_videoProperties);

	/* attach the test factory to the /test url */
	gst_rtsp_mount_points_add_factory (mounts, "/test", factory);

	/* don't need the ref to the mounts anymore */
	g_object_unref (mounts);

	/* attach the server to the default maincontext */
	const auto server_id = gst_rtsp_server_attach (s_server, NULL);

	g_signal_connect(s_server, "client-connected", reinterpret_cast<GCallback>(clientConnected), nullptr);

	videoRtspQueue.reset();   

	/* start serving */
	g_print ("stream ready at rtsp://127.0.0.1:8554/test\n");
	g_main_loop_run (loop);

	printf("RTSP Server - stopped\n");

	if(g_clientCount.load() > 0)
		closeClients(s_server);

	g_source_remove(server_id);
	if(G_IS_OBJECT(s_server))
		g_object_unref(s_server);
	s_server = 0;
	if(G_IS_OBJECT(loop))
		g_object_unref(loop);

	videoRtspQueue.cancel();

	return 0;
}

static gboolean remove_func (GstRTSPSessionPool *pool, GstRTSPSession *session, GstRTSPServer *server) {
  return GST_RTSP_FILTER_REMOVE;
}

void gst_rtsp_server_close_clients() {
	if(s_server == 0)
		return;

	printf("RTSP Server - shutdown\n");
/*	
	GstRTSPSessionPool *pool;
	pool = gst_rtsp_server_get_session_pool(s_server);
	gst_rtsp_session_pool_filter(pool, (GstRTSPSessionPoolFilterFunc) remove_func, s_server);
	g_object_unref(pool);
*/	
	if(g_clientCount.load() > 0)
		closeClients(s_server); /* It's chance to hang inside if client running */
}

static thread rtspServerThread;

/*
*
*/

void VideoOutputTask(BaseType_t baseType, bool isVideoOutputScreen, bool isVideoOutputFile, 
	bool isVideoOutputRTP, const char *rtpRemoteHost, uint16_t rtpRemotePort, 
	bool isVideoOutputHLS, int width, int height, int fps)
{    
	Size videoSize = Size((int)width,(int)height);
	char gstStr[STR_SIZE];

	VideoWriter outFile;
	VideoWriter outScreen;
	VideoWriter outRTP;
	VideoWriter outHLS;
	//VideoWriter outRTSP;

	int videoOutoutIndex = 0;

	if(isVideoOutputFile) {       
		char filePath[64];
		while(videoOutoutIndex < VIDEO_OUTPUT_MAX_FILES) {
#ifdef VIDEO_OMXH265ENC
			snprintf(filePath, 64, "%s/%s%c%03d.mkv", VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#else
			snprintf(filePath, 64, "%s/%s%c%03d.mp4", VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#endif
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
#ifdef VIDEO_OMXH265ENC
		snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
omxh265enc preset-level=3 bitrate=8000000 ! h265parse ! qtmux ! filesink location=%s/%s%c%03d.mkv ", 
			VIDEO_OUTPUT_FPS, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#else
		snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420, framerate=(fraction)%d/1 ! \
nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! h265parse ! qtmux ! filesink location=%s/%s%c%03d.mp4 ",
						VIDEO_OUTPUT_FPS, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#endif
#if 0 /* NOT work, due to tee */
		snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420, framerate=(fraction)%d/1 ! \
nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! \
tee name=t \
t. ! queue ! h265parse ! qtmux ! filesink location=%s/%s%c%03d.mkv  \
t. ! queue ! video/x-h265, stream-format=byte-stream ! h265parse ! rtph265pay mtu=1400 ! \
udpsink host=224.1.1.1 port=5000 auto-multicast=true sync=false async=false ",
			VIDEO_OUTPUT_FPS, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
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
nvvidconv flip-method=3 ! video/x-raw(memory:NVMM) ! \
nvoverlaysink sync=false ", VIDEO_OUTPUT_FPS);
		outScreen.open(gstStr, VideoWriter::fourcc('I', '4', '2', '0'), 30, Size(width, height));
		cout << endl;
		cout << gstStr << endl;
		cout << endl;
		cout << "*** Start display video ***" << endl;
	}

	if(isVideoOutputRTP && rtpRemoteHost) {
#ifdef VIDEO_OMXH265ENC
		snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
omxh264enc control-rate=2 bitrate=4000000 ! video/x-h265, stream-format=byte-stream ! \
h265parse ! rtph265pay mtu=1400 config-interval=10 pt=96 ! udpsink host=%s port=%u sync=false async=false ",
			VIDEO_OUTPUT_FPS, rtpRemoteHost, rtpRemotePort);
#else
		snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420, framerate=(fraction)%d/1 ! \
nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! video/x-h265, stream-format=byte-stream ! \
h265parse ! rtph265pay mtu=1400 config-interval=10 pt=96 ! udpsink host=%s port=%u sync=false async=false ",
			VIDEO_OUTPUT_FPS, rtpRemoteHost, rtpRemotePort);
#endif
		outRTP.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), VIDEO_OUTPUT_FPS, Size(width, height));
		cout << endl;
		cout << gstStr << endl;
		cout << endl;
		cout << "*** Start RTP video ***" << endl;        
	}

	if(isVideoOutputHLS) {
#ifdef VIDEO_OMXH265ENC
		snprintf(gstStr, STR_SIZE, "appsrc is-live=true ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
omxh264enc control-rate=2 bitrate=4000000 ! h264parse ! mpegtsmux ! \
hlssink playlist-location=/tmp/playlist.m3u8 location=/tmp/segment%%05d.ts target-duration=1 max-files=10 ", VIDEO_OUTPUT_FPS);
#else
		snprintf(gstStr, STR_SIZE, "appsrc is-live=true ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420, framerate=(fraction)%d/1 ! \
nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! h264parse ! mpegtsmux ! \
hlssink playlist-location=/tmp/playlist.m3u8 location=/tmp/segment%%05d.ts target-duration=1 max-files=10 ", VIDEO_OUTPUT_FPS);
#endif
		outHLS.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), VIDEO_OUTPUT_FPS, Size(width, height));
		cout << endl;
		cout << gstStr << endl;
		cout << endl;
		cout << "*** Start HLS video ***" << endl;        
	}

	videoOutputQueue.reset();

	steady_clock::time_point t1 = steady_clock::now();

	try {
		while(1) {
			if(bShutdown)
				break;

			const Mat & frame = videoOutputQueue.front(); /* Copy frame to avoid frame queue overflow */

			if(isVideoOutputFile) {
				if(outFile.isOpened())
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
#ifdef VIDEO_OMXH265ENC
					snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)I420, framerate=(fraction)%d/1 ! \
omxh265enc preset-level=3 bitrate=8000000 ! h265parse ! qtmux ! filesink location=%s/%s%c%03d.mkv ", 
						VIDEO_OUTPUT_FPS, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#else
		snprintf(gstStr, STR_SIZE, "appsrc ! video/x-raw, format=(string)BGR ! \
videoconvert ! video/x-raw, format=(string)BGRx ! \
nvvidconv ! video/x-raw(memory:NVMM), format=(string)I420, framerate=(fraction)%d/1 ! \
nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! h265parse ! qtmux ! filesink location=%s/%s%c%03d.mp4 ",
						VIDEO_OUTPUT_FPS, VIDEO_OUTPUT_DIR, VIDEO_OUTPUT_FILE_NAME, (baseType == BASE_A) ? 'A' : 'B', videoOutoutIndex);
#endif

					outFile.open(gstStr, VideoWriter::fourcc('X', '2', '6', '4'), VIDEO_OUTPUT_FPS, videoSize);
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

			videoOutputQueue.pop();
		}
	} catch (FrameQueue::cancelled & /*e*/) {
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
	}    
}

/*
*
*/

static void ParseConfigString(string & line, vector<pair<string, string> > & cfg)
{
	auto delimiterPos = line.find("=");
	auto name = line.substr(0, delimiterPos);
	auto value = line.substr(delimiterPos + 1);

	name = std::regex_replace(name, std::regex(" +$"), ""); /* Remove tail space */
	value = std::regex_replace(value, std::regex("^ +"), ""); /* Remove leading space */

	cfg.push_back(make_pair(name, value));
}

static size_t ParseConfigFile(char *file, vector<pair<string, string> > & cfg)
{
	ifstream input(file);
	if(input.is_open()) {
		for(string line; getline( input, line ); ) {
			line = std::regex_replace(line, std::regex("^ +| +$|( ) +"), "$1"); /* Strip leading & tail space */
			if(line[0] == '#' || line.empty())
					continue;
			ParseConfigString(line, cfg);
		}
		input.close();
		return cfg.size();
	}
	return 0;
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

/*
*
*/

class Camera {
private:
	VideoCapture cap;
	char gstStr[STR_SIZE];
	int m_width, m_height;
	int m_fps;

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

	Camera() : m_width(CAMERA_WIDTH), m_height(CAMERA_HEIGHT), m_fps(CAMERA_FPS), sensor_id(0), wbmode(0), tnr_mode(1), tnr_strength(-1), ee_mode(1), ee_strength(-1),
		gainrange("1 16"), ispdigitalgainrange("1 8"), exposuretimerange("5000000 10000000"),
		exposurecompensation(0), exposurethreshold(5) {
	}

	Camera(int width, int height, int fps) : Camera() {
		m_width = width;
		m_height = height;
		m_fps = fps;
	}

	void Initialisize(int width, int height, int fps) {
		m_width = width;
		m_height = height;
		m_fps = fps;
	}

	bool Open() {
		if(cap.isOpened())
			return true;
/* Reference : nvarguscamerasrc.txt */
/* export GST_DEBUG=2 to show debug message */
#if 0
	snprintf(gstStr, STR_SIZE, "nvarguscamerasrc sensor-id=0 wbmode=0 tnr-mode=2 tnr-strength=1 ee-mode=1 ee-strength=0 gainrange=\"1 16\" ispdigitalgainrange=\"1 8\" exposuretimerange=\"5000000 10000000\" exposurecompensation=0 ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=3 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
		m_height, m_width, CAMERA_FPS);
#else
		if(sensor_id < 2) {
			snprintf(gstStr, STR_SIZE, "nvarguscamerasrc sensor-id=%d wbmode=%d tnr-mode=%d tnr-strength=%f ee-mode=%d ee-strength=%f gainrange=%s ispdigitalgainrange=%s exposuretimerange=%s exposurecompensation=%f ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
nvvidconv flip-method=3 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
				sensor_id, wbmode, tnr_mode, tnr_strength, ee_mode, ee_strength, gainrange.c_str(), ispdigitalgainrange.c_str(), exposuretimerange.c_str(), exposurecompensation,
				m_height, m_width, m_fps);
		} else { /* USB camera - MJPG */
			snprintf(gstStr, STR_SIZE, "v4l2src device=/dev/video%d io-mode=2 ! image/jpeg, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! \
nvv4l2decoder mjpeg=1 ! \
nvvidconv flip-method=3 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true", 
				sensor_id, 
				m_height, m_width, m_fps);			
		}
#endif

#if 0 /* tee works */
		snprintf(gstStr, STR_SIZE, "nvarguscamerasrc sensor-id=%d wbmode=%d tnr-mode=%d tnr-strength=%f ee-mode=%d ee-strength=%f gainrange=%s ispdigitalgainrange=%s exposuretimerange=%s exposurecompensation=%f ! \
video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! \
tee name=t \
t. ! nvv4l2h265enc bitrate=8000000 maxperf-enable=1 ! h265parse ! rtph265pay mtu=1400 ! udpsink host=127.0.0.1 port=5009 sync=false async=false \
t. ! nvvidconv flip-method=3 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=true ", 
			sensor_id, wbmode, tnr_mode, tnr_strength, ee_mode, ee_strength, gainrange.c_str(), ispdigitalgainrange.c_str(), exposuretimerange.c_str(), exposurecompensation,
			m_height, m_width, m_fps);
#endif
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

	const char *defaultConfig = "# White balence 0 : off / 1 : auto\n\
sensor-id=0\n\
wbmode=1\n\
tnr-mode=2\n\
tnr-strength=-1\n\
ee-mode=1\n\
ee-strength=-1\n\
gainrange=\"1 16\"\n\
ispdigitalgainrange=\"1 8\"\n\
exposuretimerange=\"5000000 10000000\"\n\
# exposure compensation from -2 ~ 2\n\
exposurecompensation=0\n\
# exposure threshold from 0 ~ 5\n\
exposurethreshold=3\n";

	void LoadConfig() {
		char fn[STR_SIZE];
		snprintf(fn, STR_SIZE, "%s/camera.config", CONFIG_FILE_DIR);
		ifstream in(fn);
		if(in.is_open() == false){
			cout << "!!! Load default config" << endl;
			ofstream out(fn);
			if(out.is_open()) {
				out << defaultConfig;
				out.close();
			}
		} else
			in.close();

		vector<pair<string, string> > cfg;
		if(ParseConfigFile(fn, cfg) > 0)
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
				m_width, m_height, m_fps);

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

	int Width() const { return m_width; }
	int Height() const { return m_height; }
	int Fps() const { return m_fps; }

	int ExposureThreshold() const { return exposurethreshold; }
};

//static Camera camera(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS);
static Camera camera;

/*
*
*/

#define FIRMWARE_FILEPATH "/tmp/firmware.img"
#define DRAGONEYE_FILEPATH "/usr/local/bin/dragon-eye"

static size_t write_data(void *ptr, size_t size, size_t nmemb, void *stream)
{
	size_t written = fwrite(ptr, size, nmemb, (FILE *)stream);
	return written;
}

static int progress_func(void* ptr, double TotalToDownload, double NowDownloaded, double TotalToUpload, double NowUploaded);

static int DownloadFirmware(unsigned int ip, unsigned short port, const char *pagefilename) {
	CURLcode res;
	CURL *curl_handle;
	FILE *pagefile;

	string url("http://");

	struct in_addr sin_addr;
	sin_addr.s_addr = htonl(ip);
	struct sockaddr_in sa;
	char buffer[INET_ADDRSTRLEN];
	inet_ntop( AF_INET, &sin_addr, buffer, sizeof(buffer));

	url.append(buffer);
	printf("Source UDP %s\n", buffer);
	url.append(":");
	url.append(to_string(port));
	url.append("/firmware.img");

	curl_global_init(CURL_GLOBAL_ALL);
	curl_handle = curl_easy_init(); /* init the curl session */
	curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str()); /* set URL to get here */
	curl_easy_setopt(curl_handle, CURLOPT_VERBOSE, 1L); /* Switch on full protocol/debug output while testing */
	curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 0L); // Internal CURL progressmeter must be disabled if we provide our own callback
	curl_easy_setopt(curl_handle, CURLOPT_PROGRESSFUNCTION, progress_func); // Install the callback function
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, write_data); /* send all data to this function  */

	pagefile = fopen(pagefilename, "wb+");
	if(pagefile) {
		curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, pagefile); /* write the page body to this file handle */
		res = curl_easy_perform(curl_handle); /* get it! */
		fclose(pagefile); /* close the header file */
	}
	curl_easy_cleanup(curl_handle); /* cleanup curl stuff */
	curl_global_cleanup();
 
	return res == CURLE_OK ? 0 : -1;
}

#define WLAN_STA    "wlan0"
#define WLAN_AP     "wlan9"
#define LAN_ETH		"eth0"

class F3xBase {
private:
	int m_ttyUSB0Fd, m_jy901Fd, m_ttyTHSxFd;
	int m_udpSocket, m_apMulticastSocket, m_staMulticastSocket, m_ethMulticastSocket;

	JetsonDevice_t m_jetsonDevice;
	BaseType_t m_baseType;

	jetsonGPIO m_redLED, m_greenLED, m_blueLED, m_relay;
	jetsonGPIO m_pushButton;

	bool m_isVideoOutputScreen;
	bool m_isVideoOutputFile;
	bool m_isVideoOutputRTP;
	bool m_isVideoOutputHLS;
	bool m_isVideoOutputRTSP;
	bool m_isVideoOutputResult;

	uint16_t m_udpLocalPort;

	string m_rtpRemoteHost;
	uint16_t m_rtpRemotePort;

	uint8_t m_mog2_threshold; /* 0 ~ 64 / Most senstive is 0 / Default 16 */
	bool m_isNewTargetRestriction;
	bool m_isFakeTargetDetection;
	bool m_isBugTrigger;
	uint16_t m_relayDebouence;
	uint16_t m_horizonRatio;
	bool m_isBuzzer;

	thread m_udpServerThread;
	bool m_bUdpServerRun;

	unsigned int m_srcIp;
	unsigned short m_srcPort;

	thread m_apMulticastSenderThread;
	bool m_bApMulticastSenderRun;

	thread m_staMulticastSenderThread;
	bool m_bStaMulticastSenderRun;

	thread m_ethMulticastSenderThread;
	bool m_bEthMulticastSenderRun;

	thread m_jy901sThread;

	int m_roll, m_pitch, m_yaw;

	bool m_bCompassSuspend = false;

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

	void Jy901sTask() {
		while(m_jy901Fd > 0) {
			if(bShutdown)
				break;
#if 1            
			uint8_t header[1] = { 0x00 };
			size_t r = ReadTty(m_jy901Fd, header, 1);
			if(r != 1) {
				continue;
			}
			if(header[0] == 0x55) {
				uint8_t command[1] = { 0x00 };
				r = ReadTty(m_jy901Fd, command, 1);
				if(r != 1) {
					continue;
				}
				if(command[0] >= 0x50 && command[0] <= 0x5a) {
					if(command[0] == 0x53) { /* Angle output */
						uint8_t data[9];
						r = ReadTty(m_jy901Fd, data, 9);
						if(r != 9) {
							continue;
						}
						uint8_t sum = header[0] + command[0];
						for(int i=0;i<8;i++) {
							sum += data[i];
						}
						if(sum != data[8]) {
							dprintf("0x%02x 0x%02x checksum error !!!\n", header[0], command[0]);
							continue;
						} else {
							m_roll = ((data[1] << 8) | data[0]) * 180 / 32768;
							m_pitch = ((data[3] << 8) | data[2]) * 180 / 32768;
							m_yaw = ((data[5] << 8) | data[4]) * 180 / 32768;
							dprintf("Roll %d, Pitch %d, Yaw %d\n", m_roll, m_pitch, m_yaw);
						}
					}
				} else {
					continue;
				}
			}
#else
			uint8_t data[16];
			size_t r = ReadTty(m_jy901Fd, data, 16);
			if(r > 0) {
				printf("[ JY901s ] ");
				for(size_t i=0;i<r;i++)
					printf("0x%02x ", data[i]);
				printf("\n");
			}
#endif
		}
	}

public:
	F3xBase() : m_ttyUSB0Fd(0), m_jy901Fd(0), m_ttyTHSxFd(0),
		m_udpSocket(0), m_apMulticastSocket(0), m_staMulticastSocket(0), m_ethMulticastSocket(0), 
		m_jetsonDevice(JETSON_NANO), m_baseType(BASE_A),
		m_redLED(gpio16), m_greenLED(gpio17), m_blueLED(gpio50), m_relay(gpio51), m_pushButton(gpio18),
		m_isVideoOutputScreen(false), 
		m_isVideoOutputFile(false), 
		m_isVideoOutputRTP(false), 
		m_isVideoOutputHLS(false),
		m_isVideoOutputRTSP(false),
		m_isVideoOutputResult(false),
		m_udpLocalPort(4999), 
		m_rtpRemotePort(5000),
		m_mog2_threshold(16),
		m_isNewTargetRestriction(false),
		m_isFakeTargetDetection(false),
		m_isBugTrigger(false),
		m_relayDebouence(800),
		m_horizonRatio(20),
		m_isBuzzer(true),
		m_bUdpServerRun(false),
		m_srcIp(0), m_srcPort(0),
		m_bApMulticastSenderRun(false),
		m_bStaMulticastSenderRun(false),
		m_bEthMulticastSenderRun(false),
		m_roll(0), m_pitch(0), m_yaw(0)
	{
		ifstream in;
		in.open("/proc/device-tree/model");
		if(in.is_open()) {
			string line;
			if(getline(in, line)) {
				cout << endl;
				cout << line << endl;
				cout << endl;
				if(line.find("NVIDIA Jetson Nano Developer Kit") != string::npos) {
					m_jetsonDevice = JETSON_NANO;
				} else if(line.find("NVIDIA Jetson Nano 2GB Developer Kit") != string::npos) {
					m_jetsonDevice = JETSON_NANO;
				} else if(line.find("NVIDIA Jetson Xavier NX Developer Kit") != string::npos) {
					m_jetsonDevice = JETSON_XAVIER_NX;
				}
			}
			in.close();
		}
	}

	~F3xBase() {}

	void SetupGPIO() {
/*
		cat /proc/device-tree/model
*/
		switch(m_jetsonDevice) {
			case JETSON_NANO:
				m_redLED = gpio16;
				m_greenLED = gpio17;
				m_blueLED = gpio50;
				m_relay = gpio51;
				m_pushButton = gpio18;
				break;
			case JETSON_XAVIER_NX:
				m_redLED = gpio493;
				m_greenLED = gpio492;
				m_blueLED = gpio428;
				m_relay = gpio429;
				m_pushButton = gpio491;
				break;
		}

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
	}

	int OpenTty(const char *dev, int speed, int parity) {
		int fd = open (dev, O_RDWR | O_NOCTTY | O_SYNC);
		if(fd > 0) {
			SetupTTY(fd, speed, parity);  // set speed to 9600 bps, 8n1 (no parity)
			printf("Open %s successful ...\n", dev);
		} else
			printf("Error %d opening %s: %s\n", errno, dev, strerror (errno));

		return fd;
	}

	int OpenTtyUSB0() {
		const char *ttyUSB0 = "/dev/ttyUSB0";
		m_ttyUSB0Fd = OpenTty(ttyUSB0, B9600, 0);

		return m_ttyUSB0Fd;
	}
	
	int OpenTtyJy901s() {
		switch(m_jetsonDevice) {
			case JETSON_NANO: m_jy901Fd = OpenTty("/dev/ttyTHS1", B9600, 0);
				break;
			case JETSON_XAVIER_NX: m_jy901Fd = OpenTty("/dev/ttyTHS0", B9600, 0);
				break;
		}

		if(m_jy901Fd > 0)
			m_jy901sThread = thread(&F3xBase::Jy901sTask, this);

		return m_jy901Fd;
	}

	int OpenTtyTHSx() {
		switch(m_jetsonDevice) {
			case JETSON_NANO: m_ttyTHSxFd = OpenTty("/dev/ttyTHS1", B9600, 0);
				break;
			case JETSON_XAVIER_NX: m_ttyTHSxFd = OpenTty("/dev/ttyTHS0", B9600, 0);
				break;
		}

		return m_ttyTHSxFd;
	}

	size_t ReadTty(int fd, uint8_t *data, size_t size) {
		if(fd <= 0 || data == 0 || size == 0)
			return 0;

		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000; /* 10ms */

		if(select(fd+1, &rfds, NULL, NULL, &tv) > 0) {
			size_t r = read(fd, data, size);
			return r;
		}
		return 0;
	}

	size_t WriteTty(int fd, const uint8_t *data, size_t size) {
		return (fd > 0) ? write(fd, data, size) : 0;
	}

	void TriggerTtyUSB0(bool newTrigger) {
		static uint16_t serNo = 0x1fff;
		
		if(m_ttyUSB0Fd <= 0)
			return;

		if(newTrigger) { /* It's NEW trigger */
			if(++serNo > 0x1fff)
				serNo = 0;
		}

		char raw[16] = {0};
		switch(m_baseType) {
			case BASE_A: snprintf(raw, 16, "<A%04d>", serNo);
				printf("TriggerTtyUSB0 : %s\r\n", raw);
				break;
			case BASE_B: snprintf(raw, 16, "<B%04d>", serNo);
				printf("TriggerTtyUSB0 : %s\r\n", raw);
				break;
			default:
				break;
		}
		write(m_ttyUSB0Fd, reinterpret_cast<const uint8_t *>(raw), strlen(raw));
	}

	void TriggerTtyTHSx(bool newTrigger) {
		static uint16_t serNo = 0x1fff;
		
		if(m_ttyTHSxFd <= 0)
			return;

		if(newTrigger) { /* It's NEW trigger */
			if(++serNo > 0x1fff)
				serNo = 0;
		}

		char raw[16] = {0};
		switch(m_baseType) {
			case BASE_A: snprintf(raw, 16, "<A%04d>", serNo);
				printf("TriggerTtyTHSx : %s\r\n", raw);
				break;
			case BASE_B: snprintf(raw, 16, "<B%04d>", serNo);
				printf("TriggerTtyTHSx : %s\r\n", raw);
				break;
			default:
				break;
		}
		write(m_ttyTHSxFd, reinterpret_cast<const uint8_t *>(raw), strlen(raw));
	}

	void CloseTtyUSB0() {
		if(m_ttyUSB0Fd > 0)
			close(m_ttyUSB0Fd);
		m_ttyUSB0Fd = 0;
	}

	void CloseTtyJy901s() {
		if(m_jy901Fd > 0)
			close(m_jy901Fd);
		m_jy901Fd = 0;

		if(m_jy901sThread.joinable())
			m_jy901sThread.join();
	}

	void CloseTtyTHSx() {
		if(m_ttyTHSxFd > 0)
			close(m_ttyTHSxFd);
		m_ttyTHSxFd = 0;
	}

	int OpenUdpSocket(uint16_t port) {
		int sockfd;
		struct sockaddr_in addr; 

		sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if(sockfd < 0) {
			printf("Error open UDP socket !!!\n");
			return 0;
		}

		int sock_opt = 1;
		if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void*)&sock_opt, sizeof(sock_opt)) < 0) {
			printf("UDP setsockopt failed!\n");
		}

		memset((char*) &(addr),0, sizeof((addr)));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(port);

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

		printf("Open UDP socket successful ...\n");

		return sockfd;
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

		int r = recvfrom(m_udpSocket,
			   data,
			   originalSize,
			   0,
			   (struct sockaddr *)&from,
			   (socklen_t*)&fromLen);
		if(r > 0) {
			m_srcPort = ntohs(from.sin_port);
			m_srcIp = ntohl(from.sin_addr.s_addr);

			struct sockaddr_in sa;
			char buffer[INET_ADDRSTRLEN];
			inet_ntop( AF_INET, &from.sin_addr, buffer, sizeof(buffer));
			printf("\nReceive UDP from %s:%u\n", buffer, m_srcPort);
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

	size_t WriteSourceUdpSocket(const uint8_t *data, size_t size) {
		if(!m_udpSocket || m_srcIp == 0 || m_srcPort == 0)
			return 0;

		struct sockaddr_in to;
		int toLen = sizeof(to);
		memset(&to, 0, toLen);
		
		to.sin_family = AF_INET;
		to.sin_port = htons(m_srcPort);
		to.sin_addr.s_addr = htonl(m_srcIp);
		
		int s = sendto(m_udpSocket, data, size, 0,(struct sockaddr*)&to, toLen);

		return s;
	}

	void TriggerSourceUdpSocket(bool newTrigger) {
		static uint16_t serNo = 0x1fff;
		if(newTrigger) { /* It's NEW trigger */
			if(++serNo > 0x1fff)
				serNo = 0;
		}

		char raw[16] = {0};
		switch(m_baseType) {
			case BASE_A: snprintf(raw, 16, "<A%04d>", serNo);
				printf("TriggerSourceUdpSocket : %s\r\n", raw);
				break;
			case BASE_B: snprintf(raw, 16, "<B%04d>", serNo);
				printf("TriggerSourceUdpSocket : %s\r\n", raw);
				break;
			default:
				break;
		}
		WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(raw), strlen(raw));
	}

	void CloseUdpSocket() {
		if(m_udpSocket)
			close(m_udpSocket);
		m_udpSocket = 0;
	}

	void UdpServerTask()
	{
		m_bUdpServerRun = true;
		
		int socketfd = 0;
		while(socketfd == 0) {
			if(bShutdown)
				return;
			socketfd = OpenUdpSocket(m_udpLocalPort);
			if(socketfd == 0) {
				printf("Open UDP socket fail ...\n");
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}

		m_udpSocket = socketfd;

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
				const char compass_lock[] = "#CompassLock";
				const char compass_unlock[] = "#CompassUnlock";
				const char compass_save_settings[] = "#CompassSaveSettings";
				const char compass_suspend[] = "#CompassSuspend";
				const char compass_resume[] = "#CompassResume";
				const char firmware_version[] = "#FirmwareVersion";
				const char error[] = "#Error";

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
					s_errorString.clear();
				} else if(line == "#Status") {
					if(s_errorString.size() > 0) {
						string raw("#Error:");
						raw.append(s_errorString);
						WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(raw.c_str()), raw.size());
					} else if(bStopped) /* Stopped */
						WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(stopped), strlen(stopped));
					else
						WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(started), strlen(started));
				} else if(line == "#CompassLock") {
					const uint8_t cmd[] = {0xff, 0xaa, 0x69, 0x88, 0xb5}; /* Enter command mode */
					WriteTty(m_jy901Fd, cmd, 5);
					std::this_thread::sleep_for(std::chrono::milliseconds(20));
					const uint8_t cal[] = {0xff, 0xaa, 0x01, 0x07, 0x00}; /* Magntic calibration mode */
					WriteTty(m_jy901Fd, cal, 5);
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(compass_lock), strlen(compass_lock));
				} else if(line == "#CompassUnlock") {
					const uint8_t cal[] = {0xff, 0xaa, 0x01, 0x00, 0x00}; /* Exit calibration */
					WriteTty(m_jy901Fd, cal, 5);
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(compass_unlock), strlen(compass_unlock));
				} else if(line == "#CompassSaveSettings") {
					const uint8_t cmd[] = {0xff, 0xaa, 0x00, 0x00, 0x00}; /* Save current settings */
					WriteTty(m_jy901Fd, cmd, 5);
					std::this_thread::sleep_for(std::chrono::milliseconds(200));
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(compass_save_settings), strlen(compass_save_settings));
				} else if(line == "#CompassSuspend") {
					if(m_bCompassSuspend)
						continue;
					m_bCompassSuspend = true;
					const uint8_t cal[] = {0xff, 0xaa, 0x22, 0x01, 0x00}; /* Suspend */
					WriteTty(m_jy901Fd, cal, 5);
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(compass_suspend), strlen(compass_suspend));
				} else if(line == "#CompassResume") {
					if(m_bCompassSuspend == false)
						continue;
					m_bCompassSuspend = false;
					const uint8_t cal[] = {0xff, 0xaa, 0x22, 0x01, 0x00}; /* Resume */
					WriteTty(m_jy901Fd, cal, 5);
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(compass_resume), strlen(compass_resume));
				} else if(line == "#FirmwareVersion") {
					string ans = firmware_version;
					ans.append(":");
					ans.append(VERSION);
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ans.c_str()), ans.size());
				} else if(line == "#FirmwareUpgrade") {
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ack), strlen(ack));
					if(DownloadFirmware(m_srcIp, 8080, FIRMWARE_FILEPATH) == 0) {
						printf("Download firmware successful ...\n");

						std::ifstream src(FIRMWARE_FILEPATH, ios::binary);
						std::ofstream dst(DRAGONEYE_FILEPATH, ios::binary | ios::trunc);
						if(src.is_open() && dst.is_open()) {
							dst << src.rdbuf();
							src.close();
							dst.close();
						} else {
							if(src.is_open())
								src.close();
							if(dst.is_open())
								dst.close();
							string result("#FirmwareUpgrade:Failed");
							WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(result.c_str()), result.length());
						}

						printf("Update firmware successful ...\n");

						string result("#FirmwareUpgrade:Success");
						for(int i=0;i<3;i++) {
							std::this_thread::sleep_for(std::chrono::seconds(1));
							WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(result.c_str()), result.length());
						}

						kill(getpid(), SIGINT); /* Exit then wait systemctl to restart */
					} else {
						string result("#FirmwareUpgrade:Failed");
						WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(result.c_str()), result.length());
					}
				} else if(line.find("#VideoFiles:", 0) == 0) { /* Start with #VideoFiles:*/
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ack), strlen(ack));
					stringstream ss(line);
					vector<string> rs;
					string item;
					while(getline(ss, item, ':')) {
						rs.push_back(item);
					} 
					if(rs.size() >= 2) {
						if(rs[1] == "DeleteAll") {
							printf("Delete all video files in %s\n", VIDEO_OUTPUT_DIR);
							int count;
							DIR *dir;
							struct dirent *ent;
							if((dir = opendir(VIDEO_OUTPUT_DIR)) != NULL) {
								while((ent = readdir(dir)) != NULL) {
									if(ent->d_name) {
										if((strcmp(".", ent->d_name) == 0) ||
											strcmp("..", ent->d_name) == 0)
											continue;
									}
									string fn = VIDEO_OUTPUT_DIR;
									fn.append("/");
									fn.append(ent->d_name);
									printf("Delete %s ...\n", fn.c_str());
									remove(fn.c_str());
								}
								closedir(dir);
							}
						} else if(rs[1] == "Count") {
							int count;
							DIR *dir;
							struct dirent *ent;
							if((dir = opendir(VIDEO_OUTPUT_DIR)) != NULL) {
								while((ent = readdir(dir)) != NULL) {
									printf("%s\n", ent->d_name);
									count++;
								}
								closedir(dir);
							}
						}
					}
				} else if(line.find("#SystemCommand:", 0) == 0) {
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ack), strlen(ack));
					size_t pos = line.find(':', 0);
					string cmd = line.substr(pos+1, string::npos);
					printf("Run command : %s\n", cmd.c_str());
					system(cmd.c_str());
				} else {
					WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(ack), strlen(ack));
				}
			}
		}
		CloseUdpSocket();
	}

	void StartUdpServer() {
		m_udpServerThread = thread(&F3xBase::UdpServerTask, this);
	}

	void StopUdpServer() {
		m_bUdpServerRun = false;
		if(m_udpServerThread.joinable())
			m_udpServerThread.join();
	}

	int OpenMulticastSocket(const char *group, uint16_t port, const char *ifname) {
		string result;
		const char *ip = ipv4_address(ifname, result);
		dprintf("OpenMulticastSocket : <%s> %s / %s:%u\n", ifname, result.c_str(), group, port);

		if(group == 0 || strlen(group) == 0)
			return 0;
		if(ifname == 0 || strlen(ifname) == 0)
			return 0;
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
		mreq.imr_interface.s_addr = inet_addr(result.c_str()); /* wlan9 ip address */

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

		dprintf("WriteMulticastSocket : <%s> / %s:%u\n", ifname, group, port);
		
		if(ioctl(sockfd, SIOCGIFFLAGS, &ifr) >= 0 &&
			(ifr.ifr_flags &IFF_UP)) {
			if(setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifname, strlen(ifname)) == -1) {
				printf("WriteMulticastSocket : setsocketopt %s SO_BINDTODEVICE fail !!!\n", ifname);
			} else {
				int r = sendto(sockfd, data, size, 0,(struct sockaddr*)&addr, sizeof(addr));
				if(r < 0)
					perror(ifname);
				else
					ret = r;
			}
		} else
			perror(ifname);
	}

	void TriggerMulticastSocket(bool newTrigger) {
		static uint16_t serNo = 0x1fff;
		if(newTrigger) { /* It's NEW trigger */
			if(++serNo > 0x1fff)
				serNo = 0;
		}

		char raw[16] = {0};
		switch(m_baseType) {
			case BASE_A: snprintf(raw, 16, "<A%04d>", serNo);
				printf("TriggerMulticastSocket : %s\r\n", raw);
				break;
			case BASE_B: snprintf(raw, 16, "<B%04d>", serNo);
				printf("TriggerMulticastSocket : %s\r\n", raw);
				break;
			default:
				break;
		}

		if(m_apMulticastSocket)
			WriteMulticastSocket(m_apMulticastSocket, "224.0.0.2", 9002, WLAN_AP, reinterpret_cast<const uint8_t *>(raw), strlen(raw));
		if(m_staMulticastSocket)
			WriteMulticastSocket(m_staMulticastSocket, "224.0.0.3", 9003, WLAN_STA, reinterpret_cast<const uint8_t *>(raw), strlen(raw));
		if(m_ethMulticastSocket)
			WriteMulticastSocket(m_ethMulticastSocket, "224.0.0.3", 9003, LAN_ETH, reinterpret_cast<const uint8_t *>(raw), strlen(raw));
	}

	string MulticastRaw(string & ip) {
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
		raw.append(ip.c_str());
/*
		raw.push_back(':');
		raw.append(to_string(m_yaw));
*/
		raw.push_back(':');
		raw.append(to_string(s_fps));		

		ifstream in;
		in.open("/sys/devices/virtual/thermal/thermal_zone0/temp"); // AO-therm
		if(in.is_open()) {
			raw.push_back(':');
			string s;
			in >> s;
			raw.append(s);
			in.close();
		}

		in.open("/sys/devices/gpu.0/load"); // GPU load
		if(in.is_open()) {
			raw.push_back(':');
			string s;
			in >> s;
			raw.append(s);
			in.close();
		}

		return raw;
	}

	void ApMulticastSenderTask() {
		m_bApMulticastSenderRun = true;

		while(m_apMulticastSocket == 0) {
			if(bShutdown)
				return;
			m_apMulticastSocket = OpenMulticastSocket("224.0.0.2", 9002, WLAN_AP);
			if(m_apMulticastSocket == 0) 
				std::this_thread::sleep_for(std::chrono::seconds(1));            
		}

		while(m_bApMulticastSenderRun) {
			if(bShutdown)
				break;

			if(m_apMulticastSocket) {
				string result;
				const char *ip = ipv4_address(WLAN_AP, result);
				if(ip) {
					string raw = MulticastRaw(result);
					WriteMulticastSocket(m_apMulticastSocket, "224.0.0.2", 9002, WLAN_AP, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
				} else {
					/* As AP we should NOT be here ... */
				}
			}

			std::this_thread::sleep_for(std::chrono::seconds(2));
		}
		
		if(m_apMulticastSocket) {
			close(m_apMulticastSocket);
			m_apMulticastSocket = 0;
		}
	}

	void StartApMulticastSender() {
		m_apMulticastSenderThread = thread(&F3xBase::ApMulticastSenderTask, this);
	}

	void StopApMulticastSender() {
		m_bApMulticastSenderRun = false;
		if(m_apMulticastSenderThread.joinable())
			m_apMulticastSenderThread.join();
	}

	void StaMulticastSenderTask() {
		m_bStaMulticastSenderRun = true;

		while(m_staMulticastSocket == 0) {
			if(bShutdown)
				return;
			m_staMulticastSocket = OpenMulticastSocket("224.0.0.3", 9003, WLAN_STA);
			if(m_staMulticastSocket == 0)
				std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		while(m_bStaMulticastSenderRun) {
			if(bShutdown)
				break;

			if(m_staMulticastSocket) {
				string result;
				const char *ip = ipv4_address(WLAN_STA, result);
				if(ip) {
					string raw = MulticastRaw(result);
					WriteMulticastSocket(m_staMulticastSocket, "224.0.0.3", 9003, WLAN_STA, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
				} else {
					close(m_staMulticastSocket);
					m_staMulticastSocket = 0;
				}
			} else { /* Wifi disconnected ... */
				m_staMulticastSocket = OpenMulticastSocket("224.0.0.3", 9003, WLAN_STA);
				if(m_staMulticastSocket == 0) {
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;              
				}
			}

			std::this_thread::sleep_for(std::chrono::seconds(2));
		}
		
		if(m_staMulticastSocket) {
			close(m_staMulticastSocket);
			m_staMulticastSocket = 0;
		}
	}

	void StartStaMulticastSender() {
		m_staMulticastSenderThread = thread(&F3xBase::StaMulticastSenderTask, this);
	}

	void StopStaMulticastSender() {
		m_bStaMulticastSenderRun = false;
		if(m_staMulticastSenderThread.joinable())
			m_staMulticastSenderThread.join();
	}

	void EthMulticastSenderTask() {
		m_bEthMulticastSenderRun = true;

		while(m_ethMulticastSocket == 0) {
			if(bShutdown)
				return;
			m_ethMulticastSocket = OpenMulticastSocket("224.0.0.3", 9003, LAN_ETH);
			if(m_ethMulticastSocket == 0)
				std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		while(m_bEthMulticastSenderRun) {
			if(bShutdown)
				break;

			if(m_ethMulticastSocket) {
				string result;
				const char *ip = ipv4_address(LAN_ETH, result);
				if(ip) {
					string raw = MulticastRaw(result);
					WriteMulticastSocket(m_ethMulticastSocket, "224.0.0.3", 9003, LAN_ETH, reinterpret_cast<const uint8_t *>(raw.c_str()), raw.length());
				} else {
					close(m_ethMulticastSocket);
					m_ethMulticastSocket = 0;
				}
			} else { /* Ethernet disconnected ... */
				m_ethMulticastSocket = OpenMulticastSocket("224.0.0.3", 9003, LAN_ETH);
				if(m_ethMulticastSocket == 0) {
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
			}

			std::this_thread::sleep_for(std::chrono::seconds(2));
		}

		if(m_ethMulticastSocket) {
			close(m_ethMulticastSocket);
			m_ethMulticastSocket = 0;
		}
	}

	void StartEthMulticastSender() {
		m_ethMulticastSenderThread = thread(&F3xBase::EthMulticastSenderTask, this);
	}

	void StopEthMulticastSender() {
		m_bEthMulticastSenderRun = false;
		if(m_ethMulticastSenderThread.joinable())
			m_ethMulticastSenderThread.join();
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
			} else if(it->first == "base.relay.debouence") {
				string & s = it->second;
				if(::all_of(s.begin(), s.end(), ::isdigit))
					m_relayDebouence = stoi(s);
				else
					cout << "Invalid " << it->first << "=" << s << endl;			
			} else if(it->first == "base.horizon.ratio") {
				string & s = it->second;
				if(::all_of(s.begin(), s.end(), ::isdigit))
					m_horizonRatio = stoi(s);
				else
					cout << "Invalid " << it->first << "=" << s << endl;			
			} else if(it->first == "base.buzzer") {
				if(it->second == "yes" || it->second == "1")
					m_isBuzzer = true;
				else
					m_isBuzzer = false;
			}
		}
	}

	const char *defaultConfig = "base.type=A\n\
base.rtp.remote.host=10.0.0.238\n\
base.rtp.remote.port=5000\n\
video.output.screen=no\n\
video.output.file=no\n\
video.output.rtp=no\n\
video.output.hls=no\n\
video.output.rtsp=yes\n\
video.output.result=no\n\
base.mog2.threshold=32\n\
base.new.target.restriction=no\n\
base.relay.debouence=800\n\
base.horizon.ratio=20\n\
base.buzzer=yes";

	void LoadSystemConfig() {
		char fn[STR_SIZE];
		snprintf(fn, STR_SIZE, "%s/system.config", CONFIG_FILE_DIR);
		ifstream in(fn);
		if(in.is_open() == false){
			cout << "!!! Load default config" << endl;
			ofstream out(fn);
			if(out.is_open()) {
				out << defaultConfig;
				out.close();
			}
		} else
			in.close();

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

	inline bool IsVideoOutput() const {
		return (m_isVideoOutputScreen || m_isVideoOutputFile || m_isVideoOutputRTP || m_isVideoOutputHLS);
	}

	inline bool IsVideoOutputRTSP() const {
		return m_isVideoOutputRTSP;
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

	inline uint16_t RelayDebouence() const {
		return m_relayDebouence;
	}

	inline uint16_t HorizonRatio() const {
		return m_horizonRatio;
	}

	inline bool IsBuzzer() const {
		return m_isBuzzer;
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
		static pinValues s_onOff = off;
		if(onOff != s_onOff) {
			printf("Relay %s\n", onOff ? "on" : "off");
			s_onOff = onOff;     
		}
		gpioSetValue(m_relay, onOff);
	}

	unsigned int GetPushButton() {
		unsigned int gv;
		gpioGetValue(m_pushButton, &gv);
		return gv;
	}

	//int Roll() { return m_roll; }
	int Roll() {
		int roll = m_roll;
		if(m_roll > 180) {
			roll = (360 - m_roll) * (-1);
		}
		return roll;
	}

	//int Pitch() { return m_pitch; }
	int Pitch() {
		int pitch = m_pitch;
		if(m_pitch > 180) {
			pitch = (360 - m_pitch) * (-1);
		}
		return pitch;
	}

	int Yaw() { return m_yaw; }

	void Error(const char *str) {
		string raw("#Error:");
		raw.append(str);
		WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(raw.c_str()), raw.size());
	}

	static void Initialisize();
	static void Start();
	static void Stop();
};

static F3xBase f3xBase; 

int progress_func(void* ptr, double TotalToDownload, double NowDownloaded, double TotalToUpload, double NowUploaded)
{
	// ensure that the file to be downloaded is not empty
	// because that would cause a division by zero error later on
	if(TotalToDownload <= 0.0) {
		return 0;
	}

	int totaldotz = 100; // how wide you want the progress meter to be
	double fractiondownloaded = NowDownloaded / TotalToDownload;
	// part of the progressmeter that's already "full"
	int dotz = (int) round(fractiondownloaded * totaldotz);

	string progress("#UpgradeProgress:");
	progress.append(to_string(dotz));
	f3xBase.WriteSourceUdpSocket(reinterpret_cast<const uint8_t *>(progress.c_str()), progress.length());

	return 0;
}

/*
*
*/

static Size minTargetSize(MIN_TARGET_WIDTH, MIN_TARGET_HEIGHT);
static Size maxTargetSize(MAX_TARGET_WIDTH, MAX_TARGET_HEIGHT);

void F3xBase::Initialisize()
{
	switch(f3xBase.m_jetsonDevice) {
		case JETSON_NANO:
			camera.Initialisize(720, 1280, 30); /* 720p */
			tracker.Initialisize(720, 1280);
			minTargetSize.width = 6;
			minTargetSize.height = 8;
			maxTargetSize.width = 320;
			maxTargetSize.height = 320;
			break;
		case JETSON_XAVIER_NX:
			camera.Initialisize(1080, 1920, 30); /* 1080p */
			tracker.Initialisize(1080, 1920);
			//camera.Initialisize(720, 1280, 60); /* 720p60 */
			//tracker.Initialisize(720, 1280);
			minTargetSize.width = 9;
			minTargetSize.height = 12;
			maxTargetSize.width = 480;
			maxTargetSize.height = 480;
			break;
	}
}

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
	camera.UpdateExposure();

	if(camera.Open() == false) {
		s_errorString = "Camera";
		f3xBase.Error(s_errorString.c_str());
		bStopped = true;
		return;
	}

	tracker.UpdateHorizonRatio(f3xBase.HorizonRatio());

	if(f3xBase.IsNewTargetRestriction())
		tracker.NewTargetRestriction(Rect(180, camera.Height() - 180, 360, 180));
	else
		tracker.NewTargetRestriction(Rect());

	/* background history count, varThreshold, shadow detection */
	bsModel = cuda::createBackgroundSubtractorMOG2(30, f3xBase.Mog2Threshold(), false);
	//cout << bsModel->getVarInit() << " / " << bsModel->getVarMax() << " / " << bsModel->getVarMax() << endl;
	/* Default variance of each gaussian component 15 / 75 / 75 */ 
	bsModel->setVarInit(15);
	bsModel->setVarMax(20);
	bsModel->setVarMin(4);    

	cout << endl;
	cout << "*** Object tracking started ***" << endl;

	Mat frame;
	for(int i=0;i<30;i++) /* Read out unstable frames ... */
		camera.Read(frame);

	if(f3xBase.IsVideoOutput()) { /* NOT include RTSP video output */
		F3xBase & fb = f3xBase;
		videoOutputThread = thread(&VideoOutputTask, fb.BaseType(), fb.IsVideoOutputScreen(), fb.IsVideoOutputFile(), 
			fb.IsVideoOutputRTP(), fb.RtpRemoteHost(), fb.RtpRemotePort(), 
			fb.IsVideoOutputHLS(), camera.Width(), camera.Height(), camera.Fps());
	}

	if(f3xBase.IsVideoOutputRTSP()) {
		rtspServerThread = thread(&gst_rtsp_server_task, camera.Width(), camera.Height(), camera.Fps());
		cout << endl;
		cout << "*** Start RTSP video ***" << endl;
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

	if(f3xBase.IsVideoOutputRTSP()) {
		if(f3xBase.IsVideoOutputRTSP())
			gst_rtsp_server_close_clients();
		cout << endl;
		cout << "*** Stop RTSP video ***" << endl;
		kill(getpid(), SIGUSR1);
		if(rtspServerThread.joinable())
			rtspServerThread.join();
	}

	camera.Close();

	signal(SIGUSR1, SIG_IGN); /* Ignore SIGUSR1 here or causes abnormal exit code */

	bStopped = true;
	s_fps = 0;
}

static void contour_moving_object(Mat & frame, Mat & foregroundFrame, list<Rect> & roiRect)
{
	uint32_t num_target = 0;

	vector< vector<Point> > contours;
	vector< Vec4i > hierarchy;
	findContours(foregroundFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
#if 1 /* Anti exposure burst */   
	if(contours.size() > 64)
		return;
#endif
	vector<Rect> boundRect;

    for(int i=0; i<contours.size(); i++) {
        Rect r = boundingRect(Mat(contours[i]));
        if(r.area() < 400 && r.br().y > tracker.HorizonHeight()) {
	        Point ct = Center(r);
	        bool isMerged = false;
	        for(auto it=boundRect.begin();it!=boundRect.end();++it) {
				if(it->area() < 400) { /* Less than 20x20 */
	                if((r & *it).area() > 0) { /* Merge overlaped rect */
	                    isMerged = true;
	                } else if(cv::norm(ct - Center(*it)) < 36) { /* Merge closely enough rect */
	                    isMerged = true;
	                }
	            }
	            if(isMerged) {
	                *it = MergeRect(r, *it);
	                break;
	            }
	        }
	        if(isMerged)
	            continue;
	    }
        boundRect.push_back(r);
    }

	sort(boundRect.begin(), boundRect.end(), [](const Rect & r1, const Rect & r2) {  
			return (r1.area() > r2.area()); /* Area */
		}); /* Rects sort by area, boundRect[0] is largest */

	for(int i=0; i<boundRect.size(); i++) {
		if(boundRect[i].width > maxTargetSize.width &&
			boundRect[i].height > maxTargetSize.height)
			continue; /* Extremely large object */

		if(boundRect[i].width < minTargetSize.width && 
			boundRect[i].height < minTargetSize.height)
			break; /* Rest are small objects, ignore them */

		Mat roiFrame = frame(boundRect[i]);
#if 1 /* Anti cloud ... */
		double minVal; 
		double maxVal; 
		Point minLoc; 
		Point maxLoc;

		minMaxLoc(roiFrame, &minVal, &maxVal, &minLoc, &maxLoc ); 
			/* If difference of max and min value of ROI rect is too small then it could be noise such as cloud or sea */
		if((maxVal - minVal) < 16)
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
		kill(getpid(), SIGUSR1); /* To stop RTSP server */
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

	signal(SIGUSR1, SIG_IGN);

	ofstream pf(PID_FILE); 
	if(pf) {
		pf << getpid();
		pf.close();
	} else
		cout << "Error open " << PID_FILE << endl;

	cuda::printShortCudaDeviceInfo(cuda::getDevice());
	std::cout << cv::getBuildInformation() << std::endl;

	f3xBase.Initialisize();
	f3xBase.SetupGPIO();
	f3xBase.OpenTtyUSB0();
	//f3xBase.OpenTtyJy901s();
	f3xBase.OpenTtyTHSx();
	f3xBase.LoadSystemConfig();
	f3xBase.StartUdpServer();
	f3xBase.StartApMulticastSender();
	f3xBase.StartStaMulticastSender();
	f3xBase.StartEthMulticastSender();

	camera.LoadConfig();
	camera.UpdateExposure();

	int cy = camera.Height() - 1;
	int cx = (camera.Width() / 2) - 1;

	int erosion_size = 1;   
	elementErode = cv::getStructuringElement(cv::MORPH_RECT,
					cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
					cv::Point(-1, -1) ); /* Default anchor point */

	int dilate_size = 2;   
	elementDilate = cv::getStructuringElement(cv::MORPH_RECT,
					cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), 
					cv::Point(-1, -1) ); /* Default anchor point */

	cout << endl;
	cout << "### Press button to start object tracking !!!" << endl;

	double fps = CAMERA_FPS;
	double dt_us = 1000000.0 / CAMERA_FPS;
	steady_clock::time_point t1(steady_clock::now());
	uint64_t loopCount = 0;

	if(bStopped == false)
		F3xBase::Start();

	auto lastTriggerTime(steady_clock::now());
	auto lastRelayTriggerTime(steady_clock::now());

	uint8_t doTriggerCount = 0;

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
			if(f3xBase.IsVideoOutput() || f3xBase.IsVideoOutputRTSP()) {
				capFrame.copyTo(outFrame);
				line(outFrame, Point(cx, 0), Point(cx, cy), Scalar(0, 255, 0), 1);
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
			if(f3xBase.IsVideoOutput() || f3xBase.IsVideoOutputRTSP()) {
				for(list<Rect>::iterator rr=roiRect.begin();rr!=roiRect.end();++rr)
					rectangle( outFrame, rr->tl(), rr->br(), Scalar(0, 255, 0), 2, 8, 0 );
				if(f3xBase.IsNewTargetRestriction()) {
					Rect nr = tracker.NewTargetRestrictionRect();
					rectangle(outFrame, nr.tl(), nr.br(), Scalar(127, 0, 127), 2, 8, 0 );
					writeText(outFrame, "New Target Restriction Area", Point(120, CAMERA_HEIGHT - 180));
				}
				//line(outFrame, Point(0, (camera.Height() / 5) * 4), Point(camera.Width(), (camera.Height() / 5) * 4), Scalar(127, 127, 0), 1);
				line(outFrame, Point(0, tracker.HorizonHeight()), Point(camera.Width(), tracker.HorizonHeight()), Scalar(0, 255, 255), 1);
/*
				int viewAngle = 60;
				int yOffset = 0;
				int interval = camera.Width() / viewAngle;
				int offset = f3xBase.Yaw() % 10;
				for(int i=0;i<viewAngle;i++) {
					if((offset + i) % 10 == 0) { // Long line
						line(outFrame, Point(i * interval, 0), Point(i * interval, 36), Scalar(0, 255, 0), 1);
						int yaw = (f3xBase.Yaw() + i) - (viewAngle / 2);
						if(yaw < 0)
							yaw += 360;
						String strYaw = to_string(yaw);
						putText(outFrame, strYaw.c_str(), Point(i * interval - (strYaw.size() * 10), 80), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 2, cv::LINE_8);
					 } else if((offset + i) % 10 == 5) { //
						line(outFrame, Point(i * interval, 0), Point(i * interval, 30), Scalar(0, 255, 0), 1);
					 } else { // Short line
						line(outFrame, Point(i * interval, 0), Point(i * interval, 24), Scalar(0, 255, 0), 1);
					}
				}
*/				
			}
		}

		f3xBase.RedLed(off);
		f3xBase.Relay(off);

		tracker.Update(roiRect, f3xBase.IsFakeTargetDetection());

		list< Target > & targets = tracker.TargetList();

		if(f3xBase.IsVideoOutput() || f3xBase.IsVideoOutputRTSP()) {
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
				if(f3xBase.IsVideoOutput() || f3xBase.IsVideoOutputRTSP()) {
					t->Draw(outFrame, true); /* Draw target */
				}
			}

			if(t->TriggerCount() > 0 && t->TriggerCount() < MAX_NUM_TRIGGER)
				doTrigger = true;

			if(t->ArcLength() > MIN_COURSE_LENGTH && 
				t->AbsLength() > MIN_COURSE_LENGTH && 
				t->TrackedCount() > MIN_TARGET_TRACKED_COUNT) {
				if((t->BeginCenterPoint().x > cx && t->EndCenterPoint().x <= cx) ||
						(t->BeginCenterPoint().x < cx && t->EndCenterPoint().x >= cx) ||
						(t->PreviousCenterPoint().x > cx && t->CurrentCenterPoint().x <= cx) ||
						(t->PreviousCenterPoint().x < cx && t->CurrentCenterPoint().x >= cx)) {
					bool tgr = t->Trigger(f3xBase.IsBugTrigger());
					if(doTrigger == false)
						doTrigger = tgr;
				}
			}
		}

		if(doTrigger || doTriggerCount > 0) { /* t->TriggerCount() > 0 */
			if(f3xBase.IsVideoOutputResult()) {
				if(f3xBase.IsVideoOutput() || f3xBase.IsVideoOutputRTSP())
					line(outFrame, Point(cx, 0), Point(cx, cy), Scalar(0, 0, 255), 3);
			}

			long long duration = duration_cast<milliseconds>(steady_clock::now() - lastRelayTriggerTime).count();
			//printf("duration = %lld\n" , duration);
			if(duration > f3xBase.RelayDebouence()) {
				f3xBase.Relay(on);
				lastRelayTriggerTime = steady_clock::now();
			}

			bool isNewTrigger = false;
			duration = duration_cast<milliseconds>(steady_clock::now() - lastTriggerTime).count();
			if(duration > 330) /* new trigger */
				isNewTrigger = true;

			if(isNewTrigger)
				doTriggerCount = MAX_NUM_TRIGGER;

			lastTriggerTime = steady_clock::now();

			f3xBase.TriggerMulticastSocket(isNewTrigger);
			f3xBase.TriggerSourceUdpSocket(isNewTrigger);
			f3xBase.TriggerTtyTHSx(isNewTrigger);			
			f3xBase.TriggerTtyUSB0(isNewTrigger);
			if(f3xBase.IsBuzzer())
				f3xBase.RedLed(on);

			if(doTriggerCount > 0)
				doTriggerCount--;
		} 

		if(f3xBase.IsVideoOutput() || f3xBase.IsVideoOutputRTSP()) {
			if(f3xBase.IsVideoOutputResult()) {
				writeText(outFrame, currentDateTime(), Point(40, 160));
				char str[32];
				snprintf(str, 32, "FPS %.2lf", fps);
				writeText(outFrame, string(str), Point( 40, 200 ));
				
				//snprintf(str, 32, "Yaw %d", f3xBase.Yaw());
				//writeText(outFrame, string(str), Point( 480, 200 ));
/*
				int xOffset = 400;
				int yOffset = 200;
				writeText(outFrame, "Roll", Point( xOffset, yOffset ));
				writeText(outFrame, "Pitch", Point( xOffset + 120, yOffset ));
				writeText(outFrame, "Yaw", Point( xOffset + 240, yOffset ));
				writeText(outFrame, to_string(f3xBase.Roll()), Point( xOffset, yOffset + 40 ));
				writeText(outFrame, to_string(f3xBase.Pitch()), Point( xOffset + 120, yOffset + 40 ));
				writeText(outFrame, to_string(f3xBase.Yaw()), Point( xOffset + 240, yOffset + 40 ));
*/
				snprintf(str, 32, "MOG2 threshold %d", f3xBase.Mog2Threshold());
				writeText(outFrame, str, Point( 40, 240 ));
				snprintf(str, 32, "Exposure threshold %d", camera.ExposureThreshold());
				writeText(outFrame, str, Point( 40, 280 ));

				if(f3xBase.IsVideoOutput())
					videoOutputQueue.push(outFrame);
				if(f3xBase.IsVideoOutputRTSP() && g_clientCount.load() > 0)
					videoRtspQueue.push(outFrame);
			} else {
				if(f3xBase.IsVideoOutput())
					videoOutputQueue.push(capFrame);
				if(f3xBase.IsVideoOutputRTSP() && g_clientCount.load() > 0)
					videoRtspQueue.push(capFrame);
			}
		}

		steady_clock::time_point t2(steady_clock::now());
		dt_us = (dt_us + static_cast<double>(duration_cast<microseconds>(t2 - t1).count())) / 2;

		/* t2 - t1 = interval of loop */
		/* t2 - t3 = detection process time */
		if(loopCount > 0 && loopCount % VIDEO_OUTPUT_FPS == 0) /* Display fps every second */
			std::cout << "FPS : " << fixed  << setprecision(2) <<  (1000000.0 / dt_us) << " / " << duration_cast<milliseconds>(t2 - t3).count() << " ms" << std::endl;

		s_fps = static_cast<int>(1000000.0 / dt_us);

		t1 = steady_clock::now();
	}

	f3xBase.GreenLed(off); /* Flash during frames */
	f3xBase.BlueLed(off); /* Flash during file save */
	f3xBase.RedLed(off); /* While object detected */
	f3xBase.Relay(off);

	if(bStopped == false)
		F3xBase::Stop();

	f3xBase.StopEthMulticastSender();
	f3xBase.StopStaMulticastSender();
	f3xBase.StopApMulticastSender();
	f3xBase.StopUdpServer();
	f3xBase.CloseTtyUSB0();
	//f3xBase.CloseTtyJy901s();
	f3xBase.CloseTtyTHSx();

	cout << endl;
	cout << "Finished ..." << endl;

	unlink(PID_FILE);

	return 0;     
}
