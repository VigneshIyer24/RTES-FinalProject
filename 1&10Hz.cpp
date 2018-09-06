//#define _GNU_SOURCE
#include <sys/utsname.h>
#include <unistd.h>
#include <sys/utsname.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <semaphore.h>
#include <pthread.h>
#include <opencv/highgui.h>
#include<opencv2/imgproc.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sstream>
#include <sched.h>
#include <time.h>
#include <syslog.h>
#include <sys/time.h>

#include <errno.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (1)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (2+1)
#define HRES 640
#define VRES 480
using namespace cv;
using namespace std;
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE, abortS5=FALSE, abortS6=FALSE, abortS7=FALSE;
int frames=0, savedframe=0;
char username[20];
char host_name[20];
char imgplot[]="imggraph.csv";
sem_t semS1, semS2;
struct timeval start_time_val,current_time_val;
struct  utsname sysinfo;
struct timespec timestamp;
IplImage* frame;
IplImage* save_frame;
CvCapture* capture;
typedef struct
{
    int threadIdx;
    unsigned long long sequencePeriods;
} threadParams_t;


void *Sequencer(void *threadp);

void *Service_1(void *threadp);
void *Service_2(void *threadp);
double getTimeMsec(void);
void print_scheduler(void);


int main(int argc, char** argv)
{
         
    int i, rc, scope;
	//	capture=(CvCapture*)cvCreateCameraCapture(0);
	
    cpu_set_t threadcpu;
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid;
    cpu_set_t allcpuset;
    printf("Starting Sequencer Demo\n");
    gettimeofday(&start_time_val, (struct timezone *)0);
    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

  // printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
  //  print_scheduler();


    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    for(i=0; i < NUM_THREADS; i++)
    {

      CPU_ZERO(&threadcpu);
      CPU_SET(3, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      //rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1	@ 1 Hz
    //
    rt_param[1].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1],               // pointer to thread descriptor
                      &rt_sched_attr[1],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[1]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");


    // Service_2 = RT_MAX-2	@ 1 Hz
    //
    rt_param[2].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");


 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods=60000;

    // Sequencer = RT_MAX	@ 30 Hz
    //
    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer service 0");
    else
        printf("pthread_create successful for sequeencer service 0\n");


   for(i=0;i<NUM_THREADS;i++)
       pthread_join(threads[i], NULL);

	 printf("\nTEST COMPLETE\n");
}


void *Sequencer(void *threadp)
{
    struct timeval current_time_val;
  //  struct timespec delay_time = {0,33315333}; // delay for 33.33 msec, 30 Hz
    struct timespec delay_time = {0,33333000};
    struct timespec remaining_time;
    double current_time;
    double residual;
     threadParams_t *threadParams = (threadParams_t *)threadp;
	long int check=1;
	while(1)
	{
		nanosleep(&delay_time,&remaining_time);
		check++;
		if(check%3==0)
		{
			sem_post(&semS1);
		}
	}
	
	 gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
  

}

void *Service_1(void *threadp)
{
	cvNamedWindow("Recording",CV_WINDOW_AUTOSIZE);
		capture=(CvCapture*)cvCreateCameraCapture(0);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,HRES);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,VRES);
	double takentime;
	while(1)
	{
		sem_wait(&semS1);
			std::ofstream plotting;
    		gettimeofday(&current_time_val, (struct timezone *)0);
    		syslog(LOG_CRIT, "Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    	//	printf("Frame Sampler thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
		takentime=((double)(current_time_val.tv_sec-start_time_val.tv_sec)+(double)(current_time_val.tv_usec/1000000));
		plotting.open(imgplot,std::ios_base::app);
		plotting<<frames;
		plotting<<",";
		plotting<<takentime;
		plotting<<"\r\n";
		
		plotting.close();
		frame=cvQueryFrame(capture);
		Mat mat_frame(cvarrToMat(frame));
	//	printf("Frame = %d \n",frames);
		if(!frame) break;
		//	imshow("Image",mat_frame);
		frames++;
		clock_gettime(CLOCK_REALTIME,&timestamp);
        	gettimeofday(&current_time_val, (struct timezone *)0);
//        	syslog(LOG_CRIT, "Frame Sampler release %llu @ sec=%d, msec=%d\n", S1Cnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    	   	gettimeofday(&current_time_val, (struct timezone *)0);
  		syslog(LOG_CRIT, "Time-stamp with Image Analysis thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
		printf("Frame %d,Time= sec=%d, msec=%d\n", frames,(int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
		sem_post(&semS2);
}      
}
void *Service_2(void *threadp)
{
	char fileppm[50],filejpg[50];
	int count=0;
	while(1)
	{
	sem_wait(&semS2);
	Mat mat_frame(cvarrToMat(frame));
	sprintf(fileppm,"image%d.ppm",frames);
	sprintf(filejpg,"image%d.jpg",frames);

	imwrite(fileppm,mat_frame);
	imwrite(filejpg,mat_frame);
	

	gettimeofday(&current_time_val, (struct timezone *)0);
	syslog(LOG_CRIT, "Time-stamp with Image Analysis thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
	count++;


  
	}
	  
}	





