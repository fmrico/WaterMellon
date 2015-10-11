/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#ifndef Component_H
#define Component_H

#include <string>
#include <sstream>
#include <sys/time.h>

#include "Register.h"


class Component
{
public:

	static const int DEFAULT_FREQ_TIME 	= 60; 	// Miliseconds
	static const int FRAME_RATE 		= 0; 	// Full speed

	static const int SHORT_RATE 		= 50; 	// Full speed
	static const int MEDIUM_RATE 		= 100; 	// Medium speed
	static const int LONG_RATE 			= 500; 	// Low speed

	Component();
	virtual ~Component();

	void init(const std::string newName);
	

	bool isTime2Run	();
	void setFreqTime (const int newFreqTime);

	void startDebugInfo();
	void endDebugInfo();

	void resetStopWatch();
	long getStopWatch();

	std::string getName();
	virtual int getState() { return -1;}; //to be overriden by vicode-generated components

	virtual void step() = 0;

	long getRealFreq();
	long getMeanCycleTime();
	long getMaxCycleTime();
	long getMinCycleTime();

	inline long getCurrentTime() {
		gettimeofday(&currentTime, NULL);
		return currentTime.tv_sec * 1000000 + currentTime.tv_usec;
	}

	inline long getCurrentTime2() {
		struct timespec begin;
		clock_gettime( CLOCK_MONOTONIC, &begin );

		return begin.tv_sec * 1000000000.0 + begin.tv_nsec;
	}

protected:
	struct timeval 	currentTime;
	int 		freq_time;

private:
	static const long INFO_TIME = 5000000; // 5 Seconds = 5000000 us

	
	int 		iters;
	bool 		starting;
	long 		beginStopWatch, beginStep, beginDebugStep, beginDebugCycle, meanCycle;
	std::string 		name;
	
	long 		realfreq, mcycle, Mtcycle, mtcycle;
};

#endif
