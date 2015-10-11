/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#include "wm_bica/Component.h"
#include <limits.h>
/**
 * Class constructor that initializes the controller with a default minimum frequency time.
 */
Component::Component()
{
	this->setFreqTime(DEFAULT_FREQ_TIME);
	beginStep = getCurrentTime();
	iters = meanCycle = realfreq = mcycle = Mtcycle = 0;
	mtcycle = LONG_MAX;
	
}

/**
 * Class destructor.
 **/
Component::~Component()
{
	
}


void
Component::init(const std::string newName)
{
	
	name = newName;
	
	Register::getInstance()->regist(name, this);
}


/**
 * startDebugInfo. Method that starts the timing debug information. There are two kinds
 * of parameteres measured: cycle time and iterations between an specific INFO_TIME interval.
 **/
void
Component::startDebugInfo()
{
	beginDebugCycle = getCurrentTime();
	if (iters == 0)
		beginDebugStep = beginDebugCycle;
}

/**
 * endDebugInfo. Method that calculate the cycle time and if it is time to show the real
 * frequency of the component (if the elapsed time is greater than INFO_TIME).
 **/
void
Component::endDebugInfo()
{
	long now = getCurrentTime();
	long cycleTime = now - beginDebugCycle;

	meanCycle += cycleTime;

	if(cycleTime > Mtcycle)
		Mtcycle = cycleTime;
	if(cycleTime < mtcycle)
		mtcycle = cycleTime;

	iters++;

	long dif = now - beginDebugStep;

	if (dif >= INFO_TIME){

		realfreq = iters / ((double)(dif / 1000000));
		mcycle = (meanCycle / iters) / 1000.0;

		std::cerr << "[" << name << "] Running at " << iters / ((double)(dif / 1000000)) <<
				"Hz with mean cycle time of " << (meanCycle / iters) / 1000.0 << " ms.\n";
		iters = 0;
		meanCycle = 0;
	}

}

long
Component::getRealFreq()
{
	return realfreq;
}

long
Component::getMeanCycleTime()
{
	return mcycle;
}

long
Component::getMaxCycleTime()
{
	return Mtcycle;
}

long
Component::getMinCycleTime()
{
	return mtcycle;
}

/**
 * setFreqTime. Method that sets the minimum elapsed time between two real step() calls.
 * @param newFreqTime Minimal number of milliseconds between two real step() calls.
 **/
void
Component::setFreqTime (const int newFreqTime)
{
	freq_time = newFreqTime * 1000;
}

/**
 * isTime2Run. Method that checks if at this moment the component should execute a real
 * step() call or should return without executing it.
 **/
bool
Component::isTime2Run()
{
	long ctime = getCurrentTime();

	starting = (ctime - beginStep >= freq_time);
	if (starting)
		beginStep = ctime;  

	return starting;
}

/**
 * resetStopWatch. Internal method for measure the time elapsed into an specific state.
 * This method reset the stopwatch.
 **/
void
Component::resetStopWatch()
{
	beginStopWatch = getCurrentTime();
}

/**
 * getStopWatch. Method that returns the elapsed time from the last reset of the stopwatch.
 * It is internally used by the time based transitions.
 **/
long int
Component::getStopWatch()
{
	return (getCurrentTime() - beginStopWatch) / 1000;
}

std::string
Component::getName()
{
	return name;
}

