/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#include "wm_bica/BicaDispatcher.h"

BicaDispatcher::BicaDispatcher()
{
	worktodo_c = false;
	processing = false;
}

void
BicaDispatcher::dispatch(const Ice::DispatcherCallPtr& d, const Ice::ConnectionPtr&)
{
	/*IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
	worktodo_c = true;

	if(processing)
		wait();
	*/
    d->run();

    /*worktodo_c = false;
    notify();*/
}

void
BicaDispatcher::lock()
{
	/*
	IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
    if(worktodo_c)
    	wait();

    processing = true;*/
}

void
BicaDispatcher::unlock()
{
	/*
	IceUtil::Monitor<IceUtil::Mutex>::Lock lock(*this);
	processing = false;

	if(worktodo_c)
		notify();
		*/

}
