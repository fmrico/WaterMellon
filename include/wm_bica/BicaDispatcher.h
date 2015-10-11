/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#ifndef BICADISPATCHER_H
#define BICADISPATCHER_H

#include <Ice/Ice.h>
#include "Singleton.h"


class BicaDispatcher : public Ice::Dispatcher,  public IceUtil::Monitor<IceUtil::Mutex>, public Singleton<BicaDispatcher>
{
public:
	BicaDispatcher();

    virtual void dispatch(const Ice::DispatcherCallPtr& d, const Ice::ConnectionPtr&);

    void lock();
    void unlock();

private:

    bool worktodo_c;
    bool processing;

    IceUtil::Mutex mutex_c;
    IceUtil::Mutex mutex_r;

};

#endif
