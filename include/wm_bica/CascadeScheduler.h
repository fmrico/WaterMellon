/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#ifndef CascadeScheduler_H
#define CascadeScheduler_H

#include <vector>
#include <pthread.h>

#include "Component.h"
#include "BicaDispatcher.h"
#include "Register.h"

class CascadeScheduler: public Singleton<CascadeScheduler>, public bicacomms::SchedInterface
{
public:

	CascadeScheduler();
	~CascadeScheduler();

	void step();

	std::vector<Component*> getComponents() { return components;};

	void add(Component *comp);
	void remove(Component *comp);
	bool isRunning(Component *comp);

	virtual int addSched(const std::string& id, const Ice::Current&);
	virtual int removeSched(const std::string& id, const Ice::Current&);
	virtual void removeAll(const Ice::Current&);

private:
	std::vector<Component*>  components;
	pthread_mutex_t mutex;
};

#endif
