/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#include "wm_bica/CascadeScheduler.h"


CascadeScheduler::CascadeScheduler() 
{
	pthread_mutex_init(&mutex, NULL);

	//IceComms::getAdapter()->add(CascadeScheduler::getInstance(), IceComms::getIC()->stringToIdentity("SchedInterface"));
}

CascadeScheduler::~CascadeScheduler() 
{
	components.clear();

}


void CascadeScheduler::step()
{
	pthread_mutex_lock(&mutex);
	if (components.size() > 0)
	{
		std::vector<Component*>::iterator it;

		for (it = components.begin(); it != components.end(); ++it) {

			BicaDispatcher::getInstance()->lock();
			(*it)->step();
			BicaDispatcher::getInstance()->unlock();
		}
	}
	pthread_mutex_unlock(&mutex);
}

bool CascadeScheduler::isRunning(Component *comp)
{
	bool esta = false;
	std::vector<Component*>::iterator it;
	for (it = components.begin(); it != components.end(); it++)
		if ((*it) == comp)
			esta = true;

	return esta;
}


void CascadeScheduler::add(Component *comp)
{
	//pthread_mutex_lock(&mutex);
	if (!isRunning(comp))
		components.push_back(comp);
	//pthread_mutex_unlock(&mutex);
}

void CascadeScheduler::remove(Component *comp)
{
	//pthread_mutex_lock(&mutex);
	std::vector<Component*>::iterator it;
	for (it = components.begin(); it != components.end(); it++) {
		if ((*it) == comp) {
			if (components.size() == 1) {
				//Maybe Stop robot?
				components.clear();
				break;
			} else {
				components.erase(it);
				break;
			}
		}
	}
	//pthread_mutex_unlock(&mutex);
}

int
CascadeScheduler::addSched(const std::string&  id, const Ice::Current&)
{
	pthread_mutex_lock(&mutex);

	Component* comp = Register::getInstance()->getComponent(id);


	if(comp == NULL)
	{
		pthread_mutex_unlock(&mutex);
		return bicacomms::NOTFOUND;
	}

	add(comp);

	pthread_mutex_unlock(&mutex);
	return bicacomms::SUCCESS;

}

int
CascadeScheduler::removeSched(const std::string&  id, const Ice::Current&)
{
	pthread_mutex_lock(&mutex);

	Component* comp = Register::getInstance()->getComponent(id);

	if(comp == NULL)
	{
		pthread_mutex_unlock(&mutex);
		return bicacomms::NOTFOUND;
	}
	if(!isRunning(comp))
	{
		pthread_mutex_unlock(&mutex);
		return bicacomms::NOTRUNNING;
	}

	remove(comp);

	pthread_mutex_unlock(&mutex);
	return bicacomms::SUCCESS;
}

void
CascadeScheduler::removeAll(const Ice::Current&)
{
	pthread_mutex_lock(&mutex);
	components.clear();
	pthread_mutex_unlock(&mutex);
}
