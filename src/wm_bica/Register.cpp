/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#include "wm_bica/Register.h"

Register::Register()
{
	reg.clear();

	//IceComms::getAdapter()->add(Register::getInstance(), IceComms::getIC()->stringToIdentity("VicodeDebug"));
}

Register::~Register()
{
	
}

void 
Register::regist(std::string id, Component *instance)
{
	std::cerr<<"Component registered: ["<<id<<"]"<<std::endl;
	reg[id] = instance;
}

Component *
Register::getComponent(std::string id)
{
	std::map<std::string, Component*>::iterator it = reg.find(id);
	if(it!=reg.end())
	{
		//std::cerr<<"["<<id<<"] is found"<<std::endl;
		return it->second;
	}else
	{
		//std::cerr<<"["<<id<<"] is NOT found"<<std::endl;
		return NULL;
	}
}

int
Register::getState(const std::string& id, const Ice::Current&)
{
	Component *aux = getComponent(id);

	if(aux != NULL)
	{
		return aux->getState();
	}else
	{
		return -1;
	}
}

bicacomms::ComponentsList
Register::getListComponents(const Ice::Current&)
{
	bicacomms::ComponentsList list;

	list.ListComps.clear();

	std::map<std::string, Component*>::iterator it = reg.begin();

	for(it=reg.begin(); it!=reg.end(); ++it)
		list.ListComps.push_back(it->first);

	list.numCompos = reg.size();
	return list;
}
