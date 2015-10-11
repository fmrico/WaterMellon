/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#ifndef REGISTER_H_
#define REGISTER_H_

#include "Singleton.h"
#include "Component.h"
#include "BicaIceComms.h"

#include <string>
#include <map>

class Component;

class Register: public Singleton<Register>, public bicacomms::VicodeDebug
{
public:
	Register();
	virtual ~Register();

	void regist(std::string id, Component *instance);
	Component *getComponent(std::string id);

	virtual int getState(const std::string& id, const Ice::Current&);
	virtual bicacomms::ComponentsList getListComponents(const Ice::Current&);

private:

	std::map<std::string, Component*> reg;
};

#endif /* REGISTER_H_ */
