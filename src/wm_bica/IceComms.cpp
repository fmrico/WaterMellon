/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#include "wm_bica/IceComms.h"

#define ICE_PORT 10000

Ice::CommunicatorPtr IceComms::ic;
Ice::ObjectAdapterPtr IceComms::adapter;

void* IceComms::iceServerThread(void *obj)
{
	std::stringstream endpoint;
	int icePort = ICE_PORT;

	std::string str = "bica";
	char* argv[2];
	argv[0] = new char[str.size() + 1];
	std::copy(str.begin(), str.end(), argv[0]);
	argv[0][str.size()] = '\0'; // don't forget the terminating 0

	str = "0";
	argv[1] = new char[str.size() + 1];
	std::copy(str.begin(), str.end(), argv[1]);
	argv[1][str.size()] = '\0'; // don't forget the terminating 0

	//char* argv[] = {name.c_str(), 0};
	int argc = 1;
	int *status = new int(0);

	try {

		endpoint << "default -p " << icePort;
		std::cout<<"Starting ICE ["<<endpoint.str()<<"]..."<<std::endl;

		Ice::InitializationData initData;
		initData.properties = Ice::createProperties(argc, argv);
		initData.dispatcher = BicaDispatcher::getInstance();
		ic = Ice::initialize(argc, argv, initData);

		adapter = ic->createObjectAdapterWithEndpoints("BICAadapter", endpoint.str());


		adapter->add(Register::getInstance(), ic->stringToIdentity("VicodeDebug"));
		adapter->add(CascadeScheduler::getInstance(), ic->stringToIdentity("SchedInterface"));

		adapter->activate();


		ic->waitForShutdown();
	} catch (const Ice::Exception& e) {
		std::cerr << e << std::endl;
		*status = 1;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		*status = 1;
	}
	if (ic) {
		try {
			ic->destroy();
		} catch (const Ice::Exception& e) {
			std::cerr << e << std::endl;
			*status = 1;
		}
	}
	return status;
}

IceComms::IceComms()
{
	tIceServer = 0;
}

void
IceComms::startServer()
{
	pthread_create(&tIceServer, NULL, iceServerThread, NULL);
}
