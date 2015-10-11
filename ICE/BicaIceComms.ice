#ifndef BICAR_ICE
#define BICAR_ICE

module bicacomms {

	sequence<string> ListString;

	struct ComponentsList 
	{
		ListString ListComps;
		int numCompos;
	}; 

	const int SUCCESS 		= 0;
	const int NOTRUNNING 	= 1;
	const int NOTFOUND 		= 2;	

	interface SchedInterface
	{
		idempotent int addSched(string id);
		idempotent int removeSched(string id);
		idempotent void removeAll();
	};

	interface VicodeDebug
	{
		idempotent int getState(string id);
		idempotent ComponentsList getListComponents();
	};


};

#endif