/**
 *
 * Created: Francisco Martín (fmrico@gmail.com) 26/12/2013
 *
**/


#include "ros/ros.h"

#include <sstream>
#include <vector>

#include "wm_bica/Component.h"
#include "wm_bica/CascadeScheduler.h"
#include "wm_bica/IceComms.h"

#include "components/TestSimple.h"

int main(int argc, char **argv)
{
	IceComms icecomms;
	ros::init(argc, argv, "bica");
	ros::NodeHandle n;

	icecomms.startServer();

	CascadeScheduler *bica_sched = CascadeScheduler::getInstance();

	TestSimple::getInstance();

	bica_sched->add(TestSimple::getInstance());


	if(ros::param::has("~init_component"))
	{
		std::string initc;
		ros::param::get("~init_component", initc);
		std::cout<<"Auto starting: ["<<initc<<"]"<<std::endl;

		Component *initcp = Register::getInstance()->getComponent(initc);
		if(initcp!=NULL) bica_sched->add(initcp);
	}

	while (ros::ok())
	{
		bica_sched->step();
		ros::spinOnce();
	}

	return 0;
}
