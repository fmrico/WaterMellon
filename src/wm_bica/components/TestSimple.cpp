#include "TestSimple.h"

//BUILDER COMMENT. DO NOT REMOVE. auxinclude begin
//BUILDER COMMENT. DO NOT REMOVE. auxinclude end

TestSimple::TestSimple()
{

	init(TestSimple_ID);

	_Navigator = Navigator::getInstance();
	state = Initial;
}

TestSimple::~TestSimple()
{

}

void TestSimple::Initial_state_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Initial_state_code
//BUILDER COMMENT. DO NOT REMOVE. end Initial_state_code
}

void TestSimple::Wait_state_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Wait_state_code



//BUILDER COMMENT. DO NOT REMOVE. end Wait_state_code
}

void TestSimple::Command_state_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Command_state_code

//BUILDER COMMENT. DO NOT REMOVE. end Command_state_code
}

bool TestSimple::Initial2Wait0_transition_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Initial2Wait0_transition_code
	_Navigator->setGoal(locations_.getPose(locs[npos]));

	std::cout<<"Going to ["<<locs[npos]<<"]"<<std::endl;

	npos = (npos+1)%5;
	return true;
//BUILDER COMMENT. DO NOT REMOVE. end Initial2Wait0_transition_code

}


bool TestSimple::Wait2Command0_transition_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Wait2Command0_transition_code
return _Navigator->finished();
//BUILDER COMMENT. DO NOT REMOVE. end Wait2Command0_transition_code
}

bool TestSimple::Command2Wait0_transition_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Command2Wait0_transition_code
	_Navigator->setGoal(locations_.getPose(locs[npos]));

	std::cout<<"Going to ["<<locs[npos]<<"]"<<std::endl;

	npos = (npos+1)%5;
	return true;
//BUILDER COMMENT. DO NOT REMOVE. end Command2Wait0_transition_code
}

void
TestSimple::step(void)
{
	switch (state)
	{
	case Initial:

		if (isTime2Run()) {
			Initial_state_code();

			if (Initial2Wait0_transition_code()) {
				state = Wait;
				resetStopWatch();
			}
		}

		break;
	case Wait:
		_Navigator->step();

		if (isTime2Run()) {
			Wait_state_code();

			if (Wait2Command0_transition_code()) {
				state = Command;
				resetStopWatch();
			}
		}

		break;
	case Command:

		if (isTime2Run()) {
			Command_state_code();

			if (Command2Wait0_transition_code()) {
				state = Wait;
				resetStopWatch();
			}
		}

		_Navigator->step();
		break;
	default:
		std::cout << "[TestSimple::step()] Invalid state\n";
	}
}

//BUILDER COMMENT. DO NOT REMOVE. auxcode begin

void
TestSimple::init(const std::string newName)
{
	Component::init(newName);
	setFreqTime(LONG_RATE);

	npos=0;
	locs[0] = "bedroom";
	locs[1] = "entrance";
	locs[2] = "kitchen";
	locs[3] = "livingroom";
	locs[4] = "voidroom";

	ros::NodeHandle private_nh("~");

	if(private_nh.hasParam("locations_file"))
	{
		std::string loc_file;
		private_nh.param("locations_file", loc_file,loc_file);

		ROS_INFO("Reading Locations from %s", loc_file.c_str());

		locations_.readFile(loc_file);

		locations_.printLocations();

	}

}

//BUILDER COMMENT. DO NOT REMOVE. auxcode end

