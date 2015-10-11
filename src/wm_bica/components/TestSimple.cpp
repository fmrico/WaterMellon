#include "TestSimple.h"

//BUILDER COMMENT. DO NOT REMOVE. auxinclude begin
//BUILDER COMMENT. DO NOT REMOVE. auxinclude end

TestSimple::TestSimple()
{

	init(TestSimple_ID);

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

void TestSimple::State1_state_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin State1_state_code
std::cout<<"State 1"<<std::endl;
//BUILDER COMMENT. DO NOT REMOVE. end State1_state_code
}

void TestSimple::State2_state_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin State2_state_code
std::cout<<"State 2"<<std::endl;//BUILDER COMMENT. DO NOT REMOVE. end State2_state_code
}

bool TestSimple::Initial2State10_transition_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin Initial2State10_transition_code
return true;
//BUILDER COMMENT. DO NOT REMOVE. end Initial2State10_transition_code
}

bool TestSimple::State12State20_transition_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin State12State20_transition_code
return getStopWatch()>2000;
//BUILDER COMMENT. DO NOT REMOVE. end State12State20_transition_code
}

bool TestSimple::State22State10_transition_code(void)
{
//BUILDER COMMENT. DO NOT REMOVE. begin State22State10_transition_code
return getStopWatch()>2000;
//BUILDER COMMENT. DO NOT REMOVE. end State22State10_transition_code
}

void
TestSimple::step(void)
{
	switch (state)
	{
	case Initial:

		if (isTime2Run()) {
			Initial_state_code();

			if (Initial2State10_transition_code()) {
				state = State1;
				resetStopWatch();
			}
		}

		break;
	case State1:

		if (isTime2Run()) {
			State1_state_code();

			if (State12State20_transition_code()) {
				state = State2;
				resetStopWatch();
			}
		}

		break;
	case State2:

		if (isTime2Run()) {
			State2_state_code();

			if (State22State10_transition_code()) {
				state = State1;
				resetStopWatch();
			}
		}

		break;
	default:
		std::cout << "[TestSimple::step()] Invalid state\n";
	}
}

//BUILDER COMMENT. DO NOT REMOVE. auxcode begin
//BUILDER COMMENT. DO NOT REMOVE. auxcode end

