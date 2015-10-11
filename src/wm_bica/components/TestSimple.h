#ifndef TestSimple_H
#define TestSimple_H

#include "wm_bica/Component.h"
#include "wm_bica/Singleton.h"

#define TestSimple_ID "TestSimple"

//BUILDER COMMENT. DO NOT REMOVE. auxinclude begin
//BUILDER COMMENT. DO NOT REMOVE. auxinclude end

class TestSimple : public Component, public Singleton<TestSimple>
//BUILDER COMMENT. DO NOT REMOVE. auxclass begin
//BUILDER COMMENT. DO NOT REMOVE. auxclass end
{
public:

	TestSimple();
	~TestSimple();

	void step();

	virtual int getState() { return state;};

private:

	static const int Initial	= 0;
	static const int State1	= 1;
	static const int State2	= 2;
	int state;


	void Initial_state_code(void);
	void State1_state_code(void);
	void State2_state_code(void);
	bool Initial2State10_transition_code(void);
	bool State12State20_transition_code(void);
	bool State22State10_transition_code(void);
//BUILDER COMMENT. DO NOT REMOVE. auxcode begin
//BUILDER COMMENT. DO NOT REMOVE. auxcode end
};

#endif

