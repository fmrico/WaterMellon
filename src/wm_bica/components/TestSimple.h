#ifndef TestSimple_H
#define TestSimple_H

#include "wm_bica/Component.h"
#include "Navigator.h"
#include "wm_bica/Singleton.h"

#define TestSimple_ID "TestSimple"

//BUILDER COMMENT. DO NOT REMOVE. auxinclude begin
#include "wm_navigation/Locations.h"

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
	static const int Wait	= 1;
	static const int Command	= 2;
	int state;

	Navigator *_Navigator;

	void Initial_state_code(void);
	void Wait_state_code(void);
	void Command_state_code(void);
	bool Initial2Wait0_transition_code(void);
	bool Wait2Command0_transition_code(void);
	bool Command2Wait0_transition_code(void);
//BUILDER COMMENT. DO NOT REMOVE. auxcode begin
public:
	void init(const std::string newName);

private:
	wm_navigation::Locations locations_;

	std::string locs[5];
	int npos;
//BUILDER COMMENT. DO NOT REMOVE. auxcode end
};

#endif

