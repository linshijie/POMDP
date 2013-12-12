#include "simulator.h"

using namespace std;
using namespace UTILS;

SIMULATOR::KNOWLEDGE::KNOWLEDGE()
:   TreeLevel(LEGAL),
    RolloutLevel(LEGAL),
    SmartTreeCount(10),
    SmartTreeValue(1.0)
{
}

SIMULATOR::STATUS::STATUS()
:   Phase(TREE),
    Particles(CONSISTENT)
{
}

SIMULATOR::SIMULATOR() 
:   NumActions(0),
    NumObservations(0),
    NumAgents(1),
    Discount(1.0),
    RewardRange(1.0)
{
}

SIMULATOR::SIMULATOR(int numActions, int numObservations, double discount)
:   NumActions(numActions),
    NumObservations(numObservations),
    NumAgents(1),
    NumAgentActions(numActions),
    NumAgentObservations(numObservations),
    Discount(discount)
{ 
    assert(discount > 0 && discount <= 1);
}

SIMULATOR::~SIMULATOR() 
{ 
}

void SIMULATOR::Validate(const STATE& state) const 
{ 
}

void SIMULATOR::FreeReward(REWARD_TEMPLATE* reward) const
{
    REWARD_TEMPLATE* rewardtemplate = safe_cast<REWARD_TEMPLATE*>(reward);
    RewardMemoryPool.Free(rewardtemplate);
}

bool SIMULATOR::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    return true;
}

void SIMULATOR::GenerateLegal(const STATE& state, const HISTORY& history, 
			      std::vector< int >& actions, const SIMULATOR::STATUS& status) const
{
    for (int a = 0; a < NumActions; ++a)
        actions.push_back(a);
}


void SIMULATOR::GenerateLegalAgent(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status, const int& index) const
{
    
}

void SIMULATOR::GeneratePreferred(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status) const
{
}

void SIMULATOR::GeneratePreferredAgent(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status, const int& index) const
{
}

int SIMULATOR::SelectRandom(const STATE& state, const HISTORY& history,
    const STATUS& status, const int& index) const
{
    static vector<int> actions;

    if (Knowledge.RolloutLevel >= KNOWLEDGE::SMART)
    {
        actions.clear();
	if (index == 0)
	    GeneratePreferred(state, history, actions, status);
	else
	    GeneratePreferredAgent(state, history, actions, status, index);
        if (!actions.empty())
            return actions[Random(actions.size())];
    }
        
    if (Knowledge.RolloutLevel >= KNOWLEDGE::LEGAL)
    {
        actions.clear();
	if (index == 0)
	    GenerateLegal(state, history, actions, status);
	else
	    GenerateLegalAgent(state, history, actions, status, index);
        if (!actions.empty())
            return actions[Random(actions.size())];
    }
    

    return index == 0 ? Random(NumActions) : Random(NumAgentActions);
}

void SIMULATOR::Prior(const STATE* state, const HISTORY& history,
    VNODE* vnode, const STATUS& status, const int& index) const
{
    static vector<int> actions;
    
    if (Knowledge.TreeLevel == KNOWLEDGE::PURE || state == 0)
    {
        vnode->SetChildren(0, 0);
        return;
    }
    else
    {
        vnode->SetChildren(+LargeInteger, -Infinity);
    }

    if (Knowledge.TreeLevel >= KNOWLEDGE::LEGAL)
    {
        actions.clear();
        if (index == 0)
	    GenerateLegal(*state, history, actions, status);
	else
	    GenerateLegalAgent(*state, history, actions, status, index);

        for (vector<int>::const_iterator i_action = actions.begin(); i_action != actions.end(); ++i_action)
        {
            int a = *i_action;
            QNODE& qnode = vnode->Child(a);
            qnode.Value.Set(0, 0);
            qnode.AMAF.Set(0, 0);
        }
    }
    
    if (Knowledge.TreeLevel >= KNOWLEDGE::SMART)
    {
        actions.clear();
        if (index == 0)
	    GeneratePreferred(*state, history, actions, status);
	else
	    GeneratePreferredAgent(*state, history, actions, status, index);

        for (vector<int>::const_iterator i_action = actions.begin(); i_action != actions.end(); ++i_action)
        {
            int a = *i_action;
            QNODE& qnode = vnode->Child(a);
            qnode.Value.Set(Knowledge.SmartTreeCount, Knowledge.SmartTreeValue);
            qnode.AMAF.Set(Knowledge.SmartTreeCount, Knowledge.SmartTreeValue);
        }    
    }
}

bool SIMULATOR::HasAlpha() const
{
    return false;
}

void SIMULATOR::AlphaValue(const QNODE& qnode, double& q, int& n) const
{
}

void SIMULATOR::UpdateAlpha(QNODE& qnode, const STATE& state) const
{
}

void SIMULATOR::DisplayBeliefs(const BELIEF_STATE& beliefState, 
    ostream& ostr) const
{
}

void SIMULATOR::DisplayState(const STATE& state, ostream& ostr) const 
{
}

void SIMULATOR::DisplayAction(int action, ostream& ostr) const 
{
    ostr << "Action " << action << endl;
}

void SIMULATOR::DisplayObservation(const STATE& state, int observation, ostream& ostr) const
{
    ostr << "Observation " << observation << endl;
}

void SIMULATOR::DisplayReward(double reward, std::ostream& ostr) const
{
    ostr << "Reward " << reward << endl;
}

double SIMULATOR::GetHorizon(double accuracy, int undiscountedHorizon) const 
{ 
    if (Discount == 1)
        return undiscountedHorizon;
    return log(accuracy) / log(Discount);
}
