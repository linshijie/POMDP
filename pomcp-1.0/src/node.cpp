#include "node.h"
#include "history.h"
#include "utils.h"

using namespace std;

//-----------------------------------------------------------------------------

int QNODE::NumChildren = 0;

int QNODE::NumOtherAgentValues = 0;

int QNODE::NumMessageValues = 0;

int QNODE::MaxMessageActions = 0;

void QNODE::Initialise()
{
    assert(NumChildren);
    Children.resize(NumChildren);
    for (int observation = 0; observation < QNODE::NumChildren; observation++)
        Children[observation] = 0;
    AlphaData.AlphaSum.clear();
    assert(NumOtherAgentValues);
    OtherAgentValues.resize(NumOtherAgentValues);
    for (int i = 0; i < QNODE::NumOtherAgentValues; i++)
	OtherAgentValues[i].Set(0, 0);
    assert(NumMessageValues);
    MessageValues.resize(NumMessageValues);
    for (int i = 0; i < QNODE::NumMessageValues; i++)
	MessageValues[i].Set(0,0);
    MaxMessageValue = -Infinity;
    MaxMessageSet = false;
    MessageActions.clear();
}

int QNODE::SelectRandomMessageAction() const
{
    if (MessageActions.empty())
	return -1;
    return MessageActions[UTILS::Random((int)MessageActions.size())];
}


void QNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back().Observation = observation;
            Children[observation]->DisplayValue(history, maxDepth, ostr);
        }
    }
}

void QNODE::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back().Observation = observation;
            Children[observation]->DisplayPolicy(history, maxDepth, ostr);
        }
    }
}

//-----------------------------------------------------------------------------

MEMORY_POOL<VNODE> VNODE::VNodePool;

int VNODE::NumChildren = 0;

void VNODE::Initialise()
{
    assert(NumChildren);
    Children.resize(VNODE::NumChildren);
    for (int action = 0; action < VNODE::NumChildren; action++)
        Children[action].Initialise();
}

VNODE* VNODE::Create()
{
    VNODE* vnode = VNodePool.Allocate();
    vnode->Initialise();
    return vnode;
}

void VNODE::Free(VNODE* vnode, const SIMULATOR& simulator)
{
    //std::cout << "before free\n";
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
    //std::cout << "after free\n";
    for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
            if (vnode->Child(action).Child(observation))
                Free(vnode->Child(action).Child(observation), simulator);
}

void VNODE::FreeAll()
{
	VNodePool.DeleteAll();
}

void VNODE::SetChildren(int count, double value)
{
    for (int action = 0; action < NumChildren; action++)
    {
        QNODE& qnode = Children[action];
        qnode.Value.Set(count, value);
        qnode.AMAF.Set(count, value);
    }
}

void VNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    for (int action = 0; action < NumChildren; action++)
    {
        history.Add(action);
        Children[action].DisplayValue(history, maxDepth, ostr);
        history.Pop();
    }
}

void VNODE::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    double bestq = -Infinity;
    int besta = -1;
    for (int action = 0; action < NumChildren; action++)
    {
        if (Children[action].Value.GetValue() > bestq)
        {
            besta = action;
            bestq = Children[action].Value.GetValue();
        }
    }

    if (besta != -1)
    {
        history.Add(besta);
        Children[besta].DisplayPolicy(history, maxDepth, ostr);
        history.Pop();
    }
}

//-----------------------------------------------------------------------------
