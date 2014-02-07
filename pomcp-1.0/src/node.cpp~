#include "node.h"
#include "history.h"
#include "utils.h"

using namespace std;

//-----------------------------------------------------------------------------

//int QNODE::NumChildren = 0;

int QNODE::NumOtherAgentValues = 0;

void QNODE::Initialise(const int& qChildren)
{
    NumChildren = qChildren;
    assert(NumChildren);
    Children.resize(NumChildren);
    for (int observation = 0; observation < NumChildren; observation++)
        Children[observation] = 0;
    AlphaData.AlphaSum.clear();
    assert(NumOtherAgentValues);
    OtherAgentValues.resize(NumOtherAgentValues);
    for (int i = 0; i < QNODE::NumOtherAgentValues; i++)
	OtherAgentValues[i].Set(0, 0);
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

//int VNODE::NumChildren = 0;

void VNODE::Initialise(const int& qChildren)
{
    assert(NumChildren);
    Children.resize(NumChildren);
    for (int action = 0; action < NumChildren; action++)
        Children[action].Initialise(qChildren);
}

VNODE* VNODE::Create(const int& vChildren, const int& qChildren)
{
    VNODE* vnode = VNodePool.Allocate();
    vnode->NumChildren = vChildren;
    vnode->Initialise(qChildren);
    return vnode;
}

void VNODE::Free(VNODE* vnode, const SIMULATOR& simulator)
{
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
    for (int action = 0; action < vnode->NumChildren; action++)
        for (int observation = 0; observation < vnode->Child(action).NumChildren; observation++)
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
