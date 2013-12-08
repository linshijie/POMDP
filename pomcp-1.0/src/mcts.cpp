#include "mcts.h"
#include "testsimulator.h"
#include <math.h>

#include <algorithm>

using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------

MCTS::PARAMS::PARAMS()
:   Verbose(0),
    MaxDepth(100),
    NumSimulations(1000),
    NumStartStates(20),
    UseTransforms(false),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DisableTree(false),
    MultiAgent(true),
    JointQActions(true),
    MinMax(false),
    RewardAdaptive(false)
{
}

MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0)
{
    VNODE::NumChildren = Params.MultiAgent && !Params.JointQActions ? Simulator.GetNumAgentActions() : 
			Simulator.GetNumActions();
    QNODE::NumChildren = Params.MultiAgent ? Simulator.GetNumAgentObservations() : Simulator.GetNumObservations();
    
    Statuses.clear();
    Roots.clear();
    Histories.clear();
    StatTreeDepths.clear();
    StatRolloutDepths.clear();
    StatTotalRewards.clear();
    
    
    for (int i = 0 ; i < Simulator.GetNumAgents(); i++)
    {
	//status
	SIMULATOR::STATUS status;
	status.perspindex = i;
	status.jointhistory = Params.MultiAgent ? true : false;
	Statuses.push_back(status);
	//histories
	HISTORY history;
	Histories.push_back(history);
	//statistics
	STATISTIC StatTreeDepth;
	StatTreeDepths.push_back(StatTreeDepth);
	STATISTIC StatRolloutDepth;
	StatRolloutDepths.push_back(StatRolloutDepth);
	STATISTIC StatTotalReward;
	StatTotalRewards.push_back(StatTotalReward);
	//root
	VNODE* root = ExpandNode(Simulator.CreateStartState(), Params.MultiAgent ? i+1 : i);
	Roots.push_back(root);
    }
    

    for (int i = 0 ; i < Simulator.GetNumAgents(); i++)
	for (int j = 0; j < Params.NumStartStates; j++)
	{
	    Roots[i]->Beliefs().AddSample(Simulator.CreateStartState());
	    if (Params.MultiAgent)
		Roots[i]->Beliefs().AddSample(Simulator.CreateStartState());
	}
	
	
}

MCTS::~MCTS()
{
    for (int i = 0; i < Simulator.GetNumAgents() ; i++)
	VNODE::Free(Roots[i], Simulator);
    VNODE::FreeAll();
}

bool MCTS::Update(int action, int observation, double reward, const int& index)
{
    
    if (!Params.MultiAgent)
	Histories[index == 0 ? index : index-1].Add(action, observation);
    else
    {
	if (index == 1 && Params.JointQActions)
	    Histories[index == 0 ? index : index-1].Add(Simulator.GetAgentAction(action,1), observation);
	else if (index == 1 && !Params.JointQActions)
	    Histories[index == 0 ? index : index-1].Add(action, observation);
	else if (index == 2 && Params.JointQActions)
	    Histories[index == 0 ? index : index-1].Add(Simulator.GetAgentAction(action,2), observation);
	else if (index == 2 && !Params.JointQActions)
	    Histories[index == 0 ? index : index-1].Add(action, observation);
    }
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
	
    QNODE& qnode = Roots[index == 0 ? index : index-1]->Child(action);
    VNODE* vnode = qnode.Child(observation);
    if (vnode)
    {
        if (Params.Verbose >= 1)
            cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;
        beliefs.Copy(vnode->Beliefs(), Simulator);
    }
    else
    {
        if (Params.Verbose >= 1)
            cout << "No matching node found" << endl;
    }

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms)
	AddTransforms(Roots[index == 0 ? index : index-1], beliefs, index);

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 1)
        Simulator.DisplayBeliefs(beliefs, cout);

    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else
        state = beliefs.GetSample(0);

    // Delete old tree and create new root
    VNODE::Free(Roots[index == 0 ? index : index-1], Simulator);
    VNODE* newRoot = ExpandNode(state, index);
    newRoot->Beliefs() = beliefs;
    if (!Params.MultiAgent)
	Roots[index] = newRoot;
    else
	Roots[index-1] = newRoot;
    
    return true;
}

int MCTS::SelectAction(const int& index)
{
    if (Params.DisableTree)
	RolloutSearch(index);
    else
	UCTSearch(index);
    return GreedyUCB(Roots[index == 0 ? index : index-1], false);

}

void MCTS::RolloutSearch(const int& index)
{
    std::vector<double> totals(Simulator.GetNumActions(), 0.0);
    int historyDepth = GetHistory(index).Size();
    std::vector<int> legal;
    assert(BeliefState(index).GetNumSamples() > 0);
    Simulator.GenerateLegal(*BeliefState(index).GetSample(0), GetHistory(index), legal, GetStatus(index));
    random_shuffle(legal.begin(), legal.end());

    for (int i = 0; i < Params.NumSimulations; i++)
    {
	    int action = legal[i % legal.size()];
	    STATE* state;
	    state = Roots[index == 0 ? index : index-1]->Beliefs().CreateSample(Simulator);
	    Simulator.Validate(*state);

	    int observation;
	    double immediateReward, delayedReward, totalReward;
	    bool terminal = Simulator.Step(*state, action, observation, immediateReward);	 
	    
	    int treeaction = action;
	    if (Params.MultiAgent && !Params.JointQActions)
	    {
		if (index == 1)
		    treeaction = Simulator.GetAgentAction(action,1);
		else
		    treeaction = Simulator.GetAgentAction(action,2);
	    }

	    VNODE*& vnode = Roots[index == 0 ? index : index-1]->Child(treeaction).Child(index == 0 ?
		    observation : Simulator.GetAgentObservation(observation,index));			
	    
	    if (!vnode && !terminal)
	    {
		    vnode = ExpandNode(state, index);
		    AddSample(vnode, *state);
	    }
	    
	    Histories[index == 0 ? index : index-1].Add(index == 0 ? action : Simulator.GetAgentAction(action,index), 
				       index == 0 ? observation : Simulator.GetAgentObservation(observation,index));

	    delayedReward = Rollout(*state, index);
	    totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
	    Roots[index == 0 ? index : index-1]->Child(treeaction).Value.Add(totalReward);

	    Simulator.FreeState(state);
	    Histories[index == 0 ? index : index-1].Truncate(historyDepth);
    }
}

void MCTS::UCTSearch(const int& index)
{
    ClearStatistics(index);
    int historyDepth = GetHistory(index).Size();

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = Roots[index == 0 ? index : index-1]->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
	Statuses[index == 0 ? index : index-1].Phase = SIMULATOR::STATUS::TREE;
	
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;
        double totalReward = SimulateV(*state, Roots[index == 0 ? index : index-1], index);
        StatTotalRewards[index == 0 ? index : index-1].Add(totalReward);
        StatTreeDepths[index == 0 ? index : index-1].Add(PeakTreeDepth);

        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, index, cout);

        Simulator.FreeState(state);
	Histories[index == 0 ? index : index-1].Truncate(historyDepth);
    }

    DisplayStatistics(cout, index);
}

double MCTS::SimulateV(STATE& state, VNODE* vnode, const int& index)
{
    int action = GreedyUCB(vnode, true);

    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return 0;

    if (TreeDepth == 1)
        AddSample(vnode, state);

    QNODE& qnode = vnode->Child(action);
    double totalReward = SimulateQ(state, qnode, action, index);
    vnode->Value.Add(totalReward);
    AddRave(vnode, totalReward, state, index);
    return totalReward;
}

double MCTS::SimulateQ(STATE& state, QNODE& qnode, int action, const int& index)
{
    int observation;
    double immediateReward, delayedReward = 0;
    
    if (Params.MultiAgent && !Params.JointQActions)
    {
	if (index == 1)
	    action = action + 
		Simulator.GetNumAgentActions()*Simulator.SelectRandom(state, GetHistory(index), GetStatus(index), 2);
		//action + Simulator.GetNumAgentActions()*Random(Simulator.GetNumAgentActions());
	else
	    action = Simulator.SelectRandom(state, GetHistory(index), GetStatus(index),1) + 
		Simulator.GetNumAgentActions()*action;
		//Random(Simulator.GetNumAgentActions()) + Simulator.GetNumAgentActions()*action;
    }

    if (Simulator.HasAlpha())
        Simulator.UpdateAlpha(qnode, state);
    bool terminal = Simulator.Step(state, action, observation, immediateReward);
    assert(observation >= 0 && observation < Simulator.GetNumObservations());
    Histories[index == 0 ? index : index-1].Add(index == 0 ? action : Simulator.GetAgentAction(action, index), 
			       index == 0 ? observation : Simulator.GetAgentObservation(observation, index));

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayReward(immediateReward, cout);
        Simulator.DisplayState(state, cout);
    }

    VNODE*& vnode = index == 0 ? qnode.Child(observation) : 
	    qnode.Child(Simulator.GetAgentObservation(observation, index));
    
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
	vnode = ExpandNode(&state, index);

    if (!terminal)
    {
        TreeDepth++;
        if (vnode)
            delayedReward = SimulateV(state, vnode, index);
        else
            delayedReward = Rollout(state, index);
        TreeDepth--;
    }

    double totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
    qnode.Value.Add(totalReward);
    return totalReward;
}

void MCTS::AddRave(VNODE* vnode, double totalReward, const STATE& state, const int& index)
{
    double totalDiscount = 1.0;
    int maxIter = GetHistory(index).Size();
    for (int t = TreeDepth; t < maxIter; ++t)
    {
	int action;
	if (index == 0 || (Params.MultiAgent && !Params.JointQActions))
	    action = GetHistory(index)[t].Action;
	else
	{
	    if (index == 1)
		action = GetHistory(index)[t].Action + 
		    Simulator.GetNumAgentActions()*Simulator.SelectRandom(state, GetHistory(index), GetStatus(index),2);
	    else
		action = Simulator.SelectRandom(state, GetHistory(index), GetStatus(index),1) + 
		    Simulator.GetNumAgentActions()*GetHistory(index)[t].Action;
	}
        QNODE& qnode = vnode->Child(action);
        qnode.AMAF.Add(totalReward, totalDiscount);
        totalDiscount *= Params.RaveDiscount;
    }
}

VNODE* MCTS::ExpandNode(const STATE* state, const int& index)
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, 0);
    
    if (!Params.MultiAgent)
	Simulator.Prior(state, GetHistory(index), vnode, GetStatus(index), index);
    else
    {
	if (Params.JointQActions)
	    Simulator.Prior(state, GetHistory(index), vnode, GetStatus(index), 0);
	else
	    Simulator.Prior(state, GetHistory(index), vnode, GetStatus(index), index);
    }

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
	GetHistory(index).Display(cout);
        cout << endl;
    }

    return vnode;
}

void MCTS::AddSample(VNODE* node, const STATE& state)
{
    STATE* sample = Simulator.Copy(state);
    node->Beliefs().AddSample(sample);
    if (Params.Verbose >= 2)
    {
        cout << "Adding sample:" << endl;
        Simulator.DisplayState(*sample, cout);
    }
}

int MCTS::GreedyUCB(VNODE* vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);
    bool hasalpha = Simulator.HasAlpha();
    
    int maxIter;
    if (Params.MultiAgent && !Params.JointQActions)
	maxIter = Simulator.GetNumAgentActions();
    else
	maxIter = Simulator.GetNumActions();

    for (int action = 0; action < maxIter; action++)
    {
        double q, alphaq;
        int n, alphan;

        QNODE& qnode = vnode->Child(action);
        q = qnode.Value.GetValue();
        n = qnode.Value.GetCount();

        if (Params.UseRave && qnode.AMAF.GetCount() > 0)
        {
            double n2 = qnode.AMAF.GetCount();
            double beta = n2 / (n + n2 + Params.RaveConstant * n * n2);
            q = (1.0 - beta) * q + beta * qnode.AMAF.GetValue();
        }

        if (hasalpha && n > 0)
        {
            Simulator.AlphaValue(qnode, alphaq, alphan);
            q = (n * q + alphan * alphaq) / (n + alphan);
            //cout << "N = " << n << ", alphaN = " << alphan << endl;
            //cout << "Q = " << q << ", alphaQ = " << alphaq << endl;
        }

        if (ucb)
            q += FastUCB(N, n, logN);

        if (q >= bestq)
        {
            if (q > bestq)
                besta.clear();
            bestq = q;
            besta.push_back(action);
        }
    }

    assert(!besta.empty());
    return besta[Random(besta.size())];
}

double MCTS::Rollout(STATE& state, const int& index)
{
    Statuses[index == 0 ? index : index-1].Phase = SIMULATOR::STATUS::ROLLOUT;
    if (Params.Verbose >= 3)
        cout << "Starting rollout" << endl;

    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        int observation;
        double reward;

        int action = Simulator.SelectRandom(state, GetHistory(index), GetStatus(index), 0);
	
        terminal = Simulator.Step(state, action, observation, reward);
	    
	Histories[index == 0 ? index : index-1].Add(index == 0 ? action : Simulator.GetAgentAction(action, index), 
			      index == 0 ? observation : Simulator.GetAgentObservation(observation, index));

        if (Params.Verbose >= 4)
        {
            Simulator.DisplayAction(action, cout);
            Simulator.DisplayObservation(state, observation, cout);
            Simulator.DisplayReward(reward, cout);
            Simulator.DisplayState(state, cout);
        }

        totalReward += reward * discount;
        discount *= Simulator.GetDiscount();
    }

    StatRolloutDepths[index == 0 ? index : index-1].Add(numSteps);
    if (Params.Verbose >= 3)
        cout << "Ending rollout after " << numSteps
            << " steps, with total reward " << totalReward << endl;
    return totalReward;
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs, const int& index)
{
    
    int attempts = 0, added = 0;

    // Local transformations of state that are consistent with history
    
    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        STATE* transform = CreateTransform(index);
        if (transform)
        {
            beliefs.AddSample(transform);
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }
}

STATE* MCTS::CreateTransform(const int& index) const
{
    int stepObs;
    double stepReward;

    STATE* state = Roots[index == 0 ? index : index-1]->Beliefs().CreateSample(Simulator);
    int action;
    if (index == 0)
	action = GetHistory(index).Back().Action;
    else
    {
	if (index == 1)
	    action = GetHistory(index).Back().Action + 
		Simulator.GetNumAgentActions()*Simulator.SelectRandom(*state, GetHistory(index), GetStatus(index), 2);
	else
	    action = Simulator.SelectRandom(*state, GetHistory(index), GetStatus(index),1) + 
		Simulator.GetNumAgentActions()*GetHistory(index).Back().Action;
    }
    Simulator.Step(*state, action, stepObs, stepReward);
    if (Simulator.LocalMove(*state, GetHistory(index), stepObs, GetStatus(index)))
	return state;
    Simulator.FreeState(state);
    return 0;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
    cout << "Initialising fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = Infinity;
            else
                UCB[N][n] = exploration * sqrt(log(N + 1) / n);
    cout << "done" << endl;
    InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return Infinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS::ClearStatistics(const int& index)
{
    StatTreeDepths[index == 0 ? index : index-1].Clear();
    StatRolloutDepths[index == 0 ? index : index-1].Clear();
    StatTotalRewards[index == 0 ? index : index-1].Clear();
}

void MCTS::DisplayStatistics(ostream& ostr, const int& index) const
{
    if (Params.Verbose >= 1)
    {
	StatTreeDepths[index == 0 ? index : index-1].Print("Tree depth", ostr);
	StatRolloutDepths[index == 0 ? index : index-1].Print("Rollout depth", ostr);
	StatTotalRewards[index == 0 ? index : index-1].Print("Total reward", ostr);
    }

    if (Params.Verbose >= 2)
    {
        ostr << "Policy after " << Params.NumSimulations << " simulations" << endl;
        DisplayPolicy(6, index, ostr);
        ostr << "Values after " << Params.NumSimulations << " simulations" << endl;
        DisplayValue(6, index, ostr);
    }
}

void MCTS::DisplayValue(int depth, const int& index, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Values:" << endl;
    Roots[index == 0 ? index : index-1]->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, const int& index, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Policy:" << endl;
    Roots[index == 0 ? index : index-1]->DisplayPolicy(history, depth, ostr);
}

//-----------------------------------------------------------------------------

void MCTS::UnitTest(const int& index)
{
    UnitTestGreedy(index);
    UnitTestUCB(index);
    UnitTestRollout(index);
    for (int depth = 1; depth <= 3; ++depth)
        UnitTestSearch(depth, index);
}

void MCTS::UnitTestGreedy(const int& index)
{
    TEST_SIMULATOR testSimulator(5, 5, 0);
    PARAMS params;
    MCTS mcts(testSimulator, params);
    int numAct = testSimulator.GetNumActions();
    int numObs = testSimulator.GetNumObservations();

    VNODE* vnode = mcts.ExpandNode(testSimulator.CreateStartState(), index);
    vnode->Value.Set(1, 0);
    vnode->Child(0).Value.Set(0, 1);
    for (int action = 1; action < numAct; action++)
        vnode->Child(action).Value.Set(0, 0);
    assert(mcts.GreedyUCB(vnode, false) == 0);
}

void MCTS::UnitTestUCB(const int& index)
{
    TEST_SIMULATOR testSimulator(5, 5, 0);
    PARAMS params;
    MCTS mcts(testSimulator, params);
    int numAct = testSimulator.GetNumActions();
    int numObs = testSimulator.GetNumObservations();

    // With equal value, action with lowest count is selected
    VNODE* vnode1 = mcts.ExpandNode(testSimulator.CreateStartState(), index);
    vnode1->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode1->Child(action).Value.Set(99, 0);
        else
            vnode1->Child(action).Value.Set(100 + action, 0);
    assert(mcts.GreedyUCB(vnode1, true) == 3);

    // With high counts, action with highest value is selected
    VNODE* vnode2 = mcts.ExpandNode(testSimulator.CreateStartState(), index);
    vnode2->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode2->Child(action).Value.Set(99 + numObs, 1);
        else
            vnode2->Child(action).Value.Set(100 + numAct - action, 0);
    assert(mcts.GreedyUCB(vnode2, true) == 3);

    // Action with low value and low count beats actions with high counts
    VNODE* vnode3 = mcts.ExpandNode(testSimulator.CreateStartState(), index);
    vnode3->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode3->Child(action).Value.Set(1, 1);
        else
            vnode3->Child(action).Value.Set(100 + action, 1);
    assert(mcts.GreedyUCB(vnode3, true) == 3);

    // Actions with zero count is always selected
    VNODE* vnode4 = mcts.ExpandNode(testSimulator.CreateStartState(), index);
    vnode4->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode4->Child(action).Value.Set(0, 0);
        else
            vnode4->Child(action).Value.Set(1, 1);
    assert(mcts.GreedyUCB(vnode4, true) == 3);
}

void MCTS::UnitTestRollout(const int& index)
{
    TEST_SIMULATOR testSimulator(2, 2, 0);
    PARAMS params;
    params.NumSimulations = 1000;
    params.MaxDepth = 10;
    MCTS mcts(testSimulator, params);
    double totalReward;
    for (int n = 0; n < mcts.Params.NumSimulations; ++n)
    {
        STATE* state = testSimulator.CreateStartState();
        mcts.TreeDepth = 0;
        totalReward += mcts.Rollout(*state, index);
    }
    double rootValue = totalReward / mcts.Params.NumSimulations;
    double meanValue = testSimulator.MeanValue();
    assert(fabs(meanValue - rootValue) < 0.1);
}

void MCTS::UnitTestSearch(int depth, const int& index)
{
    TEST_SIMULATOR testSimulator(3, 2, depth);
    PARAMS params;
    params.MaxDepth = depth + 1;
    params.NumSimulations = pow(10, depth + 1);
    MCTS mcts(testSimulator, params);
    mcts.UCTSearch(index);
    double rootValue = mcts.Roots[index == 0 ? index : index-1]->Value.GetValue();
    double optimalValue = testSimulator.OptimalValue();
    assert(fabs(optimalValue - rootValue) < 0.1);
}

//-----------------------------------------------------------------------------
