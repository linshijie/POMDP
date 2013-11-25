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
    NumStartStates(1),
    UseTransforms(false),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DisableTree(false),
    MultiAgent(false),
    JointQActions(true),
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

    if (!Params.MultiAgent)
	Root = ExpandNode(Simulator.CreateStartState(), 0);
    else
    {
	Root = ExpandNode(Simulator.CreateStartState(), 1);
	Root2 = ExpandNode(Simulator.CreateStartState(), 2);
    }
	
    for (int i = 0; i < Params.NumStartStates; i++)
    {
	Root->Beliefs().AddSample(Simulator.CreateStartState());
	if (Params.MultiAgent)
	    Root2->Beliefs().AddSample(Simulator.CreateStartState());
    }
}

MCTS::~MCTS()
{
    VNODE::Free(Root, Simulator);
    if (Params.MultiAgent)
	VNODE::Free(Root2, Simulator);
    VNODE::FreeAll();
}

bool MCTS::Update(int action, int observation, double reward, const int& index)
{
    if (!Params.MultiAgent)
	History.Add(action, observation);
    else
    {
	if (index == 1)
	    History.Add(Simulator.GetAgentAction(action,1), observation);
	else
	    History2.Add(Simulator.GetAgentAction(action,2), observation);
    }
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    if (Params.MultiAgent && !Params.JointQActions)
    {
	if (index == 1)
	    action = Simulator.GetAgentAction(action,1);
	else
	    action = Simulator.GetAgentAction(action,2);
    }
	
    QNODE& qnode = index > 1 ? Root2->Child(action) : Root->Child(action);
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
    {
	if (index > 1)
	    AddTransforms(Root2, beliefs, index);
	else
	    AddTransforms(Root, beliefs, index);
    }

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
    if (index > 1)
    {
	VNODE::Free(Root2, Simulator);
	VNODE* newRoot2 = ExpandNode(state, index);
	newRoot2->Beliefs() = beliefs;
	Root2 = newRoot2;
    }
    else
    {
	VNODE::Free(Root, Simulator);
	VNODE* newRoot = ExpandNode(state, index);
	newRoot->Beliefs() = beliefs;
	Root = newRoot;
    }
    
    return true;
}

int MCTS::SelectAction(const int& index)
{
    if (Params.DisableTree)
	RolloutSearch(index);
    else
	UCTSearch(index);
    if (index > 1)
	return GreedyUCB(Root2, false);
    else
	return GreedyUCB(Root, false);

}

void MCTS::RolloutSearch(const int& index)
{
    std::vector<double> totals(Simulator.GetNumActions(), 0.0);
    int historyDepth = index > 1 ? History2.Size() : History.Size();
    std::vector<int> legal;
    assert(BeliefState().GetNumSamples() > 0);
    Simulator.GenerateLegal(*BeliefState().GetSample(0), GetHistory(index), legal, GetStatus(index));
    random_shuffle(legal.begin(), legal.end());

    for (int i = 0; i < Params.NumSimulations; i++)
    {
	    int action = legal[i % legal.size()];
	    STATE* state;
	    if (index > 1)
		state = Root2->Beliefs().CreateSample(Simulator);
	    else
		state = Root->Beliefs().CreateSample(Simulator);
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

	    VNODE*& vnode = index > 1 ? Root2->Child(treeaction).Child(Simulator.GetAgentObservation(observation,1)) : 
					Root->Child(treeaction).Child(Simulator.GetAgentObservation(observation,2));
	    if (!vnode && !terminal)
	    {
		    vnode = ExpandNode(state, index);
		    AddSample(vnode, *state);
	    }
	    if (index == 0)
		History.Add(action, observation);
	    else if (index == 1)
		History.Add(Simulator.GetAgentAction(action,1), Simulator.GetAgentObservation(observation,1));
	    else
		History2.Add(Simulator.GetAgentAction(action,2), Simulator.GetAgentObservation(observation,2));

	    delayedReward = Rollout(*state, index);
	    totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
	    if (index > 1)
		Root2->Child(treeaction).Value.Add(totalReward);
	    else
		Root->Child(treeaction).Value.Add(totalReward);

	    Simulator.FreeState(state);
	    if (index > 1)
		History2.Truncate(historyDepth);
	    else
		History.Truncate(historyDepth);
    }
}

void MCTS::UCTSearch(const int& index)
{
    ClearStatistics(index);
    int historyDepth = index > 1 ? History2.Size() : History.Size();

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = index > 1 ? Root2->Beliefs().CreateSample(Simulator) : Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
	if (index > 1)
	    Status2.Phase = SIMULATOR::STATUS::TREE;
	else
	    Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;
        double totalReward = index > 1 ? SimulateV(*state, Root2, index) : SimulateV(*state, Root, index);
        StatTotalReward.Add(totalReward);
        StatTreeDepth.Add(PeakTreeDepth);

        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, index, cout);

        Simulator.FreeState(state);
        if (index > 1)
	    History2.Truncate(historyDepth);
	else
	    History.Truncate(historyDepth);
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
    AddRave(vnode, totalReward, index);
    return totalReward;
}

double MCTS::SimulateQ(STATE& state, QNODE& qnode, int action, const int& index)
{
    int observation;
    double immediateReward, delayedReward = 0;
    
    if (Params.MultiAgent && !Params.JointQActions)
    {
	if (index == 1)
	    action = Simulator.GetNumAgentActions()*action + Random(Simulator.GetNumAgentActions());
	else
	    action = Simulator.GetNumAgentActions()*Random(Simulator.GetNumAgentActions()) + action;
    }

    if (Simulator.HasAlpha())
        Simulator.UpdateAlpha(qnode, state);
    bool terminal = Simulator.Step(state, action, observation, immediateReward);
    assert(observation >= 0 && observation < Simulator.GetNumObservations());
    if (index == 0)
	History.Add(action, observation);
    else if (index == 1)
	History.Add(Simulator.GetAgentAction(action, 1), Simulator.GetAgentObservation(observation, 1));
    else
	History2.Add(Simulator.GetAgentAction(action, 2), Simulator.GetAgentObservation(observation, 2));

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayReward(immediateReward, cout);
        Simulator.DisplayState(state, cout);
    }

    VNODE*& vnode = qnode.Child(0);
    if (index == 0)
	qnode.Child(observation);
    else if (index == 1)
	vnode = qnode.Child(Simulator.GetAgentObservation(observation, 1));
    else if (index > 1)
	vnode = qnode.Child(Simulator.GetAgentObservation(observation, 2));
    
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

void MCTS::AddRave(VNODE* vnode, double totalReward, const int& index)
{
    double totalDiscount = 1.0;
    int maxIter = 0;
    if (index > 1)
	maxIter = History2.Size();
    else
	maxIter = History.Size();
    for (int t = TreeDepth; t < maxIter; ++t)
    {
	int action;
	if (index == 0 || (Params.MultiAgent && !Params.JointQActions))
	    action = History[t].Action;
	else
	{
	    std::vector<int> otherActions;
	    for (int a = 0; a < Simulator.GetNumAgentActions(); ++a)
		otherActions.push_back(a);
	    if (index == 1)
		action = Simulator.GetNumAgentActions()*History[t].Action + otherActions[Random(otherActions.size())];
	    else
		action = Simulator.GetNumAgentActions()*otherActions[Random(otherActions.size())] + History2[t].Action;
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
    if (index > 1)
	Simulator.Prior(state, History2, vnode, Status2, index);
    else
	Simulator.Prior(state, History, vnode, Status, index);

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
	if (index > 1)
	    History2.Display(cout);
	else
	    History.Display(cout);
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
    Status.Phase = SIMULATOR::STATUS::ROLLOUT;
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

        int action;
	if (index == 0)
	    action = Simulator.SelectRandom(state, History, Status, index);
	else if (index == 1)
	    action = Simulator.GetNumAgentActions()*Simulator.SelectRandom(state, History2, Status2, index) +
			Random(Simulator.GetNumAgentActions());
	else
	    action = Simulator.GetNumAgentActions()*Random(Simulator.GetNumAgentActions()) + 
			Simulator.SelectRandom(state, History, Status, index);
	
        terminal = Simulator.Step(state, action, observation, reward);
        if (index == 0)
	    History.Add(action, observation);
	else if (index == 1)
	    History.Add(Simulator.GetAgentAction(action, 1), Simulator.GetAgentObservation(observation, 1));
	else
	    History2.Add(Simulator.GetAgentAction(action, 2), Simulator.GetAgentObservation(observation, 2));

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

    if (index > 1)
	StatRolloutDepth2.Add(numSteps);
    else
	StatRolloutDepth.Add(numSteps);
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

    STATE* state;
    if (index > 1)
	state = Root2->Beliefs().CreateSample(Simulator);
    else
	state = Root->Beliefs().CreateSample(Simulator);
    int action;
    if (index == 0)
	action = History.Back().Action;
    else
    {
	std::vector<int> otherActions;
	for (int a = 0; a < Simulator.GetNumAgentActions(); ++a)
	    otherActions.push_back(a);
	if (index == 1)
	    action = Simulator.GetNumAgentActions()*History.Back().Action + otherActions[Random(otherActions.size())];
	else
	    action = Simulator.GetNumAgentActions()*otherActions[Random(otherActions.size())] + History2.Back().Action;
    }
    Simulator.Step(*state, action, stepObs, stepReward);
    if (index > 1)
    {
	if (Simulator.LocalMove(*state, History2, stepObs, Status2))
	    return state;
    }
    else
    {
	if (Simulator.LocalMove(*state, History, stepObs, Status))
	    return state;
    }
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
    if (index > 1)
    {
	StatTreeDepth2.Clear();
	StatRolloutDepth2.Clear();
	StatTotalReward2.Clear();
    }
    else
    {
	StatTreeDepth.Clear();
	StatRolloutDepth.Clear();
	StatTotalReward.Clear();
    }
}

void MCTS::DisplayStatistics(ostream& ostr, const int& index) const
{
    if (Params.Verbose >= 1)
    {
	if (index > 1)
	{
	    StatTreeDepth2.Print("Tree depth", ostr);
	    StatRolloutDepth2.Print("Rollout depth", ostr);
	    StatTotalReward2.Print("Total reward", ostr);
	}
	else
	{
	    StatTreeDepth.Print("Tree depth", ostr);
	    StatRolloutDepth.Print("Rollout depth", ostr);
	    StatTotalReward.Print("Total reward", ostr);
	}
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
    if (index > 1)
	Root2->DisplayValue(history, depth, ostr);
    else
	Root->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, const int& index, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Policy:" << endl;
    if (index > 1)
	Root2->DisplayPolicy(history, depth, ostr);
    else
	Root->DisplayPolicy(history, depth, ostr);
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
    double rootValue = mcts.Root->Value.GetValue();
    double optimalValue = testSimulator.OptimalValue();
    assert(fabs(optimalValue - rootValue) < 0.1);
}

//-----------------------------------------------------------------------------
