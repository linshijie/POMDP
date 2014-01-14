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
    NumLearnSimulations(1),
    NumStartStates(20),
    NumStartRewards(1),
    UseTransforms(false),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DoFastUCB(false),
    DisableTree(false),
    MultiAgent(true),
    RewardOffset(100.0),
    InitialRewardWeight(20.0)
{
    JointQActions.clear();
    MinMax.clear();
    RewardAdaptive.clear();
    
    JointQActions.push_back(false);
    JointQActions.push_back(false);
    
    MinMax.push_back(false);
    MinMax.push_back(false);
    
    RewardAdaptive.push_back(true);
    RewardAdaptive.push_back(true);
}

MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0)
{
    //VNODE::NumChildren = Params.MultiAgent && !Params.JointQActions[0] ? Simulator.GetNumAgentActions() : 
	//		Simulator.GetNumActions();
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
	status.RewardAdaptive = Params.RewardAdaptive[i];
	status.UpdateValues = true;
	status.LearningPhase = false;
	
	status.MultiAgentPriorValue = Simulator.GetRewardRange();
	status.MultiAgentPriorCount = 1;//(int) (ceil(sqrt(Params.NumSimulations)));
	status.SmartTreeCount = ((int) sqrt(Params.NumSimulations));
	
	if (i == 0)
	    status.HumanDefined = false;
	else
	    status.HumanDefined = false;
	
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
	VNODE* root; 
	root->NumChildren = Params.MultiAgent && !Params.JointQActions[i] ? Simulator.GetNumAgentActions() : 
			Simulator.GetNumActions();
	if (Params.RewardAdaptive[i])
	{
	    //if (Params.JointQActions[i])
		QNODE::NumOtherAgentValues = 1;
	    //else
		//QNODE::NumOtherAgentValues = Simulator.GetNumAgentActions();
	}
	STATE* state = Simulator.CreateStartState();
	root = ExpandNode(state, Params.MultiAgent ? i+1 : i, Params.MultiAgent ? i+1 : i);
	Roots.push_back(root);
    }
    

    for (int i = 0; i < Simulator.GetNumAgents(); i++)
	for (int j = 0; j < Params.NumStartStates; j++)
	    Roots[i]->Beliefs().AddSample(Simulator.CreateStartState());
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
	if (index == 1 && Params.JointQActions[index == 0 ? index : index-1])
	    Histories[index == 0 ? index : index-1].Add(Simulator.GetAgentAction(action,1), observation);
	else if (index == 1)
	    Histories[index == 0 ? index : index-1].Add(action, observation);
	else if (index == 2 && Params.JointQActions[index == 0 ? index : index-1])
	    Histories[index == 0 ? index : index-1].Add(Simulator.GetAgentAction(action,2), observation);
	else if (index == 2)
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
    VNODE* newRoot = ExpandNode(state, index, index);
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
    return GreedyUCB(Roots[index == 0 ? index : index-1], Params.DoFastUCB, index);
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
	    STATE* state = Roots[index == 0 ? index : index-1]->Beliefs().CreateSample(Simulator);
	    REWARD_TEMPLATE* rewardTemplate;
	    if (Params.RewardAdaptive[index == 0 ? index : index-1])
	    {
		rewardTemplate = Roots[index == 0 ? index : index-1]->Beliefs().CreateRewardSample(Simulator);
		Statuses[index == 0 ? index : index-1].SampledRewardValue = rewardTemplate->RewardValue;
	    }
	    Simulator.Validate(*state);

	    int observation;
	    double immediateReward, delayedReward, totalReward;
	    bool terminal = Simulator.Step(*state, action, observation, immediateReward, Statuses[index == 0 ? index : index-1]);	 
	    
	    int treeaction = action;//, othertreeaction = action;
	    if (Params.MultiAgent && !Params.JointQActions[index == 0 ? index : index-1])
	    {
		if (index == 1)
		    treeaction = Simulator.GetAgentAction(action,1);
		else
		    treeaction = Simulator.GetAgentAction(action,2);
		/*if (Params.RewardAdaptive[index == 0 ? index : index-1])
		{
		    if (index == 1)
			othertreeaction = Simulator.GetAgentAction(action,2);
		    else
			othertreeaction = Simulator.GetAgentAction(action,1);
		}*/
	    }

	    VNODE*& vnode = Roots[index == 0 ? index : index-1]->Child(treeaction).Child(index == 0 ?
		    observation : Simulator.GetAgentObservation(observation,index));			
	    
	    if (!vnode && !terminal)
	    {
		    vnode = ExpandNode(state, index, index);
		    AddSample(vnode, *state);
	    }
	    
	    Histories[index == 0 ? index : index-1].Add(index == 0 ? action : Simulator.GetAgentAction(action,index), 
				       index == 0 ? observation : Simulator.GetAgentObservation(observation,index));
	    double otherDelayedReward = 0.0;
	    delayedReward = Rollout(*state, index, otherDelayedReward);
	    totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
	    Roots[index == 0 ? index : index-1]->Child(treeaction).Value.Add(totalReward);
	    
	    if (Params.RewardAdaptive[index == 0 ? index : index-1])
	    {
		double otherTotalReward = Statuses[index == 0 ? index : index-1].CurrOtherReward + 
		    Simulator.GetDiscount()*otherDelayedReward;
		//if (Params.JointQActions[index == 0 ? index : index-1])
		    Roots[index == 0 ? index : index-1]->Child(treeaction).OtherAgentValues[0].Add(otherTotalReward);
		//else
		    //Roots[index == 0 ? index : index-1]->Child(treeaction).OtherAgentValues[othertreeaction].Add(otherTotalReward);
		Simulator.FreeReward(rewardTemplate);
	    }

	    Simulator.FreeState(state);
	    Histories[index == 0 ? index : index-1].Truncate(historyDepth);
    }
}

void MCTS::UCTSearch(const int& index)
{
    ClearStatistics(index);
    int historyDepth = GetHistory(index).Size();
    
    Statuses[index == 0 ? index : index-1].SuccessfulPlanCount = 0;
    Statuses[index == 0 ? index : index-1].PlanSequenceReward = 0.0;
    Statuses[index == 0 ? index : index-1].PlanSequenceLength = 0;

    int n = 0;
    
    while (n < Params.NumSimulations)
    {
        STATE* state = Roots[index == 0 ? index : index-1]->Beliefs().CreateSample(Simulator);
	//REWARD_TEMPLATE* rewardTemplate;
	double rewardTemplateValue;
	if (Params.MultiAgent && Params.RewardAdaptive[index == 0 ? index : index-1])
	{
	    //rewardTemplate = Roots[index == 0 ? index : index-1]->Beliefs().CreateRewardSample(Simulator);
	    rewardTemplateValue = Roots[index == 0 ? index : index-1]->Beliefs().GetRewardSample(0)->RewardValue;
	    Statuses[index == 0 ? index : index-1].SampledRewardValue = rewardTemplateValue;
	}
        Simulator.Validate(*state);
	
	Statuses[index == 0 ? index : index-1].Phase = SIMULATOR::STATUS::TREE;
	
	Statuses[index == 0 ? index : index-1].MainSequence.clear();
	Statuses[index == 0 ? index : index-1].MainFullSequence.clear();
	Statuses[index == 0 ? index : index-1].MainQValueSequence.clear();
	Statuses[index == 0 ? index : index-1].MainVValueSequence.clear();
	Statuses[index == 0 ? index : index-1].MainOtherQValueSequence.clear();
	
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;
	double otherTotalReward = 0.0;
	Statuses[index == 0 ? index : index-1].UpdateValues = true;
	Statuses[index == 0 ? index : index-1].LearningPhase = false;
	Statuses[index == 0 ? index : index-1].TerminalReached = false;
        double totalReward = SimulateV(*state, Roots[index == 0 ? index : index-1], index, 
					otherTotalReward);
	
	if (Statuses[index == 0 ? index : index-1].TerminalReached)
	{
	    Statuses[index == 0 ? index : index-1].SuccessfulPlanCount++;
	    Statuses[index == 0 ? index : index-1].PlanSequenceReward += totalReward;
	    Statuses[index == 0 ? index : index-1].PlanSequenceLength += 
		    (int) Statuses[index == 0 ? index : index-1].MainFullSequence.size()/2;
	}
	
	StatTotalRewards[index == 0 ? index : index-1].Add(totalReward);
        StatTreeDepths[index == 0 ? index : index-1].Add(PeakTreeDepth);

        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, index, cout);
	
	Simulator.FreeState(state);
	Histories[index == 0 ? index : index-1].Truncate(historyDepth);
	
	n++;
	
	
	//if (totalReward > 0)
	//    DisplaySequence(Statuses[index == 0 ? index : index-1].MainFullSequence, index);
	
	//Statuses[index == 0 ? index : index-1].UpdateValues = false;
	if (Params.MultiAgent && Params.RewardAdaptive[index == 0 ? index : index-1] && 
	    !Statuses[index == 0 ? index : index-1].TerminalReached)
	{
	    bool doLearn = true;
	    //for (int i = 0; i < Params.NumLearnSimulations; i++)
	    while (n < Params.NumSimulations && doLearn) 
	    {
		//STATE* tempState = Simulator.Copy(*initState);
		STATE* tempState = Roots[index == 0 ? index : index-1]->Beliefs().CreateSample(Simulator);
		double tempRewardTemplateValue = UTILS::Normal(rewardTemplateValue, 1.0);
		Statuses[index == 0 ? index : index-1].SampledRewardValue = tempRewardTemplateValue;
		
		double tempOtherTotalReward = 0.0;
		TreeDepth = 0;
		PeakTreeDepth = 0;
		
		Simulator.Validate(*tempState);
		
		
		Statuses[index == 0 ? index : index-1].LearnSequence.clear();
		Statuses[index == 0 ? index : index-1].LearnFullSequence.clear();
		Statuses[index == 0 ? index : index-1].LearnRewardValueSequence.clear();
		Statuses[index == 0 ? index : index-1].LearnVValueSequence.clear();
		Statuses[index == 0 ? index : index-1].LearnQValueSequence.clear();
		Statuses[index == 0 ? index : index-1].LearnOtherQValueSequence.clear();
		Statuses[index == 0 ? index : index-1].LearningPhase = true;
		
		Statuses[index == 0 ? index : index-1].Phase = SIMULATOR::STATUS::TREE;
		
		Statuses[index == 0 ? index : index-1].TerminalReached = false;
		double tempTotalReward = SimulateV(*tempState, Roots[index == 0 ? index : index-1], index, 
						   tempOtherTotalReward);
		
		n++;
		
		StatTotalRewards[index == 0 ? index : index-1].Add(tempTotalReward);
		StatTreeDepths[index == 0 ? index : index-1].Add(PeakTreeDepth);

		if (Params.Verbose >= 2)
		    cout << "Total reward = " << tempTotalReward << endl;
		if (Params.Verbose >= 3)
		    DisplayValue(4, index, cout);
		
		Simulator.FreeState(tempState);
		Histories[index == 0 ? index : index-1].Truncate(historyDepth);
		
		if (Statuses[index == 0 ? index : index-1].TerminalReached)
		{
		    Statuses[index == 0 ? index : index-1].SuccessfulPlanCount++;
		    Statuses[index == 0 ? index : index-1].PlanSequenceReward += tempTotalReward;
		    Statuses[index == 0 ? index : index-1].PlanSequenceLength += 
			    (int) Statuses[index == 0 ? index : index-1].LearnFullSequence.size()/2;
		}
		
		tempRewardTemplateValue = tempTotalReward;
		double probRatio = exp(tempTotalReward)/exp(totalReward);
		if (RandomDouble(0.0, 1.0) < min(1.0, probRatio) && 
		    Statuses[index == 0 ? index : index-1].TerminalReached)
		//if (tempTotalReward > totalReward && tempTotalReward > 0)
		{
		    //std::cout << totalReward << " " << tempTotalReward << " " << index << "\n";
		    //DisplaySequence(Statuses[index == 0 ? index : index-1].LearnFullSequence, index);
		    
		    rewardTemplateValue = tempRewardTemplateValue;
		    totalReward = tempTotalReward;
		    
		    /**
		    //Subtract old values
		    VNODE* vnode = Roots[index == 0 ? index : index-1];
		    int indV = (int) Statuses[index == 0 ? index : index-1].MainVValueSequence.size() - 1;
		    int indQ = 0;
		    Roots[index == 0 ? index : index-1]->Value.Subtract(Statuses[index == 0 ? index : 
			    index-1].MainVValueSequence[indV]);
		    indV--;
		    int size = (int) Statuses[index == 0 ? index : index-1].MainSequence.size();
		    for (int j = 0; j < size; j += 2)
		    {
			QNODE& qnode = vnode->Child(Statuses[index == 0 ? index : index-1].MainSequence[j]);
			if (indQ < (int) Statuses[index == 0 ? index : index-1].MainQValueSequence.size())
			{
			    qnode.Value.Subtract(Statuses[index == 0 ? index : index-1].MainQValueSequence[indQ]);
			    qnode.OtherAgentValues[0].Subtract(Statuses[index == 0 ? index : 
				index-1].MainOtherQValueSequence[indQ]);
			    indQ++;
			}
			if (j+1 < size)
			{
			    vnode = qnode.Child(Statuses[index == 0 ? index : index-1].MainSequence[j+1]);
			    if (vnode && indV >= 0)
			    {
				vnode->Value.Subtract(Statuses[index == 0 ? index : index-1].MainVValueSequence[indV]);
				indV--;
			    }
			}
		    }
		    
		    //Add new values
		    vnode = Roots[index == 0 ? index : index-1];
		    indV = (int) Statuses[index == 0 ? index : index-1].LearnVValueSequence.size() - 1;
		    indQ = 0;
		    Roots[index == 0 ? index : index-1]->Beliefs().SetRewardSample(rewardTemplateValue,0);
		    Roots[index == 0 ? index : index-1]->Value.Add(Statuses[index == 0 ? index : 
			    index-1].LearnVValueSequence[indV]);
		    indV--;
		    size = (int) Statuses[index == 0 ? index : index-1].LearnSequence.size();
		    for (int j = 0; j < size; j += 2)
		    {
			QNODE& qnode = vnode->Child(Statuses[index == 0 ? index : index-1].LearnSequence[j]);
			if (indQ < (int) Statuses[index == 0 ? index : index-1].LearnQValueSequence.size())
			{
			    qnode.Value.Add(Statuses[index == 0 ? index : index-1].LearnQValueSequence[indQ]);
			    qnode.OtherAgentValues[0].Add(Statuses[index == 0 ? index : 
				index-1].LearnOtherQValueSequence[indQ]);
			    indQ++;
			}
			if (j+1 < size)
			{
			    vnode = qnode.Child(Statuses[index == 0 ? index : index-1].LearnSequence[j+1]);
			    if (vnode)
			    {
				vnode->Beliefs().SetRewardSample(Statuses[index == 0 ? index : 
				    index-1].LearnRewardValueSequence[j/2], 0);
				if (indV >= 0)
				{
				    vnode->Value.Add(Statuses[index == 0 ? index : index-1].LearnVValueSequence[indV]);
				    indV--;
				}
			    }
			}
		    }
		    
		    //Reassign sequences (in case of multiple reward simulations)
		    Statuses[index == 0 ? index : index-1].MainSequence = 
			    Statuses[index == 0 ? index : index-1].LearnSequence;
		    Statuses[index == 0 ? index : index-1].MainQValueSequence = 
			    Statuses[index == 0 ? index : index-1].LearnQValueSequence;
		    Statuses[index == 0 ? index : index-1].MainVValueSequence = 
			    Statuses[index == 0 ? index : index-1].LearnVValueSequence;
		    Statuses[index == 0 ? index : index-1].MainOtherQValueSequence = 
			    Statuses[index == 0 ? index : index-1].LearnOtherQValueSequence;*/
		}
		else
		    doLearn = false;
	    }
	    Roots[index == 0 ? index : index-1]->Beliefs().SetRewardSample(rewardTemplateValue, 0);
	    //Roots[index == 0 ? index : index-1]->Beliefs().AddRewardSample(rewardTemplate);
	    
	    Statuses[index == 0 ? index : index-1].SampledRewardValue = rewardTemplateValue;
	}
	Statuses[index == 0 ? index : index-1].LearningPhase = false;
    }
    
    

    DisplayStatistics(cout, index);
}

double MCTS::SimulateV(STATE& state, VNODE* vnode, const int& index, double otherTotalReward)
{
    int action = GreedyUCB(vnode, Params.DoFastUCB, index);
    
    
    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return 0;

    if (TreeDepth == 1)
        AddSample(vnode, state);

    int ownaction = 0, otheraction = 0;
    double totalReward = 0.0;
    /*if (Params.RewardAdaptive[index == 0 ? index : index-1] && !Params.JointQActions[index == 0 ? index : index-1])
    {
	if (index == 1)
	   action = Simulator.GetAgentAction(action, 1);
	else
	   action = Simulator.GetAgentAction(action, 2);
    }*/
	
    QNODE& qnode = vnode->Child(action);
    
    if (Statuses[index == 0 ? index : index-1].LearningPhase)
    {
	Statuses[index == 0 ? index : index-1].LearnSequence.push_back(action);
	Statuses[index == 0 ? index : index-1].LearnFullSequence.push_back(action);
    }
    else
    {
	Statuses[index == 0 ? index : index-1].MainSequence.push_back(action);
	Statuses[index == 0 ? index : index-1].MainFullSequence.push_back(action);
    }
    
    totalReward = SimulateQ(state, qnode, action, index, otherTotalReward);
    
    if (Statuses[index == 0 ? index : index-1].UpdateValues)
    {
	vnode->Value.Add(totalReward);
	AddRave(vnode, totalReward, state, index);
	
	if (Statuses[index == 0 ? index : index-1].LearningPhase)
	    Statuses[index == 0 ? index : index-1].LearnVValueSequence.push_back(totalReward);
	else
	    Statuses[index == 0 ? index : index-1].MainVValueSequence.push_back(totalReward);
    }
    
    /*if (Params.RewardAdaptive[index == 0 ? index : index-1])
    {
	vnode->Value.Add(otherTotalReward);
	//rave not compatible
    }*/
	
    return totalReward;
}

double MCTS::SimulateQ(STATE& state, QNODE& qnode, int action, const int& index, double otherTotalReward)
{
    int observation;
    double immediateReward, delayedReward = 0;
    double otherImmediateReward = 0, otherDelayedReward = 0;
    
    
    if (Params.MultiAgent && !Params.JointQActions[index == 0 ? index : index-1])
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
    
    bool terminal = Simulator.Step(state, action, observation, immediateReward, Statuses[index == 0 ? index : index-1]);
    
    Statuses[index == 0 ? index : index-1].TerminalReached = terminal;
    
    if (Params.RewardAdaptive[index == 0 ? index : index-1] && Statuses[index == 0 ? index : index-1].LearningPhase)
	otherImmediateReward = immediateReward;//Statuses[index == 0 ? index : index-1].CurrOtherReward;
    
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
	  vnode = ExpandNode(&state, index, index);
    
    //if (!terminal)
    {
	if (Statuses[index == 0 ? index : index-1].LearningPhase)
	{
	    Statuses[index == 0 ? index : index-1].LearnSequence.push_back(index == 0 ? observation : 
		Simulator.GetAgentObservation(observation, index));
	    Statuses[index == 0 ? index : index-1].LearnFullSequence.push_back(index == 0 ? observation : 
		Simulator.GetAgentObservation(observation, index));
	}
	else
	{
	    Statuses[index == 0 ? index : index-1].MainSequence.push_back(index == 0 ? observation : 
		Simulator.GetAgentObservation(observation, index)); 
	    Statuses[index == 0 ? index : index-1].MainFullSequence.push_back(index == 0 ? observation : 
		Simulator.GetAgentObservation(observation, index)); 
	}
    }
    
    if (Params.RewardAdaptive[index == 0 ? index : index-1] && vnode)
    {
	/*rewardTemplate = vnode->Beliefs().CreateRewardSample(Simulator);
	    
	if (Statuses[index == 0 ? index : index-1].LearningPhase)
	{
	    rewardTemplate->RewardValue = UTILS::Normal(rewardTemplate->RewardValue, 1.0);
	    //if (!terminal)
		Statuses[index == 0 ? index : index-1].LearnRewardValueSequence.push_back(rewardTemplate->RewardValue);
	}
	Statuses[index == 0 ? index : index-1].SampledRewardValue = rewardTemplate->RewardValue;*/
	Statuses[index == 0 ? index : index-1].SampledRewardValue = vnode->Beliefs().GetRewardSample(0)->RewardValue;
	
	if (Statuses[index == 0 ? index : index-1].LearningPhase)
	    Statuses[index == 0 ? index : index-1].LearnRewardValueSequence.push_back(
		vnode->Beliefs().GetRewardSample(0)->RewardValue);
    }
    else
	Statuses[index == 0 ? index : index-1].SampledRewardValue = 0.0;

    if (!terminal)
    {
	TreeDepth++;
	if (vnode)
	    delayedReward = SimulateV(state, vnode, index, otherDelayedReward);
	else
	    delayedReward = Rollout(state, index, otherDelayedReward);
	TreeDepth--;
    }
    
    double totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
    if (Statuses[index == 0 ? index : index-1].UpdateValues)
    {
	qnode.Value.Add(totalReward);
	if (Statuses[index == 0 ? index : index-1].LearningPhase)
	    Statuses[index == 0 ? index : index-1].LearnQValueSequence.push_back(totalReward);
	else
	    Statuses[index == 0 ? index : index-1].MainQValueSequence.push_back(totalReward);
	if (Params.RewardAdaptive[index == 0 ? index : index-1])
	{
	    if (vnode)
		otherTotalReward = otherImmediateReward + Simulator.GetDiscount() * otherDelayedReward;
	    else
		otherTotalReward = otherImmediateReward + Simulator.GetDiscount() * delayedReward;
	    
	    //if (Params.JointQActions[index == 0 ? index : index-1])
		qnode.OtherAgentValues[0].Add(otherTotalReward);
	    //else
	    //{
		//int otheraction = index == 1 ? Simulator.GetAgentAction(action, 2) : Simulator.GetAgentAction(action, 1);
		//qnode.OtherAgentValues[0].Add(otherTotalReward);
	    //}
	    if (Statuses[index == 0 ? index : index-1].LearningPhase)
		Statuses[index == 0 ? index : index-1].LearnOtherQValueSequence.push_back(otherTotalReward);
	    else
		Statuses[index == 0 ? index : index-1].MainOtherQValueSequence.push_back(otherTotalReward);
	}
    }
    
    return totalReward;
}

void MCTS::AddRave(VNODE* vnode, double totalReward, const STATE& state, const int& index)
{
    double totalDiscount = 1.0;
    int maxIter = GetHistory(index).Size();
    for (int t = TreeDepth; t < maxIter; ++t)
    {
	int action;
	if (index == 0 || (Params.MultiAgent && !Params.JointQActions[index-1]))
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

VNODE* MCTS::ExpandNode(const STATE* state, const int& perspindex, const int& agentindex)
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, 0);
    
    if (!Params.MultiAgent)
	Simulator.Prior(state, GetHistory(perspindex), vnode, GetStatus(perspindex), 0);
    else
    {
	if (Params.JointQActions[perspindex == 0 ? perspindex : perspindex-1])
	    Simulator.Prior(state, GetHistory(perspindex), vnode, GetStatus(perspindex), 0);
	else
	    Simulator.Prior(state, GetHistory(perspindex), vnode, GetStatus(perspindex), agentindex);
    }
    
    //Reward samples
    double RewardTemplateWeight = (double) (1.0/Params.NumStartRewards);

    if (Params.RewardAdaptive[perspindex == 0 ? perspindex : perspindex-1])
    {
	Statuses[perspindex == 0 ? perspindex : perspindex-1].SampledRewardValue = 0.0;
	for (int j = 0; j < Params.NumStartRewards; j++)
	{
	    double otherReward = 0.0;
	    //STATE* st = Simulator.Copy(*state);
	    //double initReward = Rollout(*st, perspindex, otherReward);
	    REWARD_TEMPLATE* rewardTemplate = Simulator.CreateInitialReward(-0.1,j);
	    vnode->Beliefs().AddRewardSample(rewardTemplate);
	    //Simulator.FreeState(st);
	}
    }

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
	GetHistory(perspindex).Display(cout);
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


int MCTS::GreedyUCB(VNODE* vnode, bool ucb, const int& index) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);
    bool hasalpha = Simulator.HasAlpha();

    int maxIter;
    
    if ((Params.MultiAgent && !Params.JointQActions[index == 0 ? index : index-1]) ||
	(Params.MinMax[index == 0 ? index : index-1] && Params.JointQActions[index == 0 ? index : index-1])
    )
	maxIter = Simulator.GetNumAgentActions();
    else
	maxIter = Simulator.GetNumActions();
    
    
    std::vector<int> maxOwnActions;
    std::vector<int> maxOtherActions;
    maxOwnActions.clear();
    maxOtherActions.clear();
    double minRew = Infinity;
    
    /*int maxIter2 = (Params.MinMax[index == 0 ? index : index-1] && 
	    Params.JointQActions[index == 0 ? index : index-1]) || 
	    (Params.RewardAdaptive[index == 0 ? index : index-1] && !Params.JointQActions[index == 0 ? index : index-1]) ? 
	    Simulator.GetNumAgentActions() : 1;*/
    int maxIter2 = 
	Params.MinMax[index == 0 ? index : index-1] && Params.JointQActions[index == 0 ? index : index-1] ? 
	    Simulator.GetNumAgentActions() : 1;
	    
    int niter = 0;

    for (int action = 0; action < maxIter; action++)
    {
	    
	std::vector<int> currMaxOtherActions;
	currMaxOtherActions.clear();
	double currMaxRew = -Infinity;
	    
	for (int i = 0; i < maxIter2; i++)
	{
	    int treeaction = action;
	    if (Params.MinMax[index == 0 ? index : index-1] && Params.JointQActions[index == 0 ? index : index-1])
	    {
		if (index < 2)
		    treeaction = action + Simulator.GetNumAgentActions()*i;
		else
		    treeaction = i + Simulator.GetNumAgentActions()*action;
	    }
	    
	    double q, alphaq;
	    int n, alphan;
	    
	    double otherq = 0.0, otheralphaq = 0.0;
	    int othern, otheralphan;
	    
	    QNODE& qnode = vnode->Child(treeaction);
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
	    
	    if (Params.MultiAgent && 
		Params.RewardAdaptive[index == 0 ? index : index-1] && Statuses[index == 0 ? index : index-1].LearningPhase) 
	    {
		otherq = qnode.OtherAgentValues[0].GetValue();
		othern = qnode.OtherAgentValues[0].GetCount();
		
		q += otherq;
		
	    }

	    
	    
	    if (Params.MinMax[index == 0 ? index : index-1] && Params.JointQActions[index == 0 ? index : index-1])
	    {
		if (q >= currMaxRew)
		{
		    if (q > currMaxRew)
			currMaxOtherActions.clear();
		    currMaxRew = q;
		    currMaxOtherActions.push_back(i);
		}
	    }
	    else if (q >= bestq)
	    {
		if (q > bestq)
		    besta.clear();
		bestq = q;
		/*if (Params.RewardAdaptive[index == 0 ? index : index-1] && !Params.JointQActions[index == 0 ? index : index-1]) 
		{
		    int jointaction;
		    if (index < 2)
			jointaction = action + Simulator.GetNumAgentActions()*i;
		    else
			jointaction = i + Simulator.GetNumAgentActions()*action;
		    besta.push_back(jointaction);
		}
		else*/
		    besta.push_back(treeaction);
	    }
	}
	
	if (Params.MinMax[index == 0 ? index : index-1] && Params.JointQActions[index == 0 ? index : index-1])
	    if (currMaxRew <= minRew)
	    {
		if (currMaxRew < minRew && currMaxRew > -Infinity)
		{
		    maxOtherActions.clear();
		    maxOwnActions.clear();
		}
		minRew = currMaxRew;
		maxOtherActions.insert(maxOtherActions.end(), currMaxOtherActions.begin(), currMaxOtherActions.end());
		for (int j = 0; j < currMaxOtherActions.size(); j++)
		    maxOwnActions.push_back(action);
	    }
    }

    if (Params.MinMax[index == 0 ? index : index-1] && Params.JointQActions[index == 0 ? index : index-1])
    {
	assert(!maxOwnActions.size() && maxOwnActions.size() == maxOtherActions.size());
	int r = Random(maxOwnActions.size());
	int a;
	if (index < 2)
	    a = maxOwnActions[r] + Simulator.GetNumAgentActions()*maxOtherActions[r];
	else
	    a = maxOtherActions[r] + Simulator.GetNumAgentActions()*maxOwnActions[r];
	return a;
    }
    else
    {
	assert(!besta.empty());
	int a = besta[Random(besta.size())];
	return a;
    }
}

double MCTS::Rollout(STATE& state, const int& index, double otherReward)
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
        terminal = Simulator.Step(state, action, observation, reward, Statuses[index == 0 ? index : index-1]);
	
	Statuses[index == 0 ? index : index-1].TerminalReached = terminal;
	    
	Histories[index == 0 ? index : index-1].Add(index == 0 ? action : Simulator.GetAgentAction(action, index), 
			      index == 0 ? observation : Simulator.GetAgentObservation(observation, index));
	
	if (Statuses[index == 0 ? index : index-1].LearningPhase)
	{
	    Statuses[index == 0 ? index : index-1].LearnFullSequence.push_back(index == 0 ? action : 
		    Simulator.GetAgentAction(action, index));
	    Statuses[index == 0 ? index : index-1].LearnFullSequence.push_back(index == 0 ? observation : 
		    Simulator.GetAgentObservation(observation, index));
	}
	else
	{
	    Statuses[index == 0 ? index : index-1].MainFullSequence.push_back(index == 0 ? action : 
		    Simulator.GetAgentAction(action, index));
	    Statuses[index == 0 ? index : index-1].MainFullSequence.push_back(index == 0 ? observation : 
		    Simulator.GetAgentObservation(observation, index));
	}

        if (Params.Verbose >= 4)
        {
            Simulator.DisplayAction(action, cout);
            Simulator.DisplayObservation(state, observation, cout);
            Simulator.DisplayReward(reward, cout);
            Simulator.DisplayState(state, cout);
        }

        totalReward += reward * discount;
	if (Params.RewardAdaptive[index == 0 ? index : index-1])
	    otherReward += Statuses[index == 0 ? index : index-1].CurrOtherReward*discount;
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
	    if (Params.RewardAdaptive[index == 0 ? index : index-1])
	    {
		Statuses[index == 0 ? index : index-1].SampledRewardValue = 0.0;
		if (beliefs.GetNumRewardSamples() < Params.NumStartRewards)
		{
		    double otherReward = 0.0;
		    STATE* st = Simulator.Copy(*transform);
		    double initReward = Rollout(*st, index, otherReward);
		    REWARD_TEMPLATE* rewardTemplate = Simulator.CreateInitialReward(initReward,beliefs.GetNumRewardSamples());
		    beliefs.AddRewardSample(rewardTemplate);
		    Simulator.FreeState(st);
		}
	    }
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
    SIMULATOR::STATUS status = Statuses[index == 0 ? index : index-1];
    Simulator.Step(*state, action, stepObs, stepReward, status);
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

void MCTS::DisplaySequence(std::vector< int > sequence, const int& index) const
{
    std::cout << "\n";
    std::cout << "Robot " << index << "\n";
    for (int i = 0; i < (int) sequence.size(); i++)
    {
	if (i%2 == 0)
	    Simulator.DisplayAgentAction(sequence[i], std::cout);
	else
	    Simulator.DisplayAgentObservation(sequence[i], std::cout);
    }
    std::cout << "\n";
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

    VNODE* vnode = mcts.ExpandNode(testSimulator.CreateStartState(), index, index);
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
    VNODE* vnode1 = mcts.ExpandNode(testSimulator.CreateStartState(), index, index);
    vnode1->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode1->Child(action).Value.Set(99, 0);
        else
            vnode1->Child(action).Value.Set(100 + action, 0);
    assert(mcts.GreedyUCB(vnode1, true) == 3);

    // With high counts, action with highest value is selected
    VNODE* vnode2 = mcts.ExpandNode(testSimulator.CreateStartState(), index, index);
    vnode2->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode2->Child(action).Value.Set(99 + numObs, 1);
        else
            vnode2->Child(action).Value.Set(100 + numAct - action, 0);
    assert(mcts.GreedyUCB(vnode2, true) == 3);

    // Action with low value and low count beats actions with high counts
    VNODE* vnode3 = mcts.ExpandNode(testSimulator.CreateStartState(), index, index);
    vnode3->Value.Set(1, 0);
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode3->Child(action).Value.Set(1, 1);
        else
            vnode3->Child(action).Value.Set(100 + action, 1);
    assert(mcts.GreedyUCB(vnode3, true) == 3);

    // Actions with zero count is always selected
    VNODE* vnode4 = mcts.ExpandNode(testSimulator.CreateStartState(), index, index);
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
	double otherTotalReward = 0.0;
        totalReward += mcts.Rollout(*state, index, otherTotalReward);
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
