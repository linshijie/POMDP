#include "experiment.h"
#include "boost/timer.hpp"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(100),
    NumSteps(20),
    SimSteps(100),
    TimeOut(3600),
    MinDoubles(10),
    MaxDoubles(10),
    MinRewardDoubles(0),
    MaxRewardDoubles(1),
    EnableRewardIterations(false),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(20),
    AutoExploration(true),
    BreakOnTerminate(true),
    RealSimCommunication(false),
    RandomiseCommunication(false)
{
    RandomActions.clear();
    
    RandomActions.push_back(false);
    RandomActions.push_back(false);
}

EXPERIMENT::EXPERIMENT(SIMULATOR& real,
    SIMULATOR& simulator, const string& outputFile,
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams)
:   Real(real),
    Simulator(simulator),
    OutputFile(outputFile.c_str()),
    ExpParams(expParams),
    SearchParams(searchParams),
    UpdatePlanStatistics(false)
{
    Results.SuccessfulPlanCount.clear();
    Results.PlanSequenceReward.clear();
    Results.PlanSequenceLength.clear();
    for (int i = 0; i < real.GetNumAgents(); i++)
    {
	STATISTIC s1;
	Results.SuccessfulPlanCount.push_back(s1);
	STATISTIC s2;
	Results.PlanSequenceReward.push_back(s2);
	STATISTIC s3;
	Results.PlanSequenceLength.push_back(s3);
    }
    if (ExpParams.AutoExploration)
    {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

void EXPERIMENT::Run()
{
    boost::timer timer;
    MCTS mcts(Simulator, SearchParams);
        
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = ExpParams.RandomActions[0], outOfParticles2 = ExpParams.RandomActions[1];
    int t;
    
    STATE* state = Real.CreateStartState();
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);
    
    if (ExpParams.RandomiseCommunication)
    {
	double probLoss = UTILS::RandomDouble(0.0,0.5);
	double probDelay = UTILS::RandomDouble(0.0,0.5);
	double probMisinterp = UTILS::RandomDouble(0.0,0.5);
	Real.SetProbMessageLoss(probLoss);
	Real.SetProbMessageDelay(probDelay);
	Real.SetProbMessageMisinterp(probMisinterp);
	Simulator.SetProbMessageLoss(probLoss);
	Simulator.SetProbMessageDelay(probDelay);
	Simulator.SetProbMessageMisinterp(probMisinterp);
    }
    
    double timeFactor = 1.0/Real.GetNumAgents();
        
    std::vector<int> planCounts;
    std::vector<STATISTIC> planRewards;
    std::vector<STATISTIC> planLengths;
    int jointGoalCount = 0;
    if (UpdatePlanStatistics)
    {
	for (int i = 0; i < Real.GetNumAgents(); i++)
	{
	    planCounts.push_back(0);
	    STATISTIC s1;
	    planRewards.push_back(s1);
	    STATISTIC s2;
	    planLengths.push_back(s2);
	}
    }

    for (t = 0; t < ExpParams.NumSteps; t++)
    {
	int observation;
	double reward;
	int action;
	int action0 = 0, action1 = 0;
	if (!SearchParams.MultiAgent)
	    action = mcts.SelectAction(0);
	else
	{
	    action0 = mcts.SelectAction(1);
	    action1 = mcts.SelectAction(2);
	    /*if (SearchParams.RewardAdaptive[0] && !SearchParams.JointQActions[0])
		action0 = Simulator.GetAgentAction(action0,1);
	    if (SearchParams.RewardAdaptive[1] && !SearchParams.JointQActions[1])
		action1 = Simulator.GetAgentAction(action1,2);*/
	    action = action0 + Simulator.GetNumAgentActions()*action1;
	}
	SIMULATOR::STATUS status = mcts.GetStatus(0);
	status.JointGoalCount = 0;
	bool statusComm = status.UseCommunication;
	if (statusComm)
	{
	    status.MessagesToBeSent.clear();
	    status.MessagesReceived.clear();
	    status.MessagesToBeSent.push_back(Real.SelectMessage(mcts.GetStatus(1), mcts.GetHistory(1), action0));
	    status.MessagesToBeSent.push_back(Real.SelectMessage(mcts.GetStatus(2), mcts.GetHistory(2), action1));
	}
	status.UseCommunication = ExpParams.RealSimCommunication;
	terminal = Real.Step(*state, action, observation, reward, status);
	status.UseCommunication = statusComm;
	if (statusComm)
	{
	    for (int i = 0; i < (int) status.MessagesReceived.size(); i++)
		mcts.SetLastMessageReceived(i+1, status.MessagesReceived[i]);
	}

	Results.Reward.Add(reward);
	undiscountedReturn += reward;
	discountedReturn += reward * discount;
	discount *= Real.GetDiscount();
	
	jointGoalCount += status.JointGoalCount;
	
	if (UpdatePlanStatistics)
	{
	    if (!SearchParams.MultiAgent)
	    {
		planCounts[0] += mcts.GetStatus(0).SuccessfulPlanCount;
		if (mcts.GetStatus(0).SuccessfulPlanCount > 0)
		{
		    planRewards[0].Add(mcts.GetStatus(0).PlanSequenceReward/mcts.GetStatus(0).SuccessfulPlanCount);
		    planLengths[0].Add(mcts.GetStatus(0).PlanSequenceLength*1.0/mcts.GetStatus(0).SuccessfulPlanCount);
		}
	    }
	    else
	    {
		planCounts[0] += mcts.GetStatus(1).SuccessfulPlanCount;
		if (mcts.GetStatus(1).SuccessfulPlanCount > 0)
		{
		    planRewards[0].Add(mcts.GetStatus(1).PlanSequenceReward/mcts.GetStatus(1).SuccessfulPlanCount);
		    planLengths[0].Add(mcts.GetStatus(1).PlanSequenceLength*1.0/mcts.GetStatus(1).SuccessfulPlanCount);
		}
		planCounts[1] += mcts.GetStatus(2).SuccessfulPlanCount;
		if (mcts.GetStatus(2).SuccessfulPlanCount > 0)
		{
		    planRewards[1].Add(mcts.GetStatus(2).PlanSequenceReward/mcts.GetStatus(2).SuccessfulPlanCount);
		    planLengths[1].Add(mcts.GetStatus(2).PlanSequenceLength*1.0/mcts.GetStatus(2).SuccessfulPlanCount);
		}
	    }
	}
	
	if (SearchParams.Verbose >= 1)
	{
	    Real.DisplayAction(action, cout);
	    Real.DisplayState(*state, cout);
	    Real.DisplayObservation(*state, observation, cout);
	    Real.DisplayReward(reward, cout);
	    sleep(1);
	}

	if (terminal)
	{
	    cout << "Terminated" << endl;
	    if (ExpParams.BreakOnTerminate)
		break;
	    else
	    {
		if (!SearchParams.MultiAgent)
		    mcts.ClearHistory(0);
		else
		{
		    mcts.ClearHistory(1);
		    mcts.ClearHistory(2);
		}
		state = Real.CreateStartState();
		outOfParticles = ExpParams.RandomActions[0];
		outOfParticles2 = ExpParams.RandomActions[1];
	    }
	}
	
	if (outOfParticles || outOfParticles2)
	    break;
	
	if (!SearchParams.MultiAgent)
	    outOfParticles = !mcts.Update(action, observation, reward, 0);
	else
	{
	    outOfParticles = !mcts.Update(action0, Simulator.GetAgentObservation(observation,1), reward, 1);
	    outOfParticles2 = !mcts.Update(action1, Simulator.GetAgentObservation(observation,2), reward, 2);
	}
	    
	if (outOfParticles || outOfParticles2)
	    break;

	if (timer.elapsed() > ExpParams.TimeOut)
	{
	    cout << "Timed out after " << t << " steps in "
		<< Results.Time.GetTotal() << "seconds" << endl;
	    break;
	}
    }

    if (outOfParticles || outOfParticles2)
    {
	if (outOfParticles)
	    cout << "Out of particles, finishing episode with SelectRandom " << endl;
	if (outOfParticles2)
	    cout << "Out of particles 2, finishing episode with SelectRandom " << endl;
        HISTORY history = mcts.GetHistory(0);
	HISTORY history2;
	if (SearchParams.MultiAgent)
	    history2 = mcts.GetHistory(2);
	
        while (++t < ExpParams.NumSteps)
        {
            int observation;
            double reward;

            // This passes real state into simulator!
            // SelectRandom must only use fully observable state
            // to avoid "cheating"
            int action;
	    int action0 = 0, action1 = 0;
	    if (!SearchParams.MultiAgent)
		action = Simulator.SelectRandom(*state, history, mcts.GetStatus(0), 0);
	    else
	    {
		if (outOfParticles)
		{
		    if (SearchParams.JointQActions[0])
			action0 = Simulator.SelectRandom(*state, history, mcts.GetStatus(1), 0);
		    else
			action0 = Simulator.SelectRandom(*state, history, mcts.GetStatus(1), 1);
		}
		else
		    action0 = mcts.SelectAction(1);
		if (outOfParticles2)
		{
		    if (SearchParams.JointQActions[1])
			action1 = Simulator.SelectRandom(*state, history2, mcts.GetStatus(2), 0);
		    else
			action1 = Simulator.SelectRandom(*state, history2, mcts.GetStatus(2), 2);
		    
		}
		else
		    action1 = mcts.SelectAction(2);
		
		/*if (SearchParams.RewardAdaptive[0] && !SearchParams.JointQActions[0] && !outOfParticles)
		    action0 = Simulator.GetAgentAction(action0,1);
		if (SearchParams.RewardAdaptive[1] && !SearchParams.JointQActions[1] && !outOfParticles2)
		    action1 = Simulator.GetAgentAction(action1,2);*/
		
		action = action0 + Simulator.GetNumAgentActions()*action1;
	    }
	    SIMULATOR::STATUS status = mcts.GetStatus(0);
	    status.JointGoalCount = 0;
            bool statusComm = status.UseCommunication;
	    if (statusComm)
	    {
		status.MessagesToBeSent.clear();
		status.MessagesReceived.clear();
		status.MessagesToBeSent.push_back(Real.SelectMessage(mcts.GetStatus(1), mcts.GetHistory(1), action0));
		status.MessagesToBeSent.push_back(Real.SelectMessage(mcts.GetStatus(2), mcts.GetHistory(2), action1));
	    }
	    status.UseCommunication = ExpParams.RealSimCommunication;
            terminal = Real.Step(*state, action, observation, reward, status);
	    status.UseCommunication = statusComm;
	    if (statusComm)
	    {
		for (int i = 0; i < (int) status.MessagesReceived.size(); i++)
		    mcts.SetLastMessageReceived(i+1, status.MessagesReceived[i]);
	    }

            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();
	    
	    jointGoalCount += status.JointGoalCount;
	    
	    if (UpdatePlanStatistics)
	    {
		if (SearchParams.MultiAgent)
		{
		    if (!outOfParticles)
		    {
			planCounts[0] += mcts.GetStatus(1).SuccessfulPlanCount;
			if (mcts.GetStatus(1).SuccessfulPlanCount > 0)
			{
			    planRewards[0].Add(mcts.GetStatus(1).PlanSequenceReward/mcts.GetStatus(1).SuccessfulPlanCount);
			    planLengths[0].Add(mcts.GetStatus(1).PlanSequenceLength*1.0/mcts.GetStatus(1).SuccessfulPlanCount);
			}
		    }
		    if (!outOfParticles2)
		    {
			planCounts[1] += mcts.GetStatus(2).SuccessfulPlanCount;
			if (mcts.GetStatus(2).SuccessfulPlanCount > 0)
			{
			    planRewards[1].Add(mcts.GetStatus(2).PlanSequenceReward/mcts.GetStatus(2).SuccessfulPlanCount);
			    planLengths[1].Add(mcts.GetStatus(2).PlanSequenceLength*1.0/mcts.GetStatus(2).SuccessfulPlanCount);
			}
		    }
		}
	    }

            if (SearchParams.Verbose >= 1)
            {
                Real.DisplayAction(action, cout);
                Real.DisplayState(*state, cout);
                Real.DisplayObservation(*state, observation, cout);
                Real.DisplayReward(reward, cout);
            }

            if (terminal)
            {
                cout << "Terminated" << endl;
		if (ExpParams.BreakOnTerminate)
		    break;
		else
		{
		    if (!SearchParams.MultiAgent)
			mcts.ClearHistory(0);
		    else
		    {
			mcts.ClearHistory(1);
			mcts.ClearHistory(2);
		    }
		    state = Real.CreateStartState();
		    outOfParticles = ExpParams.RandomActions[0];
		    outOfParticles2 = ExpParams.RandomActions[1];
		}
            }

            if (!SearchParams.MultiAgent)
		history.Add(action, observation);
	    else
	    {
		if (outOfParticles)
		{
		    if (SearchParams.JointQActions[0])
			history.Add(Simulator.GetAgentAction(action,1), Simulator.GetAgentObservation(observation,1));
		    else
			history.Add(action0, Simulator.GetAgentObservation(observation,1));
		}
		else
		    outOfParticles = !mcts.Update(action0, Simulator.GetAgentObservation(observation,1), reward, 1);
		if (outOfParticles2)
		{
		    if (SearchParams.JointQActions[1])
			history2.Add(Simulator.GetAgentAction(action,2), Simulator.GetAgentObservation(observation,2));
		    else
			history2.Add(action1, Simulator.GetAgentObservation(observation,2));
		}
		else
		    outOfParticles2 = !mcts.Update(action1, Simulator.GetAgentObservation(observation,2), reward, 2);
	    }
        }
    }

    Results.Time.Add(timer.elapsed()/(t > 0 ? t*1.0 : 1.0));
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    Results.JointGoalCount.Add(jointGoalCount);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;
	
    if (!ExpParams.BreakOnTerminate)
	cout << "Total joint goals " << Results.JointGoalCount.GetTotal() << endl;
	
    if (UpdatePlanStatistics)
    {
	for (int i = 0; i < Real.GetNumAgents(); i++)
	{
	    Results.SuccessfulPlanCount[i].Add((double) planCounts[i]);
	    Results.PlanSequenceReward[i].Add(planRewards[i].GetMean());
	    Results.PlanSequenceLength[i].Add((double) planLengths[i].GetMean());
	    cout << "Successful Plan Count = " << planCounts[i]
		<< ", average = " << Results.SuccessfulPlanCount[i].GetMean() << endl;
	    cout << "Mean Plan Sequence Reward = " << planRewards[i].GetMean()
		<< ", average = " << Results.PlanSequenceReward[i].GetMean() << endl;
	    cout << "Mean Plan Sequence Length = " << planLengths[i].GetMean() 
		<< ", average = " << Results.PlanSequenceLength[i].GetMean() << endl;
	}
    }
}

void EXPERIMENT::MultiRun()
{
    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
            << SearchParams.NumSimulations << " simulations";
	if (ExpParams.EnableRewardIterations)
	    cout << " and " << SearchParams.NumLearnSimulations << " reward simulations";
	cout << "... " << endl;
        Run();
        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
}

void EXPERIMENT::DiscountedReturn()
{
    OutputFile.precision(4);
    cout << "Main runs" << endl;
    if (!ExpParams.EnableRewardIterations)
	OutputFile << "Simulations,Runs,Undiscounted return,Undiscounted error,Discounted return,Discounted error,Time,Joint Goal Count,";
    else
	OutputFile << "Simulations,Reward Simulations,Runs,Undiscounted return,Undiscounted error,Discounted return,Discounted error,Time,Joint Goal Count,";
    
    if (UpdatePlanStatistics)
    {
	for (int k = 0; k < Real.GetNumAgents(); k++)
	    OutputFile << "Plan Count " << k << ",Error,Plan Reward " << k << ",Error,Plan Length " << k << ",Error,"; 
    }
    
    OutputFile << "\n";
    
    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    //ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;
	
	int minIter, maxIter;
	
	if (ExpParams.EnableRewardIterations)
	{
	    minIter = ExpParams.MinRewardDoubles;
	    maxIter = ExpParams.MaxRewardDoubles;
	}
	else
	{
	    minIter = 0;
	    maxIter = 1;
	}
	
	for (int j = minIter; j < maxIter; j++)
	{
	    //if (ExpParams.EnableRewardIterations)
		//SearchParams.NumLearnSimulations = 1 << j;
	    Results.Clear();
	    MultiRun();

	    cout << "Simulations = " << SearchParams.NumSimulations << endl;
	    if (ExpParams.EnableRewardIterations)
		cout << "Reward Simulations = " << SearchParams.NumLearnSimulations << endl;
	    cout << "Runs = " << Results.Time.GetCount() << endl
		<< "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
		<< " +- " << Results.UndiscountedReturn.GetStdErr() << endl
		<< "Discounted return = " << Results.DiscountedReturn.GetMean()
		<< " +- " << Results.DiscountedReturn.GetStdErr() << endl
		<< "Time = " << Results.Time.GetMean() << endl
		<< "Joint goal count = " << Results.JointGoalCount.GetTotal() << endl;
	    if (UpdatePlanStatistics)
	    {
		for (int k = 0; k < Real.GetNumAgents(); k++)
		{
		    cout << "Mean Successful Plan Count = " << Results.SuccessfulPlanCount[k].GetMean() 
			<< " +- " << Results.SuccessfulPlanCount[k].GetStdErr() << endl;
		    cout << "Mean Plan Sequence Reward = " << Results.PlanSequenceReward[k].GetMean() 
			<< " +- " << Results.PlanSequenceReward[k].GetStdErr() << endl;
		    cout << "Mean Plan Sequence Length = " << Results.PlanSequenceLength[k].GetMean() 
			<< " +- " << Results.PlanSequenceLength[k].GetStdErr() << endl;
		}
	    }
	    OutputFile << SearchParams.NumSimulations << ",";
	    if (ExpParams.EnableRewardIterations)
		OutputFile << SearchParams.NumLearnSimulations << ",";
	    OutputFile << Results.Time.GetCount() << ","
		<< Results.UndiscountedReturn.GetMean() << ","
		<< Results.UndiscountedReturn.GetStdErr() << ","
		<< Results.DiscountedReturn.GetMean() << ","
		<< Results.DiscountedReturn.GetStdErr() << ","
		<< Results.Time.GetMean() << ","
		<< Results.JointGoalCount.GetTotal() << ",";
	    if (UpdatePlanStatistics)
	    {
		for (int k = 0; k < Real.GetNumAgents(); k++)
		{
		    OutputFile << Results.SuccessfulPlanCount[k].GetMean() << ","
			<< Results.SuccessfulPlanCount[k].GetStdErr() << ","
			<< Results.PlanSequenceReward[k].GetMean() << ","
			<< Results.PlanSequenceReward[k].GetStdErr() << ","
			<< Results.PlanSequenceLength[k].GetMean() << ","
			<< Results.PlanSequenceLength[k].GetStdErr() << ",";
		}
	    }
	    OutputFile << endl;
	}
    }
}

void EXPERIMENT::AverageReward()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tSteps\tAverage reward\tAverage time\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        Run();

        cout << "Simulations = " << SearchParams.NumSimulations << endl
            << "Steps = " << Results.Reward.GetCount() << endl
            << "Average reward = " << Results.Reward.GetMean()
            << " +- " << Results.Reward.GetStdErr() << endl
            << "Average time = " << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.Reward.GetCount() << "\t"
            << Results.Reward.GetMean() << "\t"
            << Results.Reward.GetStdErr() << "\t"
            << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
    }
}

//----------------------------------------------------------------------------
