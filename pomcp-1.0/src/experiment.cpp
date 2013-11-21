#include "experiment.h"
#include "boost/timer.hpp"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(50),
    NumSteps(1000),
    SimSteps(1000),
    TimeOut(3600),
    MinDoubles(0),
    MaxDoubles(15),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(20),
    AutoExploration(true)
{
}

EXPERIMENT::EXPERIMENT(const SIMULATOR& real,
    const SIMULATOR& simulator, const string& outputFile,
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams)
:   Real(real),
    Simulator(simulator),
    OutputFile(outputFile.c_str()),
    ExpParams(expParams),
    SearchParams(searchParams)
{
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
    bool outOfParticles = false, outOfParticles2 = false;
    int t;

    STATE* state = Real.CreateStartState();
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

    for (t = 0; t < ExpParams.NumSteps; t++)
    {
        int observation;
        double reward;
        int action;
	int jointaction0 = 0, jointaction1 = 0;
	if (!SearchParams.MultiAgent)
	    action = mcts.SelectAction(0);
	else
	{
	    if (SearchParams.JointQActions)
	    {
		jointaction0 = mcts.SelectAction(1);
		jointaction1 = mcts.SelectAction(2);
		action = Simulator.GetNumAgentActions()*Simulator.GetAgentAction(jointaction0,1) + 
			Simulator.GetAgentAction(jointaction1,2);
	    }
	}
        terminal = Real.Step(*state, action, observation, reward);

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
        discount *= Real.GetDiscount();

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
            break;
        }
	
	if (!SearchParams.MultiAgent)
	    outOfParticles = !mcts.Update(action, observation, reward, 0);
	else
	{
	    outOfParticles = !mcts.Update(jointaction0, Simulator.GetAgentObservation(observation,1), reward, 1);
	    outOfParticles2 = !mcts.Update(jointaction1, Simulator.GetAgentObservation(observation,2), reward, 2);
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
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
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
	    int jointaction0 = 0, jointaction1 = 0;
	    if (!SearchParams.MultiAgent)
		action = Simulator.SelectRandom(*state, history, mcts.GetStatus(0), 0);
	    else
	    {
		if (outOfParticles)
		    jointaction0 = Simulator.GetNumAgentActions()*Simulator.SelectRandom(*state, history, 
					mcts.GetStatus(1), 1) + rand()%Simulator.GetNumAgentActions();
		else
		    jointaction0 = mcts.SelectAction(1);
		if (outOfParticles2)
		    jointaction1 = Simulator.GetNumAgentActions()*(rand()%Simulator.GetNumAgentActions()) 
					+ Simulator.SelectRandom(*state, history2, mcts.GetStatus(2), 2);
		else
		    jointaction1 = mcts.SelectAction(2);
		
		action = Simulator.GetNumAgentActions()*Simulator.GetAgentAction(jointaction0,1) + 
			    Simulator.GetAgentAction(jointaction1,2);
	    }
            terminal = Real.Step(*state, action, observation, reward);

            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();

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
                break;
            }

            if (!SearchParams.MultiAgent)
		history.Add(action, observation);
	    else
	    {
		if (outOfParticles)
		    history.Add(Simulator.GetAgentAction(action,1), Simulator.GetAgentObservation(observation,1));
		else
		    outOfParticles = !mcts.Update(jointaction0, Simulator.GetAgentObservation(observation,1), reward, 1);
		if (outOfParticles2)
		    history2.Add(Simulator.GetAgentAction(action,2), Simulator.GetAgentObservation(observation,2));
		else
		    outOfParticles2 = !mcts.Update(jointaction1, Simulator.GetAgentObservation(observation,2), reward, 2);
	    }
        }
    }

    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;
}

void EXPERIMENT::MultiRun()
{
    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
            << SearchParams.NumSimulations << " simulations... " << endl;
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
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

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
        MultiRun();

        cout << "Simulations = " << SearchParams.NumSimulations << endl
            << "Runs = " << Results.Time.GetCount() << endl
            << "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
            << " +- " << Results.UndiscountedReturn.GetStdErr() << endl
            << "Discounted return = " << Results.DiscountedReturn.GetMean()
            << " +- " << Results.DiscountedReturn.GetStdErr() << endl
            << "Time = " << Results.Time.GetMean() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.Time.GetCount() << "\t"
            << Results.UndiscountedReturn.GetMean() << "\t"
            << Results.UndiscountedReturn.GetStdErr() << "\t"
            << Results.DiscountedReturn.GetMean() << "\t"
            << Results.DiscountedReturn.GetStdErr() << "\t"
            << Results.Time.GetMean() << endl;
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
