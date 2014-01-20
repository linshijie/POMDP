#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "mcts.h"
#include "simulator.h"
#include "statistic.h"
#include <fstream>

//----------------------------------------------------------------------------

struct RESULTS
{
    void Clear();

    STATISTIC Time;
    STATISTIC Reward;
    STATISTIC DiscountedReturn;
    STATISTIC UndiscountedReturn;
    std::vector<STATISTIC> SuccessfulPlanCount;
    std::vector<STATISTIC> PlanSequenceReward;
    std::vector<STATISTIC> PlanSequenceLength;
    STATISTIC JointGoalCount;
};

inline void RESULTS::Clear()
{
    Time.Clear();
    Reward.Clear();
    DiscountedReturn.Clear();
    UndiscountedReturn.Clear();
    JointGoalCount.Clear();
    for (int i = 0; i < (int) SuccessfulPlanCount.size(); i++)
    {
	SuccessfulPlanCount[i].Clear();
	PlanSequenceReward[i].Clear();
	PlanSequenceLength[i].Clear();
    }
}

//----------------------------------------------------------------------------

class EXPERIMENT
{
public:

    struct PARAMS
    {
        PARAMS();
        
        int NumRuns;
        int NumSteps;
        int SimSteps;
        double TimeOut;
        int MinDoubles, MaxDoubles;
	int MinRewardDoubles, MaxRewardDoubles;
	bool EnableRewardIterations;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
        bool AutoExploration;
	bool BreakOnTerminate;
    };

    EXPERIMENT(const SIMULATOR& real, const SIMULATOR& simulator, 
        const std::string& outputFile, 
        EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams);

    void Run();
    void MultiRun();
    void DiscountedReturn();
    void AverageReward();

private:

    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    EXPERIMENT::PARAMS& ExpParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;

    std::ofstream OutputFile;
    bool UpdatePlanStatistics;
};

//----------------------------------------------------------------------------

#endif // EXPERIMENT_H
