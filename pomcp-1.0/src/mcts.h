#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"

class MCTS
{
public:

    struct PARAMS
    {
        PARAMS();

        int Verbose;
        int MaxDepth;
        int NumSimulations;
        int NumStartStates;
        bool UseTransforms;
        int NumTransforms;
        int MaxAttempts;
        int ExpandCount;
        double ExplorationConstant;
        bool UseRave;
        double RaveDiscount;
        double RaveConstant;
        bool DisableTree;
	bool MultiAgent;
	bool JointQActions;
	bool RewardAdaptive;
    };

    MCTS(const SIMULATOR& simulator, const PARAMS& params);
    ~MCTS();

    int SelectAction(const int& index);
    bool Update(int action, int observation, double reward, const int& index);

    void UCTSearch(const int& index);
    void RolloutSearch(const int& index);

    double Rollout(STATE& state, const int& index);

    const BELIEF_STATE& BeliefState(const int& index) const { return index > 0 ? Roots[index-1]->Beliefs() :
								    Roots[index]->Beliefs(); }
    const HISTORY& GetHistory(const int& index) const { return index > 0 ? Histories[index-1] : Histories[index]; }
    const SIMULATOR::STATUS& GetStatus(const int& index) const { return index > 0 ? Statuses[index-1] : Statuses[index];}
    void ClearStatistics(const int& index);
    void DisplayStatistics(std::ostream& ostr, const int& index) const;
    void DisplayValue(int depth, const int& index, std::ostream& ostr) const;
    void DisplayPolicy(int depth, const int& index, std::ostream& ostr) const;

    static void UnitTest(const int& index);
    static void InitFastUCB(double exploration);

private:

    const SIMULATOR& Simulator;
    int TreeDepth, PeakTreeDepth;
    PARAMS Params;
    std::vector<VNODE*> Roots;
    std::vector<HISTORY> Histories;
    HISTORY myhistory;
    std::vector<SIMULATOR::STATUS> Statuses;

    std::vector<STATISTIC> StatTreeDepths;
    std::vector<STATISTIC> StatRolloutDepths;
    std::vector<STATISTIC> StatTotalRewards;
    
    int GreedyUCB(VNODE* vnode, bool ucb) const;
    int SelectRandom() const;
    double SimulateV(STATE& state, VNODE* vnode, const int& index);
    double SimulateQ(STATE& state, QNODE& qnode, int action, const int& index);
    void AddRave(VNODE* vnode, double totalReward, const STATE& state, const int& index);
    VNODE* ExpandNode(const STATE* state, const int& index);
    void AddSample(VNODE* node, const STATE& state);
    void AddTransforms(VNODE* root, BELIEF_STATE& beliefs, const int& index);
    STATE* CreateTransform(const int& index) const;
    void Resample(BELIEF_STATE& beliefs);

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

    static void UnitTestGreedy(const int& index);
    static void UnitTestUCB(const int& index);
    static void UnitTestRollout(const int& index);
    static void UnitTestSearch(int depth, const int& index);
};

#endif // MCTS_H
