#ifndef BELIEF_STATE_H
#define BELIEF_STATE_H

#include <vector>

class STATE;
class REWARD_TEMPLATE;
class SIMULATOR;

class BELIEF_STATE
{
public:

    BELIEF_STATE();

    // Free memory for all states
    void Free(const SIMULATOR& simulator);

    // Creates new state, now owned by caller
    STATE* CreateSample(const SIMULATOR& simulator) const;
    
    //Same for rewards
    REWARD_TEMPLATE* CreateRewardSample(const SIMULATOR& simulator) const;

    // Added state is owned by belief state
    void AddSample(STATE* state);
    
    //Same for rewards
    void AddRewardSample(REWARD_TEMPLATE* reward);

    // Make own copies of all samples
    void Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator);

    // Move all samples into this belief state
    void Move(BELIEF_STATE& beliefs);

    bool Empty() const { return Samples.empty(); }
    bool EmptyRewards() const { return RewardSamples.empty(); }
    int GetNumSamples() const { return Samples.size(); }
    int GetNumRewardSamples() const { return RewardSamples.size(); }
    const STATE* GetSample(int index) const { return Samples[index]; }
    const REWARD_TEMPLATE* GetRewardSample(int index) const { return RewardSamples[index]; }
    void SetRewardSample(REWARD_TEMPLATE* rewardTemplate, const int& index);
    
    //Rewards
    
private:

    std::vector<STATE*> Samples;
    std::vector<REWARD_TEMPLATE*> RewardSamples;
    //double TotalRewardWeight;
};

#endif // BELIEF_STATE_H
