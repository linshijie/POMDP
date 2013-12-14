#include "beliefstate.h"
#include "simulator.h"
#include "utils.h"

using namespace UTILS;

BELIEF_STATE::BELIEF_STATE()
: 	TotalRewardWeight(0.0)
{
    Samples.clear();
    RewardSamples.clear();
}

void BELIEF_STATE::Free(const SIMULATOR& simulator)
{
    for (std::vector<STATE*>::iterator i_state = Samples.begin();
            i_state != Samples.end(); ++i_state)
    {
        simulator.FreeState(*i_state);
    }
    Samples.clear();
    
    for (std::vector<REWARD_TEMPLATE*>::iterator r_temp = RewardSamples.begin();
	    r_temp != RewardSamples.end(); ++r_temp)
    {
	simulator.FreeReward(*r_temp);
    }
    RewardSamples.clear();
    TotalRewardWeight = 0.0;
}

STATE* BELIEF_STATE::CreateSample(const SIMULATOR& simulator) const
{
    int index = Random(Samples.size());
    return simulator.Copy(*Samples[index]);
}

REWARD_TEMPLATE* BELIEF_STATE::CreateRewardSample(const SIMULATOR& simulator) const
{
    int index = 0;
    double randRew = RandomDouble(0.0, TotalRewardWeight);
    double currRew = 0.0;
    do
    {
	currRew += RewardSamples[index]->RewardWeight;
	if (currRew >= randRew)
	    break;
	index++;
    } while(index < (int) RewardSamples.size() && currRew < TotalRewardWeight);
    assert(index < (int) RewardSamples.size());
    return simulator.Copy(*RewardSamples[index]);
}


void BELIEF_STATE::AddSample(STATE* state)
{
    Samples.push_back(state);
}

void BELIEF_STATE::AddRewardSample(REWARD_TEMPLATE* reward)
{
    RewardSamples.push_back(reward);
    TotalRewardWeight += reward->RewardWeight;
}

void BELIEF_STATE::Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(simulator.Copy(**i_state));
    }
    for (std::vector<REWARD_TEMPLATE*>::const_iterator r_temp = beliefs.RewardSamples.begin();
	r_temp != beliefs.RewardSamples.end(); ++r_temp)
    {
	AddRewardSample(simulator.Copy(**r_temp));
    }
}

void BELIEF_STATE::Move(BELIEF_STATE& beliefs)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(*i_state);
    }
    beliefs.Samples.clear();
    for (std::vector<REWARD_TEMPLATE*>::const_iterator r_temp = beliefs.RewardSamples.begin();
	 r_temp != beliefs.RewardSamples.end(); ++r_temp)
    {
	AddRewardSample(*r_temp);
    }
    beliefs.RewardSamples.clear();
    beliefs.TotalRewardWeight = 0.0;
}
