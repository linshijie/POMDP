#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "history.h"
#include "node.h"
#include "utils.h"
#include <iostream>
#include <math.h>

class BELIEF_STATE;

class STATE : public MEMORY_OBJECT
{
};

class REWARD_TEMPLATE : public MEMORY_OBJECT
{
public:
    int RewardIndex;
    //std::pair<double,double> RewardParams;
    double RewardValue;
};

class SIMULATOR
{
public:

    struct KNOWLEDGE
    {
        enum
        {
            PURE,
            LEGAL,
            SMART,
            NUM_LEVELS
        };

        KNOWLEDGE();
        
        int RolloutLevel;
        int TreeLevel;
        int SmartTreeCount;
        double SmartTreeValue;
        
        int Level(int phase) const
        {
            assert(phase < STATUS::NUM_PHASES);
            if (phase == STATUS::TREE)
                return TreeLevel;
            else
                return RolloutLevel;
        }
    };

    struct STATUS
    {
        STATUS();
        
        enum
        {
            TREE,
            ROLLOUT,
            NUM_PHASES
        };
        
        enum
        {
            CONSISTENT,
            INCONSISTENT,
            RESAMPLED,
            OUT_OF_PARTICLES
        };
        
        int Phase;
        int Particles;
	
	int agentindex;
	int perspindex;
	bool jointhistory;
	
	bool RewardAdaptive;
	
	//std::pair<double,double> RewardParams;
	double SampledRewardValue;
	double CurrOtherReward;
	
	std::vector<int> MainSequence;
	std::vector<double> MainQValueSequence;
	std::vector<double> MainVValueSequence;
	std::vector<double> MainOtherQValueSequence;
	
	std::vector<int> LearnSequence;
	std::vector<double> LearnRewardValueSequence;
	std::vector<double> LearnQValueSequence;
	std::vector<double> LearnVValueSequence;
	std::vector<double> LearnOtherQValueSequence;
	bool LearningPhase;
	
	int RolloutLevel;
	
	bool UpdateValues;
    };
    
    /*struct INITIAL_REWARD_PARAMS
    {
	INITIAL_REWARD_PARAMS();
	
	double Alpha;
	double Beta;
    };*/

    SIMULATOR();
    SIMULATOR(int numActions, int numObservations, double discount = 1.0);    
    virtual ~SIMULATOR();

    // Create start start state (can be stochastic)
    virtual STATE* CreateStartState() const = 0;

    // Free memory for state
    virtual void FreeState(STATE* state) const = 0;

    // Update state according to action, and get observation and reward. 
    // Return value of true indicates termination of episode (if episodic)
    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward, STATUS& status) const = 0;
        
    // Create new state and copy argument (must be same type)
    virtual STATE* Copy(const STATE& state) const = 0;
    
    //Same for rewards
    REWARD_TEMPLATE* Copy(const REWARD_TEMPLATE& reward) const;
    
    // Sanity check
    virtual void Validate(const STATE& state) const;

    // Modify state stochastically to some related state
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const;
	
    //Create initial reward template
    //std::pair<double, double> InitialiseRewardParams() const;
    REWARD_TEMPLATE* CreateInitialReward(const double& value, const int& index) const;
	
    // Free memory for reward samples
    void FreeReward(REWARD_TEMPLATE* reward) const;

    // Use domain knowledge to assign prior value and confidence to actions
    // Should only use fully observable state variables
    void Prior(const STATE* state, const HISTORY& history, VNODE* vnode,
        const STATUS& status, const int& index) const;

    // Use domain knowledge to select actions stochastically during rollouts
    // Should only use fully observable state variables
    int SelectRandom(const STATE& state, const HISTORY& history,
        const STATUS& status, const int& index) const;

    // Generate set of legal actions
    virtual void GenerateLegal(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status) const;
	
    // Generate set of preferred actions
    virtual void GeneratePreferred(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status) const;
	
    virtual void GenerateLegalAgent(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status, const int& index) const;
	
    virtual void GeneratePreferredAgent(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status, const int& index) const;

    // For explicit POMDP computation only
    virtual bool HasAlpha() const;
    virtual void AlphaValue(const QNODE& qnode, double& q, int& n) const;
    virtual void UpdateAlpha(QNODE& qnode, const STATE& state) const;

    // Textual display
    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayReward(double reward, std::ostream& ostr) const;
    
    virtual void DisplayAgentAction(int action, std::ostream& ostr) const;
    virtual void DisplayAgentObservation(int observation, std::ostream& ostr) const;

    // Accessors
    void SetKnowledge(const KNOWLEDGE& knowledge) { Knowledge = knowledge; }
    int GetRolloutLevel() const { return Knowledge.RolloutLevel; }
    int GetNumActions() const { return NumActions; }
    int GetNumObservations() const { return NumObservations; }
    int GetNumAgents() const { return NumAgents; }
    int GetNumAgentActions() const { return NumAgentActions; }
    int GetNumAgentObservations() const { return NumAgentObservations; }
    int GetAgentAction(const int& action, const int& index) const { return index > 1 ? action/NumAgentActions : 
									    action%NumAgentActions;}
    int GetAgentObservation(const int& observation, const int& index) const { return index > 1 ? 
						    observation/NumAgentObservations : observation%NumAgentObservations;}
    bool IsEpisodic() const { return false; }
    double GetDiscount() const { return Discount; }
    double GetRewardRange() const { return RewardRange; }
    double GetHorizon(double accuracy, int undiscountedHorizon = 100) const;
    
protected:

    int NumActions, NumObservations, NumAgents, NumAgentActions, NumAgentObservations;
    double Discount, RewardRange;
    KNOWLEDGE Knowledge;
    mutable MEMORY_POOL<REWARD_TEMPLATE> RewardMemoryPool;
    //INITIAL_REWARD_PARAMS InitialRewardParams;
};

#endif // SIMULATOR_H
