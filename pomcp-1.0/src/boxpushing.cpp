#include "boxpushing.h"
#include <iomanip>

using namespace std;
using namespace UTILS;

boost::math::beta_distribution<> beta(0.9,0.9);

BOXPUSHING::BOXPUSHING(int numsmallboxes, double probLargeAgentBox)
: XSize(4),
  YSize(3),
  NumSmallBoxes(numsmallboxes),
  NumLargeBoxes(1),
  RandomiseInitialState(false),
  ResetAtCompletion(false),
  ProbLargeAgentBox(probLargeAgentBox),
  ProbObservation(0.9)
{
    //NumSmallBoxes = 0;
    //NumLargeBoxes = 2;
    //XSize = 5;
    
    NumAgentActions = 4;
    NumAgentObservations = 6;
    
    NumAgentMessages = 3;
    NumMessages = pow(NumAgentMessages, NumAgents);
    
    NumAgents = 2;
    
    NumActions = pow(NumAgentActions, NumAgents);
    NumObservations = pow(NumAgentObservations, NumAgents);
    
    RewardRange = 99.9;
    Discount = 1.0;
    
    quantiles.clear();
    
    for (int i = 0; i < 100; i++)
	quantiles.push_back(boost::math::quantile(beta, RandomDouble(0.0, 1.0)));
    
    /*MultiAgentLabels.clear();
    for (int i = 0; i < NumAgentActions; i++)
    {
	std::vector< std::vector<bool> > outerLabels;
	outerLabels.clear();
	for (int j = 0; j < NumAgentObservations; j++)
	{
	    std::vector<bool> innerLabels;
	    innerLabels.clear();
	    for (int k = 0; k < NumAgentObservations; k++)
	    {
		if (i == MOVE && j == LARGE_BOX_OBS && k == AGENT_OBS)
		    innerLabels.push_back(true);
		else
		    innerLabels.push_back(false);
	    }
	    outerLabels.push_back(innerLabels);
	}
	MultiAgentLabels.push_back(outerLabels);
    }*/
}

STATE* BOXPUSHING::Copy(const STATE& state) const
{
    const BOXPUSHING_STATE& boxpushingstate = safe_cast<const BOXPUSHING_STATE&>(state);
    BOXPUSHING_STATE* newstate = MemoryPool.Allocate();
    *newstate = boxpushingstate;
    return newstate; 
}

void BOXPUSHING::Validate(const STATE& state) const
{
    const BOXPUSHING_STATE& bpstate = safe_cast<const BOXPUSHING_STATE&>(state);
    for (int i = 0; i < (int) bpstate.Agents.size(); i++)
	assert(Inside(bpstate.Agents[i].Position));
}

STATE* BOXPUSHING::CreateStartState() const
{
    BOXPUSHING_STATE* bpstate = MemoryPool.Allocate();
    bpstate->Cells.Resize(XSize, YSize);
    for (int i = 0; i < XSize * YSize; ++i)
    {
	BOXPUSHING_STATE::CELL& cell = bpstate->Cells(i);
	cell.Occupied = false;
	cell.Content = NONE;
    }
    bpstate->Agents.clear();
    bpstate->SmallBoxes.clear();
    bpstate->LargeBoxes.clear();
    
    for (int i = 0; i < NumLargeBoxes; i++)
    {
	PUSH_ENTITY pe;
	do
	{
	    pe.Direction = 1;
	    if (NumLargeBoxes == 1)
		pe.Position = COORD(1, 1);
	    else
	    {
		pe.Position = COORD(i == 0 ? 0 : 3, 1);
	    }
	    pe.Length = 2;
	}
	while (Collision(*bpstate, pe));
	
	MarkPushEntity(*bpstate, pe, LARGE_BOX);
	bpstate->LargeBoxes.push_back(pe);
    }
    
    for (int i = 0; i < NumSmallBoxes; i++)
    {
	PUSH_ENTITY pe;
	do
	{
	    pe.Direction = 1;
	    pe.Position = COORD(Random(XSize), 1);
	    pe.Length = 1;
	}
	while (Collision(*bpstate, pe));
	
	MarkPushEntity(*bpstate, pe, SMALL_BOX);
	bpstate->SmallBoxes.push_back(pe);
    }
    
    
    for (int i = 0; i < NumAgents; i++)
    {
	PUSH_ENTITY pe;
	do
	{
	    //if (!RandomiseInitialState)
		pe.Direction = i==0?1:3;
	    //else
		//pe.Direction = Random(4);
	    if (!RandomiseInitialState)
		pe.Position = COORD(i==0 ? 0 : XSize-1, 0);
	    else
		pe.Position = COORD(i==0? Random(1) : Random(1)+2, 0);
	    pe.Length = 1;
	}
	while (Collision(*bpstate, pe));
	
	MarkPushEntity(*bpstate, pe, AGENT);
	bpstate->Agents.push_back(pe);
    }
    
    bpstate->NumBoxesRemaining = NumSmallBoxes + NumLargeBoxes;
    return bpstate;
}

void BOXPUSHING::FreeState(STATE* state) const
{
    BOXPUSHING_STATE* bpstate = safe_cast<BOXPUSHING_STATE*>(state);
    MemoryPool.Free(bpstate);
}

bool BOXPUSHING::Step(STATE& state, int action,
    int& observation, double& reward, STATUS& status) const
{
    BOXPUSHING_STATE& bpstate = safe_cast<BOXPUSHING_STATE&>(state);
    
    int actions[] = {action%NumAgentActions, action/NumAgentActions};
    int messages[] = {0, 0};
    
    if (status.UseCommunication)
    {
	int action0 = GetActionComponentFromAction(actions[0]);
	int action1 = GetActionComponentFromAction(actions[1]);
	messages[0] = GetMessageComponentFromAction(actions[0]);
	messages[1] = GetMessageComponentFromAction(actions[1]);
	actions[0] = action0;
	actions[1] = action1;
	for (int i = 0; i < NumAgents; i++)
	    if (UTILS::RandomDouble(0.0,1.0) > ProbMessageLoss)
	    {
		std::string mess = MessageToString(messages[i]);
		STATE::MESSAGE message;
		message.Message = mess;
		message.AgentID = i;
		if (UTILS::RandomDouble(0.0,1.0) > ProbMessageDelay)
		    bpstate.MessageQueue.push_front(message);
		else
		    bpstate.MessageQueue.push_back(message);
	    }
    }
    
    
    
    COORD next0 = bpstate.Agents[0].Position + COORD::Compass[bpstate.Agents[0].Direction];
    COORD next1 = bpstate.Agents[1].Position + COORD::Compass[bpstate.Agents[1].Direction];
    COORD after0 = next0 + COORD::Compass[bpstate.Agents[0].Direction];
    COORD after1 = next1 + COORD::Compass[bpstate.Agents[1].Direction];
    
    if (actions[0] == MOVE && actions[1] == MOVE && 
	COORD::EuclideanDistance(bpstate.Agents[0].Position,bpstate.Agents[1].Position) <= 1.0 &&
	bpstate.Cells.Inside(next0) && bpstate.Cells.Inside(next1) &&
	bpstate.Cells(next0).Content == LARGE_BOX && 
	bpstate.Cells(next1).Content == LARGE_BOX &&
	bpstate.Cells.Inside(after0) && bpstate.Cells.Inside(after1) &&
	bpstate.Cells(after0).Content == NONE &&
	bpstate.Cells(after1).Content == NONE)
    {
	//can move large box
	if (RandomDouble(0.0,1.0) < 0.9)
	{
	    bool found = false;
	    int i = 0;
	    while (i < NumLargeBoxes && !found)
	    {
		if (bpstate.LargeBoxes.at(i).Position == next0 || bpstate.LargeBoxes.at(i).Position == next1)
		    found = true;
		else
		    i++;
	    }
	    UnmarkPushEntity(bpstate, bpstate.Agents[0]);
	    UnmarkPushEntity(bpstate, bpstate.Agents[1]);
	    UnmarkCell(bpstate, next0);
	    UnmarkCell(bpstate, next1);
	    bpstate.Agents[0].Position = next0;
	    bpstate.Agents[1].Position = next1;
	    MarkPushEntity(bpstate, bpstate.Agents[0], AGENT);
	    MarkPushEntity(bpstate, bpstate.Agents[1], AGENT);
	    MarkCell(bpstate, after0, LARGE_BOX);
	    MarkCell(bpstate, after1, LARGE_BOX);
	    bpstate.LargeBoxes[i].Position += COORD::Compass[bpstate.Agents[0].Direction];
	    if (after0.Y == YSize-1){
		//std::cout << "large box goal " << status.perspindex << "\n";
		reward = 99.8;
		bpstate.NumBoxesRemaining--;
		status.JointGoalCount++;
	    }
	    else
		reward = -0.2;
	}
	else
	    reward = -0.2;
    }
    else
    {
	if (RandomDouble(0.0,1.0) < 0.5)
	    reward = StepAgent(bpstate, 0, actions[0]) + StepAgent(bpstate, 1, actions[1]);
	else
	    reward = StepAgent(bpstate, 1, actions[1]) + StepAgent(bpstate, 0, actions[0]);
    }
    
    bool terminated = bpstate.NumBoxesRemaining < NumLargeBoxes+NumSmallBoxes;
    
    /*if (terminated && ResetAtCompletion)
    {
	terminated = false;
	for (int i = 0; i < NumSmallBoxes; i++)
	{
	    bpstate.SmallBoxes[i].Direction = 1;
	    bpstate.SmallBoxes[i].Position = COORD(i == 0 ? 0 : 3, 1);
	    bpstate.SmallBoxes[i].Length = 1;
	}
	for (int i = 0; i < NumLargeBoxes; i++)
	{
	    bpstate.LargeBoxes[i].Direction = 1;
	    bpstate.LargeBoxes[i].Position = COORD(1, 1);
	    bpstate.LargeBoxes[i].Length = 1;
	}
	for (int i = 0; i < NumAgents; i++)
	{
	    bpstate.Agents[i].Direction = i==0?1:3;;
	    bpstate.Agents[i].Position = COORD(i == 0 ? 1 : XSize-2, 0);
	    bpstate.Agents[i].Length = 1;
	}
    }*/
    
    int obs0 = GetAgentObservation(bpstate, 0);
    int obs1 = GetAgentObservation(bpstate, 1);
    
    if (obs0 == LARGE_BOX_OBS && obs1 == LARGE_BOX_OBS && RandomDouble(0.0,1.0) < ProbLargeAgentBox)
    {
	obs0 = LARGE_BOX_AGENT_OBS;
	obs1 = LARGE_BOX_AGENT_OBS;
    }
    
    if (status.UseCommunication)
    {
	for (int i = 0; i < NumAgents; i++)
	{
	    int m = 0;
	    for (int j = 0; j < (int) bpstate.MessageQueue.size(); j++)
		if (bpstate.MessageQueue.at(j).AgentID != i)
		{
		    if (RandomDouble(0.0,1.0) > ProbMessageMisinterp)
			m = MessageToInt(bpstate.MessageQueue.at(j).Message);
		    else
			m = Random(NumAgentMessages);
		    break;
		}
	    if (i == 0)
		obs0 = obs0 + m*NumAgentObservations;
	    else
		obs1 = obs1 + m*NumAgentObservations;
	}
    }
    
    observation = obs0 + NumAgentObservations*obs1;
    
    if (status.RewardAdaptive && status.LearningPhase)
	status.CurrOtherReward = quantiles[Random(quantiles.size())]*reward*2;
    else
	status.CurrOtherReward = 0.0;
	//status.CurrOtherReward = reward + UTILS::RandomDouble(-10.0,10.0);//UTILS::Normal(reward, 1.0);
    
    return terminated;
}

double BOXPUSHING::StepAgent(BOXPUSHING_STATE& bpstate, int agentindex, int agentaction) const
{
    switch(agentaction){
	case(STAY):
	    return -0.1;
	case(TURN_CCW):
	    if (RandomDouble(0.0,1.0) < 0.9)
		bpstate.Agents[agentindex].Direction = COORD::Anticlockwise(bpstate.Agents[agentindex].Direction);
	    return -0.1;
	case(TURN_CW):
	    if (RandomDouble(0.0,1.0) < 0.9)
		bpstate.Agents[agentindex].Direction = COORD::Clockwise(bpstate.Agents[agentindex].Direction);
	    return -0.1;
	default:
	    //move
	    if (RandomDouble(0.0,1.0) < 0.9){
		COORD next = bpstate.Agents[agentindex].Position + COORD::Compass[bpstate.Agents[agentindex].Direction];
		if (!bpstate.Cells.Inside(next) || bpstate.Cells(next).Content == AGENT ||
		    bpstate.Cells(next).Content == LARGE_BOX)
		    return -5.1;
		if (bpstate.Cells(next).Content == NONE)
		{
		    UnmarkPushEntity(bpstate, bpstate.Agents[agentindex]);
		    bpstate.Agents[agentindex].Position = next;
		    MarkPushEntity(bpstate, bpstate.Agents[agentindex], AGENT);
		    return -0.1;
		}
		if (bpstate.Cells(next).Content == SMALL_BOX)
		{
		    COORD after = next + COORD::Compass[bpstate.Agents[agentindex].Direction];
		    if (bpstate.Cells.Inside(after) && 
			bpstate.Cells(after).Content == NONE)
		    {
			bool found = false;
			int i = 0;
			while (i < NumSmallBoxes && !found)
			{
			    if (bpstate.SmallBoxes.at(i).Position == next)
				found = true;
			    else
				i++;
			}
			UnmarkPushEntity(bpstate, bpstate.Agents[agentindex]);
			UnmarkCell(bpstate, next);
			bpstate.Agents[agentindex].Position = next;
			MarkPushEntity(bpstate, bpstate.Agents[agentindex], AGENT);
			MarkCell(bpstate, after, SMALL_BOX);
			bpstate.SmallBoxes[i].Position += COORD::Compass[bpstate.Agents[agentindex].Direction];
			if (after.Y == YSize - 1 && next.Y < YSize - 1){
			    bpstate.NumBoxesRemaining--;
			    return 9.9;
			}
			return -0.1;
		    }
		    return -0.1;
		}  
		return -0.1;
	    }
	    return -0.1;
	
    }
}

int BOXPUSHING::GetAgentObservation(const BOXPUSHING_STATE& bpstate, const int& agentindex) const
{
    if (RandomDouble(0.0,1.0) > ProbObservation)
	return Random(NumAgentObservations);
    PUSH_ENTITY agent = bpstate.Agents[agentindex];
    COORD adjacent = agent.Position + COORD::Compass[agent.Direction];
    if (!bpstate.Cells.Inside(adjacent))
	return WALL_OBS;
    if (bpstate.Cells(adjacent).Content == AGENT)
	return AGENT_OBS;
    if (bpstate.Cells(adjacent).Content == SMALL_BOX)
	return SMALL_BOX_OBS;
    if (bpstate.Cells(adjacent).Content == LARGE_BOX)
	return LARGE_BOX_OBS;
    return EMPTY_OBS;
}

bool BOXPUSHING::Collision(const BOXPUSHING_STATE& bpstate,
    const PUSH_ENTITY& pushentity) const
{
    COORD pos = pushentity.Position;
    for (int i = 0; i < pushentity.Length; ++i)
    {
	if (!bpstate.Cells.Inside(pos))
            return true;
	const BOXPUSHING_STATE::CELL& cell = bpstate.Cells(pos);
        if (cell.Occupied)
            return true;
	pos += COORD::Compass[pushentity.Direction];
    }
    
    return false;
}

bool BOXPUSHING::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    BOXPUSHING_STATE& bpstate = safe_cast<BOXPUSHING_STATE&>(state);

    int boxtype = Random(2);
    int boxindex;
    PUSH_ENTITY pe;
    
    if (NumSmallBoxes == 0 && NumLargeBoxes == 0)
	return false;
    
    if (boxtype == 0  && NumSmallBoxes > 0)
    {
	boxindex = Random(NumSmallBoxes);
	pe = bpstate.SmallBoxes.at(boxindex);
    }
    else if (boxtype == 1 && NumLargeBoxes > 0)
    {
	boxindex = Random(NumLargeBoxes);
	pe = bpstate.LargeBoxes.at(boxindex);
    }
    else
	return false;
    
    if (pe.Position.Y == YSize-1)
	//box has already been pushed to the end
	return false;
    
    UnmarkPushEntity(bpstate, pe);
    
    COORD oldposition = pe.Position;
    
    do
    {
	pe.Position = COORD(Random(XSize), Random(YSize-1));
    }
    while(Collision(bpstate, pe));
    
    
    
    if (boxtype == 0)
    {
	bpstate.SmallBoxes[boxindex] = pe;
	MarkPushEntity(bpstate, pe, SMALL_BOX);
    }
    else if (boxtype == 1)
    {
	bpstate.LargeBoxes[boxindex] = pe;
	MarkPushEntity(bpstate, pe, LARGE_BOX);
    }

    int realObs = history.Back().Observation;
    int obs0 = GetAgentObservation(bpstate, 0);
    int obs1 = GetAgentObservation(bpstate, 1);
    
    if (obs0 == LARGE_BOX_OBS && obs1 == LARGE_BOX_OBS && RandomDouble(0.0,1.0) < ProbLargeAgentBox)
    {
	obs0 = LARGE_BOX_AGENT_OBS;
	obs1 = LARGE_BOX_AGENT_OBS;
    }
    
    int simObs = obs0 + NumAgentObservations*obs1;
    
    return realObs == simObs;
    
    
}

bool BOXPUSHING::IsActionMultiagent(const int& action, const HISTORY& history) const
{
    if (history.Size() == 0)
	return false;
    if (action == MOVE && (history.Back().Observation == LARGE_BOX_AGENT_OBS))
	return true;
    return false;
    
    /*int t = history.Size();
    if (action >= (int)MultiAgentLabels.size() || action < 0 || t < 2)
	return false;
    
    int observation1 = history[t-1].Observation;
    int observation2 = history[t-2].Observation;
    
    if (action == MOVE && observation1 == LARGE_BOX_OBS && observation2 == AGENT_OBS && RandomDouble(0.0,1.0) < pow(0.9,4))
	return true;
    return false;*/
    
    //return MultiAgentLabels[action][observation1][observation2];
}


void BOXPUSHING::GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const
{
    int maxIter = NumActions;
    if (status.UseCommunication)
	maxIter *= NumMessages;
    for (int a = 0; a < maxIter; a++)
        legal.push_back(a);
}

void BOXPUSHING::GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status) const
{
    const BOXPUSHING_STATE& bpstate = safe_cast<const BOXPUSHING_STATE&>(state);
    
    if (history.Size() == 0)
    {
	actions.push_back(MOVE + NumAgentActions*MOVE);
        return;
    }
    
    int lastObs = history.Back().Observation;
    int lastObs0 = lastObs%NumAgentObservations;
    int lastObs1 = lastObs/NumAgentObservations;
    
    std::vector<int> actions0;
    std::vector<int> actions1;
    
    GeneratePreferredAgentAction(bpstate, lastObs0, actions0, 0, status.HumanDefined);
    //GenerateLegalAgent(state,history,actions1,status);
    GeneratePreferredAgentAction(bpstate, lastObs1, actions1, 1, status.HumanDefined);
    
    for (int i = 0; i < (int)actions0.size(); i++)
	for (int j = 0; j < (int)actions1.size(); j++)
	    actions.push_back(actions0[i] + NumAgentActions*actions1[j]);
}

void BOXPUSHING::GenerateLegalAgent(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status, const int& index) const
{
    //GeneratePreferredAgent(state, history, actions, status, index);
    if (history.Size() == 0 && ProbLargeAgentBox > 0.5)
    {
	actions.push_back(MOVE);
        return;
    }
    int maxIter = NumAgentActions;
    if (status.UseCommunication)
	maxIter *= NumAgentMessages;
    for (int a = 0; a < NumAgentActions; a++)
        actions.push_back(a);
}

void BOXPUSHING::GeneratePreferredAgentAction(const BOXPUSHING_STATE& bpstate, const int& lastObs, 
					      std::vector<int>& actions, const int& index, const bool& humanDefined) const
{ 
    switch(lastObs)
    {
	case(EMPTY_OBS):
	    //actions.push_back(STAY);
	    actions.push_back(TURN_CW);
	    actions.push_back(TURN_CCW);
	    actions.push_back(MOVE);
	    break;
	case(WALL_OBS):
	    actions.push_back(TURN_CW);
	    actions.push_back(TURN_CCW);
	    break;
	case(AGENT_OBS):
	    //actions.push_back(STAY);
	    actions.push_back(TURN_CW);
	    actions.push_back(TURN_CCW);
	    //actions.push_back(MOVE);
	    break;
	case(SMALL_BOX_OBS):
	    actions.push_back(MOVE);
	    actions.push_back(TURN_CW);
	    actions.push_back(TURN_CCW);
	    //actions.push_back(STAY);
	    break;
	case(LARGE_BOX_OBS):
	    actions.push_back(MOVE);
	    actions.push_back(STAY);
	    actions.push_back(TURN_CW);
	    actions.push_back(TURN_CCW);
	case(LARGE_BOX_AGENT_OBS):
	    actions.push_back(MOVE);
	    if (!humanDefined)
	    {
		actions.push_back(STAY);
		actions.push_back(TURN_CW);
		actions.push_back(TURN_CCW);
	    }
	    break;
	default:
	    break;
    }
}

void BOXPUSHING::GeneratePreferredAgent(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status, const int& index) const
{
    if (history.Size() == 0)
    {
	actions.push_back(MOVE);
	    //actions.push_back(STAY);
	    //actions.push_back(TURN_CW);
	    //actions.push_back(TURN_CCW);
        return;
    }
    const BOXPUSHING_STATE& bpstate = safe_cast<const BOXPUSHING_STATE&>(state);
    int lastObs = history.Back().Observation;
    GeneratePreferredAgentAction(bpstate, lastObs, actions, index-1, status.HumanDefined);
}



void BOXPUSHING::MarkCell(BOXPUSHING_STATE& bpstate, const COORD& coord, const CellContent& content) const
{
    BOXPUSHING_STATE::CELL& cell = bpstate.Cells(coord);
    assert(!cell.Occupied);
    cell.Occupied = true;
    cell.Content = content;
}

void BOXPUSHING::UnmarkCell(BOXPUSHING_STATE& bpstate, const COORD& coord) const
{
    BOXPUSHING_STATE::CELL& cell = bpstate.Cells(coord);
    assert(cell.Occupied);
    cell.Occupied = false;
    cell.Content = NONE;
}

void BOXPUSHING::MarkPushEntity(BOXPUSHING_STATE& bpstate, 
    const PUSH_ENTITY& pushentity, const CellContent& content) const
{
    COORD pos = pushentity.Position;
    for (int i = 0; i < pushentity.Length; ++i)
    {
        MarkCell(bpstate, pos, content);
        pos += COORD::Compass[pushentity.Direction];
    }
}

void BOXPUSHING::UnmarkPushEntity(BOXPUSHING_STATE& bpstate, 
    const PUSH_ENTITY& pushentity) const
{
    COORD pos = pushentity.Position;
    for (int i = 0; i < pushentity.Length; ++i)
    {
        UnmarkCell(bpstate, pos);
        pos += COORD::Compass[pushentity.Direction];
    }
}

void BOXPUSHING::DisplayBeliefs(const BELIEF_STATE& beliefState, 
    std::ostream& ostr) const
{
}

void BOXPUSHING::DisplayState(const STATE& state, ostream& ostr) const
{
    const BOXPUSHING_STATE& bpstate = safe_cast<const BOXPUSHING_STATE&>(state);
    ostr << endl << "  ";
    for (int x = 0; x < XSize; x++)
        ostr << setw(1) << ' ' << ' ';
    ostr << "  " << endl;
    for (int y = YSize - 1; y >= 0; y--)
    {
        ostr << setw(1) << ' ' << ' ';
        for (int x = 0; x < XSize; x++)
        {
            const BOXPUSHING_STATE::CELL& cell = bpstate.Cells(x, y);
            char c = '.';
            if (cell.Content == AGENT)
	    {
		int direction;
		if (bpstate.Agents[0].Position.X == x && bpstate.Agents[0].Position.Y == y)
		    direction = bpstate.Agents[0].Direction;
		else
		    direction = bpstate.Agents[1].Direction;
		switch(direction)
		{
		    case(0):
			c = '^';
			break;
		    case(1):
			c = '>';
			break;
		    case(2):
			c = 'v';
			break;
		    default:
			c = '<';
			break;
		}
	    }
            else if (cell.Content == SMALL_BOX)
                c = 'S';
            else if (cell.Content == LARGE_BOX)
                c = 'L';
            ostr << c << ' ';
        }
        ostr << setw(1) << ' ' << endl;
    }
    ostr << "  ";
    for (int x = 0; x < XSize; x++)
        ostr << setw(1) << ' ' << ' ';
    ostr << "  " << endl;
    ostr << "NumRemaining = " << bpstate.NumBoxesRemaining << endl;
}


void BOXPUSHING::DisplayObservation(const STATE& state, int observation, ostream& ostr) const
{
    string observationNames[6] = {"EMPTY_OBS", "WALL_OBS", "AGENT_OBS", "SMALL_BOX_OBS", "LARGE_BOX_OBS", "LARGE_BOX_AGENT_OBS"};
    string messageNames[3] = {"NO_MES", "SMALL_BOX_MES", "LARGE_BOX_MES"};
    
    int observation0 = observation%NumAgentObservations;
    int observation1 = observation/NumAgentObservations;
    
    ostr << "OBSERVATIONS: " << observationNames[GetObservationComponentFromObservation(observation0)] << ", " << 
	observationNames[GetObservationComponentFromObservation(observation1)] << "\n";
    ostr << "MESSAGES RECEIVED: " << messageNames[GetMessageComponentFromObservation(observation0)] << ", " << 
	messageNames[GetMessageComponentFromObservation(observation1)] << "\n";
}

void BOXPUSHING::DisplayAction(int action, std::ostream& ostr) const
{
    string actionNames[4] = {"STAY", "TURN_CW", "TURN_CCW", "MOVE"};
    string messageNames[3] = {"NO_MES", "SMALL_BOX_MES", "LARGE_BOX_MES"};
    
    int action0 = action%NumAgentActions;
    int action1 = action/NumAgentActions;
    
    ostr << "ACTIONS: " << actionNames[GetActionComponentFromAction(action0)] << ", " 
	    << actionNames[GetActionComponentFromAction(action1)] << "\n";
    ostr << "MESSAGES SENT: " << messageNames[GetMessageComponentFromAction(action0)] << ", " 
	    << messageNames[GetMessageComponentFromAction(action1)] << "\n";
}

void BOXPUSHING::DisplayMessage(int message, ostream& ostr) const
{
    string messageNames[3] = {"NO_MES", "SMALL_BOX_MES", "LARGE_BOX_MES"};
    
    int message0 = message%NumAgentMessages;
    int message1 = message/NumAgentMessages;
    
    ostr << messageNames[message0] << ", " << messageNames[message1] << "\n";
}

string BOXPUSHING::MessageToString(const int& message) const
{
    assert(message >= 0 && message < NumAgentMessages);
    string messageNames[3] = {"NO_MES", "SMALL_BOX_MES", "LARGE_BOX_MES"};
    
    return messageNames[message];
}

int BOXPUSHING::MessageToInt(const string& message) const
{
    if (message == "NO_MES")
	return 0;
    else if (message == "SMALL_BOX_MES")
	return 1;
    else if (message == "LARGE_BOX_MES")
	return 2;
    else
	return 0;
}



