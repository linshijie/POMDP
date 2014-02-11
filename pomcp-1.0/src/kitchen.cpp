#include "kitchen.h"
#include <iomanip>
#include <tr1/unordered_set>

boost::math::beta_distribution<> betak(0.9,0.9);

KITCHEN::KITCHEN(bool testTrayOnStove, bool testCerealInCupboard): 
  NumPlates(0), NumCups(0), NumLocations(5),
  LOCATION_OFFSET(static_cast<int>(CUPBOARD)), 
  GRIPPER_OFFSET(static_cast<int>(LEFT)), ACTION_OFFSET(static_cast<int>(CLOSE)),
  HaveAppleJuice(false), HaveCalgonit(false), HaveGranini(false),
  HaveMeasuringCup(false), HaveRiceBox(false), HaveCereal(false),
  HaveTray(false),
  TestCerealInCupboard(testCerealInCupboard), TestPlate1InDishwasher(false), TestAppleJuiceInFridge(false),
  TestCerealAndAppleJuice(false), TestTrayAndPlate(false),
  TestTrayOnStove(testTrayOnStove),
  NonDeterministicActions(true),
  ProbClose(0.95), ProbGrasp(0.95), ProbGrapsFromEdge(0.95), ProbMove(0.95), ProbNudge(0.95), ProbOpen(0.95),
  ProbOpenPartial(0.95), ProbOpenComplete(0.95), ProbPassObject(0.95), 
  ProbPlaceUpright(0.95), ProbPutDown(0.95), ProbPutIn(0.95), ProbRemoveFrom(0.95),
  ProbGraspJoint(0.9), ProbPutDownJoint(0.9), ProbMoveJoint(0.9), 
  ProbToppleOnFailPutDown(0.9),
  StartLocationsFixed(true),
  ProbObservation(0.95)
{
    NumObjects = NumPlates+NumCups;
    
    if (TestTrayOnStove || TestCerealAndAppleJuice || TestTrayAndPlate || (TestAppleJuiceInFridge && TestCerealInCupboard))
	NumAgents = 2;
    else
	NumAgents = 1;
    
    if (TestTrayOnStove || TestTrayAndPlate)
	HaveTray = true;
    
    if (TestCerealInCupboard || TestCerealAndAppleJuice)
	HaveCereal = true;
    
    if (TestAppleJuiceInFridge || TestCerealAndAppleJuice)
	HaveAppleJuice = true;
    
    ObjectTypes.clear();
    int index = 0;
    if (HaveAppleJuice)
    {
	ObjectTypes.push_back(APPLEJUICE);
	NumObjects++;
	AppleJuiceIndex = index;
	index++;
    }
    if (HaveCalgonit)
    {
	ObjectTypes.push_back(CALGONIT);
	NumObjects++;
	index++;
    }
    if (HaveGranini)
    {
	ObjectTypes.push_back(GRANINI);
	NumObjects++;
	index++;
    }
    if (HaveMeasuringCup)
    {
	ObjectTypes.push_back(MEASURINGCUP);
	NumObjects++;
	index++;
    }
    if (HaveRiceBox)
    {
	ObjectTypes.push_back(RICEBOX);
	NumObjects++;
	index++;
    }
    if (HaveCereal)
    {
	ObjectTypes.push_back(CEREAL);
	NumObjects++;
	CerealIndex = index;
	index++;
    }
    if (HaveTray)
    {
	ObjectTypes.push_back(TRAY);
	NumObjects++;
	TrayIndex = index;
	index++;
    }
    
    NumAgentActions = NumObjects*(NumLocations*18+4) + 8*NumLocations + NumLocations*NumLocations + 
		NumLocations*NumLocations + 1;
    NumAgentObservations = pow(2,NumObjects)*pow(2,NumAgents);
	    //*pow((NumObjects+1)*(NumObjects+1), NumAgents);
	//*NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects)*(NumObjects+1)*(NumObjects+1);
    
    NumActions = pow(NumAgentActions, NumAgents);
    NumObservations = pow(NumAgentObservations, NumAgents);
    
    for (int i=0; i<NumPlates; i++)
    {
	ObjectTypes.push_back(PLATE);
	if (i == 0)
	    Plate1Index = index;
	index++;
    }
    for (int i=0; i<NumCups; i++)
    {
	ObjectTypes.push_back(CUP);
	index++;
    }
    
    MultiAgentLabels.clear();
    for (int i=0; i<NumAgentActions; i++)
    {
	KitchenAction ka = IntToAction(i);
	if (ka.type == GRASP_JOINT || ka.type == PUT_DOWN_JOINT || ka.type == MOVE_ROBOT_JOINT)
	    MultiAgentLabels.push_back(true);
	else
	    MultiAgentLabels.push_back(false);
    }
	
    RewardRange = 99.9;
    MinReward = -0.1;
    MaxReward = 100.0;
    Discount = 1.0;
    
    for (int i = 0; i < 100; i++)
	quantiles.push_back(boost::math::quantile(betak, UTILS::RandomDouble(0.0, 1.0)));
}

STATE* KITCHEN::Copy(const STATE& state) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    KITCHEN_STATE* newstate = MemoryPool.Allocate();
    *newstate = kitchenstate;
    return newstate; 
}

void KITCHEN::Validate(const STATE& state) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    assert(IsLocation(kitchenstate.RobotLocation));
    for (int i = 0; i<NumObjects; i++)
	assert(IsLocation(kitchenstate.ObjectLocations[i]));
}

void KITCHEN::FreeState(STATE* state) const
{
    KITCHEN_STATE* kitchenstate = safe_cast<KITCHEN_STATE*>(state);
    MemoryPool.Free(kitchenstate);
}

STATE* KITCHEN::CreateStartState() const
{
    KITCHEN_STATE* kitchenstate = MemoryPool.Allocate();

    kitchenstate->RobotLocations.clear(); 
    kitchenstate->InWhichGripper.clear();
    kitchenstate->AtEdge.clear();
    kitchenstate->IsToppled.clear();
    kitchenstate->ObjectLocations.clear();
    kitchenstate->LocationOpen.clear();
    kitchenstate->LocationPartiallyOpen.clear();
    kitchenstate->GripperEmpty.clear();
    
    kitchenstate->TraySecondGripper = std::make_pair(-1, NO_GRIPPER);
    
    //first robot location
    if (StartLocationsFixed)
	kitchenstate->RobotLocations.push_back(SIDEBOARD);
    else
	kitchenstate->RobotLocations.push_back(static_cast<LocationType>(UTILS::Random(0,NumLocations)+LOCATION_OFFSET));
    
    if (StartLocationsFixed)
    {
	for (int i = 1; i < NumAgents; i++)
	    kitchenstate->RobotLocations.push_back(SIDEBOARD);
    }
    else
    {
	for (int i = 1; i < NumAgents; i++)
	{
	    LocationType agentlocation;
	    do
	    {
		agentlocation = static_cast<LocationType>(UTILS::Random(0,NumLocations)+LOCATION_OFFSET);
	    }while(Collision(*kitchenstate,agentlocation,i));
	    kitchenstate->RobotLocations.push_back(agentlocation);
	}
    }
    
    for (int i = 0; i < NumObjects; i++)
    {
	if (ObjectTypes[i] == CEREAL)
	{
	    kitchenstate->ObjectLocations.push_back(SIDEBOARD);
	    kitchenstate->IsToppled.push_back(false);
	}
	else if (ObjectTypes[i] == APPLEJUICE)
	{
	    kitchenstate->ObjectLocations.push_back(SIDEBOARD);
	    kitchenstate->IsToppled.push_back(true);
	}
	else if (ObjectTypes[i] == PLATE)
	{
	    kitchenstate->ObjectLocations.push_back(SIDEBOARD);
	    kitchenstate->IsToppled.push_back(false);
	}
	else if (ObjectTypes[i] == CUP)
	{
	    kitchenstate->ObjectLocations.push_back(static_cast<LocationType>(UTILS::Random(NumLocations)));
	    kitchenstate->IsToppled.push_back(false);
	}
	else if (ObjectTypes[i] == TRAY)
	{
	    kitchenstate->ObjectLocations.push_back(SIDEBOARD);
	    kitchenstate->IsToppled.push_back(false);
	}
	kitchenstate->AtEdge.push_back(false);
	kitchenstate->InWhichGripper.push_back(std::make_pair(-1,NO_GRIPPER));
    }
    
    for (int i = 0; i < NumLocations; i++)
    {
	kitchenstate->LocationOpen.push_back(false);
	kitchenstate->LocationPartiallyOpen.push_back(false);
    }
    for (int i = 0; i < NumAgents; i++)
    {
	//left first, then right
	kitchenstate->GripperEmpty.push_back(true);
	kitchenstate->GripperEmpty.push_back(true);
    }
    
    return kitchenstate;
}

bool KITCHEN::IsActionMultiagent(const int& action, const HISTORY& history) const
{
    if (action >= (int)MultiAgentLabels.size() || action < 0)
	return false;
    return MultiAgentLabels[action];
}


bool KITCHEN::Step(STATE& state, int action, int& observation, double& reward, STATUS& status) const
{
    KITCHEN_STATE& kitchenstate = safe_cast<KITCHEN_STATE&>(state);
    
    std::vector< std::pair< int, int > > agentactions;
    
    double additionalRew = 0.0;
    
    reward = -0.1;
    
    if (NumAgents == 1)
    {
	agentactions.push_back(std::make_pair(0,action));
	StepAgent(kitchenstate, action, observation, reward, 0);
    }
    else
    {
	for (int i = NumAgents-1 ; i >= 0 ; i--)
	{
	    agentactions.push_back(std::make_pair(i,action/((int) pow(NumAgentActions, i))));
	    action = action%((int) pow(NumAgentActions, i)); 
	}
	std::random_shuffle(agentactions.begin(), agentactions.end());
	
	//Check for joint actions
	std::vector< std::pair<int, int> > jointinds;
	std::vector<KitchenAction> kitchenactions;
	for (int i = 0; i < (int) agentactions.size(); i++)
	{
	    KitchenAction ka = IntToAction(agentactions[i].second);
	    kitchenactions.push_back(ka);
	    if (ka.type == GRASP_JOINT || ka.type == PUT_DOWN_JOINT || 
		ka.type == MOVE_ROBOT_JOINT)
	    {
		if (jointinds.empty())
		    jointinds.push_back(std::make_pair(i,-1));
		else
		{
		    bool assigned = false;
		    for (int j = 0; j < jointinds.size() ; j++)
			if (!assigned && jointinds[j].second == -1)
			{
			    KitchenAction otherka = kitchenactions[jointinds[j].first];
			    if (ka.type == otherka.type && ka.location == otherka.location &&
				(((ka.type == GRASP_JOINT || ka.type == PUT_DOWN_JOINT) && 
				    ka.objectindex == otherka.objectindex) ||
				    ka.type == MOVE_ROBOT_JOINT)
				)
			    {
				jointinds[j].second = i;
				assigned = true;
				break;
			    }
			}
		    if (!assigned)
			jointinds.push_back(std::make_pair(i,-1));
		}
	    }
	}
	
	bool executedjointactions[NumAgents];
	for (int i = 0 ; i < NumAgents ; i++)
	    executedjointactions[i] = false;
	
	for (int i = 0 ; i < (int) jointinds.size() ; i++)
	{
	    int firstaction = jointinds[i].first, secondaction = jointinds[i].second;

	    if (firstaction != -1 && secondaction != -1)
	    {
		int firstagent = agentactions[firstaction].first, secondagent = agentactions[secondaction].first;
		executedjointactions[firstagent] = true;
		executedjointactions[secondagent] = true;
		KitchenAction firstka = kitchenactions[firstaction], secondka = kitchenactions[secondaction];
		if (firstka.type == GRASP_JOINT)
		{
		    if (ObjectTypes[firstka.objectindex] == TRAY && firstka.objectindex == secondka.objectindex &&
			firstka.location == secondka.location &&
			kitchenstate.RobotLocations[firstagent] == firstka.location &&
			kitchenstate.RobotLocations[secondagent] == secondka.location &&
			kitchenstate.ObjectLocations[firstka.objectindex] == firstka.location && 
			kitchenstate.GripperEmpty[firstagent*2+firstka.gripper-GRIPPER_OFFSET] &&
			kitchenstate.GripperEmpty[secondagent*2+secondka.gripper-GRIPPER_OFFSET] &&
			!kitchenstate.IsToppled[firstka.objectindex]
		    )
		    {
			if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbGraspJoint)
			{
			    kitchenstate.InWhichGripper[firstka.objectindex].first = firstagent;
			    kitchenstate.InWhichGripper[firstka.objectindex].second = firstka.gripper;
			    kitchenstate.TraySecondGripper = std::make_pair(secondagent, secondka.gripper);
			    kitchenstate.GripperEmpty[firstagent*2+firstka.gripper-GRIPPER_OFFSET] = false;
			    kitchenstate.GripperEmpty[secondagent*2+secondka.gripper-GRIPPER_OFFSET] = false;
			    kitchenstate.ObjectLocations[firstka.objectindex] = NO_LOCATION;
			}
		    }
		    else
			reward = -10.0;
		}
		else if (firstka.type == PUT_DOWN_JOINT)
		{
		    if (ObjectTypes[firstka.objectindex] == TRAY && firstka.objectindex == secondka.objectindex &&
			firstka.location == secondka.location &&
			kitchenstate.RobotLocations[firstagent] == firstka.location && 
			kitchenstate.RobotLocations[secondagent] == secondka.location && 
			((kitchenstate.InWhichGripper[firstka.objectindex].first == firstagent &&
			  kitchenstate.InWhichGripper[firstka.objectindex].second == firstka.gripper && 
			  kitchenstate.TraySecondGripper.first == secondagent &&
			  kitchenstate.TraySecondGripper.second == secondka.gripper) ||
			 (kitchenstate.InWhichGripper[firstka.objectindex].first == secondagent &&
			  kitchenstate.InWhichGripper[firstka.objectindex].second == secondka.gripper && 
			  kitchenstate.TraySecondGripper.first == firstagent &&
			  kitchenstate.TraySecondGripper.second == firstka.gripper)    
			)
		    )
		    {
			if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPutDownJoint)
			{
			    kitchenstate.GripperEmpty[firstagent*2+firstka.gripper-GRIPPER_OFFSET] = true;
			    kitchenstate.GripperEmpty[secondagent*2+secondka.gripper-GRIPPER_OFFSET] = true;
			    kitchenstate.ObjectLocations[firstka.objectindex] = firstka.location;
			    kitchenstate.InWhichGripper[firstka.objectindex].first = -1;
			    kitchenstate.InWhichGripper[firstka.objectindex].second = NO_GRIPPER;
			    kitchenstate.TraySecondGripper.first = -1;
			    kitchenstate.TraySecondGripper.second = NO_GRIPPER;
			}
		    }
		    else
			reward = -10.0;
		}
		else if (firstka.type == MOVE_ROBOT_JOINT)
		{
		    if (firstka.location == secondka.location &&
			kitchenstate.RobotLocations[firstagent] == firstka.location && 
			firstka.location2 == secondka.location2 &&
			firstka.location != firstka.location2 && 
			((kitchenstate.InWhichGripper[TrayIndex].first == firstagent &&
			  kitchenstate.TraySecondGripper.first == secondagent) ||
			 (kitchenstate.InWhichGripper[TrayIndex].first == secondagent &&
			  kitchenstate.TraySecondGripper.first == firstagent)) 
		    )
		    {
			if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbMoveJoint)
			{
			    kitchenstate.RobotLocations[firstagent] = firstka.location2;
			    kitchenstate.RobotLocations[secondagent] = secondka.location2;
			}
		    }
		    else
			reward = -10.0;
		}
	    }
	    /*else if (firstaction != -1 && kitchenactions[firstaction].type == MOVE_ROBOT)
	    {
		//only one robot is trying to move while holding the tray
		executedjointactions[agentactions[firstaction].first] = true;
		reward = -10.0;
	    }*/
	}
	
	//No joint actions, proceed independently
	for (int i = 0; i < (int) agentactions.size(); i++)
	    if (!executedjointactions[agentactions.at(i).first])
	    {
		double tempreward = 0.0;
		StepAgent(kitchenstate, agentactions.at(i).second, observation, tempreward, agentactions.at(i).first);
		reward += tempreward;
	    }
	    
	if (executedjointactions[0] && executedjointactions[1])
	    additionalRew = 100.0;
    }
    
    observation = 0;
    for (int i = 0; i < NumAgents; i++)
	observation += pow(NumAgentObservations,i)*MakeObservation(kitchenstate, i);
    
    //Just executed an unsuccessful action
    //if (reward < 0.0)
	//return false;
    
    bool reachedGoal = false;
    
    if (TestCerealInCupboard)
	reachedGoal = reachedGoal || IsCerealInCupboard(kitchenstate, reward);
    if (TestPlate1InDishwasher)
	reachedGoal = reachedGoal || IsPlate1InDishwasher(kitchenstate, reward);
    if (TestAppleJuiceInFridge)
	reachedGoal = reachedGoal || IsAppleJuiceInFridge(kitchenstate, reward);
    if (TestCerealAndAppleJuice)
	reachedGoal = reachedGoal || IsCerealAndAppleJuice(kitchenstate, reward);
    if (TestTrayOnStove)
	reachedGoal = reachedGoal || IsTrayOnStove(kitchenstate, reward);
    
    /*if (reachedGoal)
	reward = 99.9;
    else*/
    if (!reachedGoal)
	reward = -0.1;
    
    //other agent reward
    //if (status.RewardAdaptive)
	//status.CurrOtherReward = reward + UTILS::RandomDouble(-1.0,1.0);//UTILS::Normal(reward, 1.0);
    if (status.RewardAdaptive && status.LearningPhase && !status.HumanDefined)
	status.CurrOtherReward = quantiles[UTILS::Random(quantiles.size())]*reward*2;
    else if (status.HumanDefined)
	status.CurrOtherReward = reward;
    else
	status.CurrOtherReward = 0;
    
    //if (reachedGoal)
	//std::cout << "terminal" << status.perspindex << "\n";
    
    return reachedGoal;
}

bool KITCHEN::StepAgent(KITCHEN_STATE& kitchenstate, int action, 
			int& observation, double& reward, const int& index) const
{
    KitchenAction ka = IntToAction(action);
    
    int otherAgentIndex = -1, otherAgentGripper = -1;
    int agentIndex = -1, gripperIndex = -1;
    
    reward = 0.1;
    
    switch(ka.type)
    {
	case(CLOSE):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] &&
		(kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] || 
		kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET])
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbClose)
		{
		    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = false;
		    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = false;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(GRASP):
	    if (kitchenstate.RobotLocations[index] == ka.location && !kitchenstate.IsToppled[ka.objectindex] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location && 
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbGrasp)
		{
		    kitchenstate.InWhichGripper[ka.objectindex].first = index;
		    kitchenstate.InWhichGripper[ka.objectindex].second = ka.gripper;
		    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = false;
		    kitchenstate.ObjectLocations[ka.objectindex] = NO_LOCATION;
		}
		else
		{
		    if (ObjectTypes[ka.objectindex] != PLATE)
			kitchenstate.IsToppled[ka.objectindex] = true;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(GRASP_FROM_EDGE):
	    if (kitchenstate.RobotLocations[index] == ka.location && kitchenstate.AtEdge[ka.objectindex] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbGrapsFromEdge)
		{
		    kitchenstate.InWhichGripper[ka.objectindex].first = index;
		    kitchenstate.InWhichGripper[ka.objectindex].second = ka.gripper;
		    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = false;
		    kitchenstate.AtEdge[ka.objectindex] = false;
		    kitchenstate.ObjectLocations[ka.objectindex] = NO_LOCATION;
		}
		else
		{
		    if (ObjectTypes[ka.objectindex] != PLATE)
			kitchenstate.IsToppled[ka.objectindex] = true;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(MOVE_ROBOT):
	    //in a multi-agent setting, if a robot 
	    if (NumAgents > 1 && HaveTray)
		for (int h = 0 ; h < 2 ; h++)
		    if ((kitchenstate.InWhichGripper.at(TrayIndex).first  == index && 
			kitchenstate.InWhichGripper.at(TrayIndex).second == h+GRIPPER_OFFSET) || 
			(kitchenstate.TraySecondGripper.first  == index && 
			kitchenstate.TraySecondGripper.second == h+GRIPPER_OFFSET))
		    {
			kitchenstate.ObjectLocations[TrayIndex] = ka.location;
			if (kitchenstate.InWhichGripper[TrayIndex].first != -1)
			{
			    agentIndex = kitchenstate.InWhichGripper[TrayIndex].first;
			    gripperIndex = kitchenstate.InWhichGripper[TrayIndex].second;
			    kitchenstate.GripperEmpty[agentIndex*2+gripperIndex-GRIPPER_OFFSET] = true;
			    kitchenstate.InWhichGripper[TrayIndex].first = -1;
			    kitchenstate.InWhichGripper[TrayIndex].second = NO_GRIPPER;
			}
			if (kitchenstate.TraySecondGripper.first != -1)
			{
			    agentIndex = kitchenstate.TraySecondGripper.first;
			    gripperIndex = kitchenstate.TraySecondGripper.second;
			    kitchenstate.GripperEmpty[agentIndex*2+gripperIndex-GRIPPER_OFFSET] = true;
			    kitchenstate.TraySecondGripper.first = -1;
			    kitchenstate.TraySecondGripper.second = NO_GRIPPER;
			}
			
			if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbMove)
			    kitchenstate.RobotLocations[index] = ka.location2;
			
			if (UTILS::RandomDouble(0.0,1.0) < ProbToppleOnFailMove)
			    kitchenstate.IsToppled[TrayIndex] = true;
			
			reward = -5.0;
		    }
	    
	    if (reward > 0.0 && 
		kitchenstate.RobotLocations[index] == ka.location && kitchenstate.RobotLocations[index] != ka.location2
	    )
	    {
		int countsideboard = 0, countstove = 0;
		for (int i = 0; i < NumAgents; i++)
		    if (i != index && kitchenstate.RobotLocations[i] == ka.location2 && reward > 0.0) 
		    {
			if (kitchenstate.RobotLocations[i] == CUPBOARD || kitchenstate.RobotLocations[i] == FRIDGE ||
			    kitchenstate.RobotLocations[i] == DISHWASHER)
			{
			    reward = -5.0;
			    break;
			}
			else if (kitchenstate.RobotLocations[i] == SIDEBOARD)
			{
			    countsideboard++;
			    if (countsideboard > 1)
			    {
				reward = -5.0;
				break;
			    }
			}
			else if (kitchenstate.RobotLocations[i] == STOVE)
			{
			    countstove++;
			    if (countstove > 1)
			    {
				reward = -5.0;
				break;
			    }
			}
		    }
		if (reward > 0.0 && (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbMove))
		    kitchenstate.RobotLocations[index] = ka.location2;
	    }
	    else
		reward = -5.0;
	    break;
	case(NUDGE):
	    if (kitchenstate.RobotLocations[index] == ka.location && !kitchenstate.AtEdge[ka.objectindex] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbNudge)
		    kitchenstate.AtEdge[ka.objectindex] = true;
		else
		{
		    if (ObjectTypes[ka.objectindex] != PLATE)
			kitchenstate.IsToppled[ka.objectindex] = true;
		}
	    }
	    else
		reward = -5.0;    
	    break;
	case(OPEN):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		!kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbOpen)
		    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = true;
	    }
	    else
		reward = -5.0;
	    break;
	case(OPEN_PARTIAL):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		!kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		!kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] && 
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbOpenPartial)
		    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = true;
	    }
	    else
		reward = -5.0;
	    break;
	case(OPEN_COMPLETE):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		!kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] && 
		kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbOpenComplete)
		{
		    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = true;
		    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = false;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(PASS_OBJECT):
	    if (kitchenstate.InWhichGripper[ka.objectindex].first == index &&
		kitchenstate.InWhichGripper[ka.objectindex].second == ka.gripper &&
		kitchenstate.GripperEmpty[index*2+ka.gripper2-GRIPPER_OFFSET] &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPassObject)
		{
		    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = true;
		    kitchenstate.GripperEmpty[index*2+ka.gripper2-GRIPPER_OFFSET] = false;
		    kitchenstate.InWhichGripper[ka.objectindex].first = index;
		    kitchenstate.InWhichGripper[ka.objectindex].second = ka.gripper2;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(PLACE_UPRIGHT):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		kitchenstate.IsToppled[ka.objectindex] && kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPlaceUpright)
		    kitchenstate.IsToppled[ka.objectindex] = false;
	    }
	    else
		reward = -5.0;
	    break;
	case(PUT_DOWN):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		kitchenstate.InWhichGripper[ka.objectindex].first == index &&
		kitchenstate.InWhichGripper[ka.objectindex].second == ka.gripper &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPutDown)
		{
		    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = true;
		    kitchenstate.ObjectLocations[ka.objectindex] = ka.location;
		    kitchenstate.InWhichGripper[ka.objectindex].first = -1;
		    kitchenstate.InWhichGripper[ka.objectindex].second = NO_GRIPPER;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(PUT_IN):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.InWhichGripper[ka.objectindex].first == index &&
		kitchenstate.InWhichGripper[ka.objectindex].second == ka.gripper &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPutIn)
		{
		    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = true;
		    kitchenstate.ObjectLocations[ka.objectindex] = ka.location;
		    kitchenstate.InWhichGripper[ka.objectindex].first = -1;
		    kitchenstate.InWhichGripper[ka.objectindex].second = NO_GRIPPER;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(REMOVE_FROM):
	    if (kitchenstate.RobotLocations[index] == ka.location && 
		kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		!kitchenstate.IsToppled[ka.objectindex] && kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] &&
		ObjectTypes[ka.objectindex] != TRAY
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbRemoveFrom)
		{
		    kitchenstate.InWhichGripper[ka.objectindex].first = index;
		    kitchenstate.InWhichGripper[ka.objectindex].second = ka.gripper;
		    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = false;
		    kitchenstate.ObjectLocations[ka.objectindex] = NO_LOCATION;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(STAY_PUT):
	    reward = -0.1;
	    break;
	//for joint actions, if we've reached this point, it means that the preconditions were not met
	//i.e. only one agent was trying to execute the action
	case(GRASP_JOINT):
	    reward = -5.0;
	    break;
	case(PUT_DOWN_JOINT):
	    kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] = true;
	    kitchenstate.ObjectLocations[ka.objectindex] = SIDEBOARD;
	    otherAgentIndex = -1;
	    otherAgentGripper = -1;
	    if (kitchenstate.InWhichGripper[ka.objectindex].first != -1 &&
		    kitchenstate.InWhichGripper[ka.objectindex].first != index)
	    {
		otherAgentIndex = kitchenstate.InWhichGripper[ka.objectindex].first;
		otherAgentGripper = kitchenstate.InWhichGripper[ka.objectindex].second;
	    }
	    else if (ObjectTypes[ka.objectindex] == TRAY && 
		kitchenstate.TraySecondGripper.first != -1 &&
		kitchenstate.TraySecondGripper.first != index)
	    {
		otherAgentIndex = kitchenstate.TraySecondGripper.first;
		otherAgentGripper = kitchenstate.TraySecondGripper.second;
	    }
	    if (otherAgentIndex > -1 && otherAgentGripper > -1)
		kitchenstate.GripperEmpty[otherAgentIndex*2+otherAgentGripper-GRIPPER_OFFSET] = true;
	    
	    kitchenstate.InWhichGripper[ka.objectindex].first = -1;
	    kitchenstate.InWhichGripper[ka.objectindex].second = NO_GRIPPER;
	    if (ObjectTypes[ka.objectindex] == TRAY)
	    {
		kitchenstate.TraySecondGripper.first = -1;
		kitchenstate.TraySecondGripper.second = NO_GRIPPER;
	    }
	    
	    if (UTILS::RandomDouble(0.0,1.0) < ProbToppleOnFailPutDown)
		kitchenstate.IsToppled[ka.objectindex] = true;
	    
	    reward = -5.0;
	    break;
	case(MOVE_ROBOT_JOINT):
	    kitchenstate.ObjectLocations[TrayIndex] = SIDEBOARD;
	    if (kitchenstate.InWhichGripper[TrayIndex].first != -1)
	    {
		agentIndex = kitchenstate.InWhichGripper[TrayIndex].first;
		gripperIndex = kitchenstate.InWhichGripper[TrayIndex].second;
		kitchenstate.GripperEmpty[agentIndex*2+gripperIndex-GRIPPER_OFFSET] = true;
		kitchenstate.InWhichGripper[TrayIndex].first = -1;
		kitchenstate.InWhichGripper[TrayIndex].second = NO_GRIPPER;
	    }
	    if (kitchenstate.TraySecondGripper.first != -1)
	    {
		agentIndex = kitchenstate.TraySecondGripper.first;
		gripperIndex = kitchenstate.TraySecondGripper.second;
		kitchenstate.GripperEmpty[agentIndex*2+gripperIndex-GRIPPER_OFFSET] = true;
		kitchenstate.TraySecondGripper.first = -1;
		kitchenstate.TraySecondGripper.second = NO_GRIPPER;
	    }
	    
	    if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbMove)
		kitchenstate.RobotLocations[index] = ka.location2;
	    
	    if (UTILS::RandomDouble(0.0,1.0) < ProbToppleOnFailMove)
		kitchenstate.IsToppled[TrayIndex] = true;
	    
	    reward = -5.0;
	    break;
	default:
	    break;
    }
}

bool KITCHEN::LocalMove(STATE& state, const HISTORY& history, int stepObs, const SIMULATOR::STATUS& status) const
{
    /*KITCHEN_STATE& kitchenstate = safe_cast<KITCHEN_STATE&>(state);
    
    std::tr1::unordered_set<int> UnseenObjects;
    std::tr1::unordered_set<int> UnexploredLocations;
    
    for (int i = 0; i < NumObjects; i++)
	UnseenObjects.insert(i);
    for (int i = 0; i < NumLocations; i++)
	UnexploredLocations.insert(i+LOCATION_OFFSET);
    
    int index = status.perspindex;
    
    for (int i = 0; i < history.Size(); i++)
    {
	KitchenObservation ko = IntToObservation(history[i].Observation);
	for (int j = 0; j < NumObjects; j++)
	    if (ko.ObjectVisible[j])
		UnseenObjects.erase(j);
	     
	if (kitchenstate.RobotLocations[index] == SIDEBOARD || kitchenstate.RobotLocations[index] == STOVE)
	    UnexploredLocations.erase(static_cast<int>(kitchenstate.RobotLocations[index]));
	else
	{
	    bool removefromunseen = false;
	    for (int j = i+1; j < history.Size(); j++)
	    {
		KitchenAction ka = IntToAction(history[i].Action);
		if (ka.type == MOVE_ROBOT)
		    break;
		else if (ka.type == OPEN || ka.type == OPEN_PARTIAL || ka.type == OPEN_COMPLETE)
		{
		    removefromunseen = true;
		    break;
		}
	    }
	    if (kitchenstate.RobotLocations[index] != NO_LOCATION && removefromunseen)
		UnexploredLocations.erase(static_cast<int>(kitchenstate.RobotLocations[index]));
	}
	
	if (UnexploredLocations.empty() || UnseenObjects.empty())
	    return false;
    }
    
    for (std::tr1::unordered_set<int>::iterator it = UnseenObjects.begin(); it != UnseenObjects.end(); ++it)
    {
	std::tr1::unordered_set<int>::iterator it2 = UnexploredLocations.begin();
	for (int i = 0; i < UTILS::Random(0,(int)UnexploredLocations.size()); i++)
	    ++it2;
	kitchenstate.ObjectLocations[*it] = static_cast<LocationType>(*it2);
    }*/
    
    return true;
}

int KITCHEN::MakeObservation(const KITCHEN_STATE& state, const int& index) const
{
    KitchenObservation ko;
    ko.ObjectVisible.clear();
    ko.AgentVisible.clear();
    ko.AtEdge.clear();
    ko.IsToppled.clear();
    
    ko.LeftGripperContents.clear();
    ko.RightGripperContents.clear();
    
    for (int i = 0; i < NumAgents; i++)
    {
	ko.LeftGripperContents.push_back(NumObjects);
	ko.RightGripperContents.push_back(NumObjects);
    }
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if ((state.ObjectLocations[i] == state.RobotLocations[index] &&
	    (state.RobotLocations[index] == SIDEBOARD || state.RobotLocations[index] == STOVE ||
	    state.LocationOpen[state.RobotLocations[index]-LOCATION_OFFSET])))
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation)
		ko.ObjectVisible.push_back(true);
	    else
		ko.ObjectVisible.push_back(false);
	}
	else
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation)
		ko.ObjectVisible.push_back(false);
	    else
		ko.ObjectVisible.push_back(true);
	}
	
	ko.AtEdge.push_back(state.AtEdge[i]);
	ko.IsToppled.push_back(state.IsToppled[i]);
	
	if (state.InWhichGripper[i].first == index)
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation)
	    {
		if (state.InWhichGripper[i].second == LEFT)
		    ko.LeftGripperContents[state.InWhichGripper[i].first] = i;
		else if (state.InWhichGripper[i].second == RIGHT)
		    ko.RightGripperContents[state.InWhichGripper[i].first] = i;
	    }
	}
	else if (state.RobotLocations[state.InWhichGripper[i].first] == state.RobotLocations[index])
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation*ProbObservation)
	    {
		if (state.InWhichGripper[i].second == LEFT)
		    ko.LeftGripperContents[state.InWhichGripper[i].first] = i;
		else if (state.InWhichGripper[i].second == RIGHT)
		    ko.RightGripperContents[state.InWhichGripper[i].first] = i;
	    }
	}
	else
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation*ProbObservation*ProbObservation)
	    {
		if (state.InWhichGripper[i].second == LEFT)
		    ko.LeftGripperContents[state.InWhichGripper[i].first] = i;
		else if (state.InWhichGripper[i].second == RIGHT)
		    ko.RightGripperContents[state.InWhichGripper[i].first] = i;
	    }
	}
	    
	    
	if (ObjectTypes[i] == TRAY && state.TraySecondGripper.first == index)
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation)
	    {
		if (state.TraySecondGripper.second == LEFT)
		    ko.LeftGripperContents[state.TraySecondGripper.first] = i;
		else if (state.TraySecondGripper.second == RIGHT)
		    ko.RightGripperContents[state.TraySecondGripper.first] = i;
	    }
	}
	else if (state.RobotLocations[state.TraySecondGripper.first] == state.RobotLocations[index])
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation*ProbObservation)
	    {
		if (state.TraySecondGripper.second == LEFT)
		    ko.LeftGripperContents[state.TraySecondGripper.first] = i;
		else if (state.TraySecondGripper.second == RIGHT)
		    ko.RightGripperContents[state.TraySecondGripper.first] = i;
	    }
	}
	else
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation*ProbObservation*ProbObservation)
	    {
		if (state.TraySecondGripper.second == LEFT)
		    ko.LeftGripperContents[state.TraySecondGripper.first] = i;
		else if (state.TraySecondGripper.second == RIGHT)
		    ko.RightGripperContents[state.TraySecondGripper.first] = i;
	    }
	}
    }
    
    for (int i = 0 ; i < NumAgents ; i++)
    {
	if ((index == i || state.RobotLocations[i] == state.RobotLocations[index]))
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation)
		ko.AgentVisible.push_back(true);
	    else
		ko.AgentVisible.push_back(false);
	}
	else
	{
	    if (UTILS::RandomDouble(0.0,1.0) < ProbObservation)
		ko.AgentVisible.push_back(false);
	    else
		ko.AgentVisible.push_back(true);
	}
    }
    
    ko.OwnLocation = state.RobotLocations[index];
    ko.OwnLocationOpen = state.LocationOpen[state.RobotLocations[index]-LOCATION_OFFSET];
    ko.OwnLocationPartiallyOpen = state.LocationPartiallyOpen[state.RobotLocations[index]-LOCATION_OFFSET];
    
    
    
    
    return ObservationToInt(ko);
}

int KITCHEN::ObservationToInt(const KitchenObservation& ko) const
{
    int observation = 0;
    
    //int atEdgeCount = 0, toppledCount = 0;
    
    int gripperCount = 0;
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (ko.ObjectVisible[i])
	    observation += pow(2,i);
	/*if (ko.IsToppled[i])
	    toppledCount += pow(2,i);
	if (ko.AtEdge[i])
	    atEdgeCount += pow(2,i);*/
    }
    
    int agentcount = 0;
    for (int i = 0 ; i < NumAgents ; i++)
    {
	if (ko.AgentVisible[i])
	    agentcount += pow(2,i);
	//gripperCount += pow((NumObjects+1)*(NumObjects+1),i)* 
		//	(ko.LeftGripperContents[i]*(NumObjects+1) + ko.RightGripperContents[i]);
    }
	
    observation += agentcount*pow(2,NumObjects);
    
    //observation += gripperCount*pow(2,NumObjects)*pow(2,NumAgents);
    
    /*observation += (ko.OwnLocation-LOCATION_OFFSET)*pow(2,NumObjects)*pow(2,NumAgents);
    
    observation += ((int) ko.OwnLocationOpen)*pow(2,NumObjects)*pow(2,NumAgents)*NumLocations;
    observation += ((int) ko.OwnLocationPartiallyOpen)*pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2;
    
    observation += atEdgeCount*pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2*2;
    observation += toppledCount*pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2*2*pow(2,NumObjects);
    
    observation += ko.LeftGripperContent
		*pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects);
		
    observation += ko.RightGripperContent
	    *pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects)*(NumObjects+1);*/
    
    return observation;
}

KitchenObservation KITCHEN::IntToObservation(int observation) const
{
    KitchenObservation ko;
    ko.ObjectVisible.clear();
    ko.AgentVisible.clear();
    ko.AtEdge.clear();
    ko.IsToppled.clear();
    
    ko.LeftGripperContents.clear();
    ko.RightGripperContents.clear();
    
    /*ko.RightGripperContent = observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects)*(NumObjects+1)));
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects)*(NumObjects+1)));
    
    ko.LeftGripperContent = observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects)));
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2*pow(2,NumObjects)*pow(2,NumObjects)));
    
    int toppledCount = observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2*pow(2,NumObjects))); 
    for (int i = 0 ; i < NumAgents ; i++)
    {
	if (toppledCount%2 == 1)
	    ko.IsToppled.push_back(true);
	else
	    ko.IsToppled.push_back(false);
	toppledCount = toppledCount >> 1;
    }
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2*pow(2,NumObjects)));
    
    int atEdgeCount = observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)*
	    NumLocations*2*2));
    for (int i = 0 ; i < NumAgents ; i++)
    {
	if (atEdgeCount%2 == 1)
	    ko.AtEdge.push_back(true);
	else
	    ko.AtEdge.push_back(false);
	atEdgeCount = atEdgeCount >> 1;
    }
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2*2));
    
    ko.OwnLocationPartiallyOpen = (bool) observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2));
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)*NumLocations*2));
    
    ko.OwnLocationOpen = (bool) observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)*NumLocations));
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)*NumLocations));
    
    ko.OwnLocation = IntToLocation(observation/((int) (pow(2,NumObjects)*pow(2,NumAgents))));
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)));*/
    
    /*int gripperCount = observation/((int) (pow(2,NumObjects)*pow(2,NumAgents)));
    
    observation = observation%((int) (pow(2,NumObjects)*pow(2,NumAgents)));*/
    
    int agentcount = observation/((int) pow(2,NumObjects));
    for (int i = 0 ; i < NumAgents ; i++)
    {
	if (agentcount%2 == 1)
	    ko.AgentVisible.push_back(true);
	else
	    ko.AgentVisible.push_back(false);
	agentcount = agentcount >> 1;
	
	
	/*int currCount = gripperCount/((int) pow(NumObjects+1,2));
	
	ko.LeftGripperContents.push_back(currCount/(NumObjects+1));
	ko.RightGripperContents.push_back(currCount%(NumObjects+1));
	
	gripperCount = gripperCount%((int) pow(NumObjects+1,2));*/
    }

    observation = observation%((int) pow(2,NumObjects));
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (observation%2 == 1)
	    ko.ObjectVisible.push_back(true);
	else
	    ko.ObjectVisible.push_back(false);
	observation = observation >> 1;
    }
    
    return ko; 
}


bool KITCHEN::IsCerealInCupboard(const KITCHEN_STATE& state, double& reward) const
{
    if (state.ObjectLocations[CerealIndex] == CUPBOARD && !state.LocationOpen[CUPBOARD-LOCATION_OFFSET]
	&& !state.LocationPartiallyOpen[CUPBOARD-LOCATION_OFFSET])
    {
	reward = 99.9;
	return true;
    }
    //reward = -0.1;
    return false;
}

bool KITCHEN::IsPlate1InDishwasher(const KITCHEN_STATE& state, double& reward) const
{
    if (state.ObjectLocations[Plate1Index] == DISHWASHER && !state.LocationOpen[DISHWASHER-LOCATION_OFFSET]
	&& !state.LocationPartiallyOpen[DISHWASHER-LOCATION_OFFSET])
    {
	reward = 99.9;
	return true;
    }
    //reward = -0.1;
    return false;
}

bool KITCHEN::IsAppleJuiceInFridge(const KITCHEN_STATE& state, double& reward) const
{
    if (state.ObjectLocations[AppleJuiceIndex] == FRIDGE && !state.LocationOpen[FRIDGE-LOCATION_OFFSET]
	&& !state.LocationPartiallyOpen[FRIDGE-LOCATION_OFFSET])
    {
	reward = 99.9;
	return true;
    }
    //reward = -0.1;
    return false;
}

bool KITCHEN::IsTrayOnStove(const KITCHEN_STATE& state, double& reward) const
{
    if (state.ObjectLocations[TrayIndex] == STOVE && !state.IsToppled[TrayIndex])
    {
	reward = 99.9;
	return true;
    }
    //reward = -0.1;
    return false;
}

bool KITCHEN::IsCerealAndAppleJuice(const KITCHEN_STATE& state, double& reward) const
{
    if (IsAppleJuiceInFridge(state, reward) && IsCerealInCupboard(state, reward))
	return true;
    reward = -0.1;
    return false;
}

bool KITCHEN::IsTrayAndPlate(const KITCHEN_STATE& state, double& reward) const
{
    if (IsTrayOnStove(state, reward) && IsPlate1InDishwasher(state, reward))
	return true;
    reward = -0.1;
    return false;
}


void KITCHEN::GenerateLegal(const STATE& state, const HISTORY& history, std::vector< int >& legal, 
			    const SIMULATOR::STATUS& status) const
{
    /*for (int i = 0; i < NumActions; i++)
	legal.push_back(i);
    return; */
    
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    
    for (int i = 0; i < NumAgents; i++)
    {
	if (i == 0)
	    GenerateAgentActions(kitchenstate, history, legal, status, i, status.HumanDefined);	  
	else
	{
	    std::vector<int> currlegal;
	    currlegal.clear();
	    GenerateAgentActions(kitchenstate, history, currlegal, status, i, status.HumanDefined);
	    int s = legal.size();
	    for (int k = (int)currlegal.size()-1 ; k >= 0; k--)
		for (int j = 0 ; j < s ; j++)
		{
		    if (k == 0)
			legal[j] += pow(NumAgentActions,i)*currlegal.at(k);
		    else
			legal.push_back(legal.at(j) + pow(NumAgentActions,i)*currlegal.at(k));
		}
	}
    }

}

void KITCHEN::GenerateLegalAgent(const STATE& state, const HISTORY& history, std::vector< int >& actions, 
				 const SIMULATOR::STATUS& status, const int& index) const
{
    /*for (int i = 0; i < NumAgentActions; i++)
	actions.push_back(i);
    return; */
    
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);

    GenerateAgentActions(kitchenstate, history, actions, status, index-1, false);
    
}

void KITCHEN::GenerateAgentActions(const KITCHEN_STATE& kitchenstate, const HISTORY& history, 
			std::vector< int >& actions, const SIMULATOR::STATUS& status, const int& index, 
			const bool& humanDefined) const
{
    LocationType location = kitchenstate.RobotLocations[index];
    
    if (history.Size() == 0)
    {
	KitchenAction stay;
	stay.type = STAY_PUT;
	actions.push_back(ActionToInt(stay));
	return;
    }
    
    
    std::vector<bool> ObjectHere;
    std::vector<bool> AgentHere;
    ObjectHere.clear();
    AgentHere.clear();
    
    KitchenObservation ko;
    if (history.Size() > 0 && NumObservations > 1)
    {
	//if (status.jointhistory)
	  //  ko = IntToObservation(GetAgentObservation(history.Back().Observation, index));
	//else
	    ko = IntToObservation(history.Back().Observation);
    }
    for (int i = 0; i < NumObjects; i++)
    {
	if (history.Size() > 0 && NumObservations > 1)
	{
	    if (index == status.perspindex || ko.AgentVisible[index])
		ObjectHere.push_back(ko.ObjectVisible[i]);
	    else
		ObjectHere.push_back(UTILS::RandomDouble(0.0,1.0) < 0.5 ? true : false);
	}
	/*else if (history.Size() == 0)
	    ObjectHere.push_back((kitchenstate.ObjectLocations[i] == kitchenstate.RobotLocations[index] &&
	    (kitchenstate.RobotLocations[index] == SIDEBOARD || kitchenstate.RobotLocations[index] == STOVE ||
	    kitchenstate.LocationOpen[kitchenstate.RobotLocations[index]-LOCATION_OFFSET])));*/
	else
	    ObjectHere.push_back(UTILS::RandomDouble(0.0,1.0) < 0.5 ? true : false);
    }
    for (int i = 0; i < NumAgents; i++)
    {
	if (history.Size() > 0 && NumObservations > 1)
	{
	    //if (index == status.perspindex) //planning for myself
		AgentHere.push_back(ko.AgentVisible[i]);
	    //else if (i == status.perspindex || ko.AgentVisible[i])
		//AgentHere.push_back(true);
	}
	/*else if (history.Size() == 0)
	    AgentHere.push_back(index == i || 
		    kitchenstate.RobotLocations[i] == kitchenstate.RobotLocations[index]);*/
	else
	    AgentHere.push_back(UTILS::RandomDouble(0.0,1.0) < 0.5 ? true : false);
    }
    
    
    if (humanDefined && TestTrayOnStove && status.perspindex == index)
    {
	//std::cout << "preferred " << status.perspindex << " " << index << "\n";
	//assume you only grasp tray with right hand
	
	if (location == SIDEBOARD)
	{
	    if (!AgentHere[(index+1)%2])
	    {
		KitchenAction stay;
		stay.type = STAY_PUT;
		actions.push_back(ActionToInt(stay));
	    }
	    else
	    {
		if (kitchenstate.GripperEmpty[index*2+RIGHT-GRIPPER_OFFSET])
		{
		    KitchenAction ka;
		    ka.type = GRASP_JOINT;
		    ka.location = location;
		    ka.gripper = RIGHT;
		    ka.objectindex = TrayIndex;
		    ka.objectclass = TRAY;
		    actions.push_back(ActionToInt(ka));
		}
		else
		{
		    KitchenAction ka;
		    ka.type = MOVE_ROBOT_JOINT;
		    ka.location = location;
		    ka.location2 = STOVE;
		    actions.push_back(ActionToInt(ka));
		}
	    }
	}
	else if (location == STOVE)
	{
	    if (AgentHere[(index+1)%2] && 
		!kitchenstate.GripperEmpty[index*2+RIGHT-GRIPPER_OFFSET])
	    {
		KitchenAction ka;
		ka.type = PUT_DOWN_JOINT;
		ka.objectindex = TrayIndex;
		ka.objectclass = TRAY;
		ka.location = location;
		ka.gripper = RIGHT;
		actions.push_back(ActionToInt(ka));
	    }
	    else if (AgentHere[(index+1)%2])
	    {
		//nothing in gripper
		if (kitchenstate.IsToppled[TrayIndex])
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = TrayIndex;
		    ka.objectclass = TRAY;
		    ka.location = location;
		    ka.gripper =  RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
		else
		{
		    KitchenAction stay;
		    stay.type = STAY_PUT;
		    actions.push_back(ActionToInt(stay));
		}
	    }
	    else
	    {
		if (kitchenstate.IsToppled[TrayIndex])
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = TrayIndex;
		    ka.objectclass = TRAY;
		    ka.location = location;
		    ka.gripper =  RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
		else
		{
		    KitchenAction ka;
		    ka.type = MOVE_ROBOT;
		    ka.location = location;
		    ka.location2 = SIDEBOARD;
		    actions.push_back(ActionToInt(ka));
		}
	    }
	}
	else
	{
	    KitchenAction ka;
	    ka.type = MOVE_ROBOT;
	    ka.location = location;
	    ka.location2 = SIDEBOARD;
	    actions.push_back(ActionToInt(ka));
	}
	return;
    }
    
    //Close actions
    if ( (
	    (location == CUPBOARD && kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET)) || 
	    (location == DISHWASHER && kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET)) ||
	    (location == FRIDGE && kitchenstate.GripperEmpty.at(index*2+LEFT-GRIPPER_OFFSET))
	  ) && 
	(kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) || 
	kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET))
    )
    {
	KitchenAction ka;
	ka.type = CLOSE;
	ka.location = location;
	ka.gripper = location == FRIDGE ? LEFT : RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Grasp actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(index*2+h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (!IsFlat(ObjectTypes.at(o)) && !kitchenstate.IsToppled.at(o) && ObjectHere.at(o) &&
			ObjectTypes.at(o) != TRAY
		    )
		    {
			KitchenAction ka;
			ka.type = GRASP;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		    
    //Grasp from edge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(index*2+h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && kitchenstate.AtEdge.at(o) && ObjectHere.at(o) &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = GRASP_FROM_EDGE;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
	
    //Move actions
    bool doIndividualMoveActions = true;
    
    //Should not do a single-agent move if tray in one of the hands
    if (NumAgents > 1)
    {
	for (int h = 0 ; h < 2 ; h++)
	    if ((kitchenstate.InWhichGripper.at(TrayIndex).first  == index && 
		kitchenstate.InWhichGripper.at(TrayIndex).second == h+GRIPPER_OFFSET) || 
		(kitchenstate.TraySecondGripper.first  == index && 
		kitchenstate.TraySecondGripper.second == h+GRIPPER_OFFSET))
	    {
		doIndividualMoveActions = false;
		break;
	    }
    }
   
    if (doIndividualMoveActions)
	for (int i = LOCATION_OFFSET ; i < LOCATION_OFFSET + NumLocations ; i++)
	    if (static_cast<LocationType>(i) != location)
	    {
		//only one agent at a time can be at the cupboard, fridge, and dishwasher
		bool dontgothere = false;
		int countsideboard = 0, countstove = 0;
		for (int j = 0 ; j < NumAgents ; j++)
		    if (i != j && kitchenstate.RobotLocations[j] == static_cast<LocationType>(i) && !dontgothere) 
		    {
			if (kitchenstate.RobotLocations[j] == CUPBOARD || kitchenstate.RobotLocations[j] == FRIDGE
			    || kitchenstate.RobotLocations[j] == DISHWASHER)
			{
			    dontgothere = true;
			    break;
			}
			else if (kitchenstate.RobotLocations[j] == SIDEBOARD)
			{
			    countsideboard++;
			    if (countsideboard > 1)
			    {
				dontgothere = true;
				break;
			    }
			}
			else if (kitchenstate.RobotLocations[j] == STOVE)
			{
			    countstove++;
			    if (countstove > 1)
			    {
				dontgothere = true;
				break;
			    }
			}
		    }
		if (!dontgothere)
		{
		    KitchenAction ka;
		    ka.type = MOVE_ROBOT;
		    ka.location = location;
		    ka.location2 = static_cast<LocationType>(i);
		    actions.push_back(ActionToInt(ka));
		}
	    }
		    
    //Nudge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(index*2+h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && !kitchenstate.AtEdge.at(o) && ObjectHere.at(o) &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = NUDGE;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		    
    //Open actions
    if ((location == CUPBOARD || location == DISHWASHER) && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) &&
	kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN;
	ka.location = location;
	ka.gripper = RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Open partial actions
    if (location == FRIDGE && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	!kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.GripperEmpty.at(index*2+LEFT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN_PARTIAL;
	ka.location = location;
	ka.gripper = LEFT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Open complete actions
    if (location == FRIDGE && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN_COMPLETE;
	ka.location = location;
	ka.gripper = RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Pass object actions
    for (int o = 0 ; o < NumObjects ; o++)
	if (ObjectTypes.at(o) != TRAY)
	    for (int h1 = 0 ; h1 < 2 ; h1++)
		if (kitchenstate.InWhichGripper.at(o).first == index && 
		    kitchenstate.InWhichGripper.at(o).second == h1+GRIPPER_OFFSET && 
		    kitchenstate.GripperEmpty.at(index*2+(h1+1)%2))
		{
		    KitchenAction ka;
		    ka.type = PASS_OBJECT;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.gripper = static_cast<GripperType>(h1 + GRIPPER_OFFSET);
		    ka.gripper2 = static_cast<GripperType>((h1+1)%2 + GRIPPER_OFFSET);
		    actions.push_back(ActionToInt(ka));
		}
	
    //Place upright actions
    for (int h = 0 ; h < 2 ; h++)
	if (kitchenstate.GripperEmpty.at(index*2+h))
	    for (int o = 0 ; o < NumObjects ; o++)
		if (ObjectHere.at(o) && kitchenstate.IsToppled.at(o))
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
		
    //Put down actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    for (int o = 0 ; o < NumObjects ; o++)
		if (kitchenstate.InWhichGripper.at(o).first  == index && 
		    kitchenstate.InWhichGripper.at(o).second == h+GRIPPER_OFFSET &&
			ObjectTypes.at(o) != TRAY)
		{
		    KitchenAction ka;
		    ka.type = PUT_DOWN;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    actions.push_back(ActionToInt(ka));
		}

    //Put in actions
    if (kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
    {
	for (int h = 0 ; h < 2 ; h++)
	    if ((location == CUPBOARD || (location == DISHWASHER && h == RIGHT-GRIPPER_OFFSET) || 
		(location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (kitchenstate.InWhichGripper.at(o).first  == index && 
			kitchenstate.InWhichGripper.at(o).second == h+GRIPPER_OFFSET &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = PUT_IN;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			actions.push_back(ActionToInt(ka));
		    }
    }
    
    //Remove from actions
    if (kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(index*2+h) && (location == CUPBOARD || 
		(location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (ObjectHere.at(o) && !kitchenstate.IsToppled.at(o) &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = REMOVE_FROM;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			actions.push_back(ActionToInt(ka));
		    }
		    
    //Stay actions (always possible)
    KitchenAction stay;
    stay.type = STAY_PUT;
    actions.push_back(ActionToInt(stay));
		    
    //Grasp joint actions
    if (NumAgents > 1 && (location == SIDEBOARD || location == STOVE))
	for (int i = 0; i < NumAgents; i++)
	   if (i != index && AgentHere[i]) 
		for (int h = 0 ; h < 2 ; h++)
		    if (kitchenstate.GripperEmpty.at(index*2+h))
			for (int o = 0 ; o < NumObjects ; o++)
			    if (ObjectTypes.at(o) == TRAY && ObjectHere.at(o) && !kitchenstate.IsToppled.at(o))
			    {
				KitchenAction ka;
				ka.type = GRASP_JOINT;
				ka.location = location;
				ka.gripper = h == 0 ? LEFT : RIGHT;
				ka.objectindex = o;
				ka.objectclass = ObjectTypes[o];
				actions.push_back(ActionToInt(ka));
			    }

    //Put down joint actions
    if (NumAgents > 1 && (location == SIDEBOARD || location == STOVE))
	for (int i = 0; i < NumAgents; i++)
	   if (i != index && AgentHere[i]) 
		for (int h = 0 ; h < 2 ; h++)
		    for (int o = 0 ; o < NumObjects ; o++)
			if (ObjectTypes.at(o) == TRAY && ((kitchenstate.InWhichGripper.at(o).first  == index && 
			    kitchenstate.InWhichGripper.at(o).second == h+GRIPPER_OFFSET) || 
			    (kitchenstate.TraySecondGripper.first  == index && 
			    kitchenstate.TraySecondGripper.second == h+GRIPPER_OFFSET)))
			{
			    KitchenAction ka;
			    ka.type = PUT_DOWN_JOINT;
			    ka.objectindex = o;
			    ka.objectclass = ObjectTypes[o];
			    ka.location = location;
			    ka.gripper = h == 0 ? LEFT : RIGHT;
			    actions.push_back(ActionToInt(ka));
			}
			
    //Move joint actions
    if (NumAgents > 1 && (location == SIDEBOARD || location == STOVE))
	for (int h = 0 ; h < 2 ; h++)
	    if ((kitchenstate.InWhichGripper.at(TrayIndex).first  == index && 
		kitchenstate.InWhichGripper.at(TrayIndex).second == h+GRIPPER_OFFSET) || 
		(kitchenstate.TraySecondGripper.first  == index && 
		kitchenstate.TraySecondGripper.second == h+GRIPPER_OFFSET))
		for (int location2 = LOCATION_OFFSET ; location2 < LOCATION_OFFSET + NumLocations ; location2++)
		    if (location2 != location && (location2 == SIDEBOARD || location2 == STOVE))
		    {
			KitchenAction ka;
			ka.type = MOVE_ROBOT_JOINT;
			ka.location = location;
			ka.location2 = static_cast<LocationType>(location2);
			actions.push_back(ActionToInt(ka));
		    }
}


void KITCHEN::GeneratePreferred(const STATE& state, const HISTORY& history, 
				std::vector< int >& actions, const SIMULATOR::STATUS& status) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    
    for (int i = 0; i < NumAgents; i++)
    {
	if (i == 0)
	    GenerateAgentActions(kitchenstate, history, actions, status, i, status.HumanDefined);	  
	else
	{
	    std::vector<int> currlegal;
	    currlegal.clear();
	    GenerateAgentActions(kitchenstate, history, currlegal, status, i, status.HumanDefined);
	    int s = actions.size();
	    for (int k = (int)currlegal.size()-1 ; k >= 0; k--)
		for (int j = 0 ; j < s ; j++)
		{
		    if (k == 0)
			actions[j] += pow(NumAgentActions,i)*currlegal.at(k);
		    else
			actions.push_back(actions.at(j) + pow(NumAgentActions,i)*currlegal.at(k));
		}
	}
    }
}

void KITCHEN::GeneratePreferredAgent(const STATE& state, const HISTORY& history, 
				     std::vector< int >& actions, const SIMULATOR::STATUS& status, const int& index) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);

    GenerateAgentActions(kitchenstate, history, actions, status, index-1, status.HumanDefined);
}



int KITCHEN::ActionToInt(const KitchenAction& ka) const
{
    int action = 0;
    int location = ka.location - LOCATION_OFFSET;
    int location2 = ka.location2 - LOCATION_OFFSET;
    int gripper = ka.gripper - GRIPPER_OFFSET;
    int gripper2 = ka.gripper2 - GRIPPER_OFFSET;
    int object = ka.objectindex;
    //std::tr1::unordered_map< int, int >::const_iterator got = ObjectIndexMap.find(ka.object);
    //assert(got != ObjectIndexMap.end());
    //int object = got->first;
    
    switch(ka.type)
    {
	case(CLOSE):
	    action += location*2 + gripper;
	    break;
	case(GRASP):
	    action += NumLocations*2;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(GRASP_FROM_EDGE):
	    action += NumLocations*2 + NumObjects*NumLocations*2;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(MOVE_ROBOT):
	    action += NumLocations*2 + NumObjects*NumLocations*2*2;
	    action += location*NumLocations + location2;
	    break;
	case(NUDGE):
	    action += NumLocations*2 + NumObjects*NumLocations*2*2 + NumLocations*NumLocations;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(OPEN):
	    action += NumLocations*2 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations;
	    action += location*2 + gripper;
	    break;
	case(OPEN_PARTIAL):
	    action += NumLocations*2*2 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations;
	    action += location*2 + gripper;
	    break;
	case(OPEN_COMPLETE):
	    action += NumLocations*2*3 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations;
	    action += location*2 + gripper;
	    break;
	case(PASS_OBJECT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations;
	    action += object*4 + gripper*2 + gripper2;
	    break;
	case(PLACE_UPRIGHT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations + NumObjects*4;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(PUT_DOWN):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*4 + NumLocations*NumLocations + NumObjects*4;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(PUT_IN):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*5 + NumLocations*NumLocations + NumObjects*4;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(REMOVE_FROM):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*6 + NumLocations*NumLocations + NumObjects*4;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(STAY_PUT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4;
	    break;
	case(GRASP_JOINT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4 + 1;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(PUT_DOWN_JOINT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*8 + NumLocations*NumLocations + NumObjects*4 + 1;
	    action += object*NumLocations*2 + location*2 + gripper; 
	    break;
	case(MOVE_ROBOT_JOINT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*9 + NumLocations*NumLocations + NumObjects*4 + 1;
	    action += location*NumLocations + location2; 
	    break;
	default:
	    break;
	    
    }
    
    //std::cout << ka.type << std::endl;
    //std::cout << action << std::endl;
    
    return action;
}

KitchenAction KITCHEN::IntToAction(int action) const
{
    assert(action >= 0 && action < NumAgentActions);
    KitchenAction ka;
    
    //std::cout << "Int" << action << std::endl;
    
    if (action <  NumLocations*2)
    {
	ka.type = CLOSE;
	ka.location = IntToLocation(action/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2 + NumObjects*NumLocations*2)
    {
	ka.type = GRASP;
	action -= NumLocations*2;
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2 + NumObjects*NumLocations*2*2)
    {
	ka.type = GRASP_FROM_EDGE;
	action -= (NumLocations*2 + NumObjects*NumLocations*2);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2 + NumObjects*NumLocations*2*2 + NumLocations*NumLocations)
    {
	ka.type = MOVE_ROBOT;
	action -= (NumLocations*2 + NumObjects*NumLocations*2*2);
	ka.location = IntToLocation(action/NumLocations);
	ka.location2 = IntToLocation(action%NumLocations);
    }
    else if (action < NumLocations*2 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations)
    {
	ka.type = NUDGE;
	action -= (NumLocations*2 + NumObjects*NumLocations*2*2 + NumLocations*NumLocations);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*2 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations)
    {
	ka.type = OPEN;
	action -= (NumLocations*2 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations);
	ka.location = IntToLocation(action/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*3 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations)
    {
	ka.type = OPEN_PARTIAL;
	action -= NumLocations*2*2 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations;
	ka.location = IntToLocation(action/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations)
    {
	ka.type = OPEN_COMPLETE;
	action -= (NumLocations*2*3 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations);
	ka.location = IntToLocation(action/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations + NumObjects*4)
    {
	ka.type = PASS_OBJECT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations);
	ka.objectclass = ObjectTypes[action/4];
	ka.objectindex = action/4;
	ka.gripper = IntToGripper((action%4)/2);
	ka.gripper2 = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*4 + NumLocations*NumLocations + NumObjects*4)
    {
	ka.type = PLACE_UPRIGHT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*3 + NumLocations*NumLocations + NumObjects*4);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*5 + NumLocations*NumLocations + NumObjects*4)
    {
	ka.type = PUT_DOWN;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*4 + NumLocations*NumLocations + NumObjects*4);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*6 + NumLocations*NumLocations + NumObjects*4)
    {
	ka.type = PUT_IN;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*5 + NumLocations*NumLocations + NumObjects*4);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4)
    {
	ka.type = REMOVE_FROM;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*6 + NumLocations*NumLocations + NumObjects*4);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4 + 1)
    {
	ka.type = STAY_PUT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*8 + NumLocations*NumLocations + NumObjects*4 + 1)
    {
	ka.type = GRASP_JOINT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4 + 1);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*9 + NumLocations*NumLocations + NumObjects*4 + 1)
    {
	ka.type = PUT_DOWN_JOINT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*8 + NumLocations*NumLocations + NumObjects*4 + 1);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else
    {
	ka.type = MOVE_ROBOT_JOINT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*9 + NumLocations*NumLocations + NumObjects*4 + 1);
	ka.location = IntToLocation(action/NumLocations);
	ka.location2 = IntToLocation(action%NumLocations);
    }
    
    //std::cout << ka.type << std::endl;
    
    
    return ka;
}

GripperType KITCHEN::IntToGripper(const int& i) const
{
    assert(i == 0 || i == 1);
    switch(i)
    {
	case(0):
	    return LEFT;
	default:
	    return RIGHT;
    }
}

LocationType KITCHEN::IntToLocation(const int& i) const
{
    assert(i >= 0 && i <= 4);
    switch(i)
    {
	case(0):
	    return CUPBOARD;
	case(1):
	    return DISHWASHER;
	case(2):
	    return FRIDGE;
	case(3):
	    return SIDEBOARD;
	default:
	    return STOVE;
    }
}

void KITCHEN::DisplayAction(int action, std::ostream& ostr) const
{
    ostr << "Action(s)\n";
    if (NumAgents == 1)
	DisplayAgentAction(action, ostr);
    else
    {
	for (int i = NumAgents-1 ; i >= 0 ; i--)
	{
	    ostr << "Robot " << i << ":";
	    DisplayAgentAction(action/((int) pow(NumAgentActions, i)), ostr);
	    action = action%((int) pow(NumAgentActions, i));
	}
    }
}

void KITCHEN::DisplayAgentAction(int action, std::ostream& ostr) const
{
    KitchenAction ka = IntToAction(action);
    
    switch(ka.type)
    {
	case(CLOSE):
	case(OPEN):
	case(OPEN_PARTIAL):
	case(OPEN_COMPLETE):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << GripperToString(ka.gripper) << ")\n";
	    break;
	case(GRASP):
	case(GRASP_FROM_EDGE):
	case(NUDGE):
	case(PLACE_UPRIGHT):
	case(PUT_DOWN):
	case(PUT_IN):
	case(REMOVE_FROM):
	case(GRASP_JOINT):
	case(PUT_DOWN_JOINT):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(MOVE_ROBOT):
	case(MOVE_ROBOT_JOINT):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << LocationToString(ka.location2) << ")\n";
	    break;
	case(PASS_OBJECT):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << GripperToString(ka.gripper) << "," << GripperToString(ka.gripper2) << ")\n";
	    break;
	case(STAY_PUT):
	    ostr << ActionToString(ka.type) << "\n";
	    break;
	default:
	    break;
    }
}


bool KITCHEN::Collision(const KITCHEN_STATE& state, const LocationType& location, const int& index) const
{
    for (int i = 0 ; i < (int) state.RobotLocations.size() ; i++)
	if (i != index && state.RobotLocations[i] == location && location != STOVE && location != SIDEBOARD)
	    return true;
    return false;
}


void KITCHEN::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    ostr << "\n";
    
    for (int i = 0 ; i < NumAgents ; i++)
    {
	ostr << "Robot " << i << ": " << LocationToString(kitchenstate.RobotLocations[i]) << "," <<
		(kitchenstate.GripperEmpty[i*2] ? "LEFT_EMPTY" : "LEFT_FULL") << "," <<
		(kitchenstate.GripperEmpty[i*2+1] ? "RIGHT_EMPTY" : "RIGHT_FULL") <<"\n";
    }
    for (int i = 0 ; i < NumObjects ; i++)
    {
	ostr << ObjectToString(ObjectTypes[i]) << " : " << 
	    LocationToString(kitchenstate.ObjectLocations[i]) << "," << 
	    GripperToString(kitchenstate.InWhichGripper[i].second) << kitchenstate.InWhichGripper[i].first;
	if (ObjectTypes[i] == TRAY)
	    ostr << "/" << GripperToString(kitchenstate.TraySecondGripper.second) << kitchenstate.TraySecondGripper.first;
	ostr  << "," <<
	    (kitchenstate.AtEdge[i] ? "AT_EDGE" : "NOT_AT_EDGE") << "," <<
	    (kitchenstate.IsToppled[i] ? "TOPPLED" : "NOT_TOPPLED") << "," <<
	    (IsFlat(ObjectTypes[i]) ? "FLAT" : "NOT_FLAT") <<"\n";
    }
    for (int i = 0 ; i < NumLocations ; i++)
    {
	ostr << LocationToString(IntToLocation(i)) << " : " <<
	(kitchenstate.LocationOpen[i] ? "OPEN" : "NOT_OPEN") << "," <<
	(kitchenstate.LocationPartiallyOpen[i] ? "PARTIALLY_OPEN" : "NOT_PARTIALLY_OPEN") << "\n";
    }
    
    
    
    ostr << "\n";
}

void KITCHEN::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
    ostr << "Observation(s)\n";
    
    for (int i = NumAgents-1 ; i >= 0 ; i--)
    {
    
	ostr << "Robot " << i << ": ";
	int currObservation = observation/((int) pow(NumAgentObservations, i));
	
	DisplayAgentObservation(observation, ostr);
	
	observation = observation%((int) pow(NumAgentObservations, i));
    }
    
    ostr << "\n";
}

void KITCHEN::DisplayAgentObservation(int observation, std::ostream& ostr) const
{
    KitchenObservation ko = IntToObservation(observation);
	
    for (int j = 0 ; j < NumObjects ; j++)
    {
	ostr << ObjectToString(ObjectTypes[j]) << ":" << ko.ObjectVisible[j];
	if (j < NumObjects-1)
	    ostr << ",";
	else
	    ostr << "/";
    }
    for (int j = 0 ; j < NumAgents ; j++)
    {
	ostr << "Agent " << j << ":" << ko.AgentVisible[j];
	//ostr << "-LG:" << ko.LeftGripperContents[j] << "-RG:" << ko.RightGripperContents[j];
	if (j < NumAgents-1)
	    ostr << ",";
    }
    ostr << "\n";    
}



std::string KITCHEN::ActionToString(const ActionType& t) const
{
    switch(t)
    {
	case(CLOSE):
	    return "CLOSE";
	case(GRASP):
	    return "GRASP";
	case(GRASP_FROM_EDGE):
	    return "GRASP_FROM_EDGE";
	case(MOVE_ROBOT):
	    return "MOVE_ROBOT";
	case(NUDGE):
	    return "NUDGE";
	case(OPEN):
	    return "OPEN";
	case(OPEN_PARTIAL):
	    return "OPEN_PARTIAL";
	case(OPEN_COMPLETE):
	    return "OPEN_COMPLETE";
	case(PASS_OBJECT):
	    return "PASS_OBJECT";
	case(PLACE_UPRIGHT):
	    return "PLACE_UPRIGHT";
	case(PUT_DOWN):
	    return "PUT_DOWN";
	case(PUT_IN):
	    return "PUT_IN";
	case(REMOVE_FROM):
	    return "REMOVE_FROM";
	case(STAY_PUT):
	    return "STAY_PUT";
	case(GRASP_JOINT):
	    return "GRASP_JOINT";
	case(PUT_DOWN_JOINT):
	    return "PUT_DOWN_JOINT";
	case(MOVE_ROBOT_JOINT):
	    return "MOVE_ROBOT_JOINT";
	default:
	    return "INVALID_ACTION";
    }
}

std::string KITCHEN::GripperToString(const GripperType& t) const
{
    switch(t)
    {
	case(LEFT):
	    return "LEFT";
	case(RIGHT):
	    return "RIGHT";
	case(NO_GRIPPER):
	    return "NO_GRIPPER";
	default:
	    return "INVALID_GRIPPER";
    }
}

std::string KITCHEN::LocationToString(const LocationType& t) const
{
    switch(t)
    {
	case(CUPBOARD):
	    return "CUPBOARD";
	case(DISHWASHER):
	    return "DISHWASHER";
	case(FRIDGE):
	    return "FRIDGE";
	case(SIDEBOARD):
	    return "SIDEBOARD";
	case(STOVE):
	    return "STOVE";
	case(NO_LOCATION):
	    return "NO_LOCATION";
	default:
	    return "INVALID_LOCATION";
    }
}

std::string KITCHEN::ObjectToString(const ObjectClass& t) const
{
    switch(t)
    {
	case(APPLEJUICE):
	    return "APPLEJUICE";
	case(CALGONIT):
	    return "CALGONIT";
	case(GRANINI):
	    return "GRANINI";
	case(MEASURINGCUP):
	    return "MEASURINGCUP";
	case(RICEBOX):
	    return "RICEBOX";
	case(CEREAL):
	    return "CEREAL";
	case(PLATE):
	    return "PLATE";
	case(CUP):
	    return "CUP";
	case(TRAY):
	    return "TRAY";
	case(NO_OBJECT):
	    return "NO_OBJECT";
	default:
	    return "INVALID_OBJECT";
    }
}



/**void KITCHEN::GenerateAgentActions(const KITCHEN_STATE& kitchenstate, const HISTORY& history, 
			std::vector< int >& actions, const SIMULATOR::STATUS& status, const int& index, 
			const bool& humanDefined) const
{
    LocationType location = kitchenstate.RobotLocations[index];
    
    if (history.Size() == 0)
    {
	KitchenAction stay;
	stay.type = STAY_PUT;
	actions.push_back(ActionToInt(stay));
	return;
    }
    
    
    std::vector<bool> ObjectHere;
    std::vector<bool> AgentHere;
    ObjectHere.clear();
    AgentHere.clear();
    
    KitchenObservation ko;
    if (history.Size() > 0 && NumObservations > 1)
    {
	//if (status.jointhistory)
	  //  ko = IntToObservation(GetAgentObservation(history.Back().Observation, index));
	//else
	    ko = IntToObservation(history.Back().Observation);
    }
    for (int i = 0; i < NumObjects; i++)
    {
	if (history.Size() > 0 && NumObservations > 1)
	{
	    if (index == status.perspindex || ko.AgentVisible[index])
		ObjectHere.push_back(ko.ObjectVisible[i]);
	    else
		ObjectHere.push_back(UTILS::RandomDouble(0.0,1.0) < 0.5 ? true : false);
	}
	else
	    ObjectHere.push_back(UTILS::RandomDouble(0.0,1.0) < 0.5 ? true : false);
    }
    for (int i = 0; i < NumAgents; i++)
    {
	if (history.Size() > 0 && NumObservations > 1)
	{
	    //if (index == status.perspindex) //planning for myself
		AgentHere.push_back(ko.AgentVisible[i]);
	    //else if (i == status.perspindex || ko.AgentVisible[i])
		//AgentHere.push_back(true);
	}
	else
	    AgentHere.push_back(UTILS::RandomDouble(0.0,1.0) < 0.5 ? true : false);
    }
    
    
    if (humanDefined && TestTrayOnStove && status.perspindex == index)
    {
	//std::cout << "preferred " << status.perspindex << " " << index << "\n";
	//assume you only grasp tray with right hand
	
	if (location == SIDEBOARD)
	{
	    if (!AgentHere[(index+1)%2])
	    {
		KitchenAction stay;
		stay.type = STAY_PUT;
		actions.push_back(ActionToInt(stay));
	    }
	    else
	    {
		if (ko.RightGripperContents[index] == NumObjects)
		{
		    KitchenAction ka;
		    ka.type = GRASP_JOINT;
		    ka.location = location;
		    ka.gripper = RIGHT;
		    ka.objectindex = TrayIndex;
		    ka.objectclass = TRAY;
		    actions.push_back(ActionToInt(ka));
		}
		else
		{
		    KitchenAction ka;
		    ka.type = MOVE_ROBOT_JOINT;
		    ka.location = location;
		    ka.location2 = STOVE;
		    actions.push_back(ActionToInt(ka));
		}
	    }
	}
	else if (location == STOVE)
	{
	    if (AgentHere[(index+1)%2] && 
		ko.RightGripperContents[index] != NumObjects)
	    {
		KitchenAction ka;
		ka.type = PUT_DOWN_JOINT;
		ka.objectindex = TrayIndex;
		ka.objectclass = TRAY;
		ka.location = location;
		ka.gripper = RIGHT;
		actions.push_back(ActionToInt(ka));
	    }
	    else if (AgentHere[(index+1)%2])
	    {
		//nothing in gripper
		if (kitchenstate.IsToppled[TrayIndex])
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = TrayIndex;
		    ka.objectclass = TRAY;
		    ka.location = location;
		    ka.gripper =  RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
		else
		{
		    KitchenAction stay;
		    stay.type = STAY_PUT;
		    actions.push_back(ActionToInt(stay));
		}
	    }
	    else
	    {
		if (kitchenstate.IsToppled[TrayIndex])
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = TrayIndex;
		    ka.objectclass = TRAY;
		    ka.location = location;
		    ka.gripper =  RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
		else
		{
		    KitchenAction ka;
		    ka.type = MOVE_ROBOT;
		    ka.location = location;
		    ka.location2 = SIDEBOARD;
		    actions.push_back(ActionToInt(ka));
		}
	    }
	}
	else
	{
	    KitchenAction ka;
	    ka.type = MOVE_ROBOT;
	    ka.location = location;
	    ka.location2 = SIDEBOARD;
	    actions.push_back(ActionToInt(ka));
	}
	return;
    }
    
    //Close actions
    if ( (
	    (location == CUPBOARD && ko.RightGripperContents[index] == NumObjects) || 
	    (location == DISHWASHER && ko.RightGripperContents[index] == NumObjects) ||
	    (location == FRIDGE && ko.LeftGripperContents[index] == NumObjects)
	  ) && 
	(kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) || 
	kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET))
    )
    {
	KitchenAction ka;
	ka.type = CLOSE;
	ka.location = location;
	ka.gripper = location == FRIDGE ? LEFT : RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Grasp actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if ((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
		(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (!IsFlat(ObjectTypes.at(o)) && !kitchenstate.IsToppled.at(o) && ObjectHere.at(o) &&
			ObjectTypes.at(o) != TRAY
		    )
		    {
			KitchenAction ka;
			ka.type = GRASP;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		    
    //Grasp from edge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if ((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
		(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && kitchenstate.AtEdge.at(o) && ObjectHere.at(o) &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = GRASP_FROM_EDGE;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
	
    //Move actions
    bool doIndividualMoveActions = true;
    
    //Should not do a single-agent move if tray in one of the hands
    if (NumAgents > 1)
    {
	if (ko.LeftGripperContents[index] == TrayIndex || ko.RightGripperContents[index] == TrayIndex) 
	    {
		doIndividualMoveActions = false;
		//break;
	    }
    }
   
    if (doIndividualMoveActions)
	for (int i = LOCATION_OFFSET ; i < LOCATION_OFFSET + NumLocations ; i++)
	    if (static_cast<LocationType>(i) != location)
	    {
		//only one agent at a time can be at the cupboard, fridge, and dishwasher
		bool dontgothere = false;
		int countsideboard = 0, countstove = 0;
		for (int j = 0 ; j < NumAgents ; j++)
		    if (i != j && kitchenstate.RobotLocations[j] == static_cast<LocationType>(i) && !dontgothere) 
		    {
			if (kitchenstate.RobotLocations[j] == CUPBOARD || kitchenstate.RobotLocations[j] == FRIDGE
			    || kitchenstate.RobotLocations[j] == DISHWASHER)
			{
			    dontgothere = true;
			    break;
			}
			else if (kitchenstate.RobotLocations[j] == SIDEBOARD)
			{
			    countsideboard++;
			    if (countsideboard > 1)
			    {
				dontgothere = true;
				break;
			    }
			}
			else if (kitchenstate.RobotLocations[j] == STOVE)
			{
			    countstove++;
			    if (countstove > 1)
			    {
				dontgothere = true;
				break;
			    }
			}
		    }
		if (!dontgothere)
		{
		    KitchenAction ka;
		    ka.type = MOVE_ROBOT;
		    ka.location = location;
		    ka.location2 = static_cast<LocationType>(i);
		    actions.push_back(ActionToInt(ka));
		}
	    }
		    
    //Nudge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if ((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
		(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && !kitchenstate.AtEdge.at(o) && ObjectHere.at(o) &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = NUDGE;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		    
    //Open actions
    if ((location == CUPBOARD || location == DISHWASHER) && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) &&
	ko.RightGripperContents[index] == NumObjects)
    {
	KitchenAction ka;
	ka.type = OPEN;
	ka.location = location;
	ka.gripper = RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Open partial actions
    if (location == FRIDGE && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	!kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && 
	ko.LeftGripperContents[index] == NumObjects)
    {
	KitchenAction ka;
	ka.type = OPEN_PARTIAL;
	ka.location = location;
	ka.gripper = LEFT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Open complete actions
    if (location == FRIDGE && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && 
	ko.RightGripperContents[index] == NumObjects)
    {
	KitchenAction ka;
	ka.type = OPEN_COMPLETE;
	ka.location = location;
	ka.gripper = RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    
    //Pass object actions
    for (int o = 0 ; o < NumObjects ; o++)
	if (ObjectTypes.at(o) != TRAY)
	    for (int h1 = 0 ; h1 < 2 ; h1++)
		if (((h1 == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == o) || 
		    (h1 == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == o)) && 
		    (((h1+1)%2 == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
		    ((h1+1)%2 == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects)))
		{
		    KitchenAction ka;
		    ka.type = PASS_OBJECT;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.gripper = static_cast<GripperType>(h1 + GRIPPER_OFFSET);
		    ka.gripper2 = static_cast<GripperType>((h1+1)%2 + GRIPPER_OFFSET);
		    actions.push_back(ActionToInt(ka));
		}
	
    //Place upright actions
    for (int h = 0 ; h < 2 ; h++)
	if ((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
	    (h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects))
	    for (int o = 0 ; o < NumObjects ; o++)
		if (ObjectHere.at(o) && kitchenstate.IsToppled.at(o))
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
		
    //Put down actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    for (int o = 0 ; o < NumObjects ; o++)
		if (((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == o) || 
			(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == o)) &&
			ObjectTypes.at(o) != TRAY) 
		{
		    KitchenAction ka;
		    ka.type = PUT_DOWN;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    actions.push_back(ActionToInt(ka));
		}

    //Put in actions
    if (kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
    {
	for (int h = 0 ; h < 2 ; h++)
	    if ((location == CUPBOARD || (location == DISHWASHER && h == RIGHT-GRIPPER_OFFSET) || 
		(location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == o) || 
			(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == o)) &&
			ObjectTypes.at(o) != TRAY) 
		    {
			KitchenAction ka;
			ka.type = PUT_IN;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			actions.push_back(ActionToInt(ka));
		    }
    }
    
    //Remove from actions
    if (kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
	for (int h = 0 ; h < 2 ; h++)
	    if (((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
		(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects))
		&& (location == CUPBOARD || 
		(location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (ObjectHere.at(o) && !kitchenstate.IsToppled.at(o) &&
			ObjectTypes.at(o) != TRAY)
		    {
			KitchenAction ka;
			ka.type = REMOVE_FROM;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			actions.push_back(ActionToInt(ka));
		    }
		    
    //Stay actions (always possible)
    KitchenAction stay;
    stay.type = STAY_PUT;
    actions.push_back(ActionToInt(stay));
		    
    //Grasp joint actions
    if (NumAgents > 1 && (location == SIDEBOARD || location == STOVE))
	for (int i = 0; i < NumAgents; i++)
	   if (i != index && AgentHere[i]) 
		for (int h = 0 ; h < 2 ; h++)
		    if ((h == LEFT-GRIPPER_OFFSET && ko.LeftGripperContents[index] == NumObjects) || 
			(h == RIGHT-GRIPPER_OFFSET && ko.RightGripperContents[index] == NumObjects))
			for (int o = 0 ; o < NumObjects ; o++)
			    if (ObjectTypes.at(o) == TRAY && ObjectHere.at(o) && !kitchenstate.IsToppled.at(o))
			    {
				KitchenAction ka;
				ka.type = GRASP_JOINT;
				ka.location = location;
				ka.gripper = h == 0 ? LEFT : RIGHT;
				ka.objectindex = o;
				ka.objectclass = ObjectTypes[o];
				actions.push_back(ActionToInt(ka));
			    }

    //Put down joint actions
    if (NumAgents > 1 && (location == SIDEBOARD || location == STOVE))
	for (int i = 0; i < NumAgents; i++)
	   if (i != index && AgentHere[i]) 
		for (int h = 0 ; h < 2 ; h++)
		    for (int o = 0 ; o < NumObjects ; o++)
			if (ko.LeftGripperContents[index] == TrayIndex  || ko.RightGripperContents[index] == TrayIndex) 
			{
			    KitchenAction ka;
			    ka.type = PUT_DOWN_JOINT;
			    ka.objectindex = o;
			    ka.objectclass = ObjectTypes[o];
			    ka.location = location;
			    ka.gripper = h == 0 ? LEFT : RIGHT;
			    actions.push_back(ActionToInt(ka));
			}
			
    //Move joint actions
    if (NumAgents > 1 && (location == SIDEBOARD || location == STOVE))
	if (ko.LeftGripperContents[index] == TrayIndex  || ko.RightGripperContents[index] == TrayIndex) 

		for (int location2 = LOCATION_OFFSET ; location2 < LOCATION_OFFSET + NumLocations ; location2++)
		    if (location2 != location && (location2 == SIDEBOARD || location2 == STOVE))
		    {
			KitchenAction ka;
			ka.type = MOVE_ROBOT_JOINT;
			ka.location = location;
			ka.location2 = static_cast<LocationType>(location2);
			actions.push_back(ActionToInt(ka));
		    }
}*/



