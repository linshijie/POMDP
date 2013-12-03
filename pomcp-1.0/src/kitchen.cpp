#include "kitchen.h"
#include <iomanip>
#include <tr1/unordered_set>

KITCHEN::KITCHEN(int nplates, int ncups): 
  NumPlates(nplates), NumCups(ncups), NumLocations(5),
  LOCATION_OFFSET(static_cast<int>(CUPBOARD)), GRIPPER_OFFSET(static_cast<int>(LEFT)),
  ACTION_OFFSET(static_cast<int>(CLOSE)),
  HaveAppleJuice(true), HaveCalgonit(false), HaveGranini(false),
  HaveMeasuringCup(false), HaveRiceBox(false), HaveCereal(true),
  HaveTray(false),
  TestCerealInCupboard(false), TestPlate1InDishwasher(false), TestAppleJuiceInFridge(true),
  TestTrayOnStove(false),
  NumAgents(1),
  NonDeterministicActions(true),
  ProbClose(0.95), ProbGrasp(0.95), ProbGrapsFromEdge(0.95), ProbMove(0.95), ProbNudge(0.95), ProbOpen(0.95),
  ProbOpenPartial(0.95), ProbOpenComplete(0.95), ProbPassObject(0.95), 
  ProbPlaceUpright(0.95), ProbPutDown(0.95), ProbPutIn(0.95), ProbRemoveFrom(0.95)
{
    NumObjects = NumPlates+NumCups;
    
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
    
    NumAgentActions = NumObjects*(NumLocations*18+4) + 8*NumLocations + NumLocations*NumLocations;
    NumAgentObservations = pow(2,NumObjects)*(NumLocations+1);
    
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
	
    RewardRange = 99.9;
    Discount = 1.0;
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
    kitchenstate->RobotLocations.push_back(SIDEBOARD);
    for (int i = 1; i < NumAgents; i++)
	kitchenstate->RobotLocations.push_back(static_cast<LocationType>(UTILS::Random(NumLocations)));
    for (int i = 1; i < NumAgents; i++)
    {
	LocationType agentlocation;
	do
	{
	    agentlocation = static_cast<LocationType>(UTILS::Random(0,NumLocations)+LOCATION_OFFSET);
	}while(Collision(*kitchenstate,agentlocation,i));
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
	else
	{
	    kitchenstate->ObjectLocations.push_back(static_cast<LocationType>(UTILS::Random(NumLocations)));
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

bool KITCHEN::Step(STATE& state, int action, int& observation, double& reward) const
{
    KITCHEN_STATE& kitchenstate = safe_cast<KITCHEN_STATE&>(state);
    
    if (NumAgents == 1)
	StepAgent(kitchenstate, action, observation, reward, 0);
    else
    {
	for (int i = 0; i < NumAgents; i++)
	{
	    if (UTILS::RandomDouble(0.0,1.0) < 0.5)
	    {
		StepAgent(kitchenstate, action%NumAgentActions, observation, reward, 0);
		StepAgent(kitchenstate, action/NumAgentActions, observation, reward, 1);
	    }
	    else
	    {
		StepAgent(kitchenstate, action/NumAgentActions, observation, reward, 1);
		StepAgent(kitchenstate, action%NumAgentActions, observation, reward, 0);
	    }
	}
    }
    
    observation = 0;
    for (int i = 0; i < NumAgents; i++)
	observation += pow(NumAgentObservations,i)*MakeObservation(kitchenstate, i);
    
    //Just executed an unsuccessful action
    if (reward < 0.0)
    {
	//std::cout << "fail " << ActionToString(ka.type) << std::endl;
	/*if (ka.type == MOVE_ROBOT)
	{
	    std::cout << LocationToString(kitchenstate.RobotLocation) << " "  << 
		LocationToString(ka.location) << " " <<
		LocationToString(ka.location2) << " " << std::endl;
	}*/
	return false;
    }
    
    //std::cout << "succ " << ActionToString(ka.type) << std::endl;
    
    bool reachedGoal = false;
    
    if (TestCerealInCupboard)
	reachedGoal = reachedGoal || IsCerealInCupboard(kitchenstate, reward);
    if (TestPlate1InDishwasher)
	reachedGoal = reachedGoal || IsPlate1InDishwasher(kitchenstate, reward);
    if (TestAppleJuiceInFridge)
	reachedGoal = reachedGoal || IsAppleJuiceInFridge(kitchenstate, reward);
    
    if (reachedGoal)
	reward = 99.9;
    else
	reward = -0.1;
    
    return reachedGoal;
}

bool KITCHEN::StepAgent(KITCHEN_STATE& kitchenstate, int action, int& observation, 
			double& reward, const int& index) const
{
    KitchenAction ka = IntToAction(action);
    
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
	    if (kitchenstate.RobotLocations[index] == ka.location && kitchenstate.RobotLocations[index] != ka.location2
	    )
	    {
		for (int i = 0; i < NumAgents; i++)
		    if (i != index && kitchenstate.RobotLocations[i] == kitchenstate.RobotLocations[index] &&
			(kitchenstate.RobotLocations[i] == CUPBOARD || kitchenstate.RobotLocations[i] == FRIDGE ||
			    kitchenstate.RobotLocations[i] == DISHWASHER)
		    )
		    {
			reward = -10.0;
			break;
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
		kitchenstate.IsToppled[ka.objectindex] && kitchenstate.GripperEmpty[index*2+ka.gripper-GRIPPER_OFFSET] &&
		ObjectTypes[ka.objectindex] != TRAY
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
	default:
	    break;
    }
}

bool KITCHEN::LocalMove(STATE& state, const HISTORY& history, int stepObs, const SIMULATOR::STATUS& status) const
{
    KITCHEN_STATE& kitchenstate = safe_cast<KITCHEN_STATE&>(state);
    
    std::tr1::unordered_set<int> UnseenObjects;
    std::tr1::unordered_set<int> UnexploredLocations;
    
    for (int i = 0; i < NumObjects; i++)
	UnseenObjects.insert(i);
    for (int i = 0; i < NumLocations; i++)
	UnexploredLocations.insert(i+LOCATION_OFFSET);
    
    for (int i = 0; i < history.Size(); i++)
    {
	KitchenObservation ko = IntToObservation(history[i].Observation);
	for (int j = 0; j < NumObjects; j++)
	    if (ko.objectvisible[j])
		UnseenObjects.erase(j);
	     
	if (ko.location == SIDEBOARD || ko.location == STOVE)
	    UnexploredLocations.erase(static_cast<int>(ko.location));
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
	    if (ko.location != NO_LOCATION && removefromunseen)
		UnexploredLocations.erase(static_cast<int>(ko.location));
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
    }
    
    return true;
}

int KITCHEN::MakeObservation(const KITCHEN_STATE& state, const int& index) const
{
    KitchenObservation ko;
    ko.objectvisible.clear();
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (state.ObjectLocations[i] == NO_LOCATION || (state.ObjectLocations[i] == state.RobotLocations[index] &&
	    (state.RobotLocations[index] == SIDEBOARD || state.RobotLocations[index] == STOVE ||
	    state.LocationOpen[state.RobotLocations[index]-LOCATION_OFFSET]))
	    )
	    ko.objectvisible.push_back(true);
	else
	    ko.objectvisible.push_back(false);
    }
    ko.location = state.RobotLocations[index];
    /*for (int i = 0 ; i < NumAgents ; i++)
    {
	if (index == i || state.RobotLocations[i] == state.RobotLocations[index])
	    ko.robotlocations.push_back(state.RobotLocations[i]);
	else
	    ko.robotlocations.push_back(NO_LOCATION);
    }*/
    
    return ObservatonToInt(ko);
}

int KITCHEN::ObservatonToInt(const KitchenObservation& ko) const
{
    int observation = 0;
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (ko.objectvisible[i])
	    observation += pow(2,i);
    }
    
    //for (int i = 0 ; i < NumAgents ; i++)
	//observation += pow(2,NumObjects)*pow(NumLocations+1,i)*(ko.robotlocations[i]-LOCATION_OFFSET);
    observation += pow(2,NumObjects)*(ko.location-LOCATION_OFFSET);
  
    return observation;
}

KitchenObservation KITCHEN::IntToObservation(int observation) const
{
    KitchenObservation ko;
    ko.objectvisible.clear();

    /*for (int i = NumAgents-1 ; i >= 0 ; i--)
    {
	ko.robotlocations.push_front(static_cast<LocationType>(
	    observation/((int) (pow(2,NumObjects)*pow(NumLocations+1,i)))+LOCATION_OFFSET)
	);
	observation = observation%((int) (pow(2,NumObjects)*pow(NumLocations+1,i)));
    }*/
    
    ko.location = static_cast<LocationType>(observation/((int) pow(2,NumObjects))+LOCATION_OFFSET);
    observation = observation%((int) pow(2,NumObjects));
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (observation%2 == 1)
	    ko.objectvisible.push_back(true);
	else
	    ko.objectvisible.push_back(false);
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
    reward = -0.1;
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
    reward = -0.1;
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
    reward = -0.1;
    return false;
}

bool KITCHEN::IsTrayOnStove(const KITCHEN_STATE& state, double& reward) const
{
    if (state.ObjectLocations[TrayIndex] == STOVE)
    {
	reward = 99.9;
	return true;
    }
    reward = -0.1;
    return false;
}


void KITCHEN::GenerateLegal(const STATE& state, const HISTORY& history, std::vector< int >& legal, 
			    const SIMULATOR::STATUS& status) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    for (int i = 0; i < NumAgents; i++)
	GenerateLegalAgent(kitchenstate, history, legal, status, i);	    
}

void KITCHEN::GenerateLegalAgent(const KITCHEN_STATE& kitchenstate, const HISTORY& history, 
				 std::vector< int >& legal, const SIMULATOR::STATUS& status, const int& index) const
{
    LocationType location = kitchenstate.RobotLocations[index];
    
    std::vector<bool> ObjectHere;
    KitchenObservation ko;
    if (history.Size() > 0 && NumObservations > 1)
	ko = IntToObservation(history.Back().Observation);
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (history.Size() > 0 && NumObservations > 1)
	    ObjectHere.push_back(ko.objectvisible[i]);
	else
	    ObjectHere.push_back(kitchenstate.ObjectLocations[i] == location);
    }
    
    //Close actions
    if ( (
	    (location == CUPBOARD && kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET)) || 
	    (location == DISHWASHER && kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET)) ||
	    (location == FRIDGE && kitchenstate.GripperEmpty.at(index*2+LEFT-GRIPPER_OFFSET))
    ) && 
	(kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) || kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET)))
    {
	KitchenAction ka;
	ka.type = CLOSE;
	ka.location = location;
	ka.gripper = location == FRIDGE ? LEFT : RIGHT;
	legal.push_back(ActionToInt(ka));
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
			legal.push_back(ActionToInt(ka));
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
			legal.push_back(ActionToInt(ka));
		    }
		    
    //Move actions
    for (int i = LOCATION_OFFSET ; i < LOCATION_OFFSET + NumLocations ; i++)
	if (static_cast<LocationType>(i) != location)
	{
	    //only one agent at a time can be at the cupboard, fridge, and dishwasher
	    bool otheragentthere = false;
	    for (int j = 0 ; j < NumAgents ; j++)
		if (i != j && kitchenstate.RobotLocations[j] == static_cast<LocationType>(i) &&
		    (kitchenstate.RobotLocations[j] == CUPBOARD || kitchenstate.RobotLocations[j] == FRIDGE
			|| kitchenstate.RobotLocations[j] == DISHWASHER)
		)
		{
		    otheragentthere = true;
		    break;
		}
	    if (!otheragentthere)
	    {
		KitchenAction ka;
		ka.type = MOVE_ROBOT;
		ka.location = location;
		ka.location2 = static_cast<LocationType>(i);
		legal.push_back(ActionToInt(ka));
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
			legal.push_back(ActionToInt(ka));
		    }
		    
    //Open actions
    if ((location == CUPBOARD || location == DISHWASHER) && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) &&
	kitchenstate.GripperEmpty.at(index*2+RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN;
	ka.location = location;
	ka.gripper = RIGHT;
	legal.push_back(ActionToInt(ka));
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
	legal.push_back(ActionToInt(ka));
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
	legal.push_back(ActionToInt(ka));
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
		    legal.push_back(ActionToInt(ka));
		}
	    
    //Place upright actions
    for (int h = 0 ; h < 2 ; h++)
	if (kitchenstate.GripperEmpty.at(index*2+h))
	    for (int o = 0 ; o < NumObjects ; o++)
		if (ObjectHere.at(o) && kitchenstate.IsToppled.at(o) &&
			ObjectTypes.at(o) != TRAY)
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    legal.push_back(ActionToInt(ka));
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
		    legal.push_back(ActionToInt(ka));
		}
    
    //Put in actions
    if (kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
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
			legal.push_back(ActionToInt(ka));
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
			legal.push_back(ActionToInt(ka));
		    }
		    

    //Grasp joint actions
    if (location == SIDEBOARD || location == STOVE)
	for (int i = 0; i < NumAgents; i++)
	   if (i != index && kitchenstate.RobotLocations[i] == location) 
		for (int h = 0 ; h < 2 ; h++)
		    if (kitchenstate.GripperEmpty.at(index*2+h))
			for (int o = 0 ; o < NumObjects ; o++)
			    if (ObjectTypes.at(o) == TRAY && ObjectHere.at(o))
			    {
				KitchenAction ka;
				ka.type = GRASP_JOINT;
				ka.location = location;
				ka.gripper = h == 0 ? LEFT : RIGHT;
				ka.objectindex = o;
				ka.objectclass = ObjectTypes[o];
				legal.push_back(ActionToInt(ka));
			    }
			    
    //Put down joint actions
    if (location == SIDEBOARD || location == STOVE)
	for (int i = 0; i < NumAgents; i++)
	   if (i != index && kitchenstate.RobotLocations[i] == location) 
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
			    legal.push_back(ActionToInt(ka));
			}
}


void KITCHEN::GeneratePreferred(const STATE& state, const HISTORY& history, 
				std::vector< int >& actions, const SIMULATOR::STATUS& status) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    for (int i = 0; i < NumAgents; i++)
	GeneratePreferredAgent(kitchenstate, history, actions, status, i);
}

void KITCHEN::GeneratePreferredAgent(const KITCHEN_STATE& kitchenstate, const HISTORY& history, 
				     std::vector< int >& actions, const SIMULATOR::STATUS& status, const int& index) const
{
    
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
	case(GRASP_JOINT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4;
	    action += object*NumLocations*2 + location*2 + gripper; 
	case(PUT_DOWN_JOINT):
	    action += NumLocations*2*4 + NumObjects*NumLocations*2*8 + NumLocations*NumLocations + NumObjects*4;
	    action += object*NumLocations*2 + location*2 + gripper; 
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
    else if (action < NumLocations*2*4 + NumObjects*NumLocations*2*8 + NumLocations*NumLocations + NumObjects*4)
    {
	ka.type = GRASP_JOINT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*7 + NumLocations*NumLocations + NumObjects*4);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
    }
    else
    {
	ka.type = PUT_DOWN_JOINT;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*8 + NumLocations*NumLocations + NumObjects*4);
	ka.objectclass = ObjectTypes[action/(NumLocations*2)];
	ka.objectindex = action/(NumLocations*2);
	ka.location = IntToLocation((action%(NumLocations*2))/2);
	ka.gripper = IntToGripper(action%2);
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
    if (NumAgents == 1)
	DisplayActionAgent(action, ostr);
    else
    {
	for (int i = 0 ; i < NumAgents ; i++)
	{
	    ostr << "Agent " << i << ":";
	    DisplayActionAgent(action/((int) pow(NumAgentActions, NumAgents-1-i)), ostr);
	    action = action%((int) pow(NumAgentActions, NumAgents-1-i));
	}
    }
}

void KITCHEN::DisplayActionAgent(int action, std::ostream& ostr) const
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
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << LocationToString(ka.location2) << ")\n";
	    break;
	case(PASS_OBJECT):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << GripperToString(ka.gripper) << "," << GripperToString(ka.gripper2) << ")\n";
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
	    ostr << "-" << GripperToString(kitchenstate.TraySecondGripper.second) << kitchenstate.TraySecondGripper.first;
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
    KitchenObservation ko = IntToObservation(observation);
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	ostr << ObjectToString(ObjectTypes[i]) << ":" << ko.objectvisible[i];
	if (i < NumObjects-1)
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
	case(GRASP_JOINT):
	    return "GRASP_JOINT";
	case(PUT_DOWN_JOINT):
	    return "PUT_DOWN_JOINT";
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
	default:
	    return "INVALID_OBJECT";
    }
}







