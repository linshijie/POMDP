#include "kitchen.h"
#include <iomanip>

KITCHEN::KITCHEN(int nplates, int ncups): 
  NumPlates(nplates), NumCups(ncups), NumLocations(5),
  LOCATION_OFFSET(static_cast<int>(CUPBOARD)), GRIPPER_OFFSET(static_cast<int>(LEFT)),
  ACTION_OFFSET(static_cast<int>(CLOSE)),
  HaveAppleJuice(true), HaveCalgonit(false), HaveGranini(false),
  HaveMeasuringCup(false), HaveRiceBox(false), HaveCereal(true),
  TestCerealInCupboard(false), TestPlate1InDishwasher(false), TestAppleJuiceInFridge(true),
  NonDeterministicActions(true),
  ProbClose(0.95), ProbGrasp(0.95), ProbGrapsFromEdge(0.95), ProbMove(0.95), ProbNudge(0.95), ProbOpen(0.95),
  ProbOpenPartial(0.95), ProbOpenComplete(0.95), ProbPassObject(0.95), 
  ProbPlaceUpright(0.95), ProbPutDown(0.95), ProbPutIn(0.95), ProbRemoveFrom(0.95)
{
    NumObjects = NumPlates+NumCups;
    
    PreferredLocations.clear();
    PreferredObjects.clear();
    
    ObjectTypes.clear();
    int index = 0;
    if (HaveAppleJuice)
    {
	ObjectTypes.push_back(APPLEJUICE);
	NumObjects++;
	AppleJuiceIndex = index;
	if (TestAppleJuiceInFridge)
	    PreferredObjects.push_back(index);
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
	if (TestCerealInCupboard)
	    PreferredObjects.push_back(index);
	index++;
    }
    
    NumActions = NumObjects*(NumLocations*14+4) + 8*NumLocations + NumLocations*NumLocations;
    NumObservations = pow(2,NumObjects);
    
    for (int i=0; i<NumPlates; i++)
    {
	ObjectTypes.push_back(PLATE);
	if (i == 0)
	{
	    Plate1Index = index;
	    if (TestPlate1InDishwasher)
		PreferredObjects.push_back(index);
	}
	index++;
    }
    for (int i=0; i<NumCups; i++)
    {
	ObjectTypes.push_back(CUP);
	index++;
    }
	
    if (TestAppleJuiceInFridge)
	PreferredLocations.push_back(FRIDGE);
    if (TestCerealInCupboard)
	PreferredLocations.push_back(CUPBOARD);
    if (TestPlate1InDishwasher)
	PreferredLocations.push_back(DISHWASHER);
	
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

    kitchenstate->RobotLocation = SIDEBOARD;
    kitchenstate->InWhichGripper.clear();
    kitchenstate->AtEdge.clear();
    kitchenstate->IsToppled.clear();
    kitchenstate->ObjectLocations.clear();
    kitchenstate->LocationOpen.clear();
    kitchenstate->LocationPartiallyOpen.clear();
    kitchenstate->GripperEmpty.clear();
    
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
	kitchenstate->InWhichGripper.push_back(NO_GRIPPER);
    }
    
    for (int i = 0; i < NumLocations; i++)
    {
	kitchenstate->LocationOpen.push_back(false);
	kitchenstate->LocationPartiallyOpen.push_back(false);
    }
    kitchenstate->GripperEmpty.push_back(true);
    kitchenstate->GripperEmpty.push_back(true);
    
    return kitchenstate;
}

bool KITCHEN::Step(STATE& state, int action, int& observation, double& reward) const
{
    KITCHEN_STATE& kitchenstate = safe_cast<KITCHEN_STATE&>(state);
    
    KitchenAction ka = IntToAction(action);
    
    reward = 0.1;
    
    switch(ka.type)
    {
	case(CLOSE):
	    if (kitchenstate.RobotLocation == ka.location && 
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] &&
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
	    if (kitchenstate.RobotLocation == ka.location && !kitchenstate.IsToppled[ka.objectindex] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location && 
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbGrasp)
		{
		    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper;
		    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = false;
		    kitchenstate.ObjectLocations[ka.objectindex] = NO_LOCATION;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(GRASP_FROM_EDGE):
	    if (kitchenstate.RobotLocation == ka.location && kitchenstate.AtEdge[ka.objectindex] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbGrapsFromEdge)
		{
		    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper;
		    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = false;
		    kitchenstate.AtEdge[ka.objectindex] = false;
		    kitchenstate.ObjectLocations[ka.objectindex] = NO_LOCATION;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(MOVE_ROBOT):
	    if (kitchenstate.RobotLocation == ka.location && kitchenstate.RobotLocation != ka.location2
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbMove)
		    kitchenstate.RobotLocation = ka.location2;
	    }
	    else
		reward = -5.0;
	    break;
	case(NUDGE):
	    if (kitchenstate.RobotLocation == ka.location && !kitchenstate.AtEdge[ka.objectindex] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbNudge)
		    kitchenstate.AtEdge[ka.objectindex] = true;
	    }
	    else
		reward = -5.0;    
	    break;
	case(OPEN):
	    if (kitchenstate.RobotLocation == ka.location && !kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbOpen)
		    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = true;
	    }
	    else
		reward = -5.0;
	    break;
	case(OPEN_PARTIAL):
	    if (kitchenstate.RobotLocation == ka.location && !kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		!kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] && 
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbOpenPartial)
		    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = true;
	    }
	    else
		reward = -5.0;
	    break;
	case(OPEN_COMPLETE):
	    if (kitchenstate.RobotLocation == ka.location && !kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] && 
		kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
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
	    if (kitchenstate.InWhichGripper[ka.objectindex] == ka.gripper &&
		kitchenstate.GripperEmpty[ka.gripper2-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPassObject)
		{
		    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = true;
		    kitchenstate.GripperEmpty[ka.gripper2-GRIPPER_OFFSET] = false;
		    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper2;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(PLACE_UPRIGHT):
	    if (kitchenstate.RobotLocation == ka.location && 
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		kitchenstate.IsToppled[ka.objectindex] && kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPlaceUpright)
		    kitchenstate.IsToppled[ka.objectindex] = false;
	    }
	    else
		reward = -5.0;
	    break;
	case(PUT_DOWN):
	    if (kitchenstate.RobotLocation == ka.location && 
		kitchenstate.InWhichGripper[ka.objectindex] == ka.gripper
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPutDown)
		{
		    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = true;
		    kitchenstate.ObjectLocations[ka.objectindex] = ka.location;
		    kitchenstate.InWhichGripper[ka.objectindex] = NO_GRIPPER;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(PUT_IN):
	    if (kitchenstate.RobotLocation == ka.location && kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.InWhichGripper[ka.objectindex] == ka.gripper
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbPutIn)
		{
		    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = true;
		    kitchenstate.ObjectLocations[ka.objectindex] = ka.location;
		    kitchenstate.InWhichGripper[ka.objectindex] = NO_GRIPPER;
		}
	    }
	    else
		reward = -5.0;
	    break;
	case(REMOVE_FROM):
	    if (kitchenstate.RobotLocation == ka.location && kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] &&
		kitchenstate.ObjectLocations[ka.objectindex] == ka.location &&
		!kitchenstate.IsToppled[ka.objectindex] && kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET]
	    )
	    {
		if (!NonDeterministicActions || UTILS::RandomDouble(0.0,1.0) < ProbRemoveFrom)
		{
		    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper;
		    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = false;
		    kitchenstate.ObjectLocations[ka.objectindex] = NO_LOCATION;
		}
	    }
	    else
		reward = -5.0;
	    break;
	default:
	    break;
    }
    
    observation = MakeObservation(kitchenstate);
    
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

int KITCHEN::MakeObservation(const KITCHEN_STATE& state) const
{
    KitchenObservation ko;
    ko.objectvisible.clear();
    
    for (int i = 0 ; i < NumObjects ; i++)
    {
	if (state.ObjectLocations[i] == NO_LOCATION || (state.ObjectLocations[i] == state.RobotLocation &&
	    (state.RobotLocation == SIDEBOARD || state.RobotLocation == STOVE ||
	    state.LocationOpen[state.RobotLocation-LOCATION_OFFSET]))
	    )
	    ko.objectvisible.push_back(true);
	else
	    ko.objectvisible.push_back(false);
    }
    //ko.location = state.RobotLocation;
    
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
    
    //observation += pow(2,NumObjects)*(ko.location-LOCATION_OFFSET);
    
    return observation;
}

KitchenObservation KITCHEN::IntToObservation(int observation) const
{
    KitchenObservation ko;
    ko.objectvisible.clear();
    ko.location = NO_LOCATION;
    
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

void KITCHEN::GenerateLegal(const STATE& state, const HISTORY& history, std::vector< int >& legal, 
			    const SIMULATOR::STATUS& status) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    LocationType location = kitchenstate.RobotLocation;
    
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
	    (location == CUPBOARD && kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET)) || 
	    (location == DISHWASHER && kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET)) ||
	    (location == FRIDGE && kitchenstate.GripperEmpty.at(LEFT-GRIPPER_OFFSET))
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
	    if (kitchenstate.GripperEmpty.at(h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (!IsFlat(ObjectTypes.at(o)) && !kitchenstate.IsToppled.at(o) && ObjectHere.at(o))
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
	    if (kitchenstate.GripperEmpty.at(h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && kitchenstate.AtEdge.at(o) && ObjectHere.at(o))
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
	    KitchenAction ka;
	    ka.type = MOVE_ROBOT;
	    ka.location = location;
	    ka.location2 = static_cast<LocationType>(i);
	    legal.push_back(ActionToInt(ka));
	}
		    
    //Nudge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && !kitchenstate.AtEdge.at(o) && ObjectHere.at(o))
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
	kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN;
	ka.location = location;
	ka.gripper = RIGHT;
	legal.push_back(ActionToInt(ka));
    }
    
    //Open partial actions
    if (location == FRIDGE && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	!kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && kitchenstate.GripperEmpty.at(LEFT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN_PARTIAL;
	ka.location = location;
	ka.gripper = LEFT;
	legal.push_back(ActionToInt(ka));
    }
    
    //Open complete actions
    if (location == FRIDGE && !kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN_COMPLETE;
	ka.location = location;
	ka.gripper = RIGHT;
	legal.push_back(ActionToInt(ka));
    }
    
    //Pass object actions
    for (int o = 0 ; o < NumObjects ; o++)
	for (int h1 = 0 ; h1 < 2 ; h1++)
	    if (kitchenstate.InWhichGripper.at(o) == h1+GRIPPER_OFFSET && kitchenstate.GripperEmpty.at((h1+1)%2))
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
	if (kitchenstate.GripperEmpty.at(h))
	    for (int o = 0 ; o < NumObjects ; o++)
		if (ObjectHere.at(o) && kitchenstate.IsToppled.at(o))
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
		if (kitchenstate.InWhichGripper.at(o) == h+GRIPPER_OFFSET)
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
		    if (kitchenstate.InWhichGripper.at(o) == h+GRIPPER_OFFSET)
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
	    if (kitchenstate.GripperEmpty.at(h) && (location == CUPBOARD || (location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (ObjectHere.at(o) && !kitchenstate.IsToppled.at(o))
		    {
			KitchenAction ka;
			ka.type = REMOVE_FROM;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			legal.push_back(ActionToInt(ka));
		    }
		    
}

void KITCHEN::GeneratePreferred(const STATE& state, const HISTORY& history, std::vector< int >& actions, const SIMULATOR::STATUS& status) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    actions.clear();
    
    LocationType location = kitchenstate.RobotLocation;
    
    KitchenAction lastaction;
    if (history.Size() > 0)
	lastaction = IntToAction(history.Back().Action);
    
    //Close actions
    if ( (history.Size() == 0 || (lastaction.type != OPEN && lastaction.type != OPEN_PARTIAL && lastaction.type != OPEN_COMPLETE)) && (
	    (location == CUPBOARD && kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET)) || 
	    (location == DISHWASHER && kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET)) ||
	    (location == FRIDGE && kitchenstate.GripperEmpty.at(LEFT-GRIPPER_OFFSET))
    ) && 
	(kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) || kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET)))
    {
	KitchenAction ka;
	ka.type = CLOSE;
	ka.location = location;
	ka.gripper = location == FRIDGE ? LEFT : RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Grasp actions
    if ((history.Size() == 0 || lastaction.type != PUT_DOWN) && (location == SIDEBOARD || location == STOVE))
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(h))
		for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
		{
		    int o = PreferredObjects.at(i);
		    if (!IsFlat(ObjectTypes.at(o)) && !kitchenstate.IsToppled.at(o) && kitchenstate.ObjectLocations.at(o) == location)
		    {
			KitchenAction ka;
			ka.type = GRASP;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		}
		    
    //Grasp from edge actions
    if ((history.Size() == 0 || lastaction.type == NUDGE) && (location == SIDEBOARD || location == STOVE))
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(h))
		for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
		{
		    int o = PreferredObjects.at(i);
		    if (IsFlat(ObjectTypes.at(o)) && kitchenstate.AtEdge.at(o) && kitchenstate.ObjectLocations.at(o) == location)
		    {
			KitchenAction ka;
			ka.type = GRASP_FROM_EDGE;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		}
		    
    //Move actions
    bool shouldmove = false;
    for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
    {
	if (kitchenstate.InWhichGripper[PreferredObjects[i]] != NO_GRIPPER)
	{
	    shouldmove = true;
	    break;
	}
    }
    
    if (shouldmove)
    for (int i = 0 ; i < (int)PreferredLocations.size() ; i++)
	if (PreferredLocations.at(i) != location)
	{
	    KitchenAction ka;
	    ka.type = MOVE_ROBOT;
	    ka.location = location;
	    ka.location2 = PreferredLocations.at(i);
	    actions.push_back(ActionToInt(ka));
	}
		    
    //Nudge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(h))
		for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
		{
		    int o = PreferredObjects.at(i);
		    if (IsFlat(ObjectTypes.at(o)) && !kitchenstate.AtEdge.at(o) && 
			kitchenstate.ObjectLocations.at(o) == location)
		    {
			KitchenAction ka;
			ka.type = NUDGE;
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			actions.push_back(ActionToInt(ka));
		    }
		}
		    
    //Open actions
    if ((history.Size() == 0 || lastaction.type != CLOSE) && (location == CUPBOARD || location == DISHWASHER) && 
	!kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN;
	ka.location = location;
	ka.gripper = RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Open partial actions
    if ((history.Size() == 0 || lastaction.type != CLOSE) &&  location == FRIDGE && 
	!kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	!kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.GripperEmpty.at(LEFT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN_PARTIAL;
	ka.location = location;
	ka.gripper = LEFT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Open complete actions
    if ((history.Size() == 0 || lastaction.type == OPEN_PARTIAL) &&  location == FRIDGE && 
	!kitchenstate.LocationOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.LocationPartiallyOpen.at(location-LOCATION_OFFSET) && 
	kitchenstate.GripperEmpty.at(RIGHT-GRIPPER_OFFSET))
    {
	KitchenAction ka;
	ka.type = OPEN_COMPLETE;
	ka.location = location;
	ka.gripper = RIGHT;
	actions.push_back(ActionToInt(ka));
    }
    
    //Pass object actions
    for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
    {
	int o = PreferredObjects.at(i);
	for (int h1 = 0 ; h1 < 2 ; h1++)
	    if (kitchenstate.InWhichGripper.at(o) == h1+GRIPPER_OFFSET && kitchenstate.GripperEmpty.at((h1+1)%2))
	    {
		KitchenAction ka;
		ka.type = PASS_OBJECT;
		ka.objectindex = o;
		ka.objectclass = ObjectTypes[o];
		ka.gripper = static_cast<GripperType>(h1 + GRIPPER_OFFSET);
		ka.gripper2 = static_cast<GripperType>((h1+1)%2 + GRIPPER_OFFSET);
		actions.push_back(ActionToInt(ka));
	    }
    }
	    
    //Place upright actions
    for (int h = 0 ; h < 2 ; h++)
	if (kitchenstate.GripperEmpty.at(h))
	    for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
	    {
		int o = PreferredObjects.at(i);
		if (kitchenstate.ObjectLocations.at(o) == location && kitchenstate.IsToppled.at(o))
		{
		    KitchenAction ka;
		    ka.type = PLACE_UPRIGHT;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
	    }
		
    //Put down actions
    if ((history.Size() == 0 || (lastaction.type != GRASP && lastaction.type != GRASP_FROM_EDGE)) &&  
	(location == SIDEBOARD || location == STOVE))
	for (int h = 0 ; h < 2 ; h++)
	    for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
	    {
		int o = PreferredObjects.at(i);
		if (kitchenstate.InWhichGripper.at(o) == h+GRIPPER_OFFSET)
		{
		    KitchenAction ka;
		    ka.type = PUT_DOWN;
		    ka.objectindex = o;
		    ka.objectclass = ObjectTypes[o];
		    ka.location = location;
		    ka.gripper = h == 0 ? LEFT : RIGHT;
		    actions.push_back(ActionToInt(ka));
		}
	    }
    
    //Put in actions
    if ((history.Size() == 0 || lastaction.type != REMOVE_FROM) && kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
	for (int h = 0 ; h < 2 ; h++)
	    if ((location == CUPBOARD || (location == DISHWASHER && h == RIGHT-GRIPPER_OFFSET) || 
		(location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
		{
		    int o = PreferredObjects.at(i);
		    if (kitchenstate.InWhichGripper.at(o) == h+GRIPPER_OFFSET)
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
    if ((history.Size() == 0 || lastaction.type != PUT_IN) && kitchenstate.LocationOpen.at(location-LOCATION_OFFSET))
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(h) && 
		(location == CUPBOARD || (location == FRIDGE && h == LEFT-GRIPPER_OFFSET)))
		for (int i = 0 ; i < (int)PreferredObjects.size() ; i++)
		{
		    int o = PreferredObjects.at(i);
		    if (kitchenstate.ObjectLocations.at(o) == location && !kitchenstate.IsToppled.at(o))
		    {
			KitchenAction ka;
			ka.type = REMOVE_FROM;
			ka.objectindex = o;
			ka.objectclass = ObjectTypes[o];
			ka.location = location;
			ka.gripper = h == 0 ? LEFT : RIGHT;
			actions.push_back(ActionToInt(ka));
		    }
		}
		
    
    //std::cout << "Num actions " << actions.size() << std::endl;
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
	default:
	    break;
	    
    }
    
    //std::cout << ka.type << std::endl;
    //std::cout << action << std::endl;
    
    return action;
}

KitchenAction KITCHEN::IntToAction(int action) const
{
    assert(action >= 0 && action < NumObjects*(NumLocations*14+4) + 8*NumLocations + NumLocations*NumLocations);
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
    else
    {
	ka.type = REMOVE_FROM;
	action -= (NumLocations*2*4 + NumObjects*NumLocations*2*6 + NumLocations*NumLocations + NumObjects*4);
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
    KitchenAction ka = IntToAction(action);
    
    switch(ka.type)
    {
	case(CLOSE):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << GripperToString(ka.gripper) << ")\n";
	    break;
	case(GRASP):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(GRASP_FROM_EDGE):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(MOVE_ROBOT):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << LocationToString(ka.location2) << ")\n";
	    break;
	case(NUDGE):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(OPEN):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << GripperToString(ka.gripper) << ")\n";
	    break;
	case(OPEN_PARTIAL):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << GripperToString(ka.gripper) << ")\n";
	    break;
	case(OPEN_COMPLETE):
	    ostr << ActionToString(ka.type) << "("  << LocationToString(ka.location) << ","
		    << GripperToString(ka.gripper) << ")\n";
	    break;
	case(PASS_OBJECT):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << GripperToString(ka.gripper) << "," << GripperToString(ka.gripper2) << ")\n";
	    break;
	case(PLACE_UPRIGHT):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(PUT_DOWN):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(PUT_IN):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	case(REMOVE_FROM):
	    ostr << ActionToString(ka.type) << "("  << ObjectToString(ka.objectclass) << ","
		    << LocationToString(ka.location) << "," << GripperToString(ka.gripper) << ")\n";
	    break;
	default:
	    break;
    }
}

void KITCHEN::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    ostr << "\n";
    
    ostr << "Robot : " << LocationToString(kitchenstate.RobotLocation) << "," <<
	    (kitchenstate.GripperEmpty[0] ? "LEFT_EMPTY" : "LEFT_FULL") << "," <<
	    (kitchenstate.GripperEmpty[1] ? "RIGHT_EMPTY" : "RIGHT_FULL") <<"\n";
    for (int i = 0 ; i < NumObjects ; i++)
    {
	ostr << ObjectToString(ObjectTypes[i]) << " : " << 
	    LocationToString(kitchenstate.ObjectLocations[i]) << "," << 
	    GripperToString(kitchenstate.InWhichGripper[i]) << "," <<
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
	default:
	    return "INVALID_OBJECT";
    }
}







