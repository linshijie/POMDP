#include "kitchen.h"
#include <iomanip>

KITCHEN::KITCHEN(int nplates, int ncups): 
  NumPlates(nplates),
  NumCups(ncups),
  NumLocations(5),
  NumActions(13),
  HaveAppleJuice(true),
  HaveCalgonit(false),
  HaveGranini(false),
  HaveMeasuringCup(false),
  HaveRiceBox(false),
  HaveCereal(true)
{
    NumObjects = NumPlates+NumCups;
    
    LOCATION_OFFSET = static_cast<int>(CUPBOARD);
    GRIPPER_OFFSET = static_cast<int>(LEFT);
    ACTION_OFFSET = static_cast<int>(CLOSE);
    
    ObjectTypes.clear();
    //int index = 0;
    if (HaveAppleJuice)
    {
	ObjectTypes.push_back(APPLEJUICE);
	NumObjects++;
	//ObjectIndexMap[APPLEJUICE] = index;
	//index++;
    }
    if (HaveCalgonit)
    {
	ObjectTypes.push_back(CALGONIT);
	NumObjects++;
	//ObjectIndexMap[CALGONIT] = index;
	//index++;
    }
    if (HaveGranini)
    {
	ObjectTypes.push_back(GRANINI);
	NumObjects++;
	//ObjectIndexMap[GRANINI] = index;
	//index++;
    }
    if (HaveMeasuringCup)
    {
	ObjectTypes.push_back(MEASURINGCUP);
	NumObjects++;
	//ObjectIndexMap[MEASURINGCUP] = index;
	//index++;
    }
    if (HaveRiceBox)
    {
	ObjectTypes.push_back(RICEBOX);
	NumObjects++;
	//ObjectIndexMap[RICEBOX] = index;
	//index++;
    }
    if (HaveCereal)
    {
	ObjectTypes.push_back(CEREAL);
	NumObjects++;
	//ObjectIndexMap[CEREAL] = index;
	//index++;
    }
    NumActions = NumObjects*74 + 8*NumLocations + NumLocations*NumLocations;
    NumObservations = pow(2,NumObjects);
    for (int i=0; i<NumPlates; i++)
	ObjectTypes.push_back(PLATE);
    //if (NumPlates > 0)
    //{
	//ObjectIndexMap[PLATE] = index;
	//index++;
    //}
    for (int i=0; i<NumCups; i++)
	ObjectTypes.push_back(CUP);
    //if (NumCups > 0)
	//ObjectIndexMap[CUP] = index;
    RewardRange = 99.8;
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
	kitchenstate->InWhichGripper.push_back(NONE);
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
    
    switch(ka.type)
    {
	case(CLOSE):
	    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = false;
	    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = false;
	    break;
	case(GRASP):
	    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper;
	    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = false;
	    break;
	case(GRASP_FROM_EDGE):
	    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper;
	    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = false;
	    kitchenstate.AtEdge[ka.objectindex] = false;
	    break;
	case(MOVE):
	    kitchenstate.RobotLocation = ka.location2;
	    break;
	case(NUDGE):
	    kitchenstate.AtEdge[ka.objectindex] = true;
	    break;
	case(OPEN):
	    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = true;
	    break;
	case(OPEN_PARTIAL):
	    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = true;
	    break;
	case(OPEN_COMPLETE):
	    kitchenstate.LocationOpen[ka.location-LOCATION_OFFSET] = true;
	    kitchenstate.LocationPartiallyOpen[ka.location-LOCATION_OFFSET] = false;
	    break;
	case(PASS_OBJECT):
	    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = true;
	    kitchenstate.GripperEmpty[ka.gripper2-GRIPPER_OFFSET] = false;
	    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper2;
	    break;
	case(PLACE_UPRIGHT):
	    kitchenstate.IsToppled[ka.objectindex] = false;
	    break;
	case(PUT_DOWN):
	    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = true;
	    kitchenstate.ObjectLocations[ka.objectindex] = ka.location;
	    kitchenstate.InWhichGripper[ka.objectindex] = NONE;
	    break;
	case(PUT_IN):
	    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = true;
	    kitchenstate.ObjectLocations[ka.objectindex] = ka.location;
	    kitchenstate.InWhichGripper[ka.objectindex] = NONE;
	    break;
	case(REMOVE_FROM):
	    kitchenstate.InWhichGripper[ka.objectindex] = ka.gripper;
	    kitchenstate.GripperEmpty[ka.gripper-GRIPPER_OFFSET] = false;
	    break;
	default:
	    break;
    }
    
    return false;
}

void KITCHEN::GenerateLegal(const STATE& state, const HISTORY& history, std::vector< int >& legal, 
			    const SIMULATOR::STATUS& status) const
{
    const KITCHEN_STATE& kitchenstate = safe_cast<const KITCHEN_STATE&>(state);
    
    LocationType location = kitchenstate.RobotLocation;
    
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
		    if (!IsFlat(ObjectTypes.at(o)) && !kitchenstate.IsToppled.at(o) && kitchenstate.ObjectLocations.at(o) == location)
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
		    if (IsFlat(ObjectTypes.at(o)) && kitchenstate.AtEdge.at(o) && kitchenstate.ObjectLocations.at(o) == location)
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
	    ka.type = MOVE;
	    ka.location = location;
	    ka.location2 = static_cast<LocationType>(i);
	    legal.push_back(ActionToInt(ka));
	}
		    
    //Nudge actions
    if (location == SIDEBOARD || location == STOVE)
	for (int h = 0 ; h < 2 ; h++)
	    if (kitchenstate.GripperEmpty.at(h))
		for (int o = 0 ; o < NumObjects ; o++)
		    if (IsFlat(ObjectTypes.at(o)) && !kitchenstate.AtEdge.at(o) && kitchenstate.ObjectLocations.at(o) == location)
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
		if (kitchenstate.ObjectLocations.at(o) == location && kitchenstate.IsToppled.at(o))
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
		    if (!kitchenstate.InWhichGripper.at(o) == h+GRIPPER_OFFSET)
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
		    if (kitchenstate.ObjectLocations.at(o) == location && !kitchenstate.IsToppled.at(o))
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
	case(MOVE):
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
    
    return action;
}

KitchenAction KITCHEN::IntToAction(int action) const
{
    assert(action >= 0 && action < NumObjects*74 + 8*NumLocations + NumLocations*NumLocations);
    KitchenAction ka;
    
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
	ka.type = MOVE;
	action -= (NumLocations*2 + NumObjects*NumLocations*2*2);
	ka.location = IntToLocation(action/2);
	ka.location2 = IntToLocation(action%2);
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






