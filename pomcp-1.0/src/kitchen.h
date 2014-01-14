#ifndef KITCHEN_H
#define KITCHEN_H
#include "simulator.h"
#include <tr1/unordered_map>
#include <deque>


enum ObjectClass {APPLEJUICE = 0, CALGONIT, GRANINI, MEASURINGCUP, RICEBOX, CEREAL, PLATE, CUP, TRAY, NO_OBJECT};

enum LocationType {CUPBOARD = 10, DISHWASHER, FRIDGE, SIDEBOARD, STOVE, NO_LOCATION};

enum GripperType {LEFT = 16, RIGHT, NO_GRIPPER};

enum ActionType {CLOSE = 19, GRASP, GRASP_FROM_EDGE, MOVE_ROBOT, NUDGE, OPEN, OPEN_PARTIAL, OPEN_COMPLETE,
    PASS_OBJECT, PLACE_UPRIGHT, PUT_DOWN, PUT_IN, REMOVE_FROM, STAY_PUT, GRASP_JOINT, PUT_DOWN_JOINT, MOVE_ROBOT_JOINT};

//typedef std::tr1::unordered_map< ObjectClass, int, std::tr1::hash<int> >  hashmap;
    
struct KitchenAction
{
    ActionType type;
    ObjectClass objectclass;
    int objectindex;
    LocationType location;
    LocationType location2;
    GripperType gripper;
    GripperType gripper2;
};

struct KitchenObservation
{
    std::vector<bool> ObjectVisible;
    std::vector<bool> AgentVisible;
    
    LocationType OwnLocation;
    bool OwnLocationOpen;
    bool OwnLocationPartiallyOpen;
    std::vector<bool> AtEdge;
    std::vector<bool> IsToppled;
    int LeftGripperContent;
    int RightGripperContent;
    
};

class KITCHEN_STATE : public STATE
{
public:
    std::vector<LocationType> RobotLocations;
    std::vector<bool> AtEdge;
    std::vector<bool> GripperEmpty;
    std::vector< std::pair< int, GripperType > > InWhichGripper;
    std::pair< int, GripperType > TraySecondGripper;
    std::vector<LocationType> ObjectLocations;
    std::vector<bool> LocationOpen;
    std::vector<bool> LocationPartiallyOpen;
    std::vector<bool> IsToppled;
};


class KITCHEN : public SIMULATOR
{
public:
    
    KITCHEN(int nplates = 1, int ncups = 1);
    
    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward, STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const;
	
    virtual void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status) const;
    virtual void GenerateLegalAgent(const STATE& state, const HISTORY& history, 
	std::vector<int>& actions, const STATUS& status, const int& index) const;
    virtual void GeneratePreferredAgent(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status, const int& index) const;
	
    void GenerateAgentActions(const KITCHEN_STATE& kitchenstate, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status, const int& index, const bool& humanDefined) const;
	
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    
    virtual void DisplayAgentAction(int action, std::ostream& ostr) const;
    virtual void DisplayAgentObservation(int observation, std::ostream& ostr) const;
    
    virtual bool IsActionMultiagent(const int& action, const HISTORY& history) const;
    
protected:
    
    int NumPlates;
    int NumCups;
    int NumLocations;
    std::vector<ObjectClass> ObjectTypes;
    std::vector<bool> MultiAgentLabels;
    bool IsLocation(const int& l) const { return l >= CUPBOARD && l <= STOVE; }
    bool IsObject(const int& o) const { return o >= APPLEJUICE && o <= CUP; }
    bool IsFlat(const int& o) const { return o == PLATE; }
    bool IsHand(const int& h) const {return h == LEFT || h == RIGHT;}
    bool HaveAppleJuice, HaveCalgonit, HaveGranini, HaveMeasuringCup, HaveRiceBox, HaveCereal, HaveTray;
    int NumObjects;
    int LOCATION_OFFSET, GRIPPER_OFFSET, ACTION_OFFSET;
    bool Collision(const KITCHEN_STATE& state, const LocationType& location, const int& index) const;
    bool StepAgent(KITCHEN_STATE& kitchenstate, int action, 
        int& observation, double& reward, const int& index) const;
    
    int CerealIndex, Plate1Index, AppleJuiceIndex, TrayIndex;
    //std::tr1::unordered_map< int, int > ObjectIndexMap;
    
    int ActionToInt(const KitchenAction& ka) const;
    KitchenAction IntToAction(int action) const;
    LocationType IntToLocation(const int& i) const;
    GripperType IntToGripper(const int& i) const;
    std::string LocationToString(const LocationType& t) const;
    std::string ActionToString(const ActionType& t) const;
    std::string ObjectToString(const ObjectClass& t) const;
    std::string GripperToString(const GripperType& t) const;
    int MakeObservation(const KITCHEN_STATE& state, const int& index) const;
    int ObservationToInt(const KitchenObservation& ko) const;
    KitchenObservation IntToObservation(int observation) const;
    
    bool NonDeterministicActions;
    double ProbClose, ProbGrasp, ProbGrapsFromEdge, ProbMove, ProbNudge, ProbOpen, ProbOpenPartial,
	ProbOpenComplete, ProbPassObject, ProbPlaceUpright, ProbPutDown, ProbPutIn, ProbRemoveFrom,
	ProbGraspJoint, ProbPutDownJoint, ProbMoveJoint, ProbToppleOnFailPutDown, ProbToppleOnFailMove;
	
    double ProbObservation;
    
    //GOALS
    bool TestCerealInCupboard, TestPlate1InDishwasher, TestAppleJuiceInFridge;
    bool TestCerealAndAppleJuice, TestTrayAndPlate;
    bool TestTrayOnStove;
    
    bool IsCerealInCupboard(const KITCHEN_STATE& state, double& reward) const;
    bool IsPlate1InDishwasher(const KITCHEN_STATE& state, double& reward) const;
    bool IsAppleJuiceInFridge(const KITCHEN_STATE& state, double& reward) const;
    bool IsTrayOnStove(const KITCHEN_STATE& state, double& reward) const;
    bool IsCerealAndAppleJuice(const KITCHEN_STATE& state, double& reward) const;
    bool IsTrayAndPlate(const KITCHEN_STATE& state, double& reward) const;
    
    double MinReward, MaxReward;
    
    bool StartLocationsFixed;
    
private:
    mutable MEMORY_POOL<KITCHEN_STATE> MemoryPool;
    
};

#endif