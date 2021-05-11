using MMICoSimulation;
using MMICSharp.MMIStandard.Utils;
using MMIStandard;
using MMIUnity.TargetEngine;
using MMIUnity.TargetEngine.Scene;
using UnityEngine;

public class HumanBehavior : AvatarBehavior
{
    private string carryID;

    private readonly string MOTION_CARRY = "Object/Carry";
    private readonly string MOTION_GAZE = "Pose/Gaze";
    private readonly string MOTION_IDLE = "Pose/Idle";
    private readonly string MOTION_MOVEFINGERS = "Pose/MoveFingers";
    private readonly string MOTION_MOVE = "Object/Move";
    private readonly string MOTION_REACH = "Pose/Reach";
    private readonly string MOTION_RELEASE = "Object/Release";
    private readonly string MOTION_SIMPLE = "Object/Test";
    private readonly string MOTION_TURN = "Object/Turn";
    private readonly string MOTION_WALK = "Locomotion/Walk";

    // Start is called before the first frame update
    protected override void GUIBehaviorInput()
    {

    }

    public void SetIdle()
    {
        MInstruction instruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);
        MSimulationState simstate = new MSimulationState(this.avatar.GetPosture(), this.avatar.GetPosture());

        this.CoSimulator.Abort();
        this.CoSimulator.AssignInstruction(instruction, simstate);
    }

    public string PickPlace(string pick, string place, string hand, string LastActionID)
    {
        GameObject pickGO = GameObject.Find(pick);
        string pickPose = pickGO.transform.GetChild(0).name;

        MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);

        MInstruction reachInstruction;
        if(LastActionID != null)
        {
            reachInstruction = new MInstruction(MInstructionFactory.GenerateID(), "reach", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance[pickPose].ID, "Hand", hand, "MinDistance", 2.0.ToString()),
                StartCondition = LastActionID + ":" + mmiConstants.MSimulationEvent_End + "+ 1.0"
            };
        }
        else {
            reachInstruction = new MInstruction(MInstructionFactory.GenerateID(), "reach", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance[pickPose].ID, "Hand", hand, "MinDistance", 2.0.ToString()),
            };
        }

        var carryID = MInstructionFactory.GenerateID();
        MInstruction carryInstruction = new MInstruction(carryID, "carry object", MOTION_CARRY)
        {
            Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance[pick].ID, "Hand", hand, "AddOffset", false.ToString()),
            StartCondition = reachInstruction.ID + ":" + mmiConstants.MSimulationEvent_End + "+ 0.25",
        };

        MInstruction moveObject = new MInstruction(MInstructionFactory.GenerateID(), "move object", MOTION_MOVE)
        {
            Properties = PropertiesCreator.Create("SubjectID", UnitySceneAccess.Instance[pick].ID, 
                                                  "Hand", hand, 
                                                  "TargetID", UnitySceneAccess.Instance[place].ID, 
                                                  CoSimTopic.OnStart, carryID + ":" + CoSimAction.EndInstruction),
            StartCondition = carryInstruction.ID + ":" + "PositioningFinished"
        };

        MInstruction releaseRight = new MInstruction(MInstructionFactory.GenerateID(), "release object", MOTION_RELEASE)
        {
            Properties = PropertiesCreator.Create("Hand", hand),
            StartCondition = moveObject.ID + ":" + mmiConstants.MSimulationEvent_End + "+ 0.5",
        };

        this.CoSimulator.AssignInstruction(idleInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        this.CoSimulator.AssignInstruction(reachInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        this.CoSimulator.AssignInstruction(carryInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        this.CoSimulator.AssignInstruction(moveObject, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        this.CoSimulator.AssignInstruction(releaseRight, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        this.CoSimulator.MSimulationEventHandler += this.CoSimulator_MSimulationEventHandler;
        //this.CoSimulator.Abort();

        return releaseRight.ID;
    }

    public void DoTaskSequential()
    {
        string lastActionID = null;
        lastActionID = PickPlace("A", "RedPlacementA", "Left", lastActionID);
        lastActionID = PickPlace("H", "MiddlePlacement", "Right", lastActionID);
        lastActionID = PickPlace("L", "RedPlacementL", "Left", lastActionID);
        lastActionID = PickPlace("H", "RedPlacementH", "Left", lastActionID);
        lastActionID = PickPlace("F", "MiddlePlacement", "Right", lastActionID);
        lastActionID = PickPlace("D", "RedPlacementD", "Left", lastActionID);
        lastActionID = PickPlace("F", "RedPlacementF", "Left", lastActionID);
        lastActionID = PickPlace("B", "MiddlePlacement", "Right", lastActionID);
        lastActionID = PickPlace("B", "RedPlacementB", "Left", lastActionID);

    }

    private void CoSimulator_MSimulationEventHandler(object sender, MSimulationEvent e)
    {
        Debug.Log(e.Reference + " " + e.Name + " " + e.Type);
    }

}
