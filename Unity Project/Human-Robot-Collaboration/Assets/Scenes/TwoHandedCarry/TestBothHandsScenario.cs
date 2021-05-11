using MMICoSimulation;
using MMICSharp.MMIStandard.Utils;
using MMIStandard;
using MMIUnity.TargetEngine;
using MMIUnity.TargetEngine.Scene;
using UnityEngine;

public class TestBothHandsScenario : AvatarBehavior
{

    private string carryID;
    private readonly string MOTION_CARRY ="Object/Carry";
    private readonly string MOTION_GAZE = "Pose/Gaze";
    private readonly string MOTION_IDLE = "Pose/Idle";
    private readonly string MOTION_MOVEFINGERS = "Pose/MoveFingers";
    private readonly string MOTION_MOVE= "Object/Move";
    private readonly string MOTION_REACH = "Pose/Reach";
    private readonly string MOTION_RELEASE = "Object/Release";
    private readonly string MOTION_SIMPLE = "Object/Test";
    private readonly string MOTION_TURN = "Object/Turn";
    private readonly string MOTION_WALK = "Locomotion/Walk";
protected override void GUIBehaviorInput()
{
        if (GUI.Button(new Rect(10, 10, 220, 50), "Pickup large object"))
        {
            MInstruction walkInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Walk", MOTION_WALK)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance.GetSceneObjectByName("WalkTargetLargeObject").ID)
            };

            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE)
            {
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            MInstruction reachLeft = new MInstruction(MInstructionFactory.GenerateID(), "reachLeft", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["LargeObjectGraspL"].ID, "Hand", "Left"),
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            MInstruction reachRight = new MInstruction(MInstructionFactory.GenerateID(), "reachRight", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["LargeObjectGraspR"].ID, "Hand", "Right"),
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            carryID = MInstructionFactory.GenerateID();
            MInstruction carryInstruction = new MInstruction(carryID, "carry object", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["LargeObject"].ID, "Hand", "Both", "CarryDistance", 0.4f.ToString(), "CarryTarget", UnitySceneAccess.Instance["CarryTarget"].ID),
                StartCondition = reachLeft.ID + ":" + mmiConstants.MSimulationEvent_End + " && " + reachRight.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            this.CoSimulator.AssignInstruction(walkInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(idleInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(reachLeft, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(reachRight, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });

            this.CoSimulator.AssignInstruction(carryInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.MSimulationEventHandler += this.CoSimulator_MSimulationEventHandler;
        }

        if (GUI.Button(new Rect(240, 10, 220, 50), "Move Large Object both handed"))
        {
            MInstruction moveInstruction = new MInstruction(MInstructionFactory.GenerateID(), "move object", MOTION_MOVE)
            {
                Properties = PropertiesCreator.Create("SubjectID", UnitySceneAccess.Instance["LargeObject"].ID, "Hand", "Both", "TargetID", UnitySceneAccess.Instance["LargeObjectPositioningTarget"].ID, "HoldDuration", 1.0f.ToString()),

                //Terminate the carry
                Action = CoSimTopic.OnStart + "->" + carryID + ":" + CoSimAction.EndInstruction,
            };

            MInstruction releaseLeft = new MInstruction(MInstructionFactory.GenerateID(), "release object left", MOTION_RELEASE)
            {
                Properties = PropertiesCreator.Create("Hand", "Left"),
                StartCondition = moveInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            MInstruction releaseRight = new MInstruction(MInstructionFactory.GenerateID(), "release object right", MOTION_RELEASE)
            {
                Properties = PropertiesCreator.Create("Hand", "Right"),
                StartCondition = moveInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };


            this.CoSimulator.AssignInstruction(moveInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(releaseLeft, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(releaseRight, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });


        }

        if (GUI.Button(new Rect(470, 10, 220, 50), "Carry Object both handed"))
        {
            carryID = MInstructionFactory.GenerateID();
            MInstruction carryInstruction = new MInstruction(carryID, "carry object", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Both", "CarryTarget", UnitySceneAccess.Instance["CarryTarget"].ID),
            };

            this.CoSimulator.AssignInstruction(carryInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        }
        // if (GUI.Button(new Rect(920, 70, 160, 50), "Abort"))
        // {
        //     this.CoSimulator.Abort();
        // }

}




    /// <summary>
    /// Callback for the co-simulation event handler
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    private void CoSimulator_MSimulationEventHandler(object sender, MSimulationEvent e)
    {
        Debug.Log(e.Reference + " " + e.Name + " " + e.Type);
    }
}
