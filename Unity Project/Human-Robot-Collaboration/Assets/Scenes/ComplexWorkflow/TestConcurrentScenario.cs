using MMICoSimulation;
using MMICSharp.MMIStandard.Utils;
using MMIStandard;
using MMIUnity.TargetEngine;
using MMIUnity.TargetEngine.Scene;
using UnityEngine;


public class TestConcurrentScenario : AvatarBehavior
{
    private string carryID;

    // Motion Types
    private readonly string MOTION_CARRY ="Object/Carry";
    private readonly string MOTION_IDLE = "Pose/Idle";
    private readonly string MOTION_MOVE= "Object/Move";
    private readonly string MOTION_REACH = "Pose/Reach";
    private readonly string MOTION_RELEASE = "Object/Release";
    private readonly string MOTION_WALK = "Locomotion/Walk";

    protected override void GUIBehaviorInput()
    {
        if (GUI.Button(new Rect(10, 10, 160, 50), "Concurrent scenario"))
        {

            MInstruction walkInstruction1 = new MInstruction(MInstructionFactory.GenerateID(), "Walk", MOTION_WALK)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance.GetSceneObjectByName("WalkTargetConcurrent").ID),
            };

            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE)
            {
                StartCondition = walkInstruction1.ID + ":" + mmiConstants.MSimulationEvent_End
            };


            MInstruction reachRight = new MInstruction(MInstructionFactory.GenerateID(), "reach right", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspTarget2R"].ID, "Hand", "Right"),
                StartCondition = walkInstruction1.ID + ":" + mmiConstants.MSimulationEvent_End
            };


            MInstruction reachLeft = new MInstruction(MInstructionFactory.GenerateID(), "reach left", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspTargetL"].ID, "Hand", "Left"),
                StartCondition = walkInstruction1.ID + ":" + mmiConstants.MSimulationEvent_End
            };


            MInstruction carryLeft = new MInstruction(MInstructionFactory.GenerateID(), "carry left", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Left", "AddOffset", false.ToString()),
                StartCondition = reachLeft.ID + ":" + mmiConstants.MSimulationEvent_End + "+ 0.01"
            };

            MInstruction carryRight = new MInstruction(MInstructionFactory.GenerateID(), "carry right", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject2"].ID, "Hand", "Right" ,"AddOffset", false.ToString()),
                StartCondition = reachRight.ID + ":" + mmiConstants.MSimulationEvent_End + "+ 0.01"
            };

            MInstruction walkInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Walk", MOTION_WALK)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance.GetSceneObjectByName("WalkTarget2").ID),
                //Both carries must me finished with positioning
                StartCondition = carryRight.ID + ":" + "PositioningFinished" +" && "+ carryLeft.ID +":" + "PositioningFinished",

                //Finish the idle instruction
                Action = CoSimTopic.OnStart +"->"+ idleInstruction.ID+":"+ CoSimAction.EndInstruction,
            };




            MInstruction idleInstruction2 = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE)
            {
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End,
            };


            MInstruction moveRight = new MInstruction(MInstructionFactory.GenerateID(), "move right", MOTION_MOVE)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["PositioningTargetRight"].ID,  "SubjectID", UnitySceneAccess.Instance["GraspObject2"].ID, "Hand", "Right", CoSimTopic.OnStart, carryRight.ID + ":" + CoSimAction.EndInstruction),
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            MInstruction moveLeft = new MInstruction(MInstructionFactory.GenerateID(), "move left", MOTION_MOVE)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["PositioningTargetLeft"].ID, "SubjectID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Left", CoSimTopic.OnStart, carryLeft.ID + ":" + CoSimAction.EndInstruction),
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            MInstruction releaseRight = new MInstruction(MInstructionFactory.GenerateID(), "release right", MOTION_RELEASE)
            {
                Properties = PropertiesCreator.Create("Hand", "Right"),
                StartCondition = moveRight.ID + ":" + mmiConstants.MSimulationEvent_End
            };

            MInstruction releaseLeft = new MInstruction(MInstructionFactory.GenerateID(), "release left", MOTION_RELEASE)
            {
                Properties = PropertiesCreator.Create("Hand", "Left"),
                StartCondition = moveLeft.ID + ":" + mmiConstants.MSimulationEvent_End
            };


            MInstruction walkInstruction2 = new MInstruction(MInstructionFactory.GenerateID(), "Walk2", MOTION_WALK)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance.GetSceneObjectByName("WalkTargetConcurrent").ID, CoSimTopic.OnStart, idleInstruction.ID + ":" + CoSimAction.EndInstruction),
                StartCondition = releaseLeft.ID + ":" + mmiConstants.MSimulationEvent_End// + "|" + releaseLeft.ID + ":" + mmiConstants.MSimulationEvent_End,
            };

            MInstruction idleInstruction3 = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE)
            {
                StartCondition = walkInstruction2.ID + ":" + mmiConstants.MSimulationEvent_End,
            };


            MInstruction reachRight2 = new MInstruction(MInstructionFactory.GenerateID(), "reach right", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspTarget3R"].ID, "Hand", "Right"),
                StartCondition = walkInstruction2.ID + ":" + mmiConstants.MSimulationEvent_End// + "|>" + releaseLeft.ID + ":" + mmiConstants.MSimulationEvent_End,
            };

            MInstruction carryRight2 = new MInstruction(MInstructionFactory.GenerateID(), "carry right", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject3"].ID, "Hand", "Right", "AddOffset", false.ToString()),
                StartCondition = reachRight2.ID + ":" + mmiConstants.MSimulationEvent_End + "+ 0.01"
            };


            MInstruction walkInstruction3 = new MInstruction(MInstructionFactory.GenerateID(), "Walk2", MOTION_WALK)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance.GetSceneObjectByName("WalkTarget2").ID, CoSimTopic.OnStart, idleInstruction3.ID + ":" + CoSimAction.EndInstruction),
                StartCondition = carryRight2.ID + ":" + "PositioningFinished"// + "|" + releaseLeft.ID + ":" + mmiConstants.MSimulationEvent_End,
            };

            MInstruction idleInstruction4 = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE)
            {
                StartCondition = walkInstruction3.ID + ":" + mmiConstants.MSimulationEvent_End,
            };

            MSimulationState state = new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() };

            this.CoSimulator.Abort();
            this.CoSimulator.AssignInstruction(walkInstruction1, state);
            this.CoSimulator.AssignInstruction(idleInstruction, state);
            this.CoSimulator.AssignInstruction(reachRight, state);
            this.CoSimulator.AssignInstruction(reachLeft, state);
            this.CoSimulator.AssignInstruction(carryRight, state);
            this.CoSimulator.AssignInstruction(carryLeft, state);
            this.CoSimulator.AssignInstruction(walkInstruction, state);
            this.CoSimulator.AssignInstruction(idleInstruction2, state);
            this.CoSimulator.AssignInstruction(moveRight, state);
            this.CoSimulator.AssignInstruction(moveLeft, state);
            this.CoSimulator.AssignInstruction(releaseLeft, state);
            this.CoSimulator.AssignInstruction(releaseRight, state);
            this.CoSimulator.AssignInstruction(walkInstruction2, state);
            this.CoSimulator.AssignInstruction(idleInstruction3, state);
            this.CoSimulator.AssignInstruction(reachRight2, state);
            this.CoSimulator.AssignInstruction(carryRight2, state);
            this.CoSimulator.AssignInstruction(walkInstruction3, state);
            this.CoSimulator.AssignInstruction(idleInstruction4, state);


        }


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
