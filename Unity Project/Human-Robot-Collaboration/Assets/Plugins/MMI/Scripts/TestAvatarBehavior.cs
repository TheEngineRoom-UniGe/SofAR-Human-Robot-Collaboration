using MMICoSimulation;
using MMICSharp.MMIStandard.Utils;
using MMIStandard;
using MMIUnity.TargetEngine;
using MMIUnity.TargetEngine.Scene;
using UnityEngine;


public class TestAvatarBehavior : AvatarBehavior
{
    private string carryID;

    // Motion Types
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
        if (GUI.Button(new Rect(10, 10, 120, 50), "Idle"))
        {
            MSkeletonAccess.Iface skeletonAccess = this.avatar.GetSkeletonAccess();
            skeletonAccess.SetChannelData(this.avatar.GetPosture());



            MInstruction instruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);
            //MInstruction instruction = new MInstruction(MInstructionFactory.GenerateID(), "MMUTest", "Object/Move");
            MSimulationState simstate = new MSimulationState(this.avatar.GetPosture(), this.avatar.GetPosture());

           

            this.CoSimulator.Abort();
            this.CoSimulator.AssignInstruction(instruction, simstate);
        }


        if (GUI.Button(new Rect(140, 10, 120, 50), "Walk to"))
        {
            MInstruction walkInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Walk", MOTION_WALK)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance.GetSceneObjectByName("WalkTarget").ID)
            };

            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE)
            {
                //Start idle after walk has been finished
                StartCondition = walkInstruction.ID + ":" + mmiConstants.MSimulationEvent_End //synchronization constraint similar to bml "id:End"  (bml original: <bml start="id:End"/>
            };

            this.CoSimulator.Abort();


            MSimulationState currentState = new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() };

            //Assign walk and idle instruction
            this.CoSimulator.AssignInstruction(walkInstruction, currentState);
            this.CoSimulator.AssignInstruction(idleInstruction, currentState);
            this.CoSimulator.MSimulationEventHandler += this.CoSimulator_MSimulationEventHandler;
        }


        if (GUI.Button(new Rect(270, 10, 120, 50), "Reach Object"))
        {

            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);


            MInstruction reachRight = new MInstruction(MInstructionFactory.GenerateID(), "reach right", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["ReachTarget"].ID, "Hand", "Left", "MinDistance", 2.0.ToString()),
            };


            //MInstruction reachLeft = new MInstruction(MInstructionFactory.GenerateID(), "reach left", "Pose/Reach")
            //{
            //    Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspTargetL"].ID, "Hand", "Left"),
            //};

            //this.CoSimulator.Abort();
            this.CoSimulator.AssignInstruction(idleInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(reachRight, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            //this.CoSimulator.AssignInstruction(reachLeft, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });


        }


        if (GUI.Button(new Rect(400, 10, 120, 50), "Move Object"))
        {
            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);

            MInstruction moveObject = new MInstruction(MInstructionFactory.GenerateID(), "move object", MOTION_MOVE)
            {
                Properties = PropertiesCreator.Create("SubjectID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Left", "TargetID", UnitySceneAccess.Instance["PositioningTarget"].ID),
            };

            this.CoSimulator.Abort();
            this.CoSimulator.AssignInstruction(idleInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(moveObject, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        }


        if (GUI.Button(new Rect(530, 10, 120, 50), "Pick-up"))
        {
            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);

            MInstruction reachInstruction = new MInstruction(MInstructionFactory.GenerateID(), "reach", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["ReachTarget"].ID, "Hand", "Left"),
            };

            carryID = MInstructionFactory.GenerateID();
            MInstruction carryInstruction = new MInstruction(carryID, "carry object", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Left"),
                StartCondition = reachInstruction.ID +":"+ mmiConstants.MSimulationEvent_End + "+ 0.01"
            };

           
            this.CoSimulator.AssignInstruction(idleInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(reachInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(carryInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.MSimulationEventHandler += this.CoSimulator_MSimulationEventHandler;

        }

        if (GUI.Button(new Rect(660, 10, 120, 50), "Carry Object"))
        {
            carryID = MInstructionFactory.GenerateID();
            MInstruction carryInstruction = new MInstruction(carryID, "carry object", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Right")//, "CarryTarget", UnitySceneAccess.Instance["CarryTarget"].ID),
            };

            this.CoSimulator.AssignInstruction(carryInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        }




        if (GUI.Button(new Rect(790, 10, 120, 50), "Release Object"))
        {
            MInstruction releaseRight = new MInstruction(MInstructionFactory.GenerateID(), "release object", MOTION_RELEASE)
            {
                Properties = PropertiesCreator.Create( "Hand", "Right", CoSimTopic.OnStart, carryID + ":" + CoSimAction.EndInstruction),
            };
            MInstruction releaseLeft = new MInstruction(MInstructionFactory.GenerateID(), "release object", MOTION_RELEASE)
            {
                Properties = PropertiesCreator.Create("Hand", "Left", CoSimTopic.OnStart, carryID + ":" + CoSimAction.EndInstruction),
            };


            this.CoSimulator.AssignInstruction(releaseRight, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(releaseLeft, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });

        }


        if (GUI.Button(new Rect(920, 10, 160, 50), "Concurrent scenario"))
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


        if (GUI.Button(new Rect(10, 70, 220, 50), "Pickup large object"))
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

        if (GUI.Button(new Rect(240, 70, 220, 50), "Move Large Object both handed"))
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

        if (GUI.Button(new Rect(470, 70, 220, 50), "Carry Object both handed"))
        {
            carryID = MInstructionFactory.GenerateID();
            MInstruction carryInstruction = new MInstruction(carryID, "carry object", MOTION_CARRY)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspObject"].ID, "Hand", "Both", "CarryTarget", UnitySceneAccess.Instance["CarryTarget"].ID),
            };

            this.CoSimulator.AssignInstruction(carryInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
        }

        if (GUI.Button(new Rect(700, 70, 210, 50), "Reach Object Single"))
        {

            MInstruction idleInstruction = new MInstruction(MInstructionFactory.GenerateID(), "Idle", MOTION_IDLE);


            MInstruction reachRight = new MInstruction(MInstructionFactory.GenerateID(), "reach right", MOTION_REACH)
            {
                Properties = PropertiesCreator.Create("TargetID", UnitySceneAccess.Instance["GraspTargetR"].ID, "Hand", "Right"),
            };




            this.CoSimulator.Abort();
            this.CoSimulator.AssignInstruction(idleInstruction, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });
            this.CoSimulator.AssignInstruction(reachRight, new MSimulationState() { Initial = this.avatar.GetPosture(), Current = this.avatar.GetPosture() });


        }

        if (GUI.Button(new Rect(920, 70, 160, 50), "Abort"))
        {
            this.CoSimulator.Abort();
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
