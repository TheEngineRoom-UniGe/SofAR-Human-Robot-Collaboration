using System.Collections;
using System;
using RosMessageTypes.HumanBaxterCollaboration;
using UnityEngine;

using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;
using JointState = RosMessageTypes.Sensor.JointState;
using Header = RosMessageTypes.Std.Header;
using Time = RosMessageTypes.Std.Time;
using System.Linq;

public class BaxterController : MonoBehaviour
{
    private GameObject baxter;

    // Hardcoded variables 
    private int numRobotJoints = 7;
    private readonly float jointAssignmentWaitRest = 0.005f;
    private readonly float jointAssignmentWait = 0.005f;
    private readonly float poseAssignmentWait = 1.5f;

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;

    private ArticulationBody leftGripperL;
    private ArticulationBody rightGripperL;
    private ArticulationBody leftGripperR;
    private ArticulationBody rightGripperR;

    private Transform gripperBaseL;
    private Transform leftGripperLGameObject;
    private Transform rightGripperLGameObject;
    private Transform gripperBaseR;
    private Transform leftGripperRGameObject;
    private Transform rightGripperRGameObject;

    private string[] jointNames =
    {
        "head_pan",
        "right_s0",
        "right_s1",
        "right_e0",
        "right_e1",
        "right_w0",
        "right_w1",
        "right_w2",
        "left_s0",
        "left_s1",
        "left_e0",
        "left_e1",
        "left_w0",
        "left_w1",
        "left_w2",
        "l_gripper_l_finger_joint",
        "l_gripper_r_finger_joint",
        "r_gripper_l_finger_joint",
        "r_gripper_r_finger_joint",
    };

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    };

    // Start is called before the first frame update
    public void Init(GameObject baxter)
    {
        this.baxter = baxter;

        var side = "left";
        leftJointArticulationBodies = new ArticulationBody[numRobotJoints];
        string upper_shoulder = "base/torso/" + side + "_arm_mount/" + side + "_upper_shoulder";
        leftJointArticulationBodies[0] = baxter.transform.Find(upper_shoulder).GetComponent<ArticulationBody>();

        string lower_shoudler = upper_shoulder + "/" + side + "_lower_shoulder";
        leftJointArticulationBodies[1] = baxter.transform.Find(lower_shoudler).GetComponent<ArticulationBody>();

        string upper_elbow = lower_shoudler + "/" + side + "_upper_elbow";
        leftJointArticulationBodies[2] = baxter.transform.Find(upper_elbow).GetComponent<ArticulationBody>();

        string lower_elbow = upper_elbow + "/" + side + "_lower_elbow";
        leftJointArticulationBodies[3] = baxter.transform.Find(lower_elbow).GetComponent<ArticulationBody>();

        string upper_forearm = lower_elbow + "/" + side + "_upper_forearm";
        leftJointArticulationBodies[4] = baxter.transform.Find(upper_forearm).GetComponent<ArticulationBody>();

        string lower_forearm = upper_forearm + "/" + side + "_lower_forearm";
        leftJointArticulationBodies[5] = baxter.transform.Find(lower_forearm).GetComponent<ArticulationBody>();

        string wrist = lower_forearm + "/" + side + "_wrist";
        leftJointArticulationBodies[6] = baxter.transform.Find(wrist).GetComponent<ArticulationBody>();

        string hand = wrist + "/" + side + "_hand";
        // Find left and right fingers
        string right_gripper = hand + "/" + side + "_gripper_base/l_gripper_r_finger";
        string left_gripper = hand + "/" + side + "_gripper_base/l_gripper_l_finger";
        string gripper_base = hand + "/" + side + "_gripper_base/Collisions/unnamed";

        gripperBaseL = baxter.transform.Find(gripper_base);
        leftGripperLGameObject = baxter.transform.Find(left_gripper);
        rightGripperLGameObject = baxter.transform.Find(right_gripper);

        rightGripperL = rightGripperLGameObject.GetComponent<ArticulationBody>();
        leftGripperL = leftGripperLGameObject.GetComponent<ArticulationBody>();


        side = "right";
        rightJointArticulationBodies = new ArticulationBody[numRobotJoints];
        upper_shoulder = "base/torso/" + side + "_arm_mount/" + side + "_upper_shoulder";
        rightJointArticulationBodies[0] = baxter.transform.Find(upper_shoulder).GetComponent<ArticulationBody>();

        lower_shoudler = upper_shoulder + "/" + side + "_lower_shoulder";
        rightJointArticulationBodies[1] = baxter.transform.Find(lower_shoudler).GetComponent<ArticulationBody>();

        upper_elbow = lower_shoudler + "/" + side + "_upper_elbow";
        rightJointArticulationBodies[2] = baxter.transform.Find(upper_elbow).GetComponent<ArticulationBody>();

        lower_elbow = upper_elbow + "/" + side + "_lower_elbow";
        rightJointArticulationBodies[3] = baxter.transform.Find(lower_elbow).GetComponent<ArticulationBody>();

        upper_forearm = lower_elbow + "/" + side + "_upper_forearm";
        rightJointArticulationBodies[4] = baxter.transform.Find(upper_forearm).GetComponent<ArticulationBody>();

        lower_forearm = upper_forearm + "/" + side + "_lower_forearm";
        rightJointArticulationBodies[5] = baxter.transform.Find(lower_forearm).GetComponent<ArticulationBody>();

        wrist = lower_forearm + "/" + side + "_wrist";
        rightJointArticulationBodies[6] = baxter.transform.Find(wrist).GetComponent<ArticulationBody>();

        hand = wrist + "/" + side + "_hand";
        // Find left and right fingers
        right_gripper = hand + "/" + side + "_gripper_base/r_gripper_r_finger";
        left_gripper = hand + "/" + side + "_gripper_base/r_gripper_l_finger";
        gripper_base = hand + "/" + side + "_gripper_base/Collisions/unnamed";

        gripperBaseR = baxter.transform.Find(gripper_base);
        leftGripperRGameObject = baxter.transform.Find(left_gripper);
        rightGripperRGameObject = baxter.transform.Find(right_gripper);

        rightGripperR = rightGripperRGameObject.GetComponent<ArticulationBody>();
        leftGripperR = leftGripperRGameObject.GetComponent<ArticulationBody>();
    }

    private void CloseGripper(string side)
    {

        if (side == "left")
        {
            var leftDrive = leftGripperL.xDrive;
            var rightDrive = rightGripperL.xDrive;

            leftDrive.target = 0.0025f;
            rightDrive.target = -0.0025f;

            leftGripperL.xDrive = leftDrive;
            rightGripperL.xDrive = rightDrive;
        }

        else
        {
            var leftDrive = leftGripperR.xDrive;
            var rightDrive = rightGripperR.xDrive;

            leftDrive.target = 0.005f;
            rightDrive.target = -0.005f;

            leftGripperR.xDrive = leftDrive;
            rightGripperR.xDrive = rightDrive;
        }
    }

    private void OpenGripper(string side)
    {
        if (side == "left")
        {
            var leftDrive = leftGripperL.xDrive;
            var rightDrive = rightGripperL.xDrive;

            leftDrive.target = 0.025f;
            rightDrive.target = -0.025f;

            leftGripperL.xDrive = leftDrive;
            rightGripperL.xDrive = rightDrive;
        }

        else
        {
            var leftDrive = leftGripperR.xDrive;
            var rightDrive = rightGripperR.xDrive;

            leftDrive.target = 0.025f;
            rightDrive.target = -0.025f;

            leftGripperR.xDrive = leftDrive;
            rightGripperR.xDrive = rightDrive;
        }
    }

    BaxterMoveitJoints CurrentJointConfig(string side)
    {
        BaxterMoveitJoints joints = new BaxterMoveitJoints();
        var articulationBodies = leftJointArticulationBodies;
        if (side == "right")
        {
            articulationBodies = rightJointArticulationBodies;
        }
        joints.joint_00 = articulationBodies[0].xDrive.target;
        joints.joint_01 = articulationBodies[1].xDrive.target;
        joints.joint_02 = articulationBodies[2].xDrive.target;
        joints.joint_03 = articulationBodies[3].xDrive.target;
        joints.joint_04 = articulationBodies[4].xDrive.target;
        joints.joint_05 = articulationBodies[5].xDrive.target;
        joints.joint_06 = articulationBodies[6].xDrive.target;

        return joints;
    }

    public void GoToRestPosition(string whichArm)
    {
        if (whichArm == "left")
        {
            StartCoroutine(GoToRestLeft());
        }
        else if (whichArm == "right")
        {
            StartCoroutine(GoToRestRight());
        }
        else
        {
            StartCoroutine(GoToRestLeft());
            StartCoroutine(GoToRestRight());
        }
    }

    private IEnumerator GoToRestLeft()
    {
        float[] target = { -30f, -70f, 0f, 99f, 0f, 43f, 0f };
        var currentJointConfig = CurrentJointConfig("left");
        float[] lastJointState = {
                (float)currentJointConfig.joint_00,
                (float)currentJointConfig.joint_01,
                (float)currentJointConfig.joint_02,
                (float)currentJointConfig.joint_03,
                (float)currentJointConfig.joint_04,
                (float)currentJointConfig.joint_05,
                (float)currentJointConfig.joint_06,
        };
        for (int i = 0; i <= 50; i++)
        {
            for (int joint = 0; joint < leftJointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = leftJointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * 0.02f * (float)i;
                leftJointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWaitRest);
        }
        CloseGripper("left");
    }

    private IEnumerator GoToRestRight()
    {
        float[] target = { 30.0f, -70.0f, 0f, 99.0f, 0f, 43.0f, 0f };
        var currentJointConfig = CurrentJointConfig("right");
        float[] lastJointState = {
                (float)currentJointConfig.joint_00,
                (float)currentJointConfig.joint_01,
                (float)currentJointConfig.joint_02,
                (float)currentJointConfig.joint_03,
                (float)currentJointConfig.joint_04,
                (float)currentJointConfig.joint_05,
                (float)currentJointConfig.joint_06,
        };
        for (int i = 0; i <= 50; i++)
        {
            for (int joint = 0; joint < rightJointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = rightJointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * 0.02f * (float)i;
                rightJointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWaitRest);
        }
        CloseGripper("right");
    }

    private double[] GetJointsPositions()
    {
        var jointPositions = new double[19];
        jointPositions[0] = 0.0f;
        int i = 1;
        int j = 0;
        foreach (ArticulationBody joint in rightJointArticulationBodies)
        {
            jointPositions[i] = Math.PI * rightJointArticulationBodies[j].xDrive.target / 180.0f;
            i++;
            j++;
        }
        j = 0;
        foreach (ArticulationBody joint in leftJointArticulationBodies)
        {
            jointPositions[i] = Math.PI * leftJointArticulationBodies[j].xDrive.target / 180.0f;
            i++;
            j++;
        }
        jointPositions[15] = jointPositions[16] = jointPositions[17] = jointPositions[18] = 0.0f;
        return jointPositions;
    }

    public JointState GetBaxterJointState()
    {
        var secs = DateTimeOffset.Now.ToUnixTimeSeconds();
        var header = new Header
        {
            seq = (uint)1,
            stamp = new Time
            {
                secs = (uint)secs,
                nsecs = (uint)(DateTimeOffset.Now.ToUnixTimeSeconds()) / 10,
            },
            frame_id = ""
        };
        var jointState = new JointState();
        jointState.header = header;
        jointState.name = jointNames;
        jointState.position = GetJointsPositions();

        return jointState;
    }

    public void TrajectoryResponse(BaxterTrajectory response)
    {
        if (response.trajectory.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    private IEnumerator ExecuteTrajectories(BaxterTrajectory response)
    {
        if (response.trajectory != null)
        {
            var currentJointConfig = CurrentJointConfig(response.arm);
            float[] lastJointState = {
                (float)currentJointConfig.joint_00,
                (float)currentJointConfig.joint_01,
                (float)currentJointConfig.joint_02,
                (float)currentJointConfig.joint_03,
                (float)currentJointConfig.joint_04,
                (float)currentJointConfig.joint_05,
                (float)currentJointConfig.joint_06,
            };
            // For every trajectory plan returned
            int steps = 100;
            var articulationBodies = leftJointArticulationBodies;
            if (response.arm == "right")
            {
                articulationBodies = rightJointArticulationBodies;
            }
            for (int poseIndex = 0; poseIndex < response.trajectory.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectory[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectory[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();
                    for (int i = 0; i <= steps; i++)
                    {
                        for (int joint = 0; joint < articulationBodies.Length; joint++)
                        {
                            var joint1XDrive = articulationBodies[joint].xDrive;
                            joint1XDrive.target = lastJointState[joint] + (result[joint] - lastJointState[joint]) * (1.0f / steps) * i;
                            articulationBodies[joint].xDrive = joint1XDrive;
                        }

                        yield return new WaitForSeconds(jointAssignmentWait);

                    }

                    // Wait for robot to achieve pose for all joint assignments
                    lastJointState = result;

                }
                if (poseIndex == (int)Poses.PreGrasp)
                    OpenGripper(response.arm);
                // Close the gripper if completed executing the trajectory for the Grasp pose
                else if (poseIndex == (int)Poses.Grasp)
                    CloseGripper(response.arm);

                // Wait for the robot to achieve the final pose from joint assignment                
            }
            // All trajectories have been executed, open the gripper to place the target cube
            yield return new WaitForSeconds(poseAssignmentWait);
            OpenGripper(response.arm);
            yield return new WaitForSeconds(poseAssignmentWait);
            GoToRestPosition(response.arm);
        }
    }
}
