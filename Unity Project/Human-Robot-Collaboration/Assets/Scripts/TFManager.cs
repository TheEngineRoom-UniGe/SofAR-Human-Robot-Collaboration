using UnityEngine;
using ROSGeometry;
using System;
using System.Collections.Generic;
using PoseStamped = RosMessageTypes.Geometry.PoseStamped;
using Header = RosMessageTypes.Std.Header;
using Time = RosMessageTypes.Std.Time;
using UnityTf = RosMessageTypes.HumanBaxterCollaboration.UnityTf;

public class TFManager : MonoBehaviour
{
    private GameObject avatar;
    private UnityEngine.Transform origin;

    private UnityTf unityTf;

    private UnityEngine.Transform[] humanTransforms;
    private string[] humanFrameNames = { "pelvis", "spine", "neck", "h_head", "upperarm_l", "lowerarm_l", "hand_l", "upperarm_r", "lowerarm_r", "hand_r" };

    private UnityEngine.Transform[] objectsTransforms;
    private string[] objectsFrameNames;

    // Start is called before the first frame update
    public void Init(GameObject avatar, UnityEngine.Transform origin)
    {
        // Get reference to gameobjects
        this.avatar = avatar;
        this.origin = origin;

        // Get reference to frames to be published on ROS
        GetHumanFrames();
        GetObjectsFrames();
        unityTf = new UnityTf();
        unityTf.frames = new PoseStamped[humanTransforms.Length + objectsTransforms.Length];
    }

    // Update is called once per frame
    void GetHumanFrames()
    {
        humanTransforms = new UnityEngine.Transform[humanFrameNames.Length];
        string pelvis = "Game_engine/Root/pelvis";
        humanTransforms[0] = avatar.transform.Find(pelvis);
        string spine = pelvis + "/spine_01/spine_02/spine_03";
        humanTransforms[1] = avatar.transform.Find(spine);
        string neck = spine + "/neck_01";
        humanTransforms[2] = avatar.transform.Find(neck);
        string head = neck + "/head";
        humanTransforms[3] = avatar.transform.Find(head);
        string upper_arm_l = spine + "/clavicle_l/upperarm_l";
        humanTransforms[4] = avatar.transform.Find(upper_arm_l);
        string lower_arm_l = upper_arm_l + "/lowerarm_l";
        humanTransforms[5] = avatar.transform.Find(lower_arm_l);
        string hand_l = lower_arm_l + "/hand_l";
        humanTransforms[6] = avatar.transform.Find(hand_l);
        string upper_arm_r = spine + "/clavicle_r/upperarm_r";
        humanTransforms[7] = avatar.transform.Find(upper_arm_r);
        string lower_arm_r = upper_arm_r + "/lowerarm_r";
        humanTransforms[8] = avatar.transform.Find(lower_arm_r);
        string hand_r = lower_arm_r + "/hand_r";
        humanTransforms[9] = avatar.transform.Find(hand_r);
    }

    void GetObjectsFrames()
    {
        int objectsCount = origin.childCount;
        objectsFrameNames = new string[objectsCount];
        objectsTransforms = new Transform[objectsCount];
        for (int i = 0; i < objectsCount; ++i)
        {
            Transform childTransform = origin.GetChild(i);
            objectsFrameNames[i] = childTransform.name;
            objectsTransforms[i] = childTransform;
        }
    }

    public UnityTf GetUnityTfMessage()
    {
        var now = DateTime.Now;
        for (int i = 0; i < humanFrameNames.Length; i++)
        {
            var header = new Header(seq: (uint)1, stamp: new Time((uint)now.Second, 0), frame_id: humanFrameNames[i]);
            unityTf.frames[i] = new PoseStamped
            {
                header = header,
                pose = new RosMessageTypes.Geometry.Pose
                {
                    position = (humanTransforms[i].position).To<FLU>(),
                    orientation = (humanTransforms[i].rotation).To<FLU>()
                }
            };
        }
        var j = 0;
        for (int i = humanFrameNames.Length; i < unityTf.frames.Length; i++)
        {
            var header = new Header(seq: (uint)1, stamp: new Time((uint)now.Second, 0), frame_id: objectsFrameNames[j]);
            unityTf.frames[i] = new PoseStamped
            {
                header = header,
                pose = new RosMessageTypes.Geometry.Pose
                {
                    position = (objectsTransforms[j].position).To<FLU>(),
                    orientation = Quaternion.Euler(180, -objectsTransforms[j].rotation.eulerAngles.y, 0).To<FLU>()
                }
            };
            j++;
        }
        return unityTf;
    }
}
