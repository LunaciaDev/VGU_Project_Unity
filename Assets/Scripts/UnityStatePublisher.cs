using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.RosUnityMessages;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class UnityStatePublisher : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;
    string[] LinkNames = {
        "arm_base_link/arm_base_link_inertia/arm_shoulder_link",
        "/arm_upper_arm_link",
        "/arm_forearm_link",
        "/arm_wrist_1_link",
        "/arm_wrist_2_link",
        "/arm_wrist_3_link"
    };

    // Variables required for ROS communication
    [SerializeField]
    string m_RosTopicName = "/unity_target_listener";

    [SerializeField]
    GameObject m_Ur10e;
    public GameObject Ur10e { get => m_Ur10e; set => m_Ur10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Robot Articulation Body
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<UnityRequestMsg>(m_RosTopicName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++) {
            linkName += LinkNames[i];
            var articulationBody = m_Ur10e.transform.Find(linkName).GetComponent<ArticulationBody>();

            if (articulationBody != null) {
                m_JointArticulationBodies[i] = articulationBody;
            }
        }
    }

    public void PublishTarget() {
        var message = new UnityRequestMsg();

        for (var i = 0; i < k_NumRobotJoints; i++) {
            message.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        // Pick Pose
        message.pick_pose = new PoseMsg {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        message.place_pose = new PoseMsg {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.Publish(m_RosTopicName, message);
    }
}
