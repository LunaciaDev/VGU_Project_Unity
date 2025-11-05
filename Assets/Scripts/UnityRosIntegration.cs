using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.RosUnityMessages;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class UnityRosIntegration : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    string[] LinkNames = {
        "arm_base_link/arm_base_link_inertia/arm_shoulder_link",
        "/arm_upper_arm_link",
        "/arm_forearm_link",
        "/arm_wrist_1_link",
        "/arm_wrist_2_link",
        "/arm_wrist_3_link"
    };

    [SerializeField]
    GameObject m_Ur10e;
    public GameObject Ur10e { get => m_Ur10e; set => m_Ur10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }
    [SerializeField]
    string m_TargetsRosTopicName = "/unity_bridge/unity_targets";
    public string TargetsRosTopicName { get => m_TargetsRosTopicName; set => m_TargetsRosTopicName = value; }
    [SerializeField]
    string m_JointStateRosTopicName = "/joint_states";
    public string JointStateRosTopicName { get => m_JointStateRosTopicName; set => m_JointStateRosTopicName = value; }
    [SerializeField]
    string m_WorkspaceObjectRosTopicName = "/unity_bridge/unity_objects";
    public string WorkspaceObjectRosTopicName { get => m_WorkspaceObjectRosTopicName; set => m_WorkspaceObjectRosTopicName = value;}

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Robot Articulation Body
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    // Gripper Controller
    GripperController m_GripperController;

    // Start is called before the first frame update
    void Start()
    {
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++) {
            linkName += LinkNames[i];
            var articulationBody = m_Ur10e.transform.Find(linkName).GetComponent<ArticulationBody>();

            if (articulationBody != null) {
                m_JointArticulationBodies[i] = articulationBody;
            }
        }

        m_GripperController = new GripperController(m_Ur10e);

        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();

        // Register to ROS that we are publishing target data to this topic
        m_Ros.RegisterPublisher<UnityRequestMsg>(m_TargetsRosTopicName);

        // Register to ROS that we are receiving joint state message from this topic
        m_Ros.Subscribe<JointStateMsg>(m_JointStateRosTopicName, JointStateCallback);

        // Register to ROS that we are publishing scene collision object information to this topic
        // [TODO]
    }

    /**
    * Publish the position of the target, as well as the location where the target needed to be to ROS.
    */
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

        m_Ros.Publish(m_TargetsRosTopicName, message);
    }

    /**
    * Update the joint index to value, in degrees. Remember to translate to degree before passing.
    */
    void SetArmJoint(int jointIndex, float value) {
        var joint1XDrive = m_JointArticulationBodies[jointIndex].xDrive;
        joint1XDrive.target = value;
        m_JointArticulationBodies[jointIndex].xDrive = joint1XDrive;
    }

    /**
    * Update the robot joint configuration to that reflected by /joint_states
    */
    void JointStateCallback(JointStateMsg robot_joint_config) {
        var jointNames = robot_joint_config.name;
        var jointPositions = robot_joint_config.position.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

        // For each joint in the config
        for (var jointIndex = 0; jointIndex < robot_joint_config.name.Length; jointIndex++) {
            // Contract does NOT guarantee joint order
            // So an expensive switch-case it is
            // Cant even do length counting to make it easier, unless if I make the joint name
            // in URDF hideous. Sad.
            switch (jointNames[jointIndex]) {
                case "arm_elbow_joint": {
                    // This joint connect upper_arm_link and forearm_link
                    // => Control arm_forearm_link, index 2
                    SetArmJoint(2, jointPositions[jointIndex]);
                    break;
                }
                case "arm_shoulder_lift_joint": {
                    // This joint connect shoulder and upper_arm
                    // => Control arm_upper_arm_link, index 1
                    SetArmJoint(1, jointPositions[jointIndex]);
                    break;
                }
                case "arm_shoulder_pan_joint": {
                    // This joint connect base_link_inertia and shoulder
                    // => Control arm_shoulder_link, index 0
                    SetArmJoint(0, jointPositions[jointIndex]);
                    break;
                }
                case "arm_wrist_1_joint": {
                    // This joint connect forearm and wrist_1
                    // => Control arm_wrist_1_link, index 3
                    SetArmJoint(3, jointPositions[jointIndex]);
                    break;
                }
                case "arm_wrist_2_joint": {
                    // I dont think I need to explain why this has index 4?
                    SetArmJoint(4, jointPositions[jointIndex]);
                    break;
                }
                case "arm_wrist_3_joint": {
                    // Same here, wonder why this has index 5?
                    SetArmJoint(5, jointPositions[jointIndex]);
                    break;
                }
                case "gripper_finger_joint": {
                    // Control the gripper, call the corresponding controller
                    m_GripperController.SetGripperPosition(jointPositions[jointIndex]);
                    break;
                }
            }
        }

        // Frankly speaking at 100Hz update rate, we dont want to wait here..?
    }
}