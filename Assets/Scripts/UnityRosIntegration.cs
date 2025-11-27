using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.RosUnityMessages;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.SceneManagement;

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
    GameObject m_PickLocation;
    public GameObject PickLocation { get => m_PickLocation; set => m_PickLocation = value; }
    [SerializeField]
    GameObject m_PlaceLocation;
    public GameObject PlaceLocation { get => m_PlaceLocation; set => m_PlaceLocation = value; }
    [SerializeField]
    GameObject m_PrePickLocation;
    public GameObject PrePickLocation { get => m_PrePickLocation; set => m_PrePickLocation = value; }
    [SerializeField]
    GameObject m_PrePlaceLocation;
    public GameObject PrePlaceLocation { get => m_PrePlaceLocation; set => m_PrePlaceLocation = value; }
    [SerializeField]
    string m_TargetsRosTopicName = "/unity_bridge/unity_targets";
    public string TargetsRosTopicName { get => m_TargetsRosTopicName; set => m_TargetsRosTopicName = value; }
    [SerializeField]
    string m_JointStateRosTopicName = "/joint_states";
    public string JointStateRosTopicName { get => m_JointStateRosTopicName; set => m_JointStateRosTopicName = value; }

    // 20cm away from the cube before lowering
    readonly Vector3 m_Offset = Vector3.up * 0.2f;

    // Robot Articulation Body
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    // Gripper Controller
    GripperController m_GripperController;

    // Collision Objects

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
    }

    /**
    * Publish the position of the target, as well as the location where the target needed to be to ROS.
    */
    public void PublishTarget() {
        var message = new UnityRequestMsg();

        // Scene name
        message.scene_name = new StringMsg(SceneManager.GetActiveScene().name);

        // Pre Pick Location
        message.pre_pick_location = new PoseMsg {
            position = m_PrePickLocation.transform.position.To<FLU>(),
            orientation = m_PrePickLocation.transform.rotation.To<FLU>()
        };

        // Pick Location
        message.pick_location = new PoseMsg {
            position = m_PickLocation.transform.position.To<FLU>(),
            orientation = m_PrePickLocation.transform.rotation.To<FLU>()
        };

        // Pre Place Location
        message.pre_place_location = new PoseMsg {
            position = m_PrePlaceLocation.transform.position.To<FLU>(),
            orientation = m_PrePlaceLocation.transform.rotation.To<FLU>()
        };

        // Place Location
        message.place_location = new PoseMsg {
            position = m_PlaceLocation.transform.position.To<FLU>(),
            orientation = m_PrePlaceLocation.transform.rotation.To<FLU>()
        };

        // Add static objects
        var sceneObjects = GameObject.FindGameObjectsWithTag("StaticSceneObject");
        var encodedObjects = new UnityObjectMsg[sceneObjects.Length];
        var index = 0;

        foreach (GameObject sceneObject in sceneObjects) {
            encodedObjects[index] = new UnityObjectMsg {
                id = new StringMsg(sceneObject.name),
                position = sceneObject.transform.position.To<FLU>(),
                orientation = sceneObject.transform.rotation.To<FLU>(),
                scale = sceneObject.transform.localScale.To<FLU>()
            };
            index += 1;
        }

        message.static_objects = encodedObjects;
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