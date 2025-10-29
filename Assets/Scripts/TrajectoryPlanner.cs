using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.RosUnityIntegration;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;
    string[] LinkNames = {
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link",
        "/robot_upper_arm_link",
        "/robot_forearm_link",
        "/robot_wrist_1_link",
        "/robot_wrist_2_link",
        "/robot_wrist_3_link"
    };

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "robot_planner_service";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

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

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftOuterGripper;
    ArticulationBody m_RightOuterGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<IntegrationServiceRequest, IntegrationServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            var articulationBody = m_Ur10e.transform.Find(linkName).GetComponent<ArticulationBody>();

            if (articulationBody != null) {
                m_JointArticulationBodies[i] = articulationBody;
            }
        }

        // Find left and right fingers
        var gripper_base_path = linkName + "/robot_flange/gripper_onrobot_rg2_base_link";

        var rightOuterGripper = gripper_base_path + "/gripper_right_outer_knuckle/gripper_right_inner_finger";
        var leftOuterGripper = gripper_base_path + "/gripper_left_outer_knuckle/gripper_left_inner_finger";

        m_RightOuterGripper = m_Ur10e.transform.Find(rightOuterGripper).GetComponent<ArticulationBody>();
        m_LeftOuterGripper = m_Ur10e.transform.Find(leftOuterGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        // Set the target position to close the gripper
        var leftOuterDrive = m_LeftOuterGripper.xDrive;
        var rightOuterDrive = m_RightOuterGripper.xDrive;

        leftOuterDrive.target = Mathf.Clamp(-30.0f, leftOuterDrive.lowerLimit, leftOuterDrive.upperLimit);
        rightOuterDrive.target = Mathf.Clamp(-30.0f, rightOuterDrive.lowerLimit, rightOuterDrive.upperLimit);

        m_LeftOuterGripper.xDrive = leftOuterDrive;
        m_RightOuterGripper.xDrive = rightOuterDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftOuterDrive = m_LeftOuterGripper.xDrive;
        var rightOuterDrive = m_RightOuterGripper.xDrive;

        leftOuterDrive.target = Mathf.Clamp(30.0f, leftOuterDrive.lowerLimit, leftOuterDrive.upperLimit);
        rightOuterDrive.target = Mathf.Clamp(30.0f, rightOuterDrive.lowerLimit, rightOuterDrive.upperLimit);

        m_LeftOuterGripper.xDrive = leftOuterDrive;
        m_RightOuterGripper.xDrive = rightOuterDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    Ur10eJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    ///     Create a new IntegrationServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new IntegrationServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<IntegrationServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(IntegrationServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> IntegrationServiceResponse received from niryo_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(IntegrationServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}
