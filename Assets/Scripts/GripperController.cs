using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripperController {
    ArticulationBody m_LeftOuterGripper;
    ArticulationBody m_LeftInnerGripper;
    ArticulationBody m_LeftFinger;
    ArticulationBody m_RightOuterGripper;
    ArticulationBody m_RightInnerGripper;
    ArticulationBody m_RightFinger;

    string[] LinkNames = {
        "arm_base_link/arm_base_link_inertia/arm_shoulder_link",
        "/arm_upper_arm_link",
        "/arm_forearm_link",
        "/arm_wrist_1_link",
        "/arm_wrist_2_link",
        "/arm_wrist_3_link"
    };

    const float neutral_angle = 0.0f;
    const float open_angle = -10.0f;
    const float close_angle = 17.0f;

    public enum GripperStatus {
        Neutral,
        Open,
        Closed
    }

    void SetGripperPosition(float position) {
        ArticulationDrive drive = m_LeftInnerGripper.xDrive;
        drive.target = -position;
        m_LeftInnerGripper.xDrive = drive;

        drive = m_RightInnerGripper.xDrive;
        drive.target = -position;
        m_RightInnerGripper.xDrive = drive;

        drive = m_LeftOuterGripper.xDrive;
        drive.target = position;
        m_LeftOuterGripper.xDrive = drive;

        drive = m_RightOuterGripper.xDrive;
        drive.target = -position;
        m_RightOuterGripper.xDrive = drive;

        drive = m_LeftFinger.xDrive;
        drive.target = position;
        m_LeftFinger.xDrive = drive;

        drive = m_RightFinger.xDrive;
        drive.target = position;
        m_RightFinger.xDrive = drive;
    }

    public void SetGripperStatus(GripperStatus status) {
        switch (status) {
            case GripperStatus.Neutral: {
                SetGripperPosition(neutral_angle);
                break;
            }

            case GripperStatus.Open: {
                SetGripperPosition(open_angle);
                break;
            }

            case GripperStatus.Closed: {
                SetGripperPosition(close_angle);
                break;
            }
        }
    }

    public GripperController(GameObject m_Ur10e) {
        var link_name = string.Empty;

        for (var i = 0; i < 6; i++) {
            link_name += LinkNames[i];
        }

        var gripper_base_path = link_name + "/arm_flange/arm_tool0/gripper_onrobot_rg2_base_link";
        var right_outer_gripper = gripper_base_path + "/gripper_right_outer_knuckle";
        var left_outer_gripper = gripper_base_path + "/gripper_left_outer_knuckle";
        var right_inner_gripper = gripper_base_path + "/gripper_right_inner_knuckle";
        var left_inner_gripper = gripper_base_path + "/gripper_left_inner_knuckle";
        var right_finger = right_outer_gripper + "/gripper_right_inner_finger";
        var left_finger = left_outer_gripper + "/gripper_left_inner_finger";

        m_LeftOuterGripper = m_Ur10e.transform.Find(left_outer_gripper).GetComponent<ArticulationBody>();
        m_LeftInnerGripper = m_Ur10e.transform.Find(left_inner_gripper).GetComponent<ArticulationBody>();
        m_LeftFinger = m_Ur10e.transform.Find(left_finger).GetComponent<ArticulationBody>();
        m_RightOuterGripper = m_Ur10e.transform.Find(right_outer_gripper).GetComponent<ArticulationBody>();
        m_RightInnerGripper = m_Ur10e.transform.Find(right_inner_gripper).GetComponent<ArticulationBody>();
        m_RightFinger = m_Ur10e.transform.Find(right_finger).GetComponent<ArticulationBody>();
    }
}