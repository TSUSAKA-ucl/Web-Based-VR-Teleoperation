"use client";
import 'aframe'
import * as React from 'react'
import { WebRTC_Video_Recv, WebRTC_Video_Send, WebRTC_Video_Send_Data, WebRTC_Data_Recv } from '../lib/WebRTC_Sora';
import RobotScene from './RobotScene';
import registerAframeComponents from './registerAframeComponents'; 
import MQTT_Sub from './MQTT_Sub';
import { mqttclient, idtopic, publishMQTT, subscribeMQTT, codeType } from '../lib/MetaworkMQTT'

// On Windows, run the following command to allow script execution at first:
// Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned

/* ============================= Static Global Variables ==========================================*/
const THREE = window.AFRAME.THREE;
const mr = require('../modern_robotics/modern_robotics_core.js');
const spK = require('../modern_robotics/spatialKinematics.js');
const RobotKinematics = require('../modern_robotics/modern_robotics_Kinematics.js');

// Robot Dynamics module just used for dynamics control testing(not used in this project)
// const RobotDynamcis = require('../modern_robotics/modern_robotics_Dynamics.js');

// Load Robot Model
// Change your robot model: jaka_zu_5, agilex_piper, myCobot280, etc.
const robot_list = [
  { robotId: "left_arm", robot_model: "agilex_piper" },
  { robotId: "right_arm", robot_model: "agilex_piper" },
  { robotId: "cam", robot_model: "myCobot280" }
];

const toolLimit = { min: -1, max: 89 }; 

const Euler_order = 'ZYX'; // Euler angle order

const dt = 25/1000; // VR input period in seconds 

// MQTT Topics
const MQTT_REQUEST_TOPIC = "mgr/request";
const MQTT_DEVICE_TOPIC = "dev/" + idtopic;
const MQTT_CTRL_TOPIC = "control/"; 
const MQTT_ROBOT_STATE_TOPIC = "robot/";
const MQTT_SHARE_TOPIC = "share/";
const MQTT_VR_TOPIC = "vr/";

// IK State Codes
const STATE_CODES = {
  NORMAL: 0x00,
  IK_FAILED: 0x01,
  VELOCITY_LIMIT: 0x02,
  JOINT_LIMIT: 0x03,
};

/* ============================= Functions ==========================================*/
const loadRobotParams = (robot_model) => {
  const rk = new RobotKinematics(robot_model);
  const M = rk.get_M();
  const Slist = rk.get_Slist();
  const Blist = mr.SlistToBlist(M, Slist);
  const jointLimits = rk.jointLimits;
  const jointInitial = rk.get_jointInitial();
  return {
    M, Slist, Blist,
    jointLimits, jointInitial
  };
};

const deg2rad = deg => {
  if (Array.isArray(deg)) {
    return deg.map(d => d * Math.PI / 180); 
  }
  return deg * Math.PI / 180; 
};

function limitDualVelocity(left, right, maxVel) {
  const absLeft = Math.abs(left);
  const absRight = Math.abs(right);
  const maxVal = Math.max(absLeft, absRight);
  if (maxVal <= maxVel) {
    return [left, right];
  }
  const scale = maxVel / maxVal;
  return [left * scale, right * scale];
}


/**
 * Forward Kinematics
 * @param {Object} robotParams // Robot parameters including M, Slist, Blist, jointLimits, jointInitial
 * @param {Array<number>} theta_body // Current joint angles, and also used as initial guess
 * @param {string} VR_Control_Mode // VR control mode: 'inSpace' or 'inBody'
 * @returns {Array<number>}   return T // Return end-effector pose
 */
function FK(robotParams, theta_body, VR_Control_Mode) {
  const M = robotParams.M;
  const Slist = robotParams.Slist;
  const Blist = robotParams.Blist;

  let T;
  if (VR_Control_Mode === 'inBody') {
    T = mr.FKinBody(M, Blist, theta_body);
  }
  else if (VR_Control_Mode === 'inSpace') {
    T = mr.FKinSpace(M, Slist, theta_body);
  } else {
    throw new Error(`Invalid VR_Control_Mode: ${VR_Control_Mode}. Use 'inSpace' or 'inBody'.`);
  }
  return T;
}

/**
 * Inverse Kinematics with Joint Velocity Limit
 * @param {Array<number>} T_sd // Target end-effector pose
 * @param {Object} robotParams // Robot parameters including M, Slist, Blist, jointLimits, jointInitial
 * @param {Array<number>} theta_body // Current joint angles, and also used as initial guess
 * @param {string} VR_Control_Mode // VR control mode: 'inSpace' or 'inBody'
 * @returns {Array<number>} { new_theta_body, error_code }   // Return new joint angles and error code of IK
 */
function IK_joint_velocity_limit(T_sd, robotParams, theta_body, VR_Control_Mode) {
  let thetalist_sol, ik_success;
  const max_joint_velocity = 5.5; // Maximum joint velocity limit

  const M = robotParams.M;
  const Slist = robotParams.Slist;
  const Blist = robotParams.Blist;
  const jointLimits = robotParams.jointLimits;

  let error_code = STATE_CODES.NORMAL;

  if (VR_Control_Mode === 'inBody') {
    [thetalist_sol, ik_success] = mr.IKinBody(Blist, M, T_sd, theta_body, 1e-5, 1e-5);
  } 
  else if (VR_Control_Mode === 'inSpace') {
    [thetalist_sol, ik_success] = mr.IKinSpace(Slist, M, T_sd, theta_body, 1e-5, 1e-5);
  }

  if (ik_success) {
    // Solve IK
    const thetalist_sol_limited = thetalist_sol.map((theta, i) =>
      Math.max(jointLimits[i].min, Math.min(jointLimits[i].max, theta))
    );
    // check if any joint is at its limit
    const jointLimited = thetalist_sol_limited.some((theta, i) =>
      theta === jointLimits[i].min || theta === jointLimits[i].max
    );
    // Compute joint velocity
    const delta_theta_body = thetalist_sol_limited.map((theta, i) => theta - theta_body[i]);
    const d_theta_body = delta_theta_body.map((val) => val / dt);

    const [joint_ometahat, joint_theta] = mr.AxisAng3(d_theta_body);

    const joint_velocity = Math.max(0, Math.min(max_joint_velocity, joint_theta));
    const new_theta_body = theta_body.map((theta, i) => theta + joint_ometahat[i] * joint_velocity * dt);

    if (joint_velocity === max_joint_velocity) {
      console.warn("Joint Velocity Limit Reached");
      error_code = STATE_CODES.VELOCITY_LIMIT;
      return { new_theta_body, error_code }; 
    } else if (jointLimited) {
      console.warn("Joint Angle Limit Reached");
      error_code = STATE_CODES.JOINT_LIMIT;
      return { new_theta_body, error_code };
    } else {
      return { new_theta_body, error_code };
    }
  } else {
    console.warn("IK failed to converge");
    error_code = STATE_CODES.IK_FAILED;
    return { new_theta_body: theta_body, error_code };
  }
}

function IK_joint_velocity_dual(T_sd, robotParams, theta_body, VR_Control_Mode) {
  let thetalist_sol, ik_success;
  const max_joint_velocity = 3.14; // Maximum joint velocity limit

  const M = robotParams.M;
  const Slist = robotParams.Slist;
  const Blist = robotParams.Blist;
  const jointLimits = robotParams.jointLimits;

  let error_code = STATE_CODES.NORMAL;
  let [joint_ometahat, joint_theta] = [Array(theta_body.length).fill(0), 0];

  if (VR_Control_Mode === 'inBody') {
    [thetalist_sol, ik_success] = mr.IKinBody(Blist, M, T_sd, theta_body, 1e-5, 1e-5);
  } 
  else if (VR_Control_Mode === 'inSpace') {
    [thetalist_sol, ik_success] = mr.IKinSpace(Slist, M, T_sd, theta_body, 1e-5, 1e-5);
  }

  if (ik_success) {
    // Solve IK
    const thetalist_sol_limited = thetalist_sol.map((theta, i) =>
      Math.max(jointLimits[i].min, Math.min(jointLimits[i].max, theta))
    );
    // check if any joint is at its limit
    const jointLimited = thetalist_sol_limited.some((theta, i) =>
      theta === jointLimits[i].min || theta === jointLimits[i].max
    );
    // Compute joint velocity
    const delta_theta_body = thetalist_sol_limited.map((theta, i) => theta - theta_body[i]);
    const d_theta_body = delta_theta_body.map((val) => val / dt);

    const [joint_ometahat, joint_theta] = mr.AxisAng3(d_theta_body);
    const joint_velocity = Math.max(0, Math.min(max_joint_velocity, joint_theta));

    if (joint_velocity === max_joint_velocity) {
      console.warn("Joint Velocity Limit Reached");
      error_code = STATE_CODES.VELOCITY_LIMIT;
    } 
    else if (jointLimited) {
      console.warn("Joint Angle Limit Reached");
      error_code = STATE_CODES.JOINT_LIMIT;
    } else {
      error_code = STATE_CODES.NORMAL;
    }  
    return { joint_ometahat, joint_theta, error_code }; 

  } else {
    console.warn("IK failed to converge");
    error_code = STATE_CODES.IK_FAILED;
    return { joint_ometahat, joint_theta, error_code }; 
  }
}

/**
 * VR Controller Relative Rotation Matrix Update
 * @param {Array<number>} controller_object.quaternion // controller quaternion in 3D space
 * @returns {Array<number>} vr_controller_R_relative   // Return the relative Rotation Matrix of the VR controller
 */
function vrquatToR(vr_controller_quat, VR_Control_Mode, hand, mode) {
  const offset_left = [0, Math.PI/2, -Math.PI]; // Predefined offset angles for different modes
  const offset_right = [0, Math.PI/2, -Math.PI]; // Predefined offset angles for different modes

  // Initial offset quaternion to correct the VR controller orientation
  const initialOffsetQuat = new THREE.Quaternion().setFromEuler(
    new THREE.Euler((0.6654549523360951 * -1), 0, 0, Euler_order)
    // new THREE.Euler(0, 0, 0, Euler_order)
  );
  const vrQuat = new THREE.Quaternion().multiplyQuaternions(vr_controller_quat, initialOffsetQuat);

  // Transform vr controller's frame to world frame
  const worldOffsetQuat = new THREE.Quaternion().setFromEuler(
    new THREE.Euler((Math.PI/2 * -1), Math.PI/2, 0, Euler_order)
  );
  const spaceQuat = new THREE.Quaternion().multiplyQuaternions(vrQuat, worldOffsetQuat);

  // Offset quaternion to correct the end effector orientation
  const bodyOffsetQuat = new THREE.Quaternion().setFromEuler(
    // new THREE.Euler( Math.PI, 0, Math.PI, Euler_order)
    new THREE.Euler( -Math.PI/2, 0, 0, Euler_order)
  );
  const bodyQuat = new THREE.Quaternion().multiplyQuaternions(spaceQuat, bodyOffsetQuat);

  // Transform the corrected quaternion to a rotation matrix
  const matrix = new THREE.Matrix4();
  if (VR_Control_Mode === 'inSpace') {
    matrix.makeRotationFromQuaternion(spaceQuat);
  } else if (VR_Control_Mode === 'inBody') {
    if (hand === 'right') {
      const bodyOffsetQuat_right = new THREE.Quaternion().setFromEuler(
        new THREE.Euler( 0, 0, -Math.PI/2 + offset_right[mode], Euler_order)
      );
      const bodyQuat_right = new THREE.Quaternion().multiplyQuaternions(bodyQuat, bodyOffsetQuat_right);
      matrix.makeRotationFromQuaternion(bodyQuat_right);
    }
    else if (hand === 'left') {
      const bodyOffsetQuat_left = new THREE.Quaternion().setFromEuler(
        new THREE.Euler( Math.PI, 0, -Math.PI/2 + offset_left[mode], Euler_order)
      );
      const bodyQuat_left = new THREE.Quaternion().multiplyQuaternions(bodyQuat, bodyOffsetQuat_left);
      matrix.makeRotationFromQuaternion(bodyQuat_left);
    }
  }

  // The columns of the rotation matrix represent the axes of the controller
  const elements = matrix.elements;
  const vr_controller_R = [
    [elements[0], elements[4], elements[8]],
    [elements[1], elements[5], elements[9]],
    [elements[2], elements[6], elements[10]]
  ];
  return vr_controller_R;
}


export default function DynamicHome(props) {
  const [now, setNow] = React.useState(Date.now());
  const [rendered, set_rendered] = React.useState(false)

  const robotNameList = ["Model"]
  const [robotName,set_robotName] = React.useState(robotNameList[0])

  const [robot_model_left, setRobotModelLeft] = React.useState("agilex_piper");
  const [robot_model_right, setRobotModelRight] = React.useState("agilex_piper");
  const [robot_model_cam, setRobotModelCam] = React.useState("myCobot280");

  const [robotParams, setRobotParams] = React.useState({
    left: null,  // left control robot parameters
    right: null, // right control robot parameters
    cam: null,   // camera robot parameters
  });

  React.useEffect(() => {
    const leftParams = loadRobotParams(robot_model_left);
    const rightParams = loadRobotParams(robot_model_right);
    const camParams = loadRobotParams(robot_model_cam);
    setRobotParams((prev) => ({
      ...prev,
      left: leftParams,
      right: rightParams,
      cam: camParams,
    }));
  }, [robot_model_left, robot_model_right, robot_model_cam]);

  React.useEffect(() => {
    if (robotParams.left !== null && robotParams.right !== null && robotParams.cam !== null) {
      console.log("Load Robot Params Left:", robotParams.left);
      console.log("Load Robot Params Right:", robotParams.right);
      console.log("Load Robot Params Cam:", robotParams.cam);
    }
  }, [robotParams.left, robotParams.right, robotParams.cam]);

  React.useEffect(() => {
    console.log("Viewer mounted");
    return () => console.log("Viewer unmounted");
  }, []);

  const [error_code, setErrorCode] = React.useState(STATE_CODES.NORMAL);
  const [error_code_left, setErrorCodeLeft] = React.useState(STATE_CODES.NORMAL);
  const [error_code_cam, setErrorCodeCam] = React.useState(STATE_CODES.NORMAL);

  // VR controller state
  const vrModeRef = React.useRef(false);
  
  // Right Controller
  const [trigger_on,set_trigger_on] = React.useState(false)
  const [grip_on, set_grip_on] = React.useState(false)
  const [button_a_on, set_button_a_on] = React.useState(false)
  const [button_b_on, set_button_b_on] = React.useState(false)
  const [thumbstick_right, setThumbstickRight] = React.useState([0, 0]);
  const [thumbstick_down_right, setThumbstickDownRight] = React.useState(false);
  const [controller_object, set_controller_object] = React.useState(() => {
    const controller_object = new THREE.Object3D();
    console.log("Right Controller Object Created:", controller_object);
    return controller_object;
    });

  // Left Controller
  const [trigger_on_left,set_trigger_on_left] = React.useState(false)
  const [grip_on_left, set_grip_on_left] = React.useState(false)
  const [button_x_on, set_button_x_on] = React.useState(false)
  const [button_y_on, set_button_y_on] = React.useState(false)
  const [thumbstick_left, setThumbstickLeft] = React.useState([0, 0]);
  const [thumbstick_down_left, setThumbstickDownLeft] = React.useState(false);
  const [controller_object_left, set_controller_object_left] = React.useState(() => {
    const controller_object_left = new THREE.Object3D();
    console.log("Left Controller Object Created:", controller_object_left);
    return controller_object_left;
  });

  // Menu
  const [showMenu, setShowMenu] = React.useState(false);
  const [left_arm_mode, setLeftArmMode] = React.useState('free'); // 'free' or 'assist'
  const [VR_Control_Mode, setControlMode] = React.useState('inSpace');
  const [indicator, setIndicator] = React.useState(false);
  const [shareControl, setShareControl] = React.useState(false);

 // MQTT
  const [selectedMode, setSelectedMode] = React.useState('control'); 
  const robotIDRef = React.useRef(idtopic);
  const browserIdRef = React.useRef(idtopic); 

  // View Camera Pose
  const [view_cam_pose, setViewCamPose] = React.useState([0.24, 0.3, -0.67, 0, 150, 0]);

  // Message Display
  const [dsp_message, set_dsp_message] = React.useState("")

  // WebRTC Recv
  const [webcamStream1, setWebcamStream1] = React.useState(null);
  const [webcamStream2, setWebcamStream2] = React.useState(null);
  const [webcamStream3, setWebcamStream3] = React.useState(null);

  // WebRTC Send
  const [vrLeftStream, setVrLeftStream] = React.useState(null);
  const [vrRightStream, setVrRightStream] = React.useState(null);
  const [controlData, setControlData] = React.useState(null);
  const [DataRecv, setDataRecv] = React.useState(null);

  // Robot Tool
  const toolNameList = ["No tool"]
  const [toolName,set_toolName] = React.useState(toolNameList[0])

  // Frame ID
  const reqIdRef = React.useRef()
  
  // Animation loop
  const loop = ()=>{
    reqIdRef.current = window.requestAnimationFrame(loop) 
  }
  React.useEffect(() => {
    loop()
    return () => window.cancelAnimationFrame(reqIdRef.current) 
  },[])

  // Change Robot
  const robotChange = ()=>{
    const get = (robotName)=>{
      let changeIdx = robotNameList.findIndex((e)=>e===robotName) + 1
      if(changeIdx >= robotNameList.length){
        changeIdx = 0
      }
      return robotNameList[changeIdx]
    }
    set_robotName(get)
  }

  // Set Model Opacity
  const [modelOpacity, setModelOpacity] = React.useState(1.0); 
    React.useEffect(() => {
    const scene = document.querySelector('a-scene');
    if (!scene) return;
    
    // In VR mode, set model opacity to 0.3
    function handleEnterVR() {
      setModelOpacity(0.3);
    }
    // In viewer mode, set model opacity to 1.0
    function handleExitVR() {
      setModelOpacity(1.0);
    }

    scene.addEventListener('enter-vr', handleEnterVR);
    scene.addEventListener('exit-vr', handleExitVR);

    return () => {
      scene.removeEventListener('enter-vr', handleEnterVR);
      scene.removeEventListener('exit-vr', handleExitVR);
    };
  }, []);


  /* ---------------------- Control Parameters ------------------------------------*/
  // Right Arm 
  const [robot_state, setRobotState] = React.useState(null);
  const [theta_body, setThetaBody] = React.useState([0, 0, 0, 0, 0, 0]);
  const [theta_tool, setThetaTool] = React.useState(89);
  const [rightArmPosition, setRightArmPosition] = React.useState([0.3, 0.0, 0]);
  const [spinor_omegahat_right, setSpinorOmegaHatRight] = React.useState([0, 0, 0, 0, 0, 0]);
  const [spinor_theta_right, setSpinorThetaRight] = React.useState(0);

  // Left Arm
  const [robot_state_left, setRobotStateLeft] = React.useState(null);
  const [theta_body_left, setThetaBodyLeft] = React.useState([0, 0, 0, 0, 0, 0]);
  const [theta_tool_left, setThetaToolLeft] = React.useState(89);
  const [leftArmPosition, setLeftArmPosition] = React.useState([-0.3, 0.0, 0]);
  const [spinor_omegahat_left, setSpinorOmegaHatLeft] = React.useState([0, 0, 0, 0, 0, 0]);
  const [spinor_theta_left, setSpinorThetaLeft] = React.useState(0);

  // CAM Arm
  const [robot_state_cam, setRobotStateCam] = React.useState(null);
  const [theta_body_cam, setThetaBodyCam] = React.useState([0, 0, 0, 0, 0, 0]);

  // Collision Check
  const [collision, setCollision] = React.useState(false);

  // Controller Euler Offset
  const [offset_mode_left, setOffsetModeL] = React.useState(0);
  const [offset_mode_right, setOffsetModeR] = React.useState(0);

  /* ---------------------- Right Arm Initialize ------------------------------------*/
  const [position_ee, setPositionEE] = React.useState([0,0,0]);
  const [euler_ee, setEuler] = React.useState([0,0,0]);
  const [R_ee, setREE] = React.useState(
    [[1,0,0],
    [0,1,0],
    [0,0,1]]
  );

  const position_ee_Three = mr.worlr2three(position_ee);
  const euler_ee_Three = mr.worlr2three(euler_ee);

  const [rightArmInitialized, setRightArmInitialized] = React.useState(false);

  React.useEffect(() => {
    if (robotParams.right !== null) {
      const jointInitial_right = deg2rad([0, 115-90, -42+169.997, 90, 58, 0]);
      if (!rightArmInitialized) {
        setThetaBody(jointInitial_right);
        setRightArmInitialized(true);
        console.log("Right Robot Arm Initialized");
      }
    }
  }, [robotParams]);

  React.useEffect(() => {
    if (rendered) {
      const T_right = FK(robotParams.right, theta_body, VR_Control_Mode);
      const [R_right, p] = mr.TransToRp(T_right);
      setPositionEE(p);
      setEuler(mr.RotMatToEuler(R_right, Euler_order)); 
      setREE(R_right);
    }
  }, [robotParams, theta_body]);

  /* ---------------------- Left Arm Initialize ------------------------------------*/
  const [position_ee_left, setPositionEELeft] = React.useState([0,0,0]);
  const [euler_ee_left, setEulerEELeft] = React.useState([0,0,0]);
  const [R_ee_left, setREELeft] = React.useState(
    [[1,0,0],
     [0,1,0],
     [0,0,1]]
  );

  const position_ee_Three_left = mr.worlr2three(position_ee_left);
  const euler_ee_Three_left = mr.worlr2three(euler_ee_left);

  const [leftArmInitialized, setLeftArmInitialized] = React.useState(false);

  React.useEffect(() => {
    if (robotParams.left !== null) {
      const jointInitial_left = deg2rad([0, 115-90, -42+169.997, -90, 58, 0])
      if (!leftArmInitialized) {
        setThetaBodyLeft(jointInitial_left);
        setLeftArmInitialized(true);
        console.log("Left Robot Arm Initialized", jointInitial_left);
      } 
    }
  }, [robotParams]);

  React.useEffect(() => {
    if (leftArmInitialized){
      const T_left = FK(robotParams.left, theta_body_left, VR_Control_Mode);
      const [R_left, p_left] = mr.TransToRp(T_left);
      setPositionEELeft(p_left);
      setEulerEELeft(mr.RotMatToEuler(R_left, Euler_order)); 
      setREELeft(R_left);
    }
  }, [theta_body_left, left_arm_mode, robotParams]);

  /*======================= VR Right Robot Arm Control ====================================*/
  /* ---------------------- Right VR Controller Motion ------------------------------------*/
  // Update VR controller position and rotation matrix
  // !! Do not use Euler angle for control, since it can cause gimbal lock !!
  
  /*** Position Update ***/
  const vr_controller_pos = [
  controller_object.position.x,
  controller_object.position.y,
  controller_object.position.z
  ];

  // Use last VR controller position for delta calculation
  const lastVRPosRef = React.useRef(null);

  // Return delta position of VR controller for robot control
  const [vr_controller_p_diff, setVRControllerPosDiff] = React.useState([0, 0, 0]);

  React.useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on) {
      if (!lastVRPosRef.current) {
        // First time trigger is pressed, store the initial position
        lastVRPosRef.current = [...vr_controller_pos];
        setVRControllerPosDiff([0, 0, 0]); 

      } else {
        const pos_diff = [
          vr_controller_pos[0] - lastVRPosRef.current[0],
          vr_controller_pos[1] - lastVRPosRef.current[1], 
          vr_controller_pos[2] - lastVRPosRef.current[2]
        ];

        const pos_diff_world = mr.three2world(pos_diff);
        // console.log("VR Controller Position Diff:", pos_diff_world);

        // Update last frame position
        lastVRPosRef.current = [...vr_controller_pos];
        setVRControllerPosDiff(pos_diff_world);
      }
    }
  }, [
    controller_object.position.x,
    controller_object.position.y,
    controller_object.position.z,
    rendered, 
    trigger_on, 
  ]);

  /*** Rotation Update ***/
  const vr_controller_R_current = vrquatToR(controller_object.quaternion, VR_Control_Mode, 'right', offset_mode_right);

  // Store initial rotation matrix when trigger is first pressed
  const lastRotationMatrixRef = React.useRef(null);
  const [vr_controller_R_relative, setVRControllerRmatrixRelative] = React.useState(
    [[1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]]
  );
  
  React.useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on) {
      if (!lastRotationMatrixRef.current) {
        // First time trigger is pressed, store the initial rotation matrix
        lastRotationMatrixRef.current = [
          [...vr_controller_R_current[0]],
          [...vr_controller_R_current[1]],
          [...vr_controller_R_current[2]]
        ];

        const identity_matrix = [
          [1, 0, 0],
          [0, 1, 0],
          [0, 0, 1]
        ];
        setVRControllerRmatrixRelative(identity_matrix);
        // console.log("Initial rotation matrix stored:", lastRotationMatrixRef.current);
      } else {

        const vr_controller_R_relative = spK.calculateRelativeRotationMatrix(
          vr_controller_R_current, 
          lastRotationMatrixRef.current,
          VR_Control_Mode
        );

        setVRControllerRmatrixRelative(vr_controller_R_relative);
        lastRotationMatrixRef.current = [
          [...vr_controller_R_current[0]],
          [...vr_controller_R_current[1]],
          [...vr_controller_R_current[2]]
        ];
      }
    }
  }, [
    controller_object.quaternion.x,
    controller_object.quaternion.y,
    controller_object.quaternion.z,
    controller_object.quaternion.w,
    rendered, 
    trigger_on,
    vrModeRef.current
  ]);

  // Reset when trigger is released
  /* 
  *  Note: 
  *  If the VR position difference and relative rotation matrix are not reset, 
  *  residual values may accumulate and cause the robot to jump.
  */
  React.useEffect(() => {
    if (rendered && vrModeRef.current && !trigger_on && lastVRPosRef.current) {
      lastVRPosRef.current = null;
      lastRotationMatrixRef.current = null;
      setVRControllerPosDiff([0, 0, 0]);
      setVRControllerRmatrixRelative(
        [[1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]]
      );
    }
  }, [
      trigger_on, 
      rendered, 
      vrModeRef.current,
      lastVRPosRef.current
    ]);

  /* ---------------------- Right Arm VR Control ------------------------------------*/
  React.useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on && !showMenu && !shareControl) {
      const currentP = [...position_ee];
      const currentR = [...R_ee];

      // Calculate the new position and orientation based on VR controller input
      const [p_v, p_theta] = mr.AxisAng3(vr_controller_p_diff);
      const p_scale = 1.0; // Scale factor for position movement

      const newP = [
        currentP[0] + p_v[0] * p_theta * p_scale, // Scale factor for position
        currentP[1] + p_v[1] * p_theta * p_scale,
        currentP[2] + p_v[2] * p_theta * p_scale
      ];

      const R_scale = 1.0
      const [R_screw, R_theta] = spK.relativeRMatrixtoScrewAxis(vr_controller_R_relative);

      // Calculate the new orientation based on the relative rotation matrix
      let newT;
      if (VR_Control_Mode === 'inSpace') {
        const R_screw_world = [-R_screw[2], -R_screw[0], R_screw[1]];
        const R_relative = spK.ScrewAxisToRelativeRMatrix(R_screw_world, R_theta * R_scale); 
        const newR_inSpace = mr.matDot(R_relative, currentR);
        newT = mr.RpToTrans(newR_inSpace, newP);
      }
      else if (VR_Control_Mode === 'inBody') {
        const R_relative = spK.ScrewAxisToRelativeRMatrix(R_screw, R_theta * R_scale); 
        const newR_inBody = mr.matDot(currentR, R_relative);
        newT = mr.RpToTrans(newR_inBody, newP);
      }
      else {
        console.warn("Invalid VR Control Mode, choose 'inSpace' or 'inBody'. Current:", VR_Control_Mode);
        return;
      }

      // const { new_theta_body, error_code } = IK_joint_velocity_limit(newT, robotParams.right, theta_body, VR_Control_Mode);
      // setThetaBody(new_theta_body);
      // setErrorCode(error_code);

      if (left_arm_mode === 'free'){
        const { new_theta_body, error_code } = IK_joint_velocity_limit(newT, robotParams.right, theta_body, VR_Control_Mode);
        setThetaBody(new_theta_body);
        setErrorCode(error_code);
      } else if (left_arm_mode === 'assist'){
        const  { joint_ometahat, joint_theta, error_code } = IK_joint_velocity_dual(newT, robotParams.right, theta_body, VR_Control_Mode);
        setSpinorOmegaHatRight(joint_ometahat);
        setSpinorThetaRight(joint_theta);
        setErrorCode(error_code);
      }

    }
  }, [
    vr_controller_p_diff,
    vr_controller_R_relative,
    rendered, 
    vrModeRef.current,
    lastVRPosRef.current,
    lastRotationMatrixRef.current,
  ]);

  /* ---------------------- Right Arm Tool VR Control ------------------------------------*/
  function clampTool(value) {
    return Math.max(toolLimit.min, Math.min(toolLimit.max, value));
  }
  React.useEffect(() => {
    let intervalId = null;
    if (button_b_on) {
      intervalId = setInterval(() => {
        setThetaTool(prev => clampTool(prev + 2.5));
      }, dt); 
    }
    else if (button_a_on) {
      intervalId = setInterval(() => {
        setThetaTool(prev => clampTool(prev - 2.5));
      }, dt); 
    }
    return () => {
      if (intervalId) clearInterval(intervalId);
    };
  }, [ button_a_on, button_b_on]);


  /*======================= VR Right Left Arm Control ====================================*/
  /* ---------------------- Left VR Controller Motion ------------------------------------*/
  /*** Position Update ***/
  const vr_controller_pos_left = [
  controller_object_left.position.x,
  controller_object_left.position.y,
  controller_object_left.position.z
  ];

  // Use last VR controller position for delta calculation
  const lastVRPosRef_left = React.useRef(null);

  // Return delta position of VR controller for robot control
  const [vr_controller_p_diff_left, setVRControllerPosDiffLeft] = React.useState([0, 0, 0]);

  React.useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on_left) {
      if (!lastVRPosRef_left.current) {
        // First time trigger is pressed, store the initial position
        lastVRPosRef_left.current = [...vr_controller_pos_left];
        setVRControllerPosDiffLeft([0, 0, 0]);

      } else {
        const pos_diff_left = [
          vr_controller_pos_left[0] - lastVRPosRef_left.current[0],
          vr_controller_pos_left[1] - lastVRPosRef_left.current[1],
          vr_controller_pos_left[2] - lastVRPosRef_left.current[2]
        ];

        const pos_diff_world_left = mr.three2world(pos_diff_left);

        // Update last frame position
        lastVRPosRef_left.current = [...vr_controller_pos_left];
        setVRControllerPosDiffLeft(pos_diff_world_left);
      }
    }
  }, [
    controller_object_left.position.x,
    controller_object_left.position.y,
    controller_object_left.position.z,
    rendered, 
    trigger_on_left, 
  ]);

  /*** Rotation Update ***/
  const vr_controller_R_current_left = vrquatToR(controller_object_left.quaternion, VR_Control_Mode, 'left', offset_mode_left);

  // Store initial rotation matrix when trigger is first pressed
  const lastRotationMatrixRef_left = React.useRef(null);
  const [vr_controller_R_relative_left, setVRControllerRmatrixRelativeLeft] = React.useState(
    [[1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]]
  );
  
  React.useEffect(() => {
    if (rendered && vrModeRef.current && trigger_on_left) {
      if (!lastRotationMatrixRef_left.current) {
        // First time trigger is pressed, store the initial rotation matrix
        lastRotationMatrixRef_left.current = [
          [...vr_controller_R_current_left[0]],
          [...vr_controller_R_current_left[1]],
          [...vr_controller_R_current_left[2]]
        ];

        const identity_matrix = [
          [1, 0, 0],
          [0, 1, 0],
          [0, 0, 1]
        ];
        setVRControllerRmatrixRelativeLeft(identity_matrix);
        // console.log("Initial rotation matrix stored:", lastRotationMatrixRef.current);
      } else {

        const vr_controller_R_relative = spK.calculateRelativeRotationMatrix(
          vr_controller_R_current_left,
          lastRotationMatrixRef_left.current,
          VR_Control_Mode
        );

        setVRControllerRmatrixRelativeLeft(vr_controller_R_relative);
        lastRotationMatrixRef_left.current = [
          [...vr_controller_R_current_left[0]],
          [...vr_controller_R_current_left[1]],
          [...vr_controller_R_current_left[2]]
        ];
      }
    }
  }, [
    controller_object_left.quaternion.x,
    controller_object_left.quaternion.y,
    controller_object_left.quaternion.z,
    controller_object_left.quaternion.w,
    rendered, 
    trigger_on_left,
    vrModeRef.current
  ]);

  // Reset when trigger is released
  /* 
  *  Note: 
  *  If the VR position difference and relative rotation matrix are not reset, 
  *  residual values may accumulate and cause the robot to jump.
  */
  React.useEffect(() => {
    if (rendered && vrModeRef.current && !trigger_on_left && lastVRPosRef_left.current) {
      lastVRPosRef_left.current = null;
      lastRotationMatrixRef_left.current = null;
      setVRControllerPosDiffLeft([0, 0, 0]);
      setVRControllerRmatrixRelativeLeft(
        [[1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]]
      );
    }
  }, [trigger_on_left, rendered, vrModeRef.current]);

  /*---------------------------- Left Arm Control ----------------------------------------*/
  React.useEffect(() => {
    const originalM = [
        [1, 0, 0, -0.0219],
        [0, 1, 0, 0],
        [0, 0, 1, 0.88558],
        [0, 0, 0, 1]
    ]

    const position_ee_left_world = [
      position_ee_left[0] ,
      position_ee_left[1] - leftArmPosition[0],
      position_ee_left[2]
    ]

    const position_ee_right_world = [
      position_ee[0],
      position_ee[1] - rightArmPosition[0],
      position_ee[2]
    ]
 
    const arm_offset_world = [
      position_ee_right_world[0] - position_ee_left_world[0],
      position_ee_right_world[1] - position_ee_left_world[1],
      position_ee_right_world[2] - position_ee_left_world[2]
    ]

    const arm_offset_body_left = mr.matDot(mr.RotInv(R_ee_left), arm_offset_world);
    const arm_offset_body_right = mr.matDot(mr.RotInv(R_ee), arm_offset_world);

    const newM_left = [
        [1, 0, 0, -0.0219 + arm_offset_body_left[0]/2],
        [0, 1, 0, 0       + arm_offset_body_left[1]/2],
        [0, 0, 1, 0.88558 + arm_offset_body_left[2]/2],
        [0, 0, 0, 1]
    ]

    const newM_right = [
        [1, 0, 0, -0.0219 - arm_offset_body_right[0]/2],
        [0, 1, 0, 0       - arm_offset_body_right[1]/2],
        [0, 0, 1, 0.88558 - arm_offset_body_right[2]/2],
        [0, 0, 0, 1]
    ]

    if (left_arm_mode === 'free') {
      setRobotParams(prev => ({
        ...prev,
        left: {
          ...prev.left,
          M: originalM,
        },
        right: {
          ...prev.right,
          M: originalM,
        }
      }));
    } else if (left_arm_mode === 'assist') {
      setRobotParams(prev => ({
        ...prev,
        left: {
          ...prev.left,
          M: newM_left,
        },
        right: {
          ...prev.right,
          M: newM_right,
        }
      }));
    }
  }, [left_arm_mode]);


  React.useEffect(() => {
    if (rendered && vrModeRef.current && !showMenu && !shareControl) {
      let vr_p, vr_R;
      if (left_arm_mode === 'free') {
        vr_p = vr_controller_p_diff_left;
        vr_R = vr_controller_R_relative_left;
      } else if (left_arm_mode === 'assist') {
        vr_p = vr_controller_p_diff;
        vr_R = vr_controller_R_relative;
      }

      const currentP_left = [...position_ee_left];
      const currentR_left = [...R_ee_left];

      // Calculate the new position and orientation based on VR controller input
      const [p_v_left, p_theta_left] = mr.AxisAng3(vr_p);
      const p_scale = 1.0; // Scale factor for position movement

      const newP = [
        currentP_left[0] + p_v_left[0] * p_theta_left * p_scale, // Scale factor for position
        currentP_left[1] + p_v_left[1] * p_theta_left * p_scale,
        currentP_left[2] + p_v_left[2] * p_theta_left * p_scale
      ];

      const R_scale = 1.0
      const [R_screw_left, R_theta_left] = spK.relativeRMatrixtoScrewAxis(vr_R);

      // Calculate the new orientation based on the relative rotation matrix
      let newT;
      if (VR_Control_Mode === 'inSpace') {
        const R_screw_world = [-R_screw_left[2], -R_screw_left[0], R_screw_left[1]];
        const R_relative_left = spK.ScrewAxisToRelativeRMatrix(R_screw_world, R_theta_left * R_scale); // Scale factor for rotation
        const newR_inSpace = mr.matDot(R_relative_left, currentR_left);
        newT = mr.RpToTrans(newR_inSpace, newP);
      }
      else if (VR_Control_Mode === 'inBody') {
        const R_relative_left = spK.ScrewAxisToRelativeRMatrix(R_screw_left, R_theta_left * R_scale); // Scale factor for rotation
        const newR_inBody = mr.matDot(currentR_left, R_relative_left);
        newT = mr.RpToTrans(newR_inBody, newP);
      }
      else {
        console.warn("Invalid VR Control Mode, choose 'inSpace' or 'inBody'. Current:", VR_Control_Mode);
        return;
      }

      // Update Joint Angles with IK
      if (left_arm_mode === 'free' && trigger_on_left) {
        const { new_theta_body, error_code } = IK_joint_velocity_limit(newT, robotParams.left, theta_body_left, VR_Control_Mode);
        setThetaBodyLeft(new_theta_body);
        setErrorCodeLeft(error_code);
      } else if (left_arm_mode === 'assist') {
        const { joint_ometahat, joint_theta, error_code } = IK_joint_velocity_dual(newT, robotParams.left, theta_body_left, VR_Control_Mode);
        setSpinorOmegaHatLeft(joint_ometahat);
        setSpinorThetaLeft(joint_theta);
        setErrorCodeLeft(error_code);
      }
    }
  }, [
    vr_controller_p_diff_left,
    vr_controller_R_relative_left,
    vr_controller_p_diff,
    vr_controller_R_relative,
    rendered, 
    vrModeRef.current
  ]);

  /*---------------------------- Left Arm Tool Control ----------------------------------------*/
  React.useEffect(() => {
    let intervalId = null;
    if (button_y_on) {
      intervalId = setInterval(() => {
        setThetaToolLeft(prev => clampTool(prev + 1.0));
      }, dt); 
    }
    else if (button_x_on) {
      intervalId = setInterval(() => {
        setThetaToolLeft(prev => clampTool(prev - 1.0));
      }, dt); 
    }
    return () => {
      if (intervalId) clearInterval(intervalId);
    };
  }, [ button_x_on, button_y_on]);

  React.useEffect(() => {
    if (thumbstick_down_left) {
      setShowMenu(prev => !prev);
      setThumbstickDownLeft(false);
      console.log("Show Menu:", !showMenu);
    }
  }, [thumbstick_down_left]);

  React.useEffect(() => {
    if (thumbstick_down_right) {
      setShareControl(prev => !prev);
      setThumbstickDownRight(false);
      console.log("Shared Control On:", !shareControl);
    }
  }, [thumbstick_down_right]);

  /*========================= Dual Arm Control ================================*/
  React.useEffect(() => {
    if (left_arm_mode === 'assist' && trigger_on && !showMenu && !shareControl) {

      let dual_arm_state
      if (error_code === STATE_CODES.JOINT_LIMIT || error_code_left === STATE_CODES.JOINT_LIMIT) {
        dual_arm_state = STATE_CODES.JOINT_LIMIT
      } else if (error_code === STATE_CODES.VELOCITY_LIMIT || error_code_left === STATE_CODES.VELOCITY_LIMIT) {
        dual_arm_state = STATE_CODES.VELOCITY_LIMIT
      } else if (error_code === STATE_CODES.IK_FAILED || error_code_left === STATE_CODES.IK_FAILED) {
        dual_arm_state = STATE_CODES.IK_FAILED
      } else {
        dual_arm_state = STATE_CODES.NORMAL
      }

      if (dual_arm_state === STATE_CODES.NORMAL || dual_arm_state === STATE_CODES.VELOCITY_LIMIT) {
        const maxVel = 10
        let [safe_left, safe_right] = limitDualVelocity(spinor_theta_left, spinor_theta_right, maxVel);
        const new_theta_body_left = theta_body_left.map((theta, i) => theta + spinor_omegahat_left[i] * safe_left/2 * dt);
        const new_theta_body_right = theta_body.map((theta, i) => theta + spinor_omegahat_right[i] * safe_right/2 * dt);
        setThetaBodyLeft(new_theta_body_left);
        setThetaBody(new_theta_body_right);
      } else if (dual_arm_state === STATE_CODES.JOINT_LIMIT) {
        console.warn("⚠️ Dual Arm Joint Limit Reached");
        const new_theta_body_left = theta_body_left.map((theta, i) => theta + spinor_omegahat_left[i] * -0.1 * dt);
        const new_theta_body_right = theta_body.map((theta, i) => theta + spinor_omegahat_right[i] * -0.1 * dt);
        setThetaBodyLeft(new_theta_body_left);
        setThetaBody(new_theta_body_right);
      } else if (dual_arm_state === STATE_CODES.IK_FAILED) {
        console.warn("⚠️ Dual Arm IK Failed");
      }

    }
  }, [vr_controller_p_diff, vr_controller_R_relative]);

  /*========================= Collision Check ================================*/
  const thetaHistoryRef = React.useRef([]);
  const thetaLeftHistoryRef = React.useRef([]);

  React.useEffect(() => {
    if (!collision) {
      thetaHistoryRef.current.push(theta_body);
      thetaLeftHistoryRef.current.push(theta_body_left);

      if (thetaHistoryRef.current.length > 5) thetaHistoryRef.current.shift();
      if (thetaLeftHistoryRef.current.length > 5) thetaLeftHistoryRef.current.shift();
    } else if (collision) {
      if (thetaHistoryRef.current.length > 0 && thetaLeftHistoryRef.current.length > 0) {
        const last = thetaHistoryRef.current.pop();
        const lastLeft = thetaLeftHistoryRef.current.pop();

        setThetaBody(last);
        setThetaBodyLeft(lastLeft);

        console.warn("🔁 Return to last valid theta due to collision");
      }
    }
  }, [collision, theta_body, theta_body_left]);

  /* ======================== CAM Arm Control ================================*/
  /* ------------------------- Initialize ------------------------------------*/
  const [position_ee_cam, setPositionEECam] = React.useState([0,0,0]);
  const [euler_ee_cam, setEulerEECam] = React.useState([0,0,0]);
  const [R_ee_cam, setREECam] = React.useState(
    [1,0,0],
    [0,1,0],
    [0,0,1]
  );

  const position_ee_Three_cam = mr.worlr2three(position_ee_cam);
  const euler_ee_Three_cam = mr.worlr2three(euler_ee_cam);

  React.useEffect(() => {
    if (robotParams.cam !== null) {
      const T0_cam = FK(robotParams.cam, robotParams.cam.jointInitial, VR_Control_Mode);
      const [R0_cam, p0_cam] = mr.TransToRp(T0_cam);
      setThetaBodyCam(robotParams.cam.jointInitial);
      setPositionEECam(p0_cam);
      setREECam(R0_cam);
      setEulerEECam(mr.RotMatToEuler(R0_cam, Euler_order));
      console.log("Camera Arm Initialized", robotParams.cam.jointInitial);
    }
  }, [robotParams.cam]);

  React.useEffect(() => {
    if (rendered){
      const T_cam = FK(robotParams.cam, theta_body_cam, VR_Control_Mode);
      const [R_cam, p_cam] = mr.TransToRp(T_cam);
      setPositionEECam(p_cam);
      setEulerEECam(mr.RotMatToEuler(R_cam, Euler_order)); 
      setREECam(R_cam);
      // console.log("Camera FK Updated", theta_body_cam);
    }
  }, [rendered, theta_body_cam]);


  /* ---------------------- CAM Arm Control ------------------------------------*/
  // Pose control
  React.useEffect(() => {
    if (rendered && vrModeRef.current && !showMenu) {
      const euler_sensitivity = 0.010
      const position_sensitivity = 0.002

      let euler
      if (grip_on) {
        euler = [
          thumbstick_left[0] * euler_sensitivity,
          thumbstick_right[0] * euler_sensitivity,
          thumbstick_right[1] * euler_sensitivity
        ]
      }
      else {
        euler = [0, 0, 0]
      }

      const R_relative = mr.EulerToRotMat(euler, Euler_order);

      // In Body
      const R_cam = mr.matDot(R_ee_cam, R_relative);

      // In Space
      // const R_cam = mr.matDot(R_relative, R_ee_cam);

      let position_diff
      if (grip_on_left){      
        position_diff = [ 
          thumbstick_left[0] * position_sensitivity, 
          thumbstick_left[1] * position_sensitivity, 
          -thumbstick_right[1] * position_sensitivity]
      }
      else{
        position_diff = [0, 0, 0]
      }

      const position_relative = mr.matDot(R_ee_cam, position_diff);

      const P_cam = [
        position_ee_cam[0] + position_relative[0],
        position_ee_cam[1] + position_relative[1],
        position_ee_cam[2] + position_relative[2]
      ]

      const T_cam = mr.RpToTrans(R_cam, P_cam);

      const { new_theta_body, error_code } = IK_joint_velocity_limit(T_cam, robotParams.cam, theta_body_cam, 'inBody');
      setThetaBodyCam(new_theta_body);
      setErrorCodeCam(error_code);
    }

  }, [rendered, vrModeRef.current, thumbstick_left, thumbstick_right, grip_on, grip_on_left]);


  /* ========================= Robot Scene Inputs =========================*/
  const interfacePropos = React.useMemo(() => ({
    robotName, robotNameList, set_robotName,
    toolName, toolNameList, set_toolName,
    view_cam_pose, setViewCamPose,
    vr_mode:vrModeRef.current,
    selectedMode, setSelectedMode,
    theta_body, setThetaBody,
    theta_tool, setThetaTool,
    position_ee, setPositionEE,
    euler_ee, setEuler,
    theta_body_left, setThetaBodyLeft,
    theta_tool_left, setThetaToolLeft,
    theta_body_cam, setThetaBodyCam,
  }), [
    robotName, robotNameList, set_robotName,
    toolName, toolNameList, set_toolName,
    view_cam_pose, setViewCamPose,
    vrModeRef.current,
    selectedMode, setSelectedMode,
    theta_body, setThetaBody,
    theta_tool, setThetaTool,
    position_ee, setPositionEE,
    euler_ee, setEuler,
    theta_body_left, setThetaBodyLeft,
    theta_tool_left, setThetaToolLeft,
    theta_body_cam, setThetaBodyCam,
  ]);

  /* =============== VRController Inputs (Aframe Components) =================*/
  React.useEffect(() => {
    registerAframeComponents({
      set_rendered,
      robotChange,

      // Right Controller
      set_controller_object,
      set_trigger_on,
      set_grip_on,
      set_button_a_on,
      set_button_b_on,
      setThumbstickRight,
      setThumbstickDownRight,

      // Left Controller
      set_controller_object_left,
      set_trigger_on_left,
      set_grip_on_left,
      set_button_x_on,
      set_button_y_on,
      setThumbstickLeft,
      setThumbstickDownLeft,

      //Collision Check
      collision,
      setCollision,

      // VR Camera Pose
      setViewCamPose,
      vrModeRef,
      props,
      onXRFrameMQTT,
      
      // Menu
      setShowMenu,
      setLeftArmMode,
      setControlMode,
      setIndicator,
      setShareControl,
    });
  }, []);

  /* ============================== MQTT ==========================================*/

  const thetaBodyMQTT = React.useRef(theta_body);
  React.useEffect(() => {
    thetaBodyMQTT.current = theta_body;
  }, [theta_body]);

  const thetaToolMQTT = React.useRef(theta_tool);
  React.useEffect(() => {
    thetaToolMQTT.current = theta_tool;
  }, [theta_tool]);

  const thetaBodyLeftMQTT = React.useRef(theta_body_left);
  React.useEffect(() => {
    thetaBodyLeftMQTT.current = theta_body_left;
  }, [theta_body_left]);

  const thetaToolLeftMQTT = React.useRef(theta_tool_left);
  React.useEffect(() => {
    thetaToolLeftMQTT.current = theta_tool_left;
  }, [theta_tool_left]);

  const thetaBodyCamMQTT = React.useRef(theta_body_cam);
  React.useEffect(() => {
    thetaBodyCamMQTT.current = theta_body_cam.map(val => Number(val.toFixed(2)));
  }, [theta_body_cam]);

  
  /* ---------------------- Robot Control MQTT message ------------------------------------*/

  /* Animation Frame for MQTT Publish */
  // React.useEffect(() => {
  //   window.requestAnimationFrame(onAnimationMQTT);
  // }, []);
  // const onAnimationMQTT = (time) =>{
  //   const robot_state_json = JSON.stringify({
  //     time: time,
  //     joint: thetaBodyMQTT.current,
  //     tool: thetaToolMQTT.current,
  //     joint_left: thetaBodyLeftMQTT.current,
  //     tool_left: thetaToolLeftMQTT.current,
  //     cam: thetaBodyCamMQTT.current
  //   });
  //   // publishMQTT(MQTT_ROBOT_STATE_TOPIC + robotIDRef.current , robot_state_json); 
  //   // console.log("onAnimationMQTT published:", robot_state_json);
  //   window.requestAnimationFrame(onAnimationMQTT); 
  // }

  /* VR Frame for MQTT Publish */
  const receiveStateRef = React.useRef(true); // VR MQTT switch
  const onXRFrameMQTT = (time, frame) => {
    if (vrModeRef.current){
      frame.session.requestAnimationFrame(onXRFrameMQTT);
      setNow(performance.now()); 
    }
  }
  
  // // Publish control JSON messages (entire)
  // React.useEffect(() => {
  //     const ctl_json = JSON.stringify({
  //       timestamp: Date.now(),
  //       joint: thetaBodyMQTT.current,
  //       tool: thetaToolMQTT.current,
  //       joint_left: thetaBodyLeftMQTT.current,
  //       tool_left: thetaToolLeftMQTT.current,
  //       cam: thetaBodyCamMQTT.current
  //     });
  //     if ((mqttclient != null) && receiveStateRef.current && !shareControl && !showMenu) {
  //       publishMQTT(MQTT_CTRL_TOPIC + browserIdRef.current, ctl_json);
  //       // console.log("onXRFrameMQTT published:", MQTT_CTRL_TOPIC + browserIdRef.current, ctl_json);
  //     }
  // }, [
  //   thetaBodyMQTT.current, 
  //   thetaToolMQTT.current, 
  //   thetaBodyLeftMQTT.current, 
  //   thetaToolLeftMQTT.current, 
  //   thetaBodyCamMQTT.current
  // ]);

  // Publish control JSON messages (individual)
  React.useEffect(() => {
    const json_msg = JSON.stringify({
      timestamp: Date.now(),
      joint: thetaBodyMQTT.current,
    });
    if ((mqttclient != null) && receiveStateRef.current && rendered && !showMenu && !shareControl) {
      publishMQTT(MQTT_CTRL_TOPIC + 'right/' + 'joint/' + browserIdRef.current, json_msg);
    }
  }, [thetaBodyMQTT.current]);

  React.useEffect(() => {
    const json_msg = JSON.stringify({
      timestamp: Date.now(),
      tool: thetaToolMQTT.current,
    });
    if ((mqttclient != null) && receiveStateRef.current && rendered && !showMenu) {
      publishMQTT(MQTT_CTRL_TOPIC + 'right/' + 'tool/' + browserIdRef.current, json_msg);
    }
  }, [thetaToolMQTT.current, robot_state]);

  React.useEffect(() => {
    const json_msg = JSON.stringify({
      timestamp: Date.now(),
      joint: thetaBodyLeftMQTT.current,
    });
    if ((mqttclient != null) && receiveStateRef.current && rendered && !showMenu) {
      publishMQTT(MQTT_CTRL_TOPIC + 'left/' + 'joint/' + browserIdRef.current, json_msg);
    }
  }, [thetaBodyLeftMQTT.current]);

  React.useEffect(() => {
    const json_msg = JSON.stringify({
      timestamp: Date.now(),
      tool: thetaToolLeftMQTT.current,
    });
    if ((mqttclient != null) && receiveStateRef.current && rendered && !showMenu) {
      publishMQTT(MQTT_CTRL_TOPIC + 'left/' + 'tool/' + browserIdRef.current, json_msg);
    }
  }, [thetaToolLeftMQTT.current, robot_state_left]);

  React.useEffect(() => {
    const json_msg = JSON.stringify({
      timestamp: Date.now(),
      joint: thetaBodyCamMQTT.current,
    });
    if ((mqttclient != null) && receiveStateRef.current && rendered && !showMenu) {
      publishMQTT(MQTT_CTRL_TOPIC + 'cam/' + 'joint/' + browserIdRef.current, json_msg);
    }
  }, [thetaBodyCamMQTT.current]);

  /* ---------------------- Visual Assistance MQTT message ------------------------------------*/
  React.useEffect(() => {
      let share_control_flag;
      if (shareControl) {
        share_control_flag = 1;
      } else {
        share_control_flag = 0;
      }

      let share_control_signal;
      if (thumbstick_right[1] < -0.35) {
        share_control_signal = 1;
      } else if (thumbstick_right[1] > 0.7) {
        share_control_signal = -1;
      } else {
        share_control_signal = 0;
      }

      if ((mqttclient != null) && receiveStateRef.current) {
        publishMQTT(MQTT_SHARE_TOPIC + robotIDRef.current, JSON.stringify({
          flag: share_control_flag,
          share: share_control_signal,
        }));
        console.log("Shared Control Published:", share_control_signal);
      }

  }, [thumbstick_right, shareControl]);

  // Robot Request MQTT
  const requestRobot = (mqclient) => {
    const requestInfo = {
      devId: idtopic,
      type: codeType,
    }
    publishMQTT(MQTT_REQUEST_TOPIC, JSON.stringify(requestInfo));
  }

  /* ---------------------- Feedback Update ------------------------------------*/
  // Update theta_body when robot_state is "initialize"
  const [theta_body_feedback, setThetaBodyFeedback] = React.useState([0, 0, 0, 0, 0, 0]);
  React.useEffect(() => {
    if (robot_state === "initialize") {
      setThetaBody(theta_body_feedback)
    }
  }, [robot_state, theta_body_feedback]);

  const [theta_body_left_feedback, setThetaBodyLeftFeedback] = React.useState([0, 0, 0, 0, 0, 0]);
  React.useEffect(() => {
    if (robot_state_left === "initialize") {
      setThetaBodyLeft(theta_body_left_feedback)
    }
  }, [robot_state_left]);

  const [theta_body_cam_feedback, setThetaBodyCamFeedback] = React.useState([0, 0, 0, 0, 0, 0]);
  React.useEffect(() => {
    if (robot_state_cam === "initialize") {
      setThetaBodyCam(theta_body_cam_feedback)
    }
  }, [robot_state_cam]);
  
  /* ---------------------- Option MQTT message -------------------------------*/
  // Send VR Controller Pose Message for Robot Control
  React.useEffect(() => {
    const json_msg = JSON.stringify({
      timestamp: Date.now(),
      p_diff: vr_controller_p_diff,
      R_relative: vr_controller_R_relative,
    });
    if ((mqttclient != null) && receiveStateRef.current && rendered && !showMenu) {
      publishMQTT(MQTT_VR_TOPIC + 'right/' + robotIDRef.current, json_msg);
    }
  }, [vr_controller_p_diff, vr_controller_R_relative]);

  MQTT_Sub({
    // MQTT Client and Topics
    props,
    requestRobot,
    robotIDRef,
    MQTT_DEVICE_TOPIC, 
    MQTT_CTRL_TOPIC, 
    MQTT_ROBOT_STATE_TOPIC,

    // Right Arm
    thetaBodyMQTT: setThetaBody,
    thetaToolMQTT: setThetaTool,
    thetaBodyFeedback: setThetaBodyFeedback,
    robot_state: setRobotState,

    // Left Arm
    thetaBodyLeftMQTT: setThetaBodyLeft,
    thetaToolLeftMQTT: setThetaToolLeft,
    thetaBodyLeftFeedback: setThetaBodyLeftFeedback,
    robot_state_left: setRobotStateLeft,

    // Cam Arm
    thetaBodyCamMQTT: setThetaBodyCam,
    thetaBodyCamFeedback: setThetaBodyCamFeedback,
    robot_state_cam: setRobotStateCam

  });


  /* ============================== WebRTC ==========================================*/
  
  // Get stream from media devices (For testing purpose)
  // React.useEffect(() => {
  //   navigator.mediaDevices.getUserMedia({ video: true, audio: false })
  //     .then(stream => {
  //       setVrLeftStream(stream);
  //       setVrRightStream(stream); 
  //     });
  // }, []);

  /* ------------------------ Get Stream from WebXR ------------------------*/
  React.useEffect(() => {
    if (!vrModeRef.current && !rendered) {
      console.warn('⚠️ Not in VR mode');
      return;
    }

    const scene = document.querySelector('a-scene');
    if (!scene) {
      console.warn('⚠️ a-scene not found');
      return;
    }

    const getCanvasStream = () => {
      const sceneEl = document.querySelector('a-scene');
      const canvas = sceneEl.renderer.domElement;
      const stream = canvas.captureStream(30);  // 30 FPS
      return stream;
    };

    // Listen VR enter event
    const handleEnterVR = async () => {
      console.log('🥽 Entering VR, capturing streams...');
      const stream = getCanvasStream();
      setVrLeftStream(stream);
      setVrRightStream(stream);
    };

    const handleExitVR = () => {
      console.log('🚪 Exiting VR, stopping streams...');
      
      // Stop all tracks
      if (vrLeftStream) {
        vrLeftStream.getTracks().forEach(track => track.stop());
        setVrLeftStream(null);
      }
      if (vrRightStream) {
        vrRightStream.getTracks().forEach(track => track.stop());
        setVrRightStream(null);
      }
    };

    scene.addEventListener('enter-vr', handleEnterVR);
    scene.addEventListener('exit-vr', handleExitVR);

    return () => {
      scene.removeEventListener('enter-vr', handleEnterVR);
      scene.removeEventListener('exit-vr', handleExitVR);
    };
  }, [vrModeRef.current, rendered]);

  React.useEffect(() => {
    if (!vrLeftStream && !vrRightStream) return;

    if (!shareControl && !showMenu) {
      const payload = {
        timestamp: Date.now(),
        joint: thetaBodyMQTT.current,
        tool: thetaToolMQTT.current,
        joint_left: thetaBodyLeftMQTT.current,
        tool_left: thetaToolLeftMQTT.current,
      };
      setControlData(payload);
    }

    // const interval = setInterval(() => {
    //   const payload = {
    //     timestamp: Date.now(),
    //     joint: thetaBodyMQTT.current,
    //     tool: thetaToolMQTT.current,
    //     joint_left: thetaBodyLeftMQTT.current,
    //     tool_left: thetaToolLeftMQTT.current,
    //   };
    //   setControlData(payload);
    // }, 45); // 25Hz

    // return () => clearInterval(interval);
  }, [
    // vrLeftStream, 
    // vrRightStream,
    thetaBodyMQTT.current,
    thetaToolMQTT.current,
    thetaBodyLeftMQTT.current,
    thetaToolLeftMQTT.current
  ]);


  /* ============================= Robot State Update ==========================================*/
  // Robot State Update Props
  const robotProps = React.useMemo(() => ({
    robotNameList, robotName, theta_body, theta_tool, theta_body_left, theta_tool_left, theta_body_cam
  }), [robotNameList, robotName, theta_body, theta_tool, theta_body_left, theta_tool_left, theta_body_cam]);

  // Robot Secene Render
  return (
    <>
      <WebRTC_Video_Recv 
        onVideoStream1={setWebcamStream1}
        onVideoStream2={setWebcamStream2}
        onVideoStream3={setWebcamStream3} />

      {/* <WebRTC_Video_Send 
        VR_Left_Stream={vrLeftStream}
        VR_Right_Stream={vrRightStream}
      /> */}

      <WebRTC_Video_Send_Data
        VR_Left_Stream={vrLeftStream}
        VR_Right_Stream={vrRightStream}
        controlData={controlData}
        recvData={DataRecv}
      />

      <WebRTC_Data_Recv
        channelId="sora_liust_vr_left"
        onControlData={setDataRecv}
      />

      <RobotScene
        robot_list={robot_list}
        rendered={rendered}

        robotProps={robotProps}
        interfacePropos={interfacePropos}
        dsp_message={dsp_message}
        view_cam_pose={view_cam_pose}
        viewer={props.viewer}
        monitor={props.monitor}

        // Right Arm
        state_codes={error_code}
        position_ee={position_ee_Three}
        euler_ee={euler_ee_Three}
        vr_controller_pos={vr_controller_pos}
        vr_controller_R={vr_controller_R_current}
        rightArmPosition={rightArmPosition}

        // Left Arm
        state_codes_left={error_code_left}
        position_ee_left={position_ee_Three_left}
        euler_ee_left={euler_ee_Three_left}
        vr_controller_pos_left={vr_controller_pos_left}
        vr_controller_R_left={vr_controller_R_current_left}
        leftArmPosition={leftArmPosition}

        // CAM Arm
        state_codes_cam={error_code_cam}
        position_ee_cam={position_ee_Three_cam}
        euler_ee_cam={euler_ee_Three_cam}

        indicator={indicator}
        modelOpacity={modelOpacity}
        webcamStream1={webcamStream1}
        webcamStream2={webcamStream2}
        webcamStream3={webcamStream3}
        showMenu={showMenu}
      />
    </>
  );
}
