using System;
using Unity.Robotics;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine.EventSystems;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine.SceneManagement;

public class CustomController : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    public ControlType control = ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    [Header("RL Parameters")]
    public string controlTopic;
    public string stateTopic;
    private ROSConnection ros;
    [Tooltip("Between each command from the agent, the simulation will run for x seconds before returning new state, reward, and stopping movement")]
    public float timeBetweenCommands = 0.05f;
    private float lastTime;
    public Transform pickupObject;
    public Transform endEffectorPosition;
    bool endSim = false;
    bool hitGround = false;
    bool recievedFirstUpdate = false;
    bool freezeTime = true;
    public LayerMask groundCollisionLayer;
    public float startTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float32MultiArrayMsg>(stateTopic);
        ros.Subscribe<EmptyMsg>("/reset", HandleReset);
        ros.Subscribe<Int8MultiArrayMsg>(controlTopic, UpdateDirections);
        lastTime = timeBetweenCommands;
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
            UpdateControlType(joint.GetComponent<JointControl>());
        }
        Time.timeScale = 0;
        startTime = Time.time;
    }

    void HandleReset(EmptyMsg msg)
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    void Update()
    {
        if (!recievedFirstUpdate)
        {
            SendCurrentState();
            return;
        }

        if (freezeTime || endSim) return;

        lastTime -= Time.deltaTime;
        if (lastTime <= 0)
        {
            Time.timeScale = 0;
            lastTime = timeBetweenCommands;
            SendCurrentState();
        }
    }

    private void SendCurrentState()
    {
        float[] state = new float[articulationChain.Length + 5];
        // the position of the object (vec3) and then the 4th spot is 0 unless terminate early (hit ground), then reward, then all the joint angles
        // if the sim ends early from bad, return -1000 for reward
        // if the sim ends early from happy, return 1000 for reward
        endSim = endSim || CheckGroundCollision();
        var rosPosition = pickupObject.transform.To<FLU>();
        state[0] = (float)rosPosition.translation.x;
        state[1] = (float)rosPosition.translation.y;
        state[2] = (float)rosPosition.translation.z;
        state[4] = CalculateReward();
        // this has to go after calculate rewards, or else it doesn't check for endsim from good reward
        state[3] = endSim ? 1 : 0;

        for (int i = 5; i < articulationChain.Length + 4; i++)
        {
            if (articulationChain[i - 5].jointPosition.dofCount > 0)
            {
                state[i] = articulationChain[i - 5].jointPosition[0];
            }
            else state[i] = 0;
        }

        Float32MultiArrayMsg msg = new Float32MultiArrayMsg();
        msg.data = state;
        ros.Publish(stateTopic, msg);
    }

    private bool CheckGroundCollision()
    {
        hitGround = hitGround || Physics.SphereCast(endEffectorPosition.position, 0.2f, Vector3.zero, out RaycastHit hit, 1, groundCollisionLayer);
        return hitGround;
    }

    private float CalculateReward()
    {
        if (!recievedFirstUpdate) return 0f;

        endSim = endSim || Time.time - startTime > 15;

        float distance = Vector3.Distance(endEffectorPosition.position, pickupObject.position);

        if (distance < 0.2)
        {
            endSim = true;
            return 100;
        }
        if (endSim && hitGround) return -1000;

        return -distance + pickupObject.position.y * 10;
    }

    /// <summary>
    /// Sets the direction of movement of the joint on every update
    /// </summary>
    /// <param name="jointIndex">Index of the link selected in the Articulation Chain</param>
    private void UpdateDirections(Int8MultiArrayMsg msg)
    {
        recievedFirstUpdate = true;
        for (int i = 0; i < articulationChain.Length; i++)
        {
            JointControl current = articulationChain[i].GetComponent<JointControl>();
            var direction = msg.data[i];
            if (direction > 0)
            {
                current.direction = RotationDirection.Positive;
            }
            else if (direction < 0)
            {
                current.direction = RotationDirection.Negative;
            }
            else
            {
                current.direction = RotationDirection.None;
            }
        }
        Time.timeScale = 1.0f; // allow time to come back after the joint updates
        freezeTime = false;
    }

    public void UpdateControlType(JointControl joint)
    {
        if (joint == null) return;
        joint.controltype = control;
        if (control == ControlType.PositionControl)
        {
            if (joint.joint == null) return;
            ArticulationDrive drive = joint.joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }
}
