using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class RosSubscriberExample : MonoBehaviour
{
    // Start is called before the first frame update

    public GameObject cube;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PoseArrayMsg>("frontiers", ShowFrontiers);
    }

    void ShowFrontiers(PoseArrayMsg msg)
    {

    }
}
