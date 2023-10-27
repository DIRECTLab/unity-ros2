using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RosPublisherExample : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pose_unity";

    // Gameobject to watch
    public GameObject cube;

    // # seconds between publishes
    public float publishMessageFrequency = 1.0f;

    private float timeElapsed;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            PointMsg point = new PointMsg(cube.transform.position.x, cube.transform.position.y, cube.transform.position.z);
            QuaternionMsg quaternionMsg = new QuaternionMsg();

            PoseMsg msg = new PoseMsg(point, quaternionMsg);

            ros.Publish(topicName, msg);

            timeElapsed = 0;
        }
    }
}
