using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

[RequireComponent(typeof(Rigidbody))]
public class TwistController : MonoBehaviour
{
    Rigidbody rb;

    Vector3 currentVelocity;
    Vector3 currentAngularVelocity;
    float maxSpeed = 10.0f;
    float maxRotationSpeed = 2f; 
    


    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("cmd_vel", SetVelocity);
        rb = GetComponent<Rigidbody>();
        currentAngularVelocity = Vector3.zero;
        currentVelocity = Vector3.zero;
    }

    private void Update()
    {
        rb.velocity = currentVelocity;
        rb.angularVelocity = currentAngularVelocity;
    }

    void SetVelocity(TwistMsg msg)
    {
        currentVelocity = new Vector3((float)msg.linear.x * maxSpeed * transform.forward.x, 0.0f, (float)msg.linear.x * maxSpeed * transform.forward.z);
        currentAngularVelocity = new Vector3(0.0f, (float)-msg.angular.z * maxRotationSpeed * maxRotationSpeed, 0.0f) ;

    }
}
