using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

// Using https://medium.com/google-developers/real-time-image-capture-in-unity-458de1364a4c

public class CameraPublisher : MonoBehaviour
{
    [Header("ROS Config")]
    ROSConnection ros;
    public string topicName = "/camera/image_compressed";

    [Header("Camera Settings")]
    public Vector2Int resolution = new Vector2Int(640, 480);
    public int frameRate = 24;
    private float cameraHz;
    private float elapsedTime = 0f;
    public Camera cam;

    // Render details
    private Texture2D tex;
    private RenderTexture rt;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(topicName);
        cameraHz = 1.0f / frameRate;
        tex = new Texture2D(resolution.x, resolution.y);
        rt = new RenderTexture(resolution.x, resolution.y, 16, RenderTextureFormat.ARGB32);
        rt.Create();
    }


    // Update is called once per frame
    void Update()
    {
        elapsedTime += Time.deltaTime;
        if (elapsedTime < cameraHz) return; // since we can have frame rates less than the rendered frame rate, we should leave if we don't need to get an image

        elapsedTime -= cameraHz;
        cam.targetTexture = rt;
        cam.Render();

        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, resolution.x, resolution.y), 0, 0);

        CompressedImageMsg msg = new CompressedImageMsg();
        msg.data = tex.EncodeToJPG(50);
        msg.format = "jpeg";
        msg.header.frame_id = "/camera";

        ros.Publish(topicName, msg);
        cam.targetTexture = null;
        Debug.Log("Published Image");

    }
}
