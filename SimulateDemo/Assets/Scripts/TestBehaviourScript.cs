using DG.Tweening;
using SQLite;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

[Serializable]
public class Information
{
    [PrimaryKey, AutoIncrement]
    [field: SerializeField] public int ID { get; set; }
    [field: SerializeField] public string Fan { get; set; }
    [field: SerializeField] public string Blade { get; set; }
    [field: SerializeField] public string Fairway { get; set; }
    [field: SerializeField] public int ImageWidth { get; set; }
    [field: SerializeField] public int ImageHeight { get; set; }
    [field: SerializeField] public double Longitude { get; set; }
    [field: SerializeField] public double Latitude { get; set; }
    [field: SerializeField] public double Altitude { get; set; }
    [field: SerializeField] public double RelativeAltitude { get; set; }
    [field: SerializeField] public double RollDegree { get; set; }
    [field: SerializeField] public double PitchDegree { get; set; }
    [field: SerializeField] public double YawDegree { get; set; }
    [field: SerializeField] public double FlightRollDegree { get; set; }
    [field: SerializeField] public double FlightPitchDegree { get; set; }
    [field: SerializeField] public double FlightYawDegree { get; set; }
    [field: SerializeField] public double FocalLength { get; set; }
    [field: SerializeField] public double Fov { get; set; }
    [field: SerializeField] public string SerialNumber { get; set; }
    [field: SerializeField] public DateTime DateTime { get; set; }
    [field: SerializeField] public string Name { get; set; }
    [field: SerializeField] public double X { get; set; }
    [field: SerializeField] public double Y { get; set; }
    [field: SerializeField] public double Z { get; set; }
}


/*
 * Qumeta：
*/
public class TestBehaviourScript : MonoBehaviour
{
    [SerializeField]
    public Information[] Informations;

    [SerializeField]
    public bool ShowFlag = true;

    //[SerializeField]
    //public bool ShowCamera = true;
    [SerializeField]
    private bool AnimationFlag = false;
    
    [SerializeField]
    private bool TestFlag = false;

    [SerializeField]
    public Transform RootCamera;

    [SerializeField]
    public Transform FlyCamera;

    //[SerializeField]
    //public Transform FlyPlane;

    [SerializeField]
    private int refImageIndex = 0; // >=0时取正向,-1时倒取第一个

    [SerializeField]
    private int imageID = 0;

    [SerializeField]
    private int imageStep = 0;

    [SerializeField]
    private float duration = .5f;

    [SerializeField]
    public string Fan = "027";

    [SerializeField]
    public bool FlightFlag = true;
    
    [SerializeField]
    public bool ConeFlag = false;
    
    void Start()
    {
        //var lookRotation = Quaternion.LookRotation(new Vector3(3, 4, 5));
        //Debug.Log(lookRotation);
        //EditorApplication.isPlaying = false;

        // 删除目录下所有jpg或者png文件
        var files = Directory.GetFiles(Application.persistentDataPath, "*.jpg");
        foreach (var file in files)
        {
            File.Delete(file);
        }

        var cameras = RootCamera.GetComponentsInChildren<Camera>();
        foreach (var camera in cameras)
        { 
            camera.enabled = false;
        }

        var meshRenderers = FlyCamera.GetComponentsInChildren<MeshRenderer>();
        foreach (var meshRenderer in meshRenderers)
        {
            meshRenderer.enabled = false;
        }

        if (TestFlag)
        {
            var cameraTest = Selection.activeTransform.gameObject.GetComponentInChildren<Camera>();
            //imageID = 201;
            DoCamera(cameraTest);
            SaveYaml(cameraTest);
            return;
        }

        if (!AnimationFlag)
            return;

        cameras = FlyCamera.GetComponentsInChildren<Camera>();
        Camera.main.transform.position = cameras[0].transform.position;
        Camera.main.transform.rotation = cameras[0].transform.rotation;
        //Camera.main.fieldOfView = cameras[0].fieldOfView;
        Camera.main.usePhysicalProperties = cameras[0].usePhysicalProperties;
        Camera.main.focalLength = cameras[0].focalLength;

        // 裁切步骤
        if (imageStep > 0 && imageStep < cameras.Count())
        {
            cameras = cameras.Take(imageStep).ToArray();
        }

        //Camera.main.DOFieldOfView(cameras[0].fieldOfView, 1);
        var sequence = DOTween.Sequence();
        //var duration = 1;
        var atPosition = 0f;
        imageID = 1;
        imageTransforms.Clear();
        imagePositions.Clear();
        imageRotations.Clear();

        // start
        sequence.AppendCallback(() =>
        {
            DoCamera();
        });

        for (var i = 1; i < cameras.Length; i++)
        {
            var tween1 = Camera.main.transform.DORotateQuaternion(cameras[i].transform.rotation, duration).SetEase(Ease.Linear);
            var tween2 = Camera.main.transform.DOMove(cameras[i].transform.position, duration).SetEase(Ease.Linear);

            sequence.Insert(atPosition, tween1);
            sequence.Insert(atPosition, tween2);

            if (i > 1)
            {
                sequence.InsertCallback(atPosition, () =>
                {
                    DoCamera();
                });
            }

            atPosition += duration;
        }

        // end
        sequence.AppendCallback(() =>
        {
            DoCamera();

            SaveYaml(Camera.main);
        });

        sequence.Play();
    }

    // Update is called once per frame
    //void Update()
    //{
    //var cameras = this.GetComponents<Camera>();
    //foreach (var camera in cameras)
    //{
    //    camera.gameObject.SetActive(ShowCamera);
    //}
    //}

    private List<Matrix4x4> imageTransforms = new List<Matrix4x4>();
    private List<Vector3> imagePositions = new List<Vector3>();
    private List<Quaternion> imageRotations = new List<Quaternion>();
    private List<Vector3> imageNormals = new List<Vector3>();
    private List<float> imageDistances = new List<float>();
    private List<Quaternion> cameraRotations = new List<Quaternion>();


    public void DoCamera(Camera camera = null)
    {
        //var camera = Selection.activeTransform.gameObject.GetComponentInChildren<Camera>();
        if (camera == null)
        {
            camera = Camera.main;
        }

        var position = camera.transform.position;
        var rotation = camera.transform.rotation;
        var scale = Vector3.one;

        var RT = get_3x4_RT_matrix_from_u3d(position, rotation, scale);
        imageTransforms.Add(RT);
        imagePositions.Add(position);
        imageRotations.Add(rotation);
        //imageTransforms.Add(camera.transform.localToWorldMatrix);

        var  FlyPlane = FlyCamera.GetChild(0);
        var pt1 = FlyPlane.GetChild(0).position;
        var pt2 = FlyPlane.GetChild(1).position;
        var pt3 = FlyPlane.GetChild(2).position;

        var plane = new Plane(pt1, pt2, pt3);
        //Debug.Log(plane.normal);
        imageNormals.Add(plane.normal);
        imageDistances.Add(plane.distance);

        // AI写
        // 计算相机到平面Plane的垂直投影点
        // Plane转化成平面一般方程
        // Ax + By + Cz + D = 0
        // 相机位置
        var P = camera.transform.position;
        // 相机到平面的垂直投影点
        // P0 = P - (A * P.x + B * P.y + C * P.z + D) / (A * A + B * B + C * C) * (A, B, C)
        var P0 = P - (plane.normal.x * P.x + plane.normal.y * P.y + plane.normal.z * P.z + plane.distance) / (plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y + plane.normal.z * plane.normal.z) * plane.normal;
        //Debug.Log(P0);

        // 从相机点看向投影点的旋转
        var lookRotation = Quaternion.LookRotation(P0 - P);
        cameraRotations.Add(lookRotation);
        //Debug.Log(lookRotation.eulerAngles);

        /* 手写
        var A = plane.normal.x;
        var B = plane.normal.y;
        var C = plane.normal.z;
        var D = plane.distance;
        var Xp = ((B * B + C * C) * P.x - A * (B * P.y + C * P.z + D)) / (A * A + B * B + C * C);
        var Yp = ((A * A + C * C) * P.y - B * (A * P.x + C * P.z + D)) / (A * A + B * B + C * C);
        var Zp = ((A * A + B * B) * P.z - C * (A * P.x + B * P.y + D)) / (A * A + B * B + C * C);
        var Pp = new Vector3(Xp, Yp, Zp);
        Debug.Log(Pp);
        */

        //var renderTexture = AssetDatabase.LoadAssetAtPath<RenderTexture>("Assets/Demo Render Texture.renderTexture");
        var renderTexture = AssetDatabase.LoadAssetAtPath<RenderTexture>("Assets/Test Render Texture.renderTexture");
        camera.targetTexture = renderTexture;
        RenderTexture.active = renderTexture;

        camera.Render();

        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();
        camera.targetTexture = null;

        var pngFlag = false;

        if (pngFlag)
        {
            byte[] bytes = image.EncodeToPNG();
            File.WriteAllBytes(Path.Combine(Application.persistentDataPath, $"{imageID++}.png"), bytes);
        }
        else
        {
            byte[] bytes = image.EncodeToJPG();
            File.WriteAllBytes(Path.Combine(Application.persistentDataPath, $"{imageID++}.jpg"), bytes);
        }
    }

    public void SaveYaml(Camera camera)
    {
        var K = get_calibration_matrix_K_from_u3d(camera);

        using (StreamWriter sw = new StreamWriter(Path.Combine(Application.persistentDataPath, (imageTransforms.Count >= 2 ? "u3d.yml" : $"{imageID}.yml"))))
        {
            sw.WriteLine("%YAML:1.0");
            sw.WriteLine("---");
            sw.WriteLine($"num: {imageTransforms.Count}");
            sw.WriteLine($"w: {camera.pixelWidth}");
            sw.WriteLine($"h: {camera.pixelHeight}");

            var fovH = Camera.VerticalToHorizontalFieldOfView(camera.fieldOfView, (float)camera.sensorSize.x / (float)camera.sensorSize.y);

            sw.WriteLine($"focalLength: {camera.focalLength}");
            sw.WriteLine($"sensorSizeX: {camera.sensorSize.x}");
            sw.WriteLine($"sensorSizeY: {camera.sensorSize.y}");

            sw.WriteLine($"fovH: {fovH}");
            sw.WriteLine($"fovV: {camera.fieldOfView}");
            // 对角fov
            var fov = Math.Atan(camera.sensorSize.y / camera.sensorSize.x) * 2 * 180 / Math.PI;
            sw.WriteLine($"fov: {fov}");

            // K
            sw.WriteLine("cameraMatrix: !!opencv-matrix");
            sw.WriteLine("   rows: 3");
            sw.WriteLine("   cols: 3");
            sw.WriteLine("   dt: d");
            sw.WriteLine($"   data: [ {K[0, 0]}, {K[0, 1]}, {K[0, 2]}, {K[1, 0]}, {K[1, 1]}, {K[1, 2]}, {K[2, 0]}, {K[2, 1]}, {K[2, 2]} ]"); // 3*3

            if (imageTransforms.Count > 0)
            {
                // RT
                var index = 0;

                if (refImageIndex >= 0)
                    index = Math.Min(refImageIndex, imageTransforms.Count - 1);
                else
                    index = imageTransforms.Count - Math.Min(Math.Abs(refImageIndex), imageTransforms.Count);

                for (var m = 0; m < imageTransforms.Count; m++)
                {
                    var RT = imageTransforms[m];
                    var position = imagePositions[m];
                    var rotation = imageRotations[m];
                    var normal = imageNormals[m];
                    var distance = imageDistances[m];
                    var cameraRotation = cameraRotations[m];

                    //Debug.Log(RT);

                    writeRT(sw, $"c{m + 1}Mo", RT, position, rotation, normal, distance, cameraRotation);

                    if(m == index)
                    {
                        Matrix4x4 avgRT = imageTransforms[m];
                        Vector3 avgPosition = imagePositions[m];
                        Quaternion avgRotation = imageRotations[m];
                        Vector3 avgNormal = imageNormals[m];
                        float avgDistance = imageDistances[m];
                        Quaternion avgCameraRotation = cameraRotations[m];

                        // 真正求平均位置及平均看向拍摄平面的角度
                        avgPosition = Vector3.zero;
                        for (var n = 0; n < imageTransforms.Count; n++)
                        {
                            avgPosition += imagePositions[m];
                        }
                        avgPosition /= imageTransforms.Count;
                        var FlyPlane = FlyCamera.GetChild(0);
                        var pt1 = FlyPlane.GetChild(0).position;
                        var pt2 = FlyPlane.GetChild(1).position;
                        var pt3 = FlyPlane.GetChild(2).position;

                        var P = avgPosition;
                        var plane = new Plane(pt1, pt2, pt3);
                        var P0 = P - (plane.normal.x * P.x + plane.normal.y * P.y + plane.normal.z * P.z + plane.distance) / (plane.normal.x * plane.normal.x + plane.normal.y * plane.normal.y + plane.normal.z * plane.normal.z) * plane.normal;
                        //Debug.Log(P0);

                        // 从相机点看向投影点的旋转
                        avgRotation = Quaternion.LookRotation(P0 - P);

                        writeRT(sw, $"cMo", avgRT, avgPosition, avgRotation, avgNormal, avgDistance, avgCameraRotation);
                    }
                }
            }
        }

        Debug.Log("OK");

        // 停止u3d的播放模式
        EditorApplication.isPlaying = false;
    }

    private void writeRT(StreamWriter sw, string name, Matrix4x4 m, 
        Vector3 position, Quaternion rotation, Vector3 normal, float distance,
        Quaternion cameraRotation)
    {
        if (false)
        {
            sw.WriteLine($"{name}: !!opencv-matrix");
            sw.WriteLine("   rows: 4");
            sw.WriteLine("   cols: 4");
            sw.WriteLine("   dt: d");

            sw.Write("   data: [ ");
            for (var i = 0; i < 4; i++)
            {
                if (i != 0)
                    sw.Write("       ");
                for (var j = 0; j < 4; j++)
                {
                    sw.Write($"{m[i, j]}");
                    if (!(i == 3 && j == 3))
                        sw.Write(",");
                }
                if (i != 3)
                    sw.WriteLine("");
            }
            sw.WriteLine(" ]");
        }

        // position
        sw.WriteLine($"{name}_position: [{position.x},{position.y},{position.z}]");

        // rotation
        var euler = rotation.eulerAngles;
        sw.WriteLine($"{name}_rotation: [{euler.x},{euler.y},{euler.z}]");

        // normal
        sw.WriteLine($"{name}_normal: [{normal.x},{normal.y},{normal.z}]");

        // distance
        sw.WriteLine($"{name}_distance: {distance}");

        // cameraRotation
        euler = cameraRotation.eulerAngles;
        sw.WriteLine($"{name}_rotation_camera: [{euler.x},{euler.y},{euler.z}]");
    }

    public static Matrix4x4 get_calibration_matrix_K_from_u3d(Camera camera)
    {
        var K = Matrix4x4.zero;

        // 50
        var f_in_mm = camera.focalLength;
        var scale = 1; // c++ 处理
        var resolution_x_in_px = camera.pixelWidth / scale;
        var resolution_y_in_px = camera.pixelHeight / scale;

        // https://docs.unity3d.com/Manual/PhysicalCameras.html
        var sensor_size_in_mm = 0f;
        var view_fac_in_px = 0f;

        if (camera.gateFit == Camera.GateFitMode.Horizontal)
        {
            sensor_size_in_mm = camera.sensorSize.x;
            view_fac_in_px = resolution_x_in_px;
        }
        else if (camera.gateFit == Camera.GateFitMode.Vertical)
        {
            sensor_size_in_mm = camera.sensorSize.y;
            view_fac_in_px = resolution_y_in_px;
        }

        var pixel_aspect_ratio = 1f;

        var pixel_size_mm_per_px = sensor_size_in_mm / f_in_mm / view_fac_in_px;
        var s_u = 1 / pixel_size_mm_per_px;
        var s_v = 1 / pixel_size_mm_per_px / pixel_aspect_ratio;

        var u_0 = resolution_x_in_px / 2;
        var v_0 = resolution_y_in_px / 2;
        var skew = 0f; // only use rectangular pixels

        K[0, 0] = s_u;
        K[0, 1] = skew;
        K[0, 2] = u_0;

        K[1, 1] = s_v;
        K[1, 2] = v_0;

        K[2, 2] = 1f;
        //K = Matrix(
        //((s_u, skew, u_0),
        //(0, s_v, v_0),
        //(0, 0, 1)))

        return K;
    }

    public static Matrix4x4 get_3x4_RT_matrix_from_u3d(Vector3 location, Quaternion rotation, Vector3 scale)
    {
        // 两种world矩阵方法
        //var t = Matrix4x4.TRS(location, rotation, scale);

        //var rotX = Matrix4x4.Rotate(Quaternion.Euler(rotation.eulerAngles.x, 0, 0));
        //var rotY = Matrix4x4.Rotate(Quaternion.Euler(0, rotation.eulerAngles.y, 0));
        //var rotZ = Matrix4x4.Rotate(Quaternion.Euler(0, 0, rotation.eulerAngles.z));
        //var m = rotY * rotX * rotZ;
        //Debug.Log(m);

        //var m1 = rotX * rotY * rotZ;
        //Debug.Log(m1);

        // Quaternion-->转换为 Matrix4x4
        //var rotMatrix = new Matrix4x4();
        //rotMatrix.SetTRS(Vector3.zero, rotation, Vector3.one);
        var rotMatrix = Matrix4x4.Rotate(rotation);
        //Debug.Log("rotMatrix");
        //Debug.Log(rotMatrix);
        //if(m == rotMatrix)

        var R_world2u3dcam = rotMatrix.transpose; // 矩阵转置
        var T_world2u3dcam = -1 * R_world2u3dcam.MultiplyVector(location);

        //Debug.Log("R_world2u3dcam(transpose)");
        //Debug.Log(R_world2u3dcam);

        var R_u3dcam2cv = Matrix4x4.identity;
        R_u3dcam2cv[0, 0] = 1;    // x 水平向右
        R_u3dcam2cv[1, 1] = -1;   // y 向上
        R_u3dcam2cv[2, 2] = 1;    // z 向前

        var R_world2cv = R_u3dcam2cv * R_world2u3dcam;
        var T_world2cv = R_u3dcam2cv * T_world2u3dcam;

        //Debug.Log("R_world2u3dcam");
        //Debug.Log(R_world2u3dcam);

        // put into 3x4 matrix
        var RT = Matrix4x4.zero;
        for (var i = 0; i < 4; i++)
        {
            RT[i, 0] = R_world2cv[i, 0];
            RT[i, 1] = R_world2cv[i, 1];
            RT[i, 2] = R_world2cv[i, 2];
            RT[i, 3] = T_world2cv[i];
        }
        RT[3, 3] = 1;

        //Debug.Log(RT);
        return RT;
    }
}
