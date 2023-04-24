using KevinCastejon.ConeMesh;
using Microsoft.SqlServer.Server;
using SQLite;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;


/*
 * Qumeta：
*/
public class TestScript
{
    private static Vector3 startPt = Vector3.zero;
    private static Vector3 endPt = Vector3.zero;

    private static Quaternion U3DQuaternion = Quaternion.Euler(0f, 0f, 0f);


    private static Vector3 getU3DPos(Information information)
    {
        var pt = new Vector3((float)information.X, (float)information.Z, (float)information.Y);
        var u3dpt= U3DQuaternion * pt;

        // 右手坐标系转换成U3D左手坐标系

        return u3dpt;
    }

    private static Quaternion getU3DQuaternion(Information information, bool flightFlag)
    {
        var q = Quaternion.identity;
        // x y z
        //x z -y
        if (flightFlag)
            q = Quaternion.Euler(-(float)information.FlightPitchDegree, (float)information.FlightYawDegree, -(float)information.FlightRollDegree);
        else
            q = Quaternion.Euler(-(float)information.PitchDegree, (float)information.YawDegree, -(float)information.RollDegree);

        // x y z 
        //q = Quaternion.Euler((float)information.RollDegree, (float)information.YawDegree, (float)information.PitchDegree);
        //q = Quaternion.Euler(-(float)information.PitchDegree, (float)information.YawDegree, (float)information.RollDegree);

        return U3DQuaternion * q;
    }

    [MenuItem("Qumeta/Test")]
    public static void Test()
    {
        var script = Selection.activeTransform.GetComponent<TestBehaviourScript>();
        if (script == null)
            return;

        var colors = new Color[] { Color.red, Color.yellow, Color.green, Color.blue };
        var fairways = new List<string> { "LE", "PS", "SS", "TE" };

        // G:\\Users\\lilin
        var profile = Environment.GetEnvironmentVariable("USERPROFILE");
        var fan = script.Fan;
        var flightFlag = script.FlightFlag;

        var dbPath = Path.Join(profile, ".icache", "Information.db");
        //dbPath = @"G:\Item\GitHub\IndustrialInternetThing\ImageApp\Simulate\Assets\StreamingAssets\Information.db";
        using (var connection = new SQLiteConnection(dbPath, SQLiteOpenFlags.ReadOnly))
        {
            var informations = connection.Table<Information>().AsEnumerable().Where(item => string.IsNullOrEmpty(fan) ? true : item.Fan == fan).ToArray();

            var root = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
            root.parent = Selection.activeTransform;
            root.name = DateTime.Now.ToString();

            var material = AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/BallMaterial.mat");
            var materials = new Material[4];// { Color.red, Color.yellow, Color.green, Color.blue };
            for (var i = 0; i < 4; i++)
            {
                materials[i] = new Material(material);
                materials[i].color = new Color[] { Color.red, Color.yellow, Color.green, Color.blue }[i];
            }

            var transformMap = new Dictionary<string, Transform>();
            var fanPointMap = new Dictionary<string, List<Vector3>>();

            foreach (var information in informations)
            {
                Transform fanTransform = null;
                if (!transformMap.TryGetValue(information.Fan, out fanTransform))
                {
                    fanTransform = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
                    fanTransform.parent = root;
                    fanTransform.name = information.Fan;
                    transformMap[information.Fan] = fanTransform;

                    fanPointMap[information.Fan] = new List<Vector3>();
                }

                Transform bladeTransform = null;
                if (!transformMap.TryGetValue(information.Fan + information.Blade, out bladeTransform))
                {
                    var bladeGo = new GameObject(information.Blade);

                    bladeTransform = bladeGo.transform;
                    bladeTransform.parent = fanTransform;
                    bladeTransform.name = information.Blade;
                    transformMap[information.Fan + information.Blade] = bladeTransform;

                    // 取得单片叶子，四个最大Y值平均作为起点，四个最小Y值作为终点
                    startPt = Vector3.zero;     // 叶根
                    endPt = Vector3.zero;       // 页稍

                    // 根据TE判断是从高到底还是从低到高
                    var items = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade && item.Fairway == "TE").ToList();
                    var orderFlag = items.First().Z > items.Last().Z ? 1 : -1;

                    foreach (var fairway in fairways)
                    {
                        items = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade && item.Fairway == fairway).OrderByDescending(item => item.Z * orderFlag).ToList();
                        var firstItem = items.First();
                        startPt += getU3DPos(firstItem);
                        var lastItem = items.Last();
                        endPt += getU3DPos(lastItem);
                    }

                    startPt /= fairways.Count;
                    endPt /= fairways.Count;

                    // 叶根
                    var ballTransform = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
                    ballTransform.gameObject.GetComponent<MeshRenderer>().sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Simple Lit"));
                    ballTransform.parent = bladeTransform;
                    ballTransform.name = "CloneRoot";
                    ballTransform.localPosition = startPt;
                    ballTransform.localScale = Vector3.one * 4;

                    // 简单方法，模拟一个柱子
                    if (script.ConeFlag)
                    {
                        var bladeConeGo = new GameObject("Clone");
                        bladeConeGo.transform.parent = bladeTransform;
                        bladeConeGo.transform.localPosition = endPt;
                        bladeConeGo.transform.rotation = Quaternion.LookRotation(startPt - endPt);

                        var cone = bladeConeGo.AddComponent<Cone>();
                        cone.Material = new Material(Shader.Find("Universal Render Pipeline/Simple Lit"));
                        cone.ConeHeight = Vector3.Distance(startPt, endPt);
                        cone.ConeRadius = 1;
                    }
                    else
                    {
                        // 复杂方法，用球模拟柱子
                        // LE-PS-SS-TE
                        var listPts = new List<Vector3[]>();
                        for (var i = 0; i < fairways.Count; i++)
                        {
                            var fairwayInformations = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade && item.Fairway == fairways[i]).
                                Select(information => getU3DPos(information));
                            
                            if (i == 0 || i == 3)
                                listPts.Add(fairwayInformations.Reverse().ToArray());
                            else
                                listPts.Add(fairwayInformations.ToArray());
                        }
                        var circle = new GameObject("Circle");
                        circle.transform.parent = bladeTransform;

                        var num = int.MaxValue;
                        foreach (var list in listPts)
                        {
                            num = Math.Min(num, list.Length);
                        }

                        for (var i = 0; i < num - 1; i++)
                        {
                            var center = FitCircleCenter(listPts[0][i], listPts[1][i],
                                listPts[2][i], listPts[3][i]);

                            var centerNext = FitCircleCenter(listPts[0][i + 1], listPts[1][i + 1],
                                listPts[2][i + 1], listPts[3][i + 1]);

                            var numSpan = 20;
                            var scale = 4f;

                            for (var j = 0; j < numSpan; j++)
                            {
                                var t = (float)j / numSpan;
                                var pt = Vector3.Lerp(center, centerNext, t);
                                //Debug.Log(pt);

                                var ptSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                                ptSphere.gameObject.GetComponent<MeshRenderer>().sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Simple Lit"));
                                ptSphere.name = $"{i}-{j}";
                                // 缩放比例变得平滑
                                ptSphere.transform.localScale = Vector3.one * (0.1f + (float)i / num * scale + (float)j / num * scale / numSpan);
                                //Debug.Log(ptSphere.transform.localScale);
                                ptSphere.transform.parent = circle.transform;
                                ptSphere.transform.position = pt;
                            }
                        }
                    }

                    fanPointMap[information.Fan].Add(startPt);
                }

                Transform fairwayTransform = null;
                if (!transformMap.TryGetValue(information.Fan + information.Blade + information.Fairway, out fairwayTransform))
                {
                    var fairwayGo = new GameObject(information.Fairway);

                    fairwayTransform = fairwayGo.transform;
                    fairwayTransform.parent = bladeTransform;
                    fairwayTransform.name = information.Fairway;
                    transformMap[information.Fan + information.Blade + information.Fairway] = fairwayTransform;

                    // Clip Plane
                    var informationPlane = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade && item.Fairway == information.Fairway).First();

                    // 叶片Normal Plane模拟计算(建立三个点)
                    var planeGo = new GameObject($"Plane:[{information.Fairway}]");
                    planeGo.transform.parent = fairwayTransform;

                    var spherePt = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    spherePt.name = $"PtA";
                    spherePt.transform.parent = planeGo.transform;
                    spherePt.transform.localPosition = startPt;

                    spherePt = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    spherePt.name = $"PtB";
                    spherePt.transform.parent = planeGo.transform;
                    spherePt.transform.localPosition = endPt;

                    var firstCameraPt = getU3DPos(informationPlane);;
                    var plane = new Plane(startPt, endPt, firstCameraPt);
                    //plane.normal;

                    // 计算clip第三点
                    var midPt = (startPt + endPt) / 2;
                    var pt = midPt + 10 * plane.normal;

                    spherePt = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    spherePt.name = $"PtC";
                    spherePt.transform.parent = planeGo.transform;
                    spherePt.transform.localPosition = pt;

                    plane = new Plane(startPt, endPt, pt);

                    var demo = GameObject.CreatePrimitive(PrimitiveType.Plane);
                    demo.name = $"ClipPlane";
                    demo.transform.parent = planeGo.transform;
                    demo.transform.localPosition = (startPt + endPt + midPt) / 3;
                    demo.transform.localScale = Vector3.one * 10;
                    //demo.transform.localRotation = calQuaternion(plane.normal);
                    demo.transform.localRotation = Quaternion.LookRotation(plane.normal) * Quaternion.Euler(90, 0, 0);

                    var planeMaterial = AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/PlaneMaterial.mat");
                    demo.GetComponent<MeshRenderer>().sharedMaterial = new Material(planeMaterial);

                    var planeBehaviourScript = demo.AddComponent<PlaneBehaviourScript>();
                    planeBehaviourScript.Information = informationPlane;
                    planeBehaviourScript.RootScript = script;

                    // 斜率线
                    //var lineRenderer = fairwayGo.AddComponent<LineRenderer>();
                    //lineRenderer.positionCount = 2;
                    //lineRenderer.sharedMaterial = materials[fairways.IndexOf(information.Fairway)];
                    //lineRenderer.startWidth = 1f;
                    //lineRenderer.endWidth = 0.2f;
                    //
                    //// 起始点-终止点 求直线斜率
                    //var items = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade && item.Fairway == information.Fairway).ToList();
                    //double k = 0;
                    //double b = 0;
                    //var falg = fitline(items, information.Fan, information.Blade, information.Fairway, ref k, ref b);
                    //var z1 = k * items[0].Y + b;
                    //var z2 = k * items[items.Count - 1].Y + b;
                    //
                    //lineRenderer.SetPosition(0, new Vector3((float)items[0].X, (float)z1, (float)items[0].Y));
                    //lineRenderer.SetPosition(1, new Vector3((float)items[items.Count - 1].X, (float)z2, (float)items[items.Count - 1].Y));
                }

                var bladeList = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade/* && item.Fairway == information.Fairway*/).OrderBy(item => item.DateTime).ToList();
                var index = bladeList.IndexOf(information);

                var fairwayList = informations.Where(item => item.Fan == information.Fan && item.Blade == information.Blade && item.Fairway == information.Fairway).OrderBy(item => item.DateTime).ToList();
                var fairwayIndex = fairwayList.IndexOf(information);

                // (0,count)-->(1,0.2)
                float map2Range(float v, float start1, float end1, float start2, float end2)
                {
                    var r = (v - start1) / (end1 - start1);
                    var t = r * (end2 - start2);
                    var ret = start2 + t;
                    return ret;
                }

                var fairwayIndexScale = map2Range(fairwayList.IndexOf(information), 0, fairwayList.Count, 1, 0.2f);

                var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                cube.name = $"{index}-{fairwayIndex}"; //(blade,fairway)
                cube.transform.parent = fairwayTransform;
                cube.transform.localPosition = getU3DPos(information);
                cube.transform.localRotation = Quaternion.identity;
                cube.transform.localScale = Vector3.one * fairwayIndexScale;

                var imageBehaviourScript = cube.AddComponent<ImageBehaviourScript>();
                imageBehaviourScript.Information = information;
                imageBehaviourScript.RootScript = script;

                var meshRenderer = cube.GetComponent<MeshRenderer>();
                meshRenderer.sharedMaterial = materials[fairways.IndexOf(information.Fairway)];
                meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

                // 模拟相机朝向
                //var cubeCameraGo = new GameObject("Camera");
                //cubeCameraGo.transform.parent = cube.transform;
                //cubeCameraGo.transform.localPosition = Vector3.zero;
                //cubeCameraGo.transform.localRotation = Quaternion.identity;

                //var camera = cubeCameraGo.AddComponent<Camera>();
                var camera = cube.AddComponent<Camera>();
                //camera.fieldOfView = (float)information.Fov;
                camera.transform.rotation = getU3DQuaternion(information, flightFlag);
                camera.gateFit = UnityEngine.Camera.GateFitMode.Horizontal;
                camera.usePhysicalProperties = true;
                camera.focalLength = (float)information.FocalLength;
                camera.clearFlags = CameraClearFlags.SolidColor;

                //var cameraLineRenderer = cubeCameraGo.AddComponent<LineRenderer>();
                //cameraLineRenderer.positionCount = 2;
                //cameraLineRenderer.sharedMaterial = materials[fairways.IndexOf(information.Fairway)];
                //cameraLineRenderer.startWidth = 0.1f;
                //cameraLineRenderer.endWidth = 0.4f;

                //cameraLineRenderer.SetPosition(0, cube.transform.localPosition + Vector3.zero);
                //cameraLineRenderer.SetPosition(1, cube.transform.localPosition + Vector3.up * 2);
                //Quaternion.
            }

            foreach (var fanPoints in fanPointMap)
            {
                if (fanPoints.Value.Count == 3)
                {
                    var fanTransform = transformMap[fanPoints.Key];

                    var pt = Vector3.zero;
                    pt += fanPoints.Value[0];
                    pt += fanPoints.Value[1];
                    pt += fanPoints.Value[2];
                    pt /= fanPoints.Value.Count;

                    var ballTransform = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
                    ballTransform.gameObject.GetComponent<MeshRenderer>().sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Simple Lit"));
                    ballTransform.parent = fanTransform;
                    ballTransform.name = "FanRoot";
                    ballTransform.localPosition = pt;
                    ballTransform.localScale = Vector3.one * 4;

                    var cylinderTransform = GameObject.CreatePrimitive(PrimitiveType.Cylinder).transform;
                    cylinderTransform.gameObject.GetComponent<MeshRenderer>().sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Simple Lit"));
                    cylinderTransform.parent = fanTransform;
                    cylinderTransform.name = "Body";
                    cylinderTransform.localPosition = new Vector3(pt.x, (pt.y / 2), pt.z);
                    cylinderTransform.localScale = new Vector3(4, Vector3.Distance(pt, Vector3.zero) / 2, 4);
                    //cylinderTransform.localRotation = Quaternion.LookRotation(Vector3.zero - pt);
                    //transformMap[fanPoints.Key]
                }
            }
        }
    }

    public static Vector3 FitCircleCenter(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
    {
        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1).normalized;
        float d = -Vector3.Dot(normal, p1);

        var plane = new Plane(normal, d);

        Vector3 center = plane.ClosestPointOnPlane((p1 + p2 + p3 + p4) / 4);
        return center;
    }

    public static CirclePlane FitCircle(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4, float pos = 1f)
    {
        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1).normalized;
        float d = -Vector3.Dot(normal, p1);

        var plane = new Plane(normal, d);

        Vector3 center = plane.ClosestPointOnPlane((p1 + p2 + p3 + p4) / 4);
        float radius = (p1 - center).magnitude * pos;

        return new CirclePlane(plane, center, radius);
    }

    private static Quaternion calQuaternion(Vector3 dir)
    {
        Quaternion cal = new Quaternion();
        Vector3 euler = Quaternion.LookRotation(dir).eulerAngles;

        //欧拉角Y: cosY = z/sqrt(x^2+z^2)
        float CosY = dir.z / Mathf.Sqrt(dir.x * dir.x + dir.z * dir.z);
        float CosYDiv2 = Mathf.Sqrt((CosY + 1) / 2);
        if (dir.x < 0) CosYDiv2 = -CosYDiv2;

        float SinYDiv2 = Mathf.Sqrt((1 - CosY) / 2);

        //欧拉角X: cosX = sqrt((x^2+z^2)/(x^2+y^2+z^2)
        float CosX = Mathf.Sqrt((dir.x * dir.x + dir.z * dir.z) / (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z));
        if (dir.z < 0) CosX = -CosX;
        float CosXDiv2 = Mathf.Sqrt((CosX + 1) / 2);
        if (dir.y > 0) CosXDiv2 = -CosXDiv2;
        float SinXDiv2 = Mathf.Sqrt((1 - CosX) / 2);

        //四元数w = cos(x/2)cos(y/2)
        cal.w = CosXDiv2 * CosYDiv2;
        //四元数x = sin(x/2)cos(y/2)
        cal.x = SinXDiv2 * CosYDiv2;
        //四元数y = cos(x/2)sin(y/2)
        cal.y = CosXDiv2 * SinYDiv2;
        //四元数z = sin(x/2)sin(y/2)
        cal.z = -SinXDiv2 * SinYDiv2;

        //CalCosX = CosX;
        //CalCosY = CosY;
        //RightCosX = Mathf.Cos(Mathf.Deg2Rad * (Quaternion.LookRotation(dir).eulerAngles.x));
        //RightCosY = Mathf.Cos(Mathf.Deg2Rad * (Quaternion.LookRotation(dir).eulerAngles.y));
        //RightEulers = Quaternion.LookRotation(dir).eulerAngles;

        return cal;
    }

    // 最小二乘法。拟合直线
    static bool fitline(List<Information> items, string fan, string blade, string fairway, ref double k, ref double b)
    {
        int n = items.Count();
        if (n < 2)
            return false;

        double xx_sum = 0;
        double x_sum = 0;
        double y_sum = 0;
        double xy_sum = 0;
        for (int i = 0; i < n; i++)
        {
            x_sum += items[i].Y; // x的累加和
            y_sum += items[i].Z; // y的累加和

            xx_sum += items[i].Y * items[i].Y; // x的平方累加和
            xy_sum += items[i].Y * items[i].Z; // x，y的累加和
        }

        k = (n * xy_sum - x_sum * y_sum) / (n * xx_sum - x_sum * x_sum); // 根据公式求解k
        b = (-x_sum * xy_sum + xx_sum * y_sum) / (n * xx_sum - x_sum * x_sum); //根据公式求解b

        return true;
    }

    [MenuItem("Qumeta/Remove")]
    public static void Remove()
    {
        GameObject.DestroyImmediate(Selection.activeTransform.gameObject);
    }

    [MenuItem("Qumeta/Cameras")]
    public static void Cameras()
    {
        var cameras = Selection.activeTransform.gameObject.GetComponentsInChildren<Camera>();
        foreach (var camera in cameras)
        {
            //Selection.activeTransform = camera.transform;
            //WaitForSeconds(0.5);

            var id = int.Parse(camera.name.Split("-")[1]) + 1;

            var renderTexture = AssetDatabase.LoadAssetAtPath<RenderTexture>("Assets/Demo Render Texture.renderTexture");
            camera.targetTexture = renderTexture;
            RenderTexture.active = renderTexture;

            camera.Render();

            Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
            image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
            image.Apply();
            camera.targetTexture = null;

            byte[] bytes = image.EncodeToPNG();

            File.WriteAllBytes(Path.Combine(Application.persistentDataPath, $"{id}.png"), bytes);
            //ScreenCapture.CaptureScreenshot($"{name}.png");
        }
    }

    [MenuItem("Qumeta/Camera")]
    public static void Camera()
    {
        var camera = Selection.activeTransform.gameObject.GetComponentInChildren<Camera>();
        if (camera == null)
            return;

        //var roll = 30;
        //var yaw = 45;
        //var pitch = 60;
        //var rotation = Quaternion.Euler(roll, yaw, pitch); 
        //var euler = rotation.eulerAngles;
        //Debug.Log(rotation);
        //Debug.Log(euler);
        //return;

        var renderTexture = AssetDatabase.LoadAssetAtPath<RenderTexture>("Assets/Demo Render Texture.renderTexture");
        camera.targetTexture = renderTexture;
        RenderTexture.active = renderTexture;

        camera.Render();

        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();
        camera.targetTexture = null;

        byte[] bytes = image.EncodeToPNG();

        var name = DateTime.Now.ToString("yyyyMMddhhmmss");

        File.WriteAllBytes(Path.Combine(Application.persistentDataPath, $"{name}.png"), bytes);
    }

    public class CirclePlane
    {
        public Plane plane;
        public Vector3 center;
        public float radius;

        public CirclePlane(Plane plane, Vector3 center, float radius)
        {
            this.plane = plane;
            this.center = center;
            this.radius = radius;
        }

        public Vector3[] GetPointsOnCircle(int numPoints)
        {
            Vector3[] points = new Vector3[numPoints];
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward,  plane.normal);
            for (int i = 0; i < numPoints; i++)
            {
                float angle = i * (2 * Mathf.PI / numPoints);
                Vector3 point = new Vector3(radius * Mathf.Cos(angle), radius * Mathf.Sin(angle), 0);
                point = rotation * point;
                points[i] = plane.ClosestPointOnPlane(center + point);
            }

            return points;
        }

        public override string ToString()
        {
            return $"center:{this.center.ToString()} radius:{this.radius}";
        }
    }
}