using DG.Tweening;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.Rendering;
using UnityEngine;
using UnityEngine.UIElements;


/*
 * Qumeta：
*/
public class TestCamera : MonoBehaviour
{
    [SerializeField]
    private bool AnimationFlag = false;

    void Start()
    {
        //TestCal();
        //return;

        var camera = Selection.activeTransform.gameObject.GetComponentInChildren<Camera>();
        imageID = 200;
        DoCamera(camera);
        return;

        Camera.main.transform.position = new Vector3(0, 0, -15);
        Camera.main.transform.rotation = Quaternion.Euler(0, -20, 0);
        //Camera.main.fieldOfView = 60;
        Camera.main.focalLength = 50;

        if (!AnimationFlag)
            return;

        var sequence = DOTween.Sequence();
        var duration = 1;
        var atPosition = 0;
        imageID = 200;

        var tween1 = Camera.main.transform.DORotateQuaternion(Quaternion.Euler(0, 20, 0), duration).SetEase(Ease.Linear);
        var tween2 = Camera.main.transform.DOMove(new Vector3(0, 0, -15), duration).SetEase(Ease.Linear);

        sequence.Insert(atPosition, tween1);
        sequence.Insert(atPosition, tween2);

        sequence.InsertCallback(atPosition, () =>
        {
            DoCamera();
        });

        atPosition += duration;

        sequence.AppendCallback(() =>
        {
            DoCamera();
        });

        sequence.Play();
    }

    private void TestCal()
    {
        // 旋转顺序zxy 
        var deg = 45;

        var m1 = test_3x4_RT_matrix_from_u3d(new Vector3(0, 0, 0), Quaternion.Euler(0, 0, 0), Vector3.one);
        var m2 = test_3x4_RT_matrix_from_u3d(new Vector3(0, 0, 0), Quaternion.Euler(deg, 0, 0), Vector3.one);
        var m3 = test_3x4_RT_matrix_from_u3d(new Vector3(0, 0, 0), Quaternion.Euler(0, deg, 0), Vector3.one);
        var m4 = test_3x4_RT_matrix_from_u3d(new Vector3(0, 0, 0), Quaternion.Euler(0, 0, deg), Vector3.one);
        var m5 = test_3x4_RT_matrix_from_u3d(new Vector3(0, 0, 0), Quaternion.Euler(deg, deg, deg), Vector3.one);
        Debug.Log(m1);
        Debug.Log(m2);
        Debug.Log(m3);
        Debug.Log(m4);
        Debug.Log(m5);
        Debug.Log(m4 * m2 * m3); // m4 * m2 * m3 == m5

        var offset = 10;
        m2 = test_3x4_RT_matrix_from_u3d(new Vector3(offset, 0, 0), Quaternion.Euler(deg, 0, 0), Vector3.one);
        m3 = test_3x4_RT_matrix_from_u3d(new Vector3(0, offset, 0), Quaternion.Euler(0, deg, 0), Vector3.one);
        m4 = test_3x4_RT_matrix_from_u3d(new Vector3(0, 0, offset), Quaternion.Euler(0, 0, deg), Vector3.one);
        m5 = test_3x4_RT_matrix_from_u3d(new Vector3(offset, offset, offset), Quaternion.Euler(deg, deg, deg), Vector3.one);
        Debug.Log(m2);
        Debug.Log(m3);
        Debug.Log(m4);
        Debug.Log(m5);
        Debug.Log(m4 * m2 * m3); // m4 * m2 * m3 != m5
    }

    private int imageID = 0;

    public void DoCamera(Camera camera = null)
    {
        // 没有传入用Main
        if (camera == null)
        {
            camera = Selection.activeTransform.gameObject.GetComponentInChildren<Camera>();
            if(camera == null)
                camera = Camera.main;
        }

        CalRT(camera);

        //var renderTexture = AssetDatabase.LoadAssetAtPath<RenderTexture>("Assets/Demo Render Texture.renderTexture");
        var renderTexture = AssetDatabase.LoadAssetAtPath<RenderTexture>("Assets/Test Render Texture.renderTexture");
        camera.targetTexture = renderTexture;
        RenderTexture.active = renderTexture;

        camera.Render();

        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();
        camera.targetTexture = null;

        byte[] bytes = image.EncodeToPNG();

        //var name = DateTime.Now.ToString("yyyyMMddhhmmss");

        File.WriteAllBytes(Path.Combine(Application.persistentDataPath, $"{imageID++}.png"), bytes);
    }

    private void CalRT(Camera camera)
    {
        var K = TestBehaviourScript.get_calibration_matrix_K_from_u3d(camera);
        var RT = TestBehaviourScript.get_3x4_RT_matrix_from_u3d(camera.transform.position, camera.transform.rotation, camera.transform.lossyScale);

        Debug.Log("K");
        Debug.Log(K); // 3*3
        Debug.Log("RT");
        Debug.Log(RT);

        using (StreamWriter sw = new StreamWriter(Path.Combine(Application.persistentDataPath, ($"{imageID}.yml"))))
        {
            sw.WriteLine("K");
            sw.WriteLine($"   data: [ {K[0, 0]}, {K[0, 1]}, {K[0, 2]}, {K[1, 0]}, {K[1, 1]}, {K[1, 2]}, {K[2, 0]}, {K[2, 1]}, {K[2, 2]} ]"); // 3*3
            //   data: [ 888.8889, 0., 320., 0., 888.8889, 240., 0., 0., 1. ]

            sw.WriteLine("RT");
            sw.Write("   data: [ ");
            for (var i = 0; i < 4; i++)
            {
                if (i != 0)
                    sw.Write("       ");
                for (var j = 0; j < 4; j++)
                {
                    sw.Write($"{RT[i, j]}");
                    if (!(i == 3 && j == 3))
                        sw.Write(",");
                }
                if (i != 3)
                    sw.WriteLine("");
            }
            sw.WriteLine(" ]");
        }
    }

    private Matrix4x4 test_3x4_RT_matrix_from_u3d(Vector3 location, Quaternion rotation, Vector3 scale)
    {
        //var matrix4X4 = transform.localToWorldMatrix;
        // 两种world矩阵方法
        var t = Matrix4x4.TRS(location, rotation, scale);

        // Quaternion-->转换为 Matrix4x4
        var rotMatrix = new Matrix4x4();
        rotMatrix.SetTRS(Vector3.zero, rotation, Vector3.one);

        var R_world2u3dcam = rotMatrix.transpose; // 矩阵转置
        var T_world2u3dcam = -1 * R_world2u3dcam.MultiplyVector(location);

        var R_u3dcam2cv = Matrix4x4.identity;
        R_u3dcam2cv[0, 0] = 1;    // x 水平向右
        R_u3dcam2cv[1, 1] = 1;// -1;   // y 向上 TODO
        R_u3dcam2cv[2, 2] = 1;    // z 向前

        var R_world2cv = R_u3dcam2cv * R_world2u3dcam;
        var T_world2cv = R_u3dcam2cv * T_world2u3dcam;

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

        return RT;
    }
}
