using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SwitchCameraPositionWithPointer : MonoBehaviour
{
    void OnMouseDown(){
        Debug.Log(transform.eulerAngles);
        Vector3 traslation = transform.localPosition;
        Quaternion rotation = transform.localRotation;

        Camera.main.gameObject.transform.SetPositionAndRotation(traslation, rotation);
        Camera.main.gameObject.transform.parent.SetPositionAndRotation(new Vector3(0, 0, 0), new Quaternion(0, 0, 0, 0));
    }
}
