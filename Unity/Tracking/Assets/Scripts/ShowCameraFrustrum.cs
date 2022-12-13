using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShowCameraFrustrum : MonoBehaviour
{
      public Camera cameraShowFrustumAlways;
      private void Start()
      {
          if (cameraShowFrustumAlways)
          {
              Gizmos.matrix = cameraShowFrustumAlways.transform.localToWorldMatrix;
              Gizmos.DrawFrustum(new Vector3(0, 0, cameraShowFrustumAlways.nearClipPlane), 
                  cameraShowFrustumAlways.fieldOfView, 
                  cameraShowFrustumAlways.farClipPlane, 
                  cameraShowFrustumAlways.nearClipPlane, 
                  cameraShowFrustumAlways.aspect);
          }
      }
}
