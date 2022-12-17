using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetMQ;

namespace PubSub
{
    public class TrackingScript : MonoBehaviour
    {

        private Listener _listener;
        private float x, y, z;

        public TRACKING_MODE TRACE_MOVEMENT;
        public int MAX_TRACE_POINTS;
        public Transform TRACE_PREFAB;

        private List<Transform> _traceList;
        private int _nextTraceIndex;

        public enum TRACKING_MODE { CAMERA_MOVEMENT, SCATTER_PLOT, FAKE_CAMERA };

        // Start is called before the first frame update
        void Start()
        {
            Transform obj;

            switch (TRACE_MOVEMENT)
            {
                case TRACKING_MODE.CAMERA_MOVEMENT:
                    break;
                case TRACKING_MODE.SCATTER_PLOT:
                    _traceList = new List<Transform>();
                    System.Random rnd = new System.Random();
                    for(int i = 0; i < MAX_TRACE_POINTS; i++){
                        obj = Instantiate(TRACE_PREFAB);
                        _traceList.Add(obj);
                        Debug.Log(_traceList.Count);
                    }
                    _nextTraceIndex = 0;
                    break;
                case TRACKING_MODE.FAKE_CAMERA:
                    _traceList = new List<Transform>();
                    obj = Instantiate(TRACE_PREFAB);
                    obj.GetComponent<TrailRenderer>().enabled=true; 
                    _traceList.Add(obj);
                    break;
            }
            _listener = new Listener("127.0.0.1", "5555", moveCameraAccordingToInput);
            OnStartClient();
        }

        // Update is called once per frame
        void Update()
        {
            _listener.DigestMessage();
        }

        void moveCameraAccordingToInput(string message){
            Debug.Log(message);
            var response = JsonUtility.FromJson<JSONResponse>(message);
            Vector3 traslation = new Vector3(response.pos_x, response.pos_y, response.pos_z);
            Quaternion rotation = new Quaternion(response.quat_x, response.quat_y, response.quat_z, response.quat_w) * Quaternion.Euler(0, 180f, 180f);

            switch (TRACE_MOVEMENT)
            {
                case TRACKING_MODE.CAMERA_MOVEMENT:
                    Camera.main.transform.localPosition = traslation;
                    Camera.main.transform.localRotation = rotation;
                    break;
                case TRACKING_MODE.SCATTER_PLOT:
                    _traceList[_nextTraceIndex % _traceList.Count].transform.localPosition = traslation;
                    _traceList[_nextTraceIndex % _traceList.Count].transform.localRotation = rotation;
                    _nextTraceIndex += 1;
                    break;
                case TRACKING_MODE.FAKE_CAMERA:
                    _traceList[0].transform.localPosition = traslation;
                    _traceList[0].transform.localRotation = rotation;
                    break;
            }
        }

        private void OnStartClient()
        {
            Debug.Log("Starting client...");
            _listener.Start();
            Debug.Log("Client started!");
        }

        private void OnStopClient()
        {
            Debug.Log("Stopping client...");
            _listener.Stop();
            Debug.Log("Client stopped!");
        }
    }

    [Serializable]
    public class JSONResponse{
        public float pos_x;
        public float pos_y;
        public float pos_z;

        public float quat_x;
        public float quat_y;
        public float quat_z;
        public float quat_w;
    }
}