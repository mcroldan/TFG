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
        private GameObject _mainCamera { get; set; }
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
                    _mainCamera = Camera.main.gameObject;
                    break;
                case TRACKING_MODE.SCATTER_PLOT:
                    _traceList = new List<Transform>();
                    System.Random rnd = new System.Random();
                    for(int i = 0; i < MAX_TRACE_POINTS; i++){
                        obj = Instantiate(TRACE_PREFAB);
                        //obj.transform.position = new Vector3(rnd.Next(1, 10), rnd.Next(1, 10), rnd.Next(1, 10));
                        _traceList.Add(obj);
                        Debug.Log(_traceList.Count);
                    }
                    _nextTraceIndex = 0;
                    _mainCamera = GameObject.Find("FakeCamera");
                    break;
                case TRACKING_MODE.FAKE_CAMERA:
                    _traceList = new List<Transform>();
                    obj = Instantiate(TRACE_PREFAB);
                    obj.GetComponent<TrailRenderer>().enabled=true; 
                    _traceList.Add(obj);
                    _mainCamera = GameObject.Find("FakeCamera");
                    break;
            }
            Debug.Log("Agua");
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
            //Vector3 rotation = new Vector3(response.pitch_x, response.roll_y, response.yaw_z);
            //x = response.pitch_x;
            //y = response.roll_y;
            //z = response.yaw_z;
            //Quaternion rotation = Quaternion.Euler(x, y, z);
            Quaternion rotation = new Quaternion(response.quat_x, response.quat_y, response.quat_z, response.quat_w) * Quaternion.Euler(0, 180f, 180f);
            Debug.Log(response);
            //_mainCamera.transform.SetPositionAndRotation(traslation, rotation);

            switch (TRACE_MOVEMENT)
            {
                case TRACKING_MODE.CAMERA_MOVEMENT:
                    Camera.main.transform.localPosition = traslation; //SetPositionAndRotation(traslation, rotation);
                    Camera.main.transform.localRotation = rotation;
                    break;
                case TRACKING_MODE.SCATTER_PLOT:
                    _traceList[_nextTraceIndex % _traceList.Count].transform.localPosition = traslation; //SetPositionAndRotation(traslation, rotation);
                    _traceList[_nextTraceIndex % _traceList.Count].transform.localRotation = rotation;
                    _nextTraceIndex += 1;
                    break;
                case TRACKING_MODE.FAKE_CAMERA:
                    _traceList[0].transform.localPosition = traslation; //SetPositionAndRotation(traslation, rotation);
                    _traceList[0].transform.localRotation = rotation;
                    break;
            }

            /*if(_traceList != null && _traceList.Count > 1){
                _traceList[_nextTraceIndex % _traceList.Count].transform.position = traslation; //SetPositionAndRotation(traslation, rotation);
                _traceList[_nextTraceIndex % _traceList.Count].transform.eulerAngles = new Vector3(response.pitch_x, response.roll_y, response.yaw_z);
                _nextTraceIndex += 1;
            } else if {
                Camera.main.transform.position = traslation; //SetPositionAndRotation(traslation, rotation);
                Camera.main.transform.eulerAngles = new Vector3(response.pitch_x, response.roll_y, response.yaw_z);
            }*/
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

        /*public float pitch_x;
        public float roll_y;
        public float yaw_z;*/

        public float quat_x;
        public float quat_y;
        public float quat_z;
        public float quat_w;
    }
}