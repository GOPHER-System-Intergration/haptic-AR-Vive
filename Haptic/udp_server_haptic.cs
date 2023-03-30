using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

using Valve.VR;

public class udp_server : MonoBehaviour
{
    Socket socket;
    EndPoint clientEnd;//obtain client information
    EndPoint send2End;//used for sending data to the client
    IPEndPoint ipEnd;
    bool connected = false;
    string recvStr;
    string sendStr;
    byte[] recvData = new byte[1024];
    byte[] sendData = new byte[1024];
    int recvLen;
    string haptic_level;
    float hl;
    float freq_level;
    Thread receiveThread;

    //link to OVRCameraRig
    public Transform targetCamera;
    public Transform leftController;
    public Transform rightController;
    public Transform Chest;
    public Transform L_Upperarm;
    public Transform R_Upperarm;
    public Transform L_Lowerarm;
    public Transform R_Lowerarm;
    //public Transform Tracker;
    System.DateTime dtFrom = System.DateTime.Now;
    //System.DateTime dtFrom = System.DateTime.Now.Ticks;
    long lastSent;

    public SteamVR_Input_Sources LeftInputSource = SteamVR_Input_Sources.LeftHand;
    public SteamVR_Input_Sources RightInputSource = SteamVR_Input_Sources.RightHand;
    //public SteamVR_Input_Sources ChestTracker = SteamVR_Input_Sources.Chest;

    public SteamVR_Action_Vibration hapticAction;

    void InitSocket()
    {
        lastSent = 0;
        //Listen to any IP, serve as a server
        ipEnd = new IPEndPoint(IPAddress.Any, 23023);
        socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        socket.Bind(ipEnd);

        //define a connection after hearing from a client and send message to that client 
        IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
        clientEnd = (EndPoint)sender;
        print("waiting for UDP dgram");

        //Vector3 trackerPos = Cube.position;
        //Quaternion trackerRot = Cube.rotation;

        //thread for hearing from client
        receiveThread = new Thread(new ThreadStart(SocketReceive));
        receiveThread.Start();

        //send 6DOFs head information to the client
        //sendThread = new Thread(new ThreadStart(DataSend));
        //sendThread.Start();
    }

    //1. Convert the vectors from the Unity coordinate to the ROS coordinate
    //2. convert the "corrected vectors" to string, prepare for the UDP package.
    //giving x, y, z, roll, pitch, yaw
    string Vec2Str(Vector3 pos, Quaternion rot)
    {
        string output = (-pos.z).ToString("0.00") + ',';
        output += pos.x.ToString("0.00") + ',';
        output += pos.y.ToString("0.00") + ',';

        output += (-rot.z).ToString("0.0000") + ',';
        output += (rot.x).ToString("0.0000") + ',';
        output += (-rot.y).ToString("0.0000") + ',';
        output += (rot.w).ToString("0.0000") + ',';
        return output;
    }

    void SocketSend(string sendStr)
    {
        //clean up send buffer
        sendData = new byte[1024];
        //data conversion
        sendData = Encoding.ASCII.GetBytes(sendStr);
        //send data
        try
        {
            socket.SendTo(sendData, sendData.Length, SocketFlags.None, clientEnd);
        }
        catch
        {
            Debug.Log("remote closed.");
            connected = false;
        }
    }

    //receive from a client
    void SocketReceive()
    {
        while (true)
        {
            //clean up buffer
            recvData = new byte[1024];
            //receive data and get client information
            try
            {
                Debug.Log("Try to receive");
                recvLen = socket.ReceiveFrom(recvData, ref clientEnd);
                Debug.Log("Message from: " + clientEnd.ToString());
                recvStr = Encoding.ASCII.GetString(recvData, 0, recvLen);
                //Debug.Log(recvStr);
                print("Print Received Value" + recvStr);
                connected = true;
                //float amplitude = float.Parse(recvStr);
                haptic_level = recvStr;
            }
            catch
            {
                Debug.Log("Remote closed. Receive failed");
                connected = false;
            }
        }
    }

    void SocketQuit()
    {
        if (receiveThread != null)
        {
            receiveThread.Interrupt();
            receiveThread.Abort();
        }
        if (socket != null)
            socket.Close();
        Debug.Log("disconnect");
    }

    // Start is called before the first frame update
    void Start()
    {
        InitSocket();
    }

    void BtnState(bool state)
    {
        if (state)
            sendStr += "1, ";
        else
            sendStr += "0, ";
    }

    // Update is called once per frame
    void Update()
    {
        //waiting for a client to connect
        if (!connected)
        {
            Thread.Sleep(20);
        }
        //after connected, send data till lost of connection
        else
        {

            //reduce data rate, only send at around 10Hz
            long currTicks = System.DateTime.Now.Ticks;

            if (currTicks - lastSent > 180 * 10000)
            {
                //print("this is the data received" + recvStr);
                print("data received" + recvStr + "type of data" + recvStr.GetType());
                hl = float.Parse(recvStr);
                if (hl == 701)
                {
                    hapticAction.Execute(0, 0.05f, 0, 75, SteamVR_Input_Sources.RightHand);
                    //hapticAction.Execute(0, 0.5f, 300, 75, SteamVR_Input_Sources.LeftHand);
                    //hapticAction.Execute(0, 0.5f, 0, 75, SteamVR_Input_Sources.LeftHand);
                }
                if (hl == 501)
                {
                    //hapticAction.Execute(0, 1f, 300, 75, SteamVR_Input_Sources.RightHand);
                    //hapticAction.Execute(0, 0.5f, 300, 75, SteamVR_Input_Sources.LeftHand);
                    hapticAction.Execute(0, 0.5f, 0, 75, SteamVR_Input_Sources.LeftHand);
                }
                else if (hl == 301)
                {
                    //hapticAction.Execute(0, 1f, 300, 75, SteamVR_Input_Sources.RightHand);
                    //hapticAction.Execute(0, 0.5f, 300, 75, SteamVR_Input_Sources.LeftHand);
                    //hapticAction.Execute(0, 0.05f, 0, 75, SteamVR_Input_Sources.RightHand);
                    hapticAction.Execute(0, 0.5f, 0, 75, SteamVR_Input_Sources.LeftHand);
                }
                else if (hl > 1000)
                {
                    hl = hl - 1000;
                    if ((hl <= 0.3F) && (hl > 0.15F))
                    {
                        freq_level = 0.1F;
                    }
                    if ((hl <= 0.45F) && (hl > 0.3F))
                    {
                        freq_level = 0.30F;
                    }
                    if ((hl > 0.45F) && (hl != 700) && (hl < 1F))
                    {
                        freq_level = 0.6F;
                    }
                    //hapticAction.Execute(0, 0.05f, freq_level * 300, 75, SteamVR_Input_Sources.RightHand);
                    //hapticAction.Execute(0, 0.05f, 0, 75, SteamVR_Input_Sources.RightHand);
                    //hapticAction.Execute(0, 0.5f, 300, 75, SteamVR_Input_Sources.LeftHand);
                    //hapticAction.Execute(0, 0.5f, 0, 75, SteamVR_Input_Sources.LeftHand);
                }
                //print(hl);
                else
                {
                    if (hl > 1000)
                    {
                        hl = hl - 1000;
                    }
                    if ((hl > 1) && (hl <= 501))
                    {
                        freq_level = 1F;
                        //freq_level = 0F;
                    }

                    /*if (hl == 500)

                    {

                        freq_level = 1F;
                    }*/

                    if (hl == 700)

                    {
                        freq_level = 0F;
                        //print("we are here !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    }

                    /*if (hl < 0.15)
                    {
                        freq_level = 0F;
                    }*/

                    if ((hl <= 0.3F) && (hl > 0.15F))
                    {
                        freq_level = 0.1F;
                    }
                    if ((hl <= 0.45F) && (hl > 0.3F))
                    {
                        freq_level = 0.30F;
                    }
                    if ((hl > 0.45F) && (hl != 700) && (hl < 1F))
                    {
                        freq_level = 0.6F;
                    }
                    //print("_____________________________________" + freq_level + hl);
                    if ((freq_level != 1F) && (freq_level != 0))
                    {
                        //hapticAction.Execute(0, 0.05f, freq_level * 300, 75, SteamVR_Input_Sources.RightHand);
                        hapticAction.Execute(0, 0, 0, 0, SteamVR_Input_Sources.RightHand);
                    }

                    else if (freq_level == 0)
                    {
                        //hapticAction.Execute(0, 0.05F, 0, 75, SteamVR_Input_Sources.RightHand);
                        hapticAction.Execute(0, 0, 0, 0, SteamVR_Input_Sources.RightHand);
                        //print("are we here tho?????????????????????????");
                    }

                    else
                    {
                        //hapticAction.Execute(0, 1f, freq_level * 300, 75, SteamVR_Input_Sources.RightHand);
                        //hapticAction.Execute(0, 1f, freq_level * 300, 75, SteamVR_Input_Sources.LeftHand);
                        //hapticAction.Execute(0, 0.5f, freq_level * 300, 75, SteamVR_Input_Sources.RightHand);
                        hapticAction.Execute(0, 0, 0, 0, SteamVR_Input_Sources.RightHand);
                    }

                }

                Vector3 camPos = targetCamera.position;
                Quaternion rotH = targetCamera.rotation;

                Vector3 rightCtrlPos = rightController.position;
                Quaternion rotR = rightController.rotation;

                Vector3 leftCtrlPos = leftController.position;
                Quaternion rotL = leftController.rotation;

                Vector3 chestPos = Chest.position;
                Quaternion chestR = Chest.rotation;

                Vector3 luaPos = L_Upperarm.position;
                Quaternion luaR = L_Upperarm.rotation;

                Vector3 llaPos = L_Lowerarm.position;
                Quaternion llaR = L_Lowerarm.rotation;

                Vector3 ruaPos = R_Upperarm.position;
                Quaternion ruaR = R_Upperarm.rotation;

                Vector3 rlaPos = R_Lowerarm.position;
                Quaternion rlaR = R_Lowerarm.rotation;



                long currMills = (currTicks - dtFrom.Ticks) / 10000;
                //  Data format: time stamp + 3 positions + 4 quaternion rotations

                sendStr = currMills.ToString() + "," + Vec2Str(camPos, rotH);
                sendStr += "right, ";
                sendStr += Vec2Str(rightCtrlPos, rotR);
                sendStr += "left, ";
                sendStr += Vec2Str(leftCtrlPos, rotL);
                sendStr += "r_ctrl, ";
                sendStr += SteamVR_Actions._default.TrackpadAxis.GetAxis(RightInputSource)[0].ToString("0.00") + ", ";
                sendStr += SteamVR_Actions._default.TrackpadAxis.GetAxis(RightInputSource)[1].ToString("0.00") + ", ";
                sendStr += SteamVR_Actions._default.TriggerAxis.GetAxis(RightInputSource).ToString("0.00") + ", ";
                BtnState(SteamVR_Actions._default.Menu.GetState(RightInputSource));
                BtnState(SteamVR_Actions._default.TriggerBtn.GetState(RightInputSource));
                BtnState(SteamVR_Actions._default.TrackpadBtn.GetState(RightInputSource));
                BtnState(SteamVR_Actions._default.Grip.GetState(RightInputSource));
                //BtnState(SteamVR_Actions._default.System.GetState(RightInputSource));

                sendStr += "l_ctrl, ";
                sendStr += SteamVR_Actions._default.TrackpadAxis.GetAxis(LeftInputSource)[0].ToString("0.00") + ", ";
                sendStr += SteamVR_Actions._default.TrackpadAxis.GetAxis(LeftInputSource)[1].ToString("0.00") + ", ";
                sendStr += SteamVR_Actions._default.TriggerAxis.GetAxis(LeftInputSource).ToString("0.00") + ", ";
                BtnState(SteamVR_Actions._default.Menu.GetState(LeftInputSource));
                BtnState(SteamVR_Actions._default.TriggerBtn.GetState(LeftInputSource));
                BtnState(SteamVR_Actions._default.TrackpadBtn.GetState(LeftInputSource));
                BtnState(SteamVR_Actions._default.Grip.GetState(LeftInputSource));

                sendStr += "tracker, ";
                sendStr += Vec2Str(chestPos, chestR);
                sendStr += Vec2Str(luaPos, luaR);
                sendStr += Vec2Str(ruaPos, ruaR);
                sendStr += Vec2Str(llaPos, llaR);
                sendStr += Vec2Str(rlaPos, rlaR);

                //Debug.Log(sendStr);
                SocketSend(sendStr);
                lastSent = currTicks;
                Thread.Sleep(50);
            }
        }
    }

    void OnApplicationQuit()
    {
        SocketQuit();
    }
}
