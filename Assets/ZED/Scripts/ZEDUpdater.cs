
using UnityEngine;
using System.Threading;
using System;
public class ZEDUpdater
{

    /// <summary>
    /// Grab parameters for multithreading
    /// </summary>
    private struct GrabParameters
    {
        public sl.SENSING_MODE mode;
        public bool depth;
    }

    //The grab parameters
    private GrabParameters grabParameters;

    //Thread doing grab call
    private Thread threadGrab;

    //True if the thread is running
    private bool running = false;

    //Timer for the thread
    private System.Diagnostics.Stopwatch timer;

    //True if threading is activated
    private bool isThreaded = false;

    //Mutex of the grab
    public object grabLock = new object();

    public static ZEDUpdater instance;
    private static object lock_ = new object();

    public delegate void OnGrabAction();
    public static event OnGrabAction OnGrab;

    //HD720 default FPS
    public uint fpsMax = 60;

    // To pause the thread
    bool pauseThread = false;

    //Number times a grab succeeded before a reset
    private int numberGrabbedSucceed = 0;
    /// <summary>
    /// Gets an instance of the ZEDUpdater
    /// </summary>
    /// <returns>The instance</returns>
    public static ZEDUpdater GetInstance()
    {
        lock (lock_)
        {
            if (instance == null)
            {
                instance = new ZEDUpdater();
            }
            return instance;
        }
    }

    private ZEDUpdater()
    {
        //Create the timer used in the grab thread
        timer = new System.Diagnostics.Stopwatch();

        //Default grab parameters for threaded mode
        grabParameters = new GrabParameters();
        grabParameters.mode = sl.SENSING_MODE.FILL;
        grabParameters.depth = true;
    }

    /// <summary>
    /// Set the grab parameters for threading mode
    /// </summary>
    /// <param name="mode"></param>
    public void SetGrabParametersThreadingMode(sl.SENSING_MODE mode, bool depth)
    {
        lock (grabLock)
        {
            grabParameters.mode = mode;
            grabParameters.depth = depth;
        }
    }

    /// <summary>
    /// Run image/data grab in a thread
    /// </summary>
    private void ThreadedGrab()
    {
        while (running)
        {

            float timePerTick = 1000.0f / fpsMax;
            timer.Reset();
            lock (grabLock)
            {
                if (!pauseThread)
                {
                    if (sl.ZEDCamera.GetInstance().Grab(grabParameters.mode, grabParameters.depth) == sl.ERROR_CODE.SUCCESS)
                    {
                        numberGrabbedSucceed++;
                    }
                }
            }
            timer.Stop();
            TimeSpan ts = timer.Elapsed;
            if (ts.Milliseconds < timePerTick)
            {
                Thread.Sleep(((int)timePerTick - ts.Milliseconds));
            }
        }
    }

    /// <summary>
    /// Initializes the grab thread
    /// </summary>
    private void InitGrabThread()
    {
        running = true;
        threadGrab = new Thread(new ThreadStart(ThreadedGrab));
        threadGrab.Start();
    }

    /// <summary>
    /// Stops the current thread
    /// </summary>
    public void Destroy()
    {
        //If the camera is threaded, first stop the thread
        if (isThreaded)
        {
            running = false;
            if (threadGrab != null)
            {
                threadGrab.Join();
            }
        }
    }

    /// <summary>
    /// Pause the Grab in threading mode
    /// </summary>
    /// <param name="value"></param>
    public void SetPauseThread(bool value)
    {
        pauseThread = value;
    }

    /// <summary>
    /// Activate or deactivate threading mode. Performance may be increased when multi thread is activated
    /// </summary>
    /// <param name="value"></param>
    public void SetMultiThread(bool value)
    {
        isThreaded = value;
        if (!running && isThreaded)
        {
            InitGrabThread();
            //dllz_set_is_threaded();
        }
        else if (running && isThreaded)
        {
            Debug.Log("Already threaded");

        }
        else if (running && !isThreaded)
        {
            running = false;
            if (threadGrab != null)
            {
                threadGrab.Join();
            }
        }
    }

    /// <summary>
    /// Check if the grab has been called at leat once, useful in threading mode
    /// </summary>
    /// <returns></returns>
    public bool IsReady()
    {
        return numberGrabbedSucceed != 0;
    }

    /// <summary>
    /// Reset the number of times, the grab has been called between resets
    /// </summary>
    public void Reset()
    {
        numberGrabbedSucceed = 0;
    }

    /// <summary>
    /// Launches a grab and update the listeners
    /// </summary>
    /// <param name="mode"></param>
    public void Update(sl.SENSING_MODE mode, bool depth = true, sl.REFERENCE_FRAME reference = sl.REFERENCE_FRAME.CAMERA)
    {
        if (!isThreaded)
        {
            if (!pauseThread && sl.ZEDCamera.GetInstance().Grab(mode, depth, reference) == sl.ERROR_CODE.SUCCESS)
            {
                SendEventGrab();
            }
        }
        else
        {
            Broadcast();
            grabParameters.mode = mode;
        }
    }

    private void SendEventGrab()
    {
        if (OnGrab != null)
        {
            OnGrab();
        }
        Reset();
    }

    /// <summary>
    /// Broadcast an event if the grab has been called once
    /// </summary>
    public void Broadcast()
    {
        if (IsReady())
        {
            SendEventGrab();
        }
    }
}
