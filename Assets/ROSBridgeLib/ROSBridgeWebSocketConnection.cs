using SimpleJSON;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using WebSocketSharp;

namespace ROSBridgeLib
{
    /// <summary>
    /// This class handles the connection with the external ROS world, deserializing
    /// json messages into appropriate instances of packets and messages.
    ///  
    /// This class also provides a mechanism for having the callback's exectued on the rendering thread.
    /// (Remember, Unity has a single rendering thread, so we want to do all of the communications stuff away
    /// from that.
    /// 
    /// The one other clever thing that is done here is that we only keep 1 (the most recent!) copy of each message type
    /// that comes along.
    ///  
    /// Version History
    /// 3.1 - changed methods to start with an upper case letter to be more consistent with c#
    /// style.
    /// 3.0 - modification from hand crafted version 2.0
    /// 
    /// @author Michael Jenkin, Robert Codd-Downey and Andrew Speers
    /// @version 3.1
    /// </summary>
    public class ROSBridgeWebSocketConnection
    {
        private class RenderTask
        {
            private Type _subscriber;
            private string _topic;
            private ROSBridgeMsg _msg;

            public RenderTask(Type subscriber, string topic, ROSBridgeMsg msg)
            {
                _subscriber = subscriber;
                _topic = topic;
                _msg = msg;
            }

            public Type getSubscriber()
            {
                return _subscriber;
            }

            public ROSBridgeMsg getMsg()
            {
                return _msg;
            }

            public string getTopic()
            {
                return _topic;
            }
        };

        private string _host;
        private int _port;
        private WebSocket _ws;
        private System.Threading.Thread _myThread;
        private List<Type> _subscribers; // our subscribers
        private List<Type> _publishers; //our publishers
        private Type _serviceResponse; // to deal with service responses
        private string _serviceName = null;
        private string _serviceValues = null;
        private List<RenderTask> _taskQ = new List<RenderTask>();
        // Adjustment to be able to add ros objects at runtime while connected
        private bool _running = false;
        private bool _sendErrorMessage = false;

        private object _queueLock = new object();

        /// <summary>
        /// list containing all publishing topics (only one message type possible -> not necessary) that have been announced so far
        /// </summary>
        private List<string> m_AnnouncedTopics = new List<string>();

        /// <summary>
        /// list containing all subscription topics (only one message type possible -> not necessary to specify) that have been announced so far
        /// </summary>
        private List<string> m_SubscribedTopics = new List<string>();

        private static string GetMessageType(Type t)
        {
            return (string)t.GetMethod("GetMessageType", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, null);
        }

        private static string GetMessageTopic(Type t)
        {
            return (string)t.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, null);
        }

        private static ROSBridgeMsg ParseMessage(Type t, JSONNode node)
        {
            return (ROSBridgeMsg)t.GetMethod("ParseMessage", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { node });
        }

        private static void Update(Type t, ROSBridgeMsg msg)
        {
            t.GetMethod("CallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { msg });
        }

        private static void ServiceResponse(Type t, string service, string yaml)
        {
            t.GetMethod("ServiceCallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy).Invoke(null, new object[] { service, yaml });
        }

        private static void IsValidServiceResponse(Type t)
        {
            if (t.GetMethod("ServiceCallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("invalid service response handler");
        }

        private static void IsValidSubscriber(Type t)
        {
            if (t.GetMethod("CallBack", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing Callback method");
            if (t.GetMethod("GetMessageType", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageType method");
            if (t.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageTopic method");
            if (t.GetMethod("ParseMessage", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing ParseMessage method");
        }

        private static void IsValidPublisher(Type t)
        {
            if (t.GetMethod("GetMessageType", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageType method");
            if (t.GetMethod("GetMessageTopic", BindingFlags.Public | BindingFlags.Static | BindingFlags.FlattenHierarchy) == null)
                throw new Exception("missing GetMessageTopic method");
        }

        /**
		 * Make a connection to a host/port. 
		 * This does not actually start the connection, use Connect to do that.
		 */
        public ROSBridgeWebSocketConnection(string host, int port)
        {
            _host = host;
            _port = port;
            _myThread = null;
            _subscribers = new List<Type>();
            _publishers = new List<Type>();
        }

        /**
		 * Add a service response callback to this connection.
		 */
        public void AddServiceResponse(Type serviceResponse)
        {
            IsValidServiceResponse(serviceResponse);
            _serviceResponse = serviceResponse;
        }

        /**
         * Not Implemented! 
         */
        public void RemoveServiceResponse(Type serviceResponse)
        {
            // Code here
            //-> somehow this method is called... No clue what it is supposed to do
        }

        /**
		 * Add a subscriber callback to this connection. There can be many subscribers.
		 */
        public void AddSubscriber(Type subscriber)
        {
            //Multiple subscribers for the same topic possible
            IsValidSubscriber(subscriber);
            string topic = GetMessageTopic(subscriber);
            _subscribers.Add(subscriber);

            if (_running)
            {
                //only announce if not announced yet, 
                if (!m_SubscribedTopics.Contains(topic))
                {
                    //Debug.Log("[ROS WEBSOCKET] Adding Subscriber. Subscribing to " + topic);
                    _ws.Send(ROSBridgeMsg.Subscribe(GetMessageTopic(subscriber), GetMessageType(subscriber)));
                    m_SubscribedTopics.Add(topic);
                }
                else
                {
                    Debug.Log("[ROS WEBSOCKET] ALREADY subscribed to " + topic);
                }
            }
            else
            {
                Debug.Log("[ROS WEBSOCKET] couldn't subscribe to " + topic + ". Websocket not running.");
            }
        }

        /**
         * Removes a subscriber and disconnects him if the connection is already running.
         */
        public void RemoveSubscriber(Type subscriber)
        {
            if (!_subscribers.Contains(subscriber))
                return;

            _subscribers.Remove(subscriber);
            string topic = GetMessageTopic(subscriber);
            m_SubscribedTopics.Remove(topic);
            if (_running)
            {
                //since multiple subscribers can subscribe to same topic, only send unsubscribe if no more subscribers on this topic
                if (!m_SubscribedTopics.Contains(topic))
                {
                    Debug.Log("[ROS WEBSOCKET]Not subcribing anymore to " + topic);
                    _ws.Send(ROSBridgeMsg.UnSubscribe(topic));
                }
            }
        }

        /**
		 * Add a publisher to this connection. There can be many publishers.
		 */
        public void AddPublisher(Type publisher)
        {
            IsValidPublisher(publisher);
            string topic = GetMessageTopic(publisher);
            _publishers.Add(publisher);

            if (_running)
            {
                if (!m_AnnouncedTopics.Contains(topic))
                { // only advertise if needed
                    Debug.Log("[ROS WEBSOCKET] Adding publisher. Advertising " + topic);
                    _ws.Send(ROSBridgeMsg.Advertise(topic, GetMessageType(publisher)));
                }
                //keep track of all publishers (no matter the topic or if the topic already exists )
                m_AnnouncedTopics.Add(topic);
            }
            else
            {
                Debug.Log("[ROS WEBSOCKET] Could not advertise publisher. Websocket not running.");
            }
        }

        /**
         * Removes a publisher from the connection. Disconnects the publisher if connection is already running.
         */
        public void RemovePublisher(Type publisher)
        {
            if (!_publishers.Contains(publisher))
                return;

            _publishers.Remove(publisher);
            //keep track of all publishers no matter if the topic 
            string topic = GetMessageTopic(publisher);
            m_AnnouncedTopics.Remove(topic);
            if (_running)
            {
                //if we hvae no more publishers on this topic
                if (!m_AnnouncedTopics.Contains(topic))
                {
                    Debug.Log("[ROS WEBSOCKET] not announcing anymore on: " + topic);
                    _ws.Send(ROSBridgeMsg.UnAdvertise(topic));
                }
            }
        }

        /**
		 * Connect to the remote ros environment.
		 */
        public void Connect()
        {
            Debug.Log("[ROS WEBSOCKET] Connecting to ROS");
            _myThread = new System.Threading.Thread(Run);
            _myThread.Start();
        }

        /**
		 * Disconnect from the remote ros environment.
		 */
        public void Disconnect()
        {
            Debug.Log("[ROS WEBSOCKET] Disconnecting from ROS");
            _myThread.Abort();
            foreach (Type p in _subscribers)
            {
                string topic = GetMessageTopic(p);
                //remove this subscriber
                if (m_SubscribedTopics.Contains(topic))
                {
                    m_SubscribedTopics.Remove(topic);
                }
                //if no mo subscribers on this topic
                if (!m_SubscribedTopics.Contains(topic))
                {
                    Debug.Log("[ROS WEBSOCKET] not subscribing anymore to " + topic);
                    _ws.Send(ROSBridgeMsg.UnSubscribe(topic));
                }
            }
            foreach (Type p in _publishers)
            {
                string topic = GetMessageTopic(p);
                //remove this advertiser
                if (m_AnnouncedTopics.Contains(topic))
                {
                    m_AnnouncedTopics.Remove(topic);
                }

                //if no more advertiser on this topic
                if (!m_AnnouncedTopics.Contains(topic))
                {
                    Debug.Log("[ROS WEBSOCKET] not publishing on topic: " + topic);
                    _ws.Send(ROSBridgeMsg.UnAdvertise(topic));
                }
            }
            Debug.Log("[ROS WEBSOCKET] Closing websocket");
            _ws.Close();
            _running = false;
        }

        private void Run()
        {
            _ws = new WebSocket(_host + ":" + _port);
            _ws.OnMessage += (sender, e) => this.OnMessage(e.Data);
            _ws.OnError += (sender, e) => OnError(sender, e);
            _ws.Connect();
            CheckConnection();

            _running = true;
            Debug.Log("[ROSBRIDGE] running = true; Starting to advertise / subscribe");
            AnnouncePublishersAndSubscribers();
        }

        /// <summary>
        /// Announces all publishers and subscribers which were previously added to the lists _publishers and _subscriers
        /// </summary>
        private void AnnouncePublishersAndSubscribers()
        {
            if (_running && _ws != null)
            {
                foreach (Type p in _subscribers)
                {
                    string topic = GetMessageTopic(p);
                    //only announce if not already known that we subscribed
                    if (!m_SubscribedTopics.Contains(topic))
                    {
                        Debug.Log("[ROS WEBSOCKET] Subscribing to " + topic);
                        _ws.Send(ROSBridgeMsg.Subscribe(topic, GetMessageType(p)));
                    }
                    m_SubscribedTopics.Add(topic);
                }
                foreach (Type p in _publishers)
                {
                    string topic = GetMessageTopic(p);
                    //only announce new publisher if we didn't already announce one for this topic
                    if (!m_AnnouncedTopics.Contains(topic))
                    {
                        Debug.Log("[ROS WEBSOCKET] Advertising " + topic);
                        _ws.Send(ROSBridgeMsg.Advertise(topic, GetMessageType(p)));
                    }
                    m_AnnouncedTopics.Add(topic);
                }
            }
            else
            {
                Debug.LogWarning("[ROS WEBSOCKET] COuld not advertise/ subscribe since websocket not running.");
            }
        }


        /**
         * Checks if the connection is open and prints a message in that case. Does not do anything in the other case as Connect() has an exception in this case and it wont be printed anyway.
         */
        private void CheckConnection()
        {
            if (_ws.ReadyState == WebSocketState.Open)
            {
                Debug.Log("[ROS WEBSOCKET] Connection to ROSBridge successful!");
            }
        }

        /**
         * Error handler method which simply prints the error message.
         */
        private void OnError(object sender, WebSocketSharp.ErrorEventArgs e)
        {
            if (!_sendErrorMessage)
            {
                Debug.Log(e.Message);
                _sendErrorMessage = true;
            }

        }

        private void OnMessage(string s)
        {
            //Debug.Log("Got a message " + s);
            if ((s != null) && !s.Equals(""))
            {
                JSONNode node;
                try
                {
                    node = JSONNode.Parse(s);
                }
                catch { return; }
                //Debug.Log ("Parsed it");
                string op = node["op"];
                if ("publish".Equals(op))
                {
                    //Message was published on this topic
                    string topic = node["topic"];
                    foreach (Type p in _subscribers)
                    {
                        //search for subscriber listening to the topic of the received message
                        if (topic.Equals(GetMessageTopic(p)))
                        {
                            //Debug.Log ("And will parse it " + GetMessageTopic (p));
                            ROSBridgeMsg msg;
                            try
                            {
                                msg = ParseMessage(p, node["msg"]);
                            }
                            catch { return; }
                            RenderTask newTask = new RenderTask(p, topic, msg);
                            lock (_queueLock)
                            {
                                bool found = false;
                                for (int i = 0; i < _taskQ.Count; i++)
                                {
                                    if (_taskQ[i].getTopic().Equals(topic))
                                    {
                                        _taskQ.RemoveAt(i);
                                        _taskQ.Insert(i, newTask);
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found)
                                    _taskQ.Add(newTask);
                            }

                        }
                    }
                }
                else if ("service_response".Equals(op))
                {
                    //Debug.Log ("Got service response " + node.ToString ());
                    _serviceName = node["service"];
                    _serviceValues = (node["values"] == null) ? "" : node["values"].ToString();
                }
                else
                    Debug.Log("Must write code here for other messages");
            }
            else
                Debug.Log("Got an empty message from the web socket");
        }

        public void Render()
        {
            RenderTask newTask = null;
            lock (_queueLock)
            {
                if (_taskQ.Count > 0)
                {
                    newTask = _taskQ[0];
                    _taskQ.RemoveAt(0);
                }
            }
            if (newTask != null)
                Update(newTask.getSubscriber(), newTask.getMsg());
            if (_serviceName != null)
            {
                ServiceResponse(_serviceResponse, _serviceName, _serviceValues);
                _serviceName = null;
            }
        }

        public void Publish(string topic, ROSBridgeMsg msg)
        {
            if (_ws != null)
            {

                if (m_AnnouncedTopics.Contains(topic))
                {
                    string s = ROSBridgeMsg.Publish(topic, msg.ToYAMLString());
                    //Debug.Log ("Sending on " + topic);
                    _ws.Send(s);
                }
                else
                {// if not announced that we're publishing on this topic 
                 //-> go through list of publishers/ subscribers and see if one is not yet advertised, try again

                    AnnouncePublishersAndSubscribers();
                    //Try second time otherwise not working
                    if (m_AnnouncedTopics.Contains(topic))
                    {
                        string s = ROSBridgeMsg.Publish(topic, msg.ToYAMLString());
                        //Debug.Log ("Sending  on " + topic);
                        _ws.Send(s);
                    }
                    else
                    {
                        Debug.LogWarning("[ROS Websocket Connection] Trying to publish on a topic that is not registered!\n Topic: " + topic);
                    }
                }
            }
        }

        public void CallService(string service, string args)
        {
            if (_ws != null)
            {
                string s = ROSBridgeMsg.CallService(service, args);
                //Debug.Log ("Sending " + s);
                _ws.Send(s);
            }
        }
    }
}
