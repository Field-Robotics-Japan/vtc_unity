using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class TwistTest : UnitySubscriber<MessageTypes.Geometry.Twist>
    {
        public enum WheelType
        {
            Left,
            Right
        }
        [SerializeField] WheelType type;
        public float WheelRadius;
        public float WheelSeparation;
        private Vector3 linearVelocity;
        private Vector3 angularVelocity;
        private bool isMessageReceived;

        private HingeJoint hinge;
        private JointMotor motor;

        protected override void Start()
        {
            base.Start();
            hinge = GetComponent<HingeJoint>();
        }

        protected override void ReceiveMessage(MessageTypes.Geometry.Twist message)
        {
            linearVelocity = ToVector3(message.linear).Ros2Unity();
            angularVelocity = -ToVector3(message.angular).Ros2Unity();
            isMessageReceived = true;
        }

        private static Vector3 ToVector3(MessageTypes.Geometry.Vector3 geometryVector3)
        {
            return new Vector3((float)geometryVector3.x, (float)geometryVector3.y, (float)geometryVector3.z);
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }
        private void ProcessMessage()
        {
            // Debug.Log(linearVelocity);
            // Debug.Log(angularVelocity);
            motor = hinge.motor;
            if (type.ToString() == "Left")
            {
                motor.targetVelocity = (float)180.0 / Mathf.PI * ( 2*linearVelocity[2] + WheelSeparation*angularVelocity[1] ) / ( 2*WheelRadius );
            }
            else if (type.ToString() == "Right")
            {
                motor.targetVelocity = (float)180.0 / Mathf.PI * ( 2*linearVelocity[2] - WheelSeparation*angularVelocity[1] ) / ( 2*WheelRadius );
            }
            else{
                motor.targetVelocity = 0;
            }
            // motor.force = 10000000000000;
            motor.freeSpin = false;
            hinge.motor = motor;
            hinge.useMotor = true;

            isMessageReceived = false;
        }
    }
}