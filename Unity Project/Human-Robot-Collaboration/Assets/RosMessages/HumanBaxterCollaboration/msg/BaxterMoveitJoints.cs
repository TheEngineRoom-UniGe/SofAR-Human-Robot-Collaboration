//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.HumanBaxterCollaboration
{
    public class BaxterMoveitJoints : Message
    {
        public const string RosMessageName = "human_baxter_collaboration/BaxterMoveitJoints";

        public double joint_00;
        public double joint_01;
        public double joint_02;
        public double joint_03;
        public double joint_04;
        public double joint_05;
        public double joint_06;

        public BaxterMoveitJoints()
        {
            this.joint_00 = 0.0;
            this.joint_01 = 0.0;
            this.joint_02 = 0.0;
            this.joint_03 = 0.0;
            this.joint_04 = 0.0;
            this.joint_05 = 0.0;
            this.joint_06 = 0.0;
        }

        public BaxterMoveitJoints(double joint_00, double joint_01, double joint_02, double joint_03, double joint_04, double joint_05, double joint_06)
        {
            this.joint_00 = joint_00;
            this.joint_01 = joint_01;
            this.joint_02 = joint_02;
            this.joint_03 = joint_03;
            this.joint_04 = joint_04;
            this.joint_05 = joint_05;
            this.joint_06 = joint_06;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_00));
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_01));
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_02));
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_03));
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_04));
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_05));
            listOfSerializations.Add(BitConverter.GetBytes(this.joint_06));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.joint_00 = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.joint_01 = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.joint_02 = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.joint_03 = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.joint_04 = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.joint_05 = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.joint_06 = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "BaxterMoveitJoints: " +
            "\njoint_00: " + joint_00.ToString() +
            "\njoint_01: " + joint_01.ToString() +
            "\njoint_02: " + joint_02.ToString() +
            "\njoint_03: " + joint_03.ToString() +
            "\njoint_04: " + joint_04.ToString() +
            "\njoint_05: " + joint_05.ToString() +
            "\njoint_06: " + joint_06.ToString();
        }
    }
}
