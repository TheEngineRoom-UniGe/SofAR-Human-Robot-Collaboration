//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.HumanBaxterCollaboration
{
    public class TrajectoryServiceResponse : Message
    {
        public const string RosMessageName = "human_baxter_collaboration/TrajectoryService";

        public Moveit.RobotTrajectory[] trajectories;

        public TrajectoryServiceResponse()
        {
            this.trajectories = new Moveit.RobotTrajectory[0];
        }

        public TrajectoryServiceResponse(Moveit.RobotTrajectory[] trajectories)
        {
            this.trajectories = trajectories;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(trajectories.Length));
            foreach(var entry in trajectories)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var trajectoriesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.trajectories= new Moveit.RobotTrajectory[trajectoriesArrayLength];
            for(var i = 0; i < trajectoriesArrayLength; i++)
            {
                this.trajectories[i] = new Moveit.RobotTrajectory();
                offset = this.trajectories[i].Deserialize(data, offset);
            }

            return offset;
        }

        public override string ToString()
        {
            return "TrajectoryServiceResponse: " +
            "\ntrajectories: " + System.String.Join(", ", trajectories.ToList());
        }
    }
}
