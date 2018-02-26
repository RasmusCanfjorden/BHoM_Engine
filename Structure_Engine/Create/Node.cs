﻿using BH.oM.Structural.Elements;
using BH.oM.Structural.Properties;
using BH.oM.Geometry;

namespace BH.Engine.Structure
{
    public static partial class Create
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        public static Node Node(Point position, string name = "", Constraint6DOF constraint = null)
        {
            return new Node
            {
                Position = new Point { X = position.X, Y = position.Y, Z = position.Z },
                Name = name,
                Constraint = constraint
            };
        }

        /***************************************************/
    }
}
