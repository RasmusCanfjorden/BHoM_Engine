﻿using BH.oM.Structural.Properties;

namespace BH.Engine.Structure
{
    public static partial class Create
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        public static SurfaceConstraint SurfaceConstraint(string name = "")
        {
            return new SurfaceConstraint { Name = name };
        }

        /***************************************************/
    }
}