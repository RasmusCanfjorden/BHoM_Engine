﻿using BH.oM.Geometry;
using System.Linq;

namespace BH.Engine.Geometry
{
    public static partial class Modify
    {
        /***************************************************/
        /**** Public Methods - Curves                   ****/
        /***************************************************/

        public static Line Extend(this Line curve, double start = 0.0, double end = 0.0)
        {
            Vector dir = curve.Direction();
            return new Line { Start = curve.Start - dir * start, End = curve.End + dir * end };
        }
    }
}
