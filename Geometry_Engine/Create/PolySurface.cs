﻿using BH.oM.Geometry;
using System.Collections.Generic;
using System.Linq;

namespace BH.Engine.Geometry
{
    public static partial class Create
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        public static PolySurface PolySurface(IEnumerable<ISurface> surfaces)
        {
            return new PolySurface { Surfaces = surfaces.ToList() };
        }

        /***************************************************/
    }
}