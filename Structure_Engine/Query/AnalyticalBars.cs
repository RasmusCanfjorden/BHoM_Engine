﻿using BH.oM.Structural.Elements;
using System.Collections.Generic;
using BH.oM.Structural.Properties;
using BH.oM.Geometry;

namespace BH.Engine.Structure
{
    public static partial class Query
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        public static List<Bar> AnalyticalBars(this FramingElement element)
        {
            return AnalyticalBars(element.Property as dynamic, element.LocationCurve as dynamic, element.Name);
        }


        /***************************************************/
        /**** Private Methods                           ****/
        /***************************************************/

        private static List<Bar> AnalyticalBars(ConstantFramingElementProperty property, Line centreLine, string name)
        {
            return new List<Bar>() { Create.Bar(centreLine, property.SectionProperty, Create.BarReleaseFixFix(), BarFEAType.Flexural, name) };
        }

        /***************************************************/
    }
}