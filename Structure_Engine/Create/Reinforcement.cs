﻿using BH.oM.Structural.Properties;

namespace BH.Engine.Structure
{
    public static partial class Create
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        public static LayerReinforcement LayerReinforcement(double diameter, double depth, int count, bool isVertical = false)
        {
            return new LayerReinforcement
            {
                Diameter = diameter,
                BarCount = count,
                Depth = depth,
                IsVertical = isVertical
            };
        }

        /***************************************************/

        public static PerimeterReinforcement PerimeterReinforcement(double diameter, int count, ReoPattern pattern)
        {
            return new PerimeterReinforcement
            {
                Diameter = diameter,
                BarCount = count,
                Pattern = pattern
            };
        }

        /***************************************************/

        public static TieReinforcement TieReinforcement(double diameter, double spacing, int count)
        {
            return new TieReinforcement
            {
                Diameter = diameter,
                Spacing = spacing,
                BarCount = count
            };
        }

        /***************************************************/
    }
}