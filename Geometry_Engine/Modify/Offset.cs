/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2019, the respective contributors. All rights reserved.
 *
 * Each contributor holds copyright over their respective contributions.
 * The project versioning (Git) records all such contribution source information.
 *                                           
 *                                                                              
 * The BHoM is free software: you can redistribute it and/or modify         
 * it under the terms of the GNU Lesser General Public License as published by  
 * the Free Software Foundation, either version 3.0 of the License, or          
 * (at your option) any later version.                                          
 *                                                                              
 * The BHoM is distributed in the hope that it will be useful,              
 * but WITHOUT ANY WARRANTY; without even the implied warranty of               
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 
 * GNU Lesser General Public License for more details.                          
 *                                                                            
 * You should have received a copy of the GNU Lesser General Public License     
 * along with this code. If not, see <https://www.gnu.org/licenses/lgpl-3.0.html>.      
 */

using BH.oM.Geometry;
using System;
using System.Collections.Generic;

namespace BH.Engine.Geometry
{
    public static partial class Modify
    {
        /***************************************************/
        /**** Public Methods - Curves                   ****/
        /***************************************************/

        public static Arc Offset(this Arc curve, double offset, Vector normal)
        {
            if (!curve.IsPlanar())
            {
                BH.Engine.Reflection.Compute.RecordError("Offset works only on planar curves");
                return null;
            }

            double radius = curve.Radius;
            Arc result = curve.Clone();

            if (normal.DotProduct(curve.Normal()) > 0)
                radius += offset;
            else
                radius -= offset;

            if (radius > 0)
            {
                result.Radius = radius;
                return result;
            }
            else
            {
                BH.Engine.Reflection.Compute.RecordError("Offset value is greater than arc radius");
                return null;
            }
        }

        /***************************************************/

        public static Circle Offset(this Circle curve, double offset, Vector normal)
        {
            double radius = curve.Radius;
            Circle result = curve.Clone();

            if (normal.DotProduct(curve.Normal()) > 0)
                radius += offset;
            else
                radius -= offset;

            if (radius > 0)
            {
                result.Radius = radius;
                return result;
            }
            else
            {
                BH.Engine.Reflection.Compute.RecordError("Offset value is greater than circle radius");
                return null;
            }

        }

        /***************************************************/

        public static Polyline Offset(this Polyline curve, double offset, Vector normal)
        {
            if (!curve.IsPlanar())
            {
                BH.Engine.Reflection.Compute.RecordError("Offset works only on planar curves");
                return null;
            }

            List<Point> cPts = new List<Point>(curve.ControlPoints);
            List<Point> tmp = new List<Point>(curve.ControlPoints);


            if (!curve.IsClosed())
            {
                for (int i = 0; i < cPts.Count - 1; i++) //moving every vertex perpendicularly to each of it's edgses. At this point only direction of move is good.
                {
                    Vector trans = (cPts[i + 1] - cPts[i]).CrossProduct(normal);
                    trans = trans.Normalise() * offset;
                    tmp[i] += trans;
                    tmp[i + 1] += trans;
                }

                for (int i = 1; i < cPts.Count - 1; i++) //adjusting move distance
                {
                    Vector trans = (tmp[i] - cPts[i]);
                    trans = trans.Normalise() * Math.Abs(offset) / Math.Sin(Math.Abs((cPts[i] - cPts[i - 1]).Angle(cPts[i] - cPts[i + 1])) / 2);
                    tmp[i] = cPts[i] + trans;
                }
                return new Polyline { ControlPoints = tmp };
            }
            else
            {
                cPts.RemoveAt(cPts.Count - 1);
                tmp.RemoveAt(tmp.Count - 1);
                for (int i = 0; i < cPts.Count; i++) //moving every vertex perpendicularly to each of it's edgses. At this point only direction of move is good.
                {
                    Vector trans = (cPts[(i + 1) % cPts.Count] - cPts[i]).CrossProduct(normal);
                    trans = trans.Normalise() * offset;
                    tmp[i] += trans;
                    tmp[(i + 1) % cPts.Count] += trans;
                }

                for (int i = 0; i < cPts.Count; i++)  // adjusting move distance
                {
                    if (!(Math.Abs(Math.Sin((cPts[i] - cPts[(i + cPts.Count - 1) % cPts.Count]).Angle(cPts[i] - cPts[(i + 1) % cPts.Count]))) < Tolerance.Angle))
                    {
                        Vector trans = (tmp[i] - cPts[i]);
                        trans = trans.Normalise() * Math.Abs(offset) / Math.Sin(Math.Abs((cPts[i] - cPts[(i + cPts.Count - 1) % cPts.Count]).Angle(cPts[i] - cPts[(i + 1) % cPts.Count])) / 2);
                        tmp[i] = cPts[i] + trans;
                    }
                }
                tmp.Add(tmp[0]);
                return new Polyline { ControlPoints = tmp };
            }
        }

        /***************************************************/
    }
}