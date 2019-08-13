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
        public static PolyCurve Offset(this PolyCurve curve, double offset, Vector normal, bool extend = false)
        {
            if (!curve.IsPlanar())
            {
                BH.Engine.Reflection.Compute.RecordError("Offset works only on planar curves");
                return null;
            }
            if (curve.IsSelfIntersecting())
            {
                BH.Engine.Reflection.Compute.RecordError("Offset works only on non-self intersecting curves");
                return null;
            }
            List<ICurve> ICrvs = new List<ICurve>(curve.Curves);
            PolyCurve result = new PolyCurve();
            if (ICrvs[0] is Circle)
            {
                result.Curves.Add(Offset((Circle)ICrvs[0], offset, normal));
                return result;
            }
            //First - offseting each individual element
            List<ICurve> crvs = new List<ICurve>();
            foreach (ICurve crv in curve.Curves)
            {
                if (crv is Arc)
                {
                    Arc arc = new Arc();
                    arc = ((Arc)crv).Clone();
                    crvs.Add(arc.Offset(offset, normal));
                }
                else
                {
                    Line ln = new Line();
                    ln = ((Line)crv).Clone();
                    Vector mv = new Vector();
                    mv = ln.TangentAtPoint(ln.Start).CrossProduct(normal);
                    double mvScale = new double();
                    mvScale = offset / mv.Length();
                    mv *= mvScale;
                    ln.Start.X += mv.X;
                    ln.Start.Y += mv.Y;
                    ln.Start.Z += mv.Z;
                    ln.End.X += mv.X;
                    ln.End.Y += mv.Y;
                    ln.End.Z += mv.Z;
                    crvs.Add(ln);
                }
            }
            List<ICurve> resultList = new List<ICurve>();
            List<Point> interPts = new List<Point>();
            // from this point on, we are looking for crossing points of curves to trim/extend them
            if (curve.IIsClosed())
            {
                Line inf1 = Create.Line(crvs[crvs.Count - 1].IEndPoint(), crvs[crvs.Count - 1].IEndDir());
                Line inf2 = Create.Line(crvs[0].IStartPoint(), -crvs[0].IStartDir());
                if (crvs[0].ICurveIntersections(crvs[crvs.Count - 1]).Count > 0)
                {
                    interPts.Add(crvs[0].ICurveIntersections(crvs[crvs.Count - 1])[0]);
                }
                else
                {
                    interPts.Add(inf1.LineIntersection(inf2));
                }
            }
            else
            {
                interPts.Add(crvs[0].IStartPoint());
            }
            for (int i = 0; i < crvs.Count - 1; i++)
            {
                Line inf1 = new Line();
                Line inf2 = new Line();
                bool complete = false;
                if (crvs[i].ICurveIntersections(crvs[i + 1]).Count > 0)
                {
                    interPts.Add(crvs[i].ICurveIntersections(crvs[i + 1])[crvs[i].ICurveIntersections(crvs[i + 1]).Count - 1]);
                    complete = true;
                }
                else if (complete == false)
                {
                    inf1 = Create.Line(crvs[i].IEndPoint(), crvs[i].IEndPoint() + crvs[i].IEndDir() * offset * crvs[i].ILength());
                    inf2 = Create.Line(crvs[i + 1].IStartPoint(), crvs[i + 1].IStartPoint() - crvs[i + 1].IStartDir() * offset * crvs[i + 1].ILength());
                    if (inf1.ILineIntersections(inf2).Count > 0)
                    {
                        interPts.Add(inf1.LineIntersection(inf2));
                        complete = true;
                    }
                }
                if (complete == false)
                {
                    inf1 = Create.Line(crvs[i].IEndPoint(), crvs[i].IEndDir());
                    if (crvs[i + 1].ILineIntersections(inf1).Count > 0)
                    {
                        interPts.Add(crvs[i + 1].ILineIntersections(inf1)[0]);
                        complete = true;
                    }
                }
                if (complete == false)
                {
                    inf2 = Create.Line(crvs[i + 1].IStartPoint(), -crvs[i + 1].IStartDir());
                    if (crvs[i].ILineIntersections(inf2).Count > 0)
                    {
                        interPts.Add(crvs[i].ILineIntersections(inf2)[0]);
                        complete = true;
                    }
                }
                if (complete == false)
                {
                    interPts.Add(inf1.LineIntersection(inf2));
                }
            }
            if (curve.IsClosed())
            {
                interPts.Add(interPts[0]);
            }
            else
            {
                interPts.Add(crvs[crvs.Count - 1].IEndPoint());
            }
            //here, we recreate the polycurve based on our control points and external extend/trim functions
            List<ICurve> temp = new List<ICurve>();
            for (int i = 0; i < crvs.Count; i++)
            {
                temp = crvs[i].TrimExtend(interPts[i], interPts[i + 1], extend);
                foreach (ICurve crv in temp)
                {
                    resultList.Add(crv);
                }
            }
            return new PolyCurve
            {
                Curves = resultList
            };
        }
    }
}