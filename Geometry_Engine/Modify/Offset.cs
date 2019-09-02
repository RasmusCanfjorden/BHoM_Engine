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
            List<ICurve> resultList = new List<ICurve>();
            bool reversed = false;
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
                    ln.Start = ln.Start + mv;
                    ln.End += mv;
                    crvs.Add(ln);
                }
            }
            int lngstIndex = 0;
            for (int i = 1; i < crvs.Count; i++)
            {
                if (crvs[i].ILength() > crvs[lngstIndex].ILength())
                {
                    lngstIndex = i;
                }
            }
            resultList.Add(crvs[lngstIndex]);
            if (curve.IsClosed())
            {
                if (crvs[(lngstIndex + 1) % crvs.Count].ILength() < crvs[(lngstIndex + crvs.Count - 1) % crvs.Count].ILength())
                {
                    crvs.Reverse();
                    reversed = true;
                }
            }
            else
            {
                if(lngstIndex==crvs.Count-1)
                {
                    crvs.Reverse();
                    reversed = true;
                }
                else if(lngstIndex!=0)
                {
                    if (crvs[(lngstIndex + 1) % crvs.Count].ILength() < crvs[(lngstIndex + crvs.Count - 1) % crvs.Count].ILength())
                    {
                        crvs.Reverse();
                        reversed = true;
                    }
                }
            }
            if (reversed)
                lngstIndex = crvs.Count - 1 - lngstIndex;
            List<Point> intPts = new List<Point>();
            List<ICurve> temp = new List<ICurve>();
            ICurve tmp;
            int rvrsNumber = 0;
            bool check = false;
            for (int i = lngstIndex + 1; i < lngstIndex + crvs.Count - 1; i++)
            {
                rvrsNumber = 0;
                check = false;
                if ((intPts = resultList[resultList.Count - 1].ICurveIntersections(crvs[i % crvs.Count])).Count > 0)
                {
                    check = true;
                }
                if (reversed)
                {
                    Line ln1 = Create.Line(resultList[resultList.Count - 1].IStartPoint(), (-1) * ((resultList[resultList.Count - 1].IStartDir() / resultList[resultList.Count - 1].IStartDir().Length()) * 100 * Math.Abs(offset)));
                    Line ln2 = Create.Line(crvs[i % crvs.Count].IEndPoint(), ((crvs[i % crvs.Count].IEndDir() / crvs[i % crvs.Count].IEndDir().Length()) * 100 * Math.Abs(offset)));
                    ln1.Infinite = false;
                    ln2.Infinite = false;
                    if (resultList[resultList.Count - 1] is Line)
                        ln1.Start = resultList[resultList.Count - 1].IEndPoint();
                    if (crvs[i % crvs.Count] is Line)
                        ln2.Start = crvs[i % crvs.Count].IStartPoint();
                    if ((intPts = ln1.ICurveIntersections(crvs[i % crvs.Count])).Count > 0)
                    {
                        check = true;
                    }
                    else if ((intPts = ln2.ICurveIntersections(resultList[resultList.Count - 1])).Count > 0)
                    {
                        check = true;
                    }
                    else if ((intPts = ln2.LineIntersections(ln1)).Count > 0)
                    {
                        check = true;
                    }
                }
                else
                {
                    Line ln1 = Create.Line(resultList[resultList.Count - 1].IEndPoint(), ((resultList[resultList.Count - 1].IEndDir() / resultList[resultList.Count - 1].IEndDir().Length()) * 100 * Math.Abs(offset)));
                    Line ln2 = Create.Line(crvs[i % crvs.Count].IStartPoint(), (-1) * ((crvs[i % crvs.Count].IStartDir() / crvs[i % crvs.Count].IStartDir().Length()) * 100 * Math.Abs(offset)));
                    ln1.Infinite = false;
                    ln2.Infinite = false;
                    if (resultList[resultList.Count - 1] is Line)
                        ln1.Start = resultList[resultList.Count - 1].IStartPoint();
                    if (crvs[i % crvs.Count] is Line)
                        ln2.Start = crvs[i % crvs.Count].IEndPoint();
                    if ((intPts = ln1.ICurveIntersections(crvs[i % crvs.Count])).Count > 0)
                    {
                        check = true;
                    }
                    else if ((intPts = ln2.ICurveIntersections(resultList[resultList.Count - 1])).Count > 0)
                    {
                        check = true;
                    }
                    else if ((intPts = ln2.LineIntersections(ln1)).Count > 0)
                    {
                        check = true;
                    }
                }
                if (check)
                {
                    if (reversed)
                    {
                        tmp = resultList[resultList.Count - 1].IClone();
                        resultList.Remove(resultList[resultList.Count - 1]);
                        temp = tmp.TrimExtend(intPts[0], tmp.IEndPoint(), extend);
                        foreach (ICurve crv in temp)
                        {
                            resultList.Add(crv);
                            rvrsNumber++;

                        }
                        resultList.Reverse(resultList.Count - temp.Count, temp.Count);
                        temp = crvs[i % crvs.Count].TrimExtend(crvs[i % crvs.Count].IStartPoint(), intPts[0], extend);
                        foreach (ICurve crv in temp)
                        {
                            resultList.Add(crv);
                            rvrsNumber++;
                        }
                        resultList.Reverse(resultList.Count - temp.Count, temp.Count);
                        resultList.Reverse(resultList.Count - rvrsNumber, rvrsNumber);
                    }
                    else
                    {
                        tmp = resultList[resultList.Count - 1].IClone();
                        resultList.Remove(resultList[resultList.Count - 1]);
                        temp = tmp.TrimExtend(tmp.IStartPoint(), intPts[0], extend);
                        foreach (ICurve crv in temp)
                        {
                            resultList.Add(crv);
                        }
                        temp = crvs[i % crvs.Count].TrimExtend(intPts[0], crvs[i % crvs.Count].IEndPoint(), extend);
                        foreach (ICurve crv in temp)
                        {
                            resultList.Add(crv);
                        }
                    }
                }
            }
            result.Curves = resultList;
                if(curve.IsClosed()&&!result.IsSelfIntersecting())
            {
                List<Point> finalIntPts = new List<Point>() ;
                if(crvs[lngstIndex-1] is Line)
                {
                    Line ln1 = Create.Line(crvs[lngstIndex - 1].IStartPoint(), crvs[lngstIndex - 1].ITangentAtParameter(0.5));
                    intPts = ln1.ICurveIntersections(Create.Line(resultList[0].IStartPoint(),resultList[0].ITangentAtParameter(0)));
                    if(intPts.Count!=0)
                    {
                        if(intPts.Count==1)
                        {
                            finalIntPts.Add(intPts[0]);
                        }
                        else
                        {
                            if(intPts[0].IDistance(resultList[resultList.Count-1])<intPts[1].IDistance(resultList[resultList.Count-1]))
                                finalIntPts.Add(intPts[0]);
                            else
                                finalIntPts.Add(intPts[1]);
                        }
                    }
                    Line ln2 = Create.Line(crvs[lngstIndex - 1].IStartPoint(), crvs[lngstIndex - 1].ITangentAtParameter(0.5));
                    intPts = ln2.ICurveIntersections(resultList[resultList.Count - 1]);
                    if (intPts.Count != 0)
                    {
                        if (intPts.Count == 1)
                        {
                            finalIntPts.Add(intPts[0]);
                        }
                        else
                        {
                            if (intPts[0].IDistance(resultList[0]) < intPts[1].IDistance(resultList[0]))
                                finalIntPts.Add(intPts[0]);
                            else
                                finalIntPts.Add(intPts[1]);
                        }
                    }
                    temp = resultList[resultList.Count - 1].TrimExtend(resultList[resultList.Count - 1].IStartPoint(), finalIntPts[1], extend);
                    if (temp.Count == 1)
                    {
                        resultList[resultList.Count - 1] = temp[0];
                    }
                    else if (temp.Count == 2)
                    {
                        resultList.Add(temp[1]);
                        resultList[resultList.Count - 1] = temp[0];
                    }
                    temp =crvs[lngstIndex-1].TrimExtend(finalIntPts[1],finalIntPts[0],extend);
                    foreach(ICurve crv in temp)
                    {
                        resultList.Add(crv);
                    }
                    temp = resultList[0].TrimExtend(finalIntPts[0], resultList[0].IEndPoint(), extend);
                    if(temp.Count==1)
                    {
                        resultList[0] = temp[0];
                    }
                    else if(temp.Count==2)
                    {
                        resultList.Add(temp[0]);
                        resultList[0] = temp[1];
                    }
                }
            }
            //result.Curves = resultList;
            //if(curve.IsClosed()&&!result.IsClosed())
            //{
            //    if(reversed)
            //    {
            //        crvs[lngstIndex
            //    }
            //}
            //List<Point> interPts = new List<Point>();
            //// from this point on, we are looking for crossing points of curves to trim/extend them
            //if (curve.IIsClosed())
            //{
            //    Line inf1 = Create.Line(crvs[crvs.Count - 1].IEndPoint(), crvs[crvs.Count - 1].IEndDir());
            //    Line inf2 = Create.Line(crvs[0].IStartPoint(), -crvs[0].IStartDir());
            //    if (crvs[0].ICurveIntersections(crvs[crvs.Count - 1]).Count > 0)
            //    {
            //        interPts.Add(crvs[0].ICurveIntersections(crvs[crvs.Count - 1])[0]);
            //    }
            //    else
            //    {
            //        interPts.Add(inf1.LineIntersection(inf2));
            //    }
            //}
            //else
            //{
            //    interPts.Add(crvs[0].IStartPoint());
            //}
            //for (int i = 0; i < crvs.Count - 1; i++)
            //{
            //    Line inf1 = new Line();
            //    Line inf2 = new Line();
            //    bool complete = false;
            //    if (crvs[i].ICurveIntersections(crvs[i + 1]).Count > 0)
            //    {
            //        interPts.Add(crvs[i].ICurveIntersections(crvs[i + 1])[crvs[i].ICurveIntersections(crvs[i + 1]).Count - 1]);
            //        complete = true;
            //    }
            //    else if (complete == false)
            //    {
            //        inf1 = Create.Line(crvs[i].IEndPoint(), crvs[i].IEndPoint() + crvs[i].IEndDir() * Math.Abs(offset) * crvs[i].ILength());
            //        inf2 = Create.Line(crvs[i + 1].IStartPoint(), crvs[i + 1].IStartPoint() - crvs[i + 1].IStartDir() * Math.Abs(offset) * crvs[i + 1].ILength());
            //        if (inf1.ILineIntersections(inf2).Count > 0)
            //        {
            //            interPts.Add(inf1.LineIntersection(inf2));
            //            complete = true;
            //        }
            //    }
            //    if (complete == false)
            //    {
            //        inf1 = Create.Line(crvs[i].IEndPoint(), crvs[i].IEndDir());
            //        if (crvs[i + 1].ILineIntersections(inf1).Count > 0)
            //        {
            //            interPts.Add(crvs[i + 1].ILineIntersections(inf1)[0]);
            //            complete = true;
            //        }
            //    }
            //    if (complete == false)
            //    {
            //        inf2 = Create.Line(crvs[i + 1].IStartPoint(), -crvs[i + 1].IStartDir());
            //        if (crvs[i].ILineIntersections(inf2).Count > 0)
            //        {
            //            interPts.Add(crvs[i].ILineIntersections(inf2)[0]);
            //            complete = true;
            //        }
            //    }
            //    if (complete == false)
            //    {
            //        interPts.Add(inf1.LineIntersection(inf2));
            //    }
            //}
            //if (curve.IsClosed())
            //{
            //    interPts.Add(interPts[0]);
            //}
            //else
            //{
            //    interPts.Add(crvs[crvs.Count - 1].IEndPoint());
            //}
            ////here, we recreate the polycurve based on our control points and external extend/trim functions
           
            //for (int i = 0; i < crvs.Count; i++)
            //{
            //    temp = crvs[i].TrimExtend(interPts[i], interPts[i + 1], extend);
            //    foreach (ICurve crv in temp)
            //    {
            //        resultList.Add(crv);
            //    }
            //}
            return new PolyCurve
            {
                Curves = resultList
            };
        }
    }
}