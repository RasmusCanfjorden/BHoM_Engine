﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Engine_Explore.BHoM.Geometry
{
    public class NurbCurve : BHoMGeometry, ICurve
    {
        /***************************************************/
        /**** Properties                                ****/
        /***************************************************/

        public List<Point> ControlPoints { get; set; } = new List<Point>();

        public List<double> Weights { get; set; } = new List<double>();

        public List<double> Knots { get; set; } = new List<double>();


        /***************************************************/
        /**** Constructors                              ****/
        /***************************************************/

        public NurbCurve() { }

        /***************************************************/

        public NurbCurve(IEnumerable<Point> controlPoints, int degree = 3)
        {
            int n = controlPoints.Count();
            int d = degree - 1;
            ControlPoints = controlPoints.ToList();
            Weights = Enumerable.Repeat(1.0, n).ToList();
            Knots = Enumerable.Repeat(0, d).Concat(Enumerable.Range(0, n - d).Concat(Enumerable.Repeat(n - d - 1, d))).Select(x => (double)x).ToList();
        }

        /***************************************************/

        public NurbCurve(IEnumerable<Point> controlPoints, IEnumerable<double> weights, int degree = 3)
        {
            int n = controlPoints.Count();
            int d = degree - 1;
            ControlPoints = controlPoints.ToList();
            Weights = weights.ToList();
            Knots = Enumerable.Repeat(0, d).Concat(Enumerable.Range(0, n - d).Concat(Enumerable.Repeat(n - d - 1, d))).Select(x => (double)x).ToList();
        }

        /***************************************************/

        public NurbCurve(IEnumerable<Point> controlPoints, IEnumerable<double> weights, IEnumerable<double> knots)
        {
            ControlPoints = controlPoints.ToList();
            Weights = weights.ToList();
            Knots = knots.ToList();
        }


        /***************************************************/
        /**** Local Methods                             ****/
        /***************************************************/

        public int GetDegree()
        {
            return 1 + Knots.Count - ControlPoints.Count;
        }
    }
}