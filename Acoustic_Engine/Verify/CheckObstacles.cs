﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Alea;
using Alea.Parallel;
using BH.oM.Base;
using BH.oM.Geometry;
using BH.oM.Acoustic;
using BH.Engine.Geometry;

namespace BH.Engine.Acoustic
{
    public static partial class Verify
    {
        public static bool CheckObstacles(Ray ray, List<Panel> surfaces, bool ClearRays = true, double tol = 0.00001)
        {
            for (int i = 0; i < surfaces.Count; i++)       //foreach surface
            {
                if (Geometry.Query.GetIntersections((Line)ray.Path, surfaces[i].Mesh) == null)    // if ray hits a surface
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Checks for obstacles in rays path. 
        /// </summary>
        /// <param name="rays">Acoustic rays list</param>
        /// <param name="surfaces">Acoustic surface list</param>
        /// <param name="ClearRays">Set true to output clear rays, false if blind rays.</param>
        /// <param name="tol">Default tolerance to 0.00001. Beware of global unit setup.</param>
        /// <returns>Returns a new list of filtered rays, either clear or blind ones.</returns>
        public static List<Ray> CheckObstacles(List<Ray> rays, List<Panel> surfaces, bool ClearRays = true, double tol = 0.00001)
        {
            List<Ray> filteredRays = new List<Ray>();
            for (int i = 0; i<rays.Count; i++)                   //foreach ray
            {
                List<bool> checker = new List<bool>();
                for (int j = 0; j < surfaces.Count; j++ )       //foreach surface
                {
                    if (Geometry.Query.GetIntersections((Line)(rays[i].Path), surfaces[i].Mesh) == null)       // if ray hits a surface
                        checker.Add(true);
                }
                if (ClearRays && checker.Any())     //if rays hits any surface and output is ClearRays
                    filteredRays.Add(rays[i]);
                else if (!ClearRays && !checker.Any())
                    filteredRays.Add(rays[i]);
            }
            return filteredRays;
        }



        /// <summary>
        /// GPU Accelerated obstacle test
        /// </summary>
        /// <param name="rays"></param>
        /// <param name="surfaces"></param>
        /// <param name="ClearRays"></param>
        /// <param name="tol"></param>
        /// <returns></returns>
        public static List<Ray> CheckObstaclesCUDA(List<Ray> rays, List<Panel> surfaces, bool ClearRays = true, double tol = 0.00001)
        {
            List<Ray> filteredRays = new List<Ray>();

            Gpu.Default.For(0, rays.Count,
                i =>
                {
                    List<bool> checker = new List<bool>();
                    for (int j = 0; j < surfaces.Count; j++)       //foreach surface
                    {
                        if (Geometry.Query.GetIntersections((Line)(rays[i].Path), surfaces[i].Mesh) == null)       // if ray hits a surface
                            checker.Add(true);
                    }
                    if (ClearRays && checker.Any())     //if rays hits any surface and output is ClearRays
                        filteredRays.Add(rays[i]);
                    else if (!ClearRays && !checker.Any())
                        filteredRays.Add(rays[i]);
                });
            return filteredRays;
        }

        /// <summary>
        /// CPU Accelerated obstacle test
        /// </summary>
        /// <param name="rays"></param>
        /// <param name="surfaces"></param>
        /// <param name="ClearRays"></param>
        /// <param name="tol"></param>
        /// <returns></returns>
        public static List<Ray> CheckObstaclesCPU(List<Ray> rays, List<Panel> surfaces, bool ClearRays = true, double tol = 0.00001)    // NOT EFFICIENT for now, since it locks the rays list to be thread safe
        {
            List<Ray> filteredRays = new List<Ray>();

            Parallel.For(0, rays.Count,
                () => new List<Ray>(),
                (int i, ParallelLoopState loop, List<Ray> localRay) =>
                {
                    List<bool> localCheck = new List<bool>();
                    for (int j = 0; j < surfaces.Count; j++)       //foreach surface
                    {
                        if (Geometry.Query.GetIntersections((Line)(rays[i].Path), surfaces[i].Mesh) == null)
                            localCheck.Add(true);
                    }
                    if (ClearRays && localCheck.Any())     //if rays hits any surface and output is ClearRays
                        localRay.Add(rays[i]);
                    else if (!ClearRays && !localCheck.Any())
                        localRay.Add(rays[i]);
                    return localRay;
                },
                (localRay) =>
                {
                    lock (rays)
                    {
                        filteredRays.AddRange(localRay);
                    }
                    
                });
            return filteredRays;
        }


        /// <summary>
        /// Filter ray 
        /// </summary>
        /// <param name="rays"></param>
        /// <param name="filter"></param>
        /// <returns></returns>
        public static List<Ray> RayFilter(List<Ray> rays, List<int> sourceFilter)
        {
            //IEnumerable<Ray> Rays = rays;
            //return Rays.Where(ray => filter.Contains(ray.SpeakerID ));
            return rays.FindAll(ray => sourceFilter.Contains(ray.SpeakerID));
        }
    }
}