﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BH.oM.Base;
using BH.oM.Structural.Elements;
using BHS = BH.oM.Structural.Design;
using BH.oM.Geometry;

namespace BH.oM.Structural
{
    public static class StructuralLayout
    {
        /// <summary>
        /// Joins a list of connected bars with the same cross-section and outputs the result as design elements
        /// </summary>
        /// <param name="bars"></param>
        /// <returns></returns>
        public static List<BHS.StructuralLayout> CreateFromConnectedBars(List<Bar> bars, double tolerance = 0.01)
        {
            List<BHS.StructuralLayout> results = new List<BHS.StructuralLayout>();
            Dictionary<Guid, int> nodeCount = new Dictionary<Guid, int>();
            int count = 0;
            for (int i = 0; i < bars.Count; i++)
            {
                results.Add(new BHS.StructuralLayout(bars[i]));
                if (nodeCount.TryGetValue(bars[i].StartNode.BHoM_Guid, out count))
                {
                    nodeCount[bars[i].StartNode.BHoM_Guid] = count + 1;
                }
                else
                {
                    nodeCount.Add(bars[i].StartNode.BHoM_Guid, 1);
                }
                if (nodeCount.TryGetValue(bars[i].EndNode.BHoM_Guid, out count))
                {
                    nodeCount[bars[i].EndNode.BHoM_Guid] = count + 1;
                }
                else
                {
                    nodeCount.Add(bars[i].EndNode.BHoM_Guid, 1);
                }
            }

            int counter = 0;
            while (counter < results.Count)
            {
                double[] ps1 = results[counter].StartPoint;
                double[] pe1 = results[counter].EndPoint;
                for (int j = counter + 1; j < results.Count; j++)
                {
                    if (results[counter].SectionProperty.Name == results[j].SectionProperty.Name)
                    {
                        double[] ps2 = results[j].StartPoint;
                        double[] pe2 = results[j].EndPoint;
                        if (ArrayUtils.Equal(pe1, ps2, tolerance) && nodeCount[results[counter].EndNode.BHoM_Guid] == 2)
                        {
                            results[j].AddBars(results[counter].AnalyticBars);
                            results.RemoveAt(counter--);
                            break;
                        }
                        else if (ArrayUtils.Equal(pe1, pe2, tolerance) && nodeCount[results[counter].EndNode.BHoM_Guid] == 2)
                        {
                            results[j].AddBars(results[counter].AnalyticBars);
                            results.RemoveAt(counter--);
                            break;
                        }
                        else if (ArrayUtils.Equal(ps1, ps2, tolerance) && nodeCount[results[counter].StartNode.BHoM_Guid] == 2)
                        {
                            results[j].AddBars(results[counter].AnalyticBars);
                            results.RemoveAt(counter--);
                            break;
                        }
                        else if (ArrayUtils.Equal(ps1, pe2, tolerance) && nodeCount[results[counter].StartNode.BHoM_Guid] == 2)
                        {
                            results[j].AddBars(results[counter].AnalyticBars);
                            results.RemoveAt(counter--);
                            break;
                        }
                    }

                }
                counter++;
            }

            for (int i = 0; i < results.Count; i++) results[i].GenerateDefaultSpans();
            return results;
        }

    }
}
