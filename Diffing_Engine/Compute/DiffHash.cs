﻿/*
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

using BH.oM.Base;
using BH.oM.Data.Collections;
using BH.oM.Diffing;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Security.Cryptography;
using System.Reflection;
using BH.Engine.Serialiser;
using BH.oM.Reflection.Attributes;
using System.ComponentModel;

namespace BH.Engine.Diffing
{
    public static partial class Compute
    {
        ///***************************************************/
        ///**** Public Fields                             ****/
        ///***************************************************/


        ///***************************************************/
        ///**** Public Methods                            ****/
        ///***************************************************/

        public static List<IBHoMObject> DiffHash(List<IBHoMObject> objs, List<string> exceptions = null, bool useDefaultExceptions = true, Stream stream = null)
        {
            // Clone the current objects
            List<IBHoMObject> objs_cloned = objs.Select(obj => BH.Engine.Base.Query.DeepClone(obj)).ToList();

            // Calculate and set the object hashes
            objs_cloned.ForEach(obj => obj.DiffHash(exceptions, useDefaultExceptions, stream));

            return objs_cloned;
        }

        public static string DiffHash(this IBHoMObject obj, List<string> exceptions = null, bool useDefaultExceptions = true, Stream stream = null)
        {
            if (exceptions == null || exceptions.Count == 0 && useDefaultExceptions)
                SetDefaultExceptions(ref exceptions);

            string hash = Compute.SHA256Hash(obj, exceptions);

            obj.Fragments.Add(
                new DiffHashFragment(hash, null, stream)
                );

            return hash;
        }


        public static void SetDefaultExceptions(ref List<string> exceptions)
        {
            if (exceptions == null)
                exceptions = defaultHashExceptions;
            else
                exceptions.AddRange(defaultHashExceptions);
        }

        ///***************************************************/
        ///**** Private Methods                           ****/
        ///***************************************************/



    }
}