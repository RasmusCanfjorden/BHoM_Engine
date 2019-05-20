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

using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using BH.oM.Reflection.Attributes;
using BH.oM.Base;

namespace BH.Engine.Reflection
{ 
    public static partial class Query
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        [Description("Return the default description of a C# class")]
        public static string DefaultDescription(this Type type)
        {
            if (type == null)
                return "";

            string desc = "This is a " + type.ToText();

            Type innerType = type;

            while (typeof(IEnumerable).IsAssignableFrom(innerType) && innerType.IsGenericType)
                innerType = innerType.GenericTypeArguments.First();

            if (innerType.IsInterface)
            {
                desc += ":";
                List<Type> t = innerType.ImplementingTypes();
                int m = Math.Min(15, t.Count);

                for (int i = 0; i < m; i++)
                    desc += $"{t[i].ToText()}, ";

                if (t.Count > m)
                    desc += "and more...";
                else
                    desc = desc.Remove(desc.Length - 2, 2);

                return desc;
            }

            return desc;
        }

        /***************************************************/
    }
}