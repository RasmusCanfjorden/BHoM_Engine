﻿using BH.oM.Reflection.Attributes;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Reflection;

namespace BH.Engine.Reflection
{
    public static partial class Query
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        [Description("Return true if a C# method has multiple outputs")]
        public static bool IsMultipleOutputs(this MethodBase method)
        {
            return (method is MethodInfo) && (typeof(oM.Reflection.Interface.IOutput).IsAssignableFrom(((MethodInfo)method).ReturnType));
        }

        /***************************************************/
    }
}
