﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace BH.Engine.Reflection
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        public static object RunExtentionMethod(object target, string methodName)
        {
            object result = null;
            Type type = target.GetType();

            // If the method has been called before, just use that
            Tuple < Type, string> key = new Tuple<Type, string>(type, methodName);
            if (m_PreviousInvokedMethods.ContainsKey(key))
                return m_PreviousInvokedMethods[key].Invoke(null, new object[] { target });

            // Otherwise, search for the method and call it if found
            foreach (MethodInfo method in type.ExtentionMethods(methodName))
            {
                if (method.GetParameters().Length == 1)
                    return method.Invoke(null, new object[] { target });
            }

            // Return null if nothing found
            return result;
        }

        /***************************************************/

        public static object RunExtentionMethod(object target, string methodName, object[] parameters)
        {
            object result = null;

            foreach (MethodInfo method in target.GetType().ExtentionMethods(methodName))
            {
                // Make sure the number of parameters is matching
                ParameterInfo[] paramInfo = method.GetParameters();
                if (paramInfo.Length != parameters.Length + 1)
                    continue;

                // Make sure the type of parameters is matching
                bool matchingTypes = true;
                for (int i = 0; i < parameters.Length; i++)
                {
                    if (!paramInfo[i + 1].ParameterType.IsAssignableFrom(parameters[i].GetType()))
                    {
                        matchingTypes = false;
                        break;
                    }
                }
                if (!matchingTypes)
                    continue;

                return method.Invoke(null, new object[] { target }.Concat(parameters).ToArray());
            }

            return result;
        }


        /***************************************************/
        /**** Private fields                            ****/
        /***************************************************/

        private static Dictionary<Tuple<Type, string>, MethodInfo> m_PreviousInvokedMethods = new Dictionary<Tuple<Type, string>, MethodInfo>();


        /***************************************************/
    }
}