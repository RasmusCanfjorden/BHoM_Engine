﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using BH.oM.Base;

namespace BH.Engine.Base
{
    public class BHoMGuidComparer : IEqualityComparer<BHoMObject>
    {
        public bool Equals(BHoMObject x, BHoMObject y)
        {
            //Check if the GUIDs are the same
            return (x.BHoM_Guid == y.BHoM_Guid);
        }

        public int GetHashCode(BHoMObject obj)
        {
            //Check whether the object is null
            if (Object.ReferenceEquals(obj, null)) return 0;

            return obj.BHoM_Guid.GetHashCode();
        }
    }
}
