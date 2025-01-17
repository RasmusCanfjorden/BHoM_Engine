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
using System.Collections.Generic;
using System.ComponentModel;
using BH.oM.Reflection.Attributes;
using BH.oM.Physical.Elements;
using BH.oM.Physical.Constructions;
using BH.oM.Geometry;

namespace BH.Engine.Physical
{
    public static partial class Create
    {
        /***************************************************/
        /**** Public Methods                            ****/
        /***************************************************/

        [Description("Creates a physical floor element. For elements for structral analytical applications look at BH.oM.Structure.Elements.Panel. For elements for environmental analytical applications look at BH.oM.Environments.Elements.Panel")]
        [Input("location", "Location surface which represents the outer geometry of the floor. Should not contain any openings")]
        [Input("construction", "Construction representing the thickness and materiality of the floor")]
        [Input("openings", "Openings of the floor. Could be simple voids or more detailed obejcts")]
        [Input("offset", "Represents the positioning of the construction in relation to the location surface of the floor")]
        [Input("name", "The name of the floor, default empty string")]
        [Output("Floor", "The created physical floor")]
        public static Floor Floor(oM.Geometry.ISurface location, IConstruction construction, List<IOpening> openings = null, Offset offset = Offset.Undefined, string name = "")
        {
            openings = openings ?? new List<IOpening>();

            return new Floor
            {
                Location = location,
                Construction = construction,
                Openings = openings,
                Offset = offset,
                Name = name
            };
        }

        /***************************************************/

        [Description("Creates a physical Wall element. For elements for structral analytical applications look at BH.oM.Structure.Elements.Panel. For elements for environmental analytical applications look at BH.oM.Environments.Elements.Panel")]
        [Input("location", "Location surface which represents the outer geometry of the Wall. Should not contain any openings")]
        [Input("construction", "Construction representing the thickness and materiality of the Wall")]
        [Input("openings", "Openings of the Wall. Could be simple voids or more detailed obejcts")]
        [Input("offset", "Represents the positioning of the construction in relation to the location surface of the Wall")]
        [Input("name", "The name of the wall, default empty string")]
        [Output("Wall", "The created physical Wall")]
        public static Wall Wall(oM.Geometry.ISurface location, IConstruction construction, List<IOpening> openings = null, Offset offset = Offset.Undefined, string name = "")
        {
            openings = openings ?? new List<IOpening>();

            return new Wall
            {
                Location = location,
                Construction = construction,
                Openings = openings,
                Offset = offset,
                Name = name
            };
        }

        /***************************************************/

        [Description("Creates a physical Wall element. For elements for structral analytical applications look at BH.oM.Structure.Elements.Panel. For elements for environmental analytical applications look at BH.oM.Environments.Elements.Panel")]
        [Input("line", "Base line of the wall")]
        [Input("height", "Height of the wall")]
        [Input("construction", "Construction representing the thickness and materiality of the Wall")]
        [Input("openings", "Openings of the Wall. Could be simple voids or more detailed obejcts")]
        [Input("offset", "Represents the positioning of the construction in relation to the location surface of the Wall")]
        [Input("name", "The name of the wall, default empty string")]
        [Output("Wall", "The created physical Wall")]
        public static Wall Wall(Line line, double height, IConstruction construction, Offset offset = Offset.Undefined, string name = "")
        {
            Polyline boundary = new Polyline();

            Vector move = Vector.ZAxis * height;

            boundary.ControlPoints.Add(line.Start);
            boundary.ControlPoints.Add(line.End);
            boundary.ControlPoints.Add(line.End + move);
            boundary.ControlPoints.Add(line.Start + move);
            boundary.ControlPoints.Add(line.Start);

            return new Wall
            {
                Location = Geometry.Create.PlanarSurface(boundary),
                Construction = construction,
                Offset = offset,
                Name = name
            };
        }

        /***************************************************/

        [Description("Creates a physical Roof element. For elements for structral analytical applications look at BH.oM.Structure.Elements.Panel. For elements for environmental analytical applications look at BH.oM.Environments.Elements.Panel")]
        [Input("location", "Location surface which represents the outer geometry of the Roof. Should not contain any openings")]
        [Input("construction", "Construction representing the thickness and materiality of the Roof")]
        [Input("openings", "Openings of the Roof. Could be simple voids or more detailed obejcts")]
        [Input("offset", "Represents the positioning of the construction in relation to the location surface of the Roof")]
        [Input("name", "The name of the roof, default empty string")]
        [Output("Roof", "The created physical Roof")]
        public static Roof Roof(oM.Geometry.ISurface location, IConstruction construction, List<IOpening> openings = null, Offset offset = Offset.Undefined, string name = "")
        {
            openings = openings ?? new List<IOpening>();

            return new Roof
            {
                Location = location,
                Construction = construction,
                Openings = openings,
                Offset = offset,
                Name = name
            };
        }

        /***************************************************/
    }
}
