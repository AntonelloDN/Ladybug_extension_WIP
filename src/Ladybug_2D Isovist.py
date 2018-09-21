# 2D Isovist
#
# Ladybug: A Plugin for Environmental Analysis (GPL) started by Mostapha Sadeghipour Roudsari
# 
# This file is part of Ladybug.
# 
# Copyright (c) 2013-2018, Antonello Di Nunzio <antonellodinunzio@gmail.com> 
# Ladybug is free software; you can redistribute it and/or modify 
# it under the terms of the GNU General Public License as published 
# by the Free Software Foundation; either version 3 of the License, 
# or (at your option) any later version. 
# 
# Ladybug is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with Ladybug; If not, see <http://www.gnu.org/licenses/>.
# 
# @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>


"""
Use this component to see the area visible from a given viewpoint and direction across a horizontal plane of vision. This component works with indoor spaces and outdoor spaces.
-
Provided by Ladybug 0.0.63
    
    Args:
        _context: Breps representing context geometry that can block the view around a given viewPoint.
        _viewPoint_: The point of vision in which to generate the view rose.
        _viewDirection_: A vector that represents the view direction.
        distanceLimits_: Set the limit of the view. If no value is supplied it will be 200 meters.
        analysisStep_: A number that represent the minimun angle between two view rays. The default value is 0.5.
        viewAngle_: Set the field of view by connecting a number. If value is not supplied it will be 62 which is the human field of view.
        -
        This input accept integer numbers or the following texts:
        25mph = it lets you analyze the field of view from a car at 25 mph.
        45mph = it lets you analyze the field of view from a car at 45 mph.
        65mph = it lets you analyze the field of view from a car at 65 mph.
        color = color recognition limit.
        sign = sign recognition limit.
        word = word recognition limit.
        _runIt: Set to "True" to run the component and generate the viewRose. 
    Returns:
        readMe!: ...
        fixationDist : The minimum distance which the human eye can see from a car.
        visibleAngle : The total angle of visibility from the viewpoint.
        viewRose: A surface representing the visible area from the viewpoint past the _context geometry.
"""

ghenv.Component.Name = "Ladybug_2D Isovist"
ghenv.Component.NickName = '2DIsovist'
ghenv.Component.Message = 'VER 0.0.63\nSEP_30_2016'
ghenv.Component.IconDisplayMode = ghenv.Component.IconDisplayMode.application
ghenv.Component.Category = "Ladybug"
ghenv.Component.SubCategory = "7 | WIP"
#compatibleLBVersion = VER 0.0.62\nJUN_07_2016
try: ghenv.Component.AdditionalHelpFromDocStrings = "0"
except: pass


import Rhino as rc
from math import radians, cos, sin
import scriptcontext as sc


class HumanEye:
    
    def __init__(self, viewPoint, viewDirection, analysisStep, distanceLimits, viewAngle_H):
        self.viewPoint = viewPoint
        self.viewDirection = viewDirection
        self.distanceLimits = distanceLimits
        self.viewAngle_H = viewAngle_H
        self.analysisStep = analysisStep
    
    
    # gimel from http://stackoverflow.com/
    def drange(self, start, stop, step):
         r = start
         while r < stop:
         	yield r
         	r += step
    
    
    def coneOfView(self):
        
        """ Use this method to generate visual lines. """
        angles = []
        lines = []
        points = []
        
        xrotation = rc.Geometry.Transform.Rotation(rc.Geometry.Plane(self.viewPoint, rc.Geometry.Vector3d.YAxis).Normal, self.viewDirection, self.viewPoint)
        
        H = self.drange((90 - self.viewAngle_H), (90 + self.viewAngle_H)+1 , self.analysisStep)
        H_values = [x for x in H]
        
        for i in H_values:
            x = self.distanceLimits * cos(radians(i)) + viewPoint.X
            y = self.distanceLimits * sin(radians(i)) + viewPoint.Y
            z = viewPoint.Z
            point = rc.Geometry.Point3d(x, y, z)
            baseLine = rc.Geometry.Line(self.viewPoint, point)
            baseLine.Transform(xrotation)
            lines.append(baseLine)
            points.append(point)
            angles.append(90-i)
        
        return lines, angles


def carAnalysis(distValue, angle, minDist):
    distanceLimits = distValue
    viewAngle_H, distanceLimits = angle, distanceLimits
    fixationDist = rc.Geometry.Circle(viewPoint, minDist) # minimumFixationDistance
    
    return fixationDist, viewAngle_H, distanceLimits


def generateSrf(points):
    points.append(viewPoint)
    curve = rc.Geometry.PolylineCurve(points)
    srf = rc.Geometry.Brep.CreatePlanarBreps(curve)
    return srf


if _viewPoint_ == None: viewPoint = rc.Geometry.Point3d.Origin
else: viewPoint = _viewPoint_
if _viewDirection_ == None: viewDirection = rc.Geometry.Vector3d.YAxis
else: viewDirection = _viewDirection_
if distanceLimits_ == None: distanceLimits = 200
else: distanceLimits = distanceLimits_
if analysisStep_ == None: analysisStep = 1
else: analysisStep = analysisStep_


if viewAngle_ != None:
    if viewAngle_ == 'color': viewAngle_H = 30
    elif viewAngle_ == 'sign': viewAngle_H = 20
    elif viewAngle_ == 'word': viewAngle_H = 10
    elif viewAngle_ == '25mph':
        fixationDist, viewAngle_H, distanceLimits = carAnalysis(182.88, 50, 15.24) # meters, , meters
    elif viewAngle_ == '45mph':
        fixationDist, viewAngle_H, distanceLimits = carAnalysis(365.76, 33, 25.908) # meters, , meters
    elif viewAngle_ == '65mph':
        fixationDist, viewAngle_H, distanceLimits = carAnalysis(609.6, 20, 35.052) # meters, , meters
    elif viewAngle_.isdigit():
        viewAngle_H = int(viewAngle_)
else: viewAngle_H = 62


UntrimmedParts = []
for geo in _context:
    brepFaces = geo.Faces
    for f in brepFaces:
        if not f.IsSurface:
            mesh = rc.Geometry.Mesh.CreateFromBrep(f.DuplicateFace(False))[0]
            UntrimmedParts.append(rc.Geometry.Brep.CreateFromMesh(mesh, False))
        else: UntrimmedParts.append(f.DuplicateFace(False))


# mod display
if _context:
    if _runIt == True:
        # mod human eye
        eye = HumanEye(viewPoint, viewDirection, analysisStep, distanceLimits, viewAngle_H)
        lines, angles = eye.coneOfView()
        
        points = [viewPoint]
        lenPoints = []
        number = []
        for i, line in enumerate(lines):
            ray = rc.Geometry.Ray3d(viewPoint, line.Direction)
            intersection = rc.Geometry.Intersect.Intersection.RayShoot(ray, UntrimmedParts, 1)
            if intersection and viewPoint.DistanceTo(intersection[0]) <= distanceLimits:
                points.extend(intersection)
            else: 
                points.append(line.PointAt(1))
                number.append('v') # visible angle
        
        visibleAngle = analysisStep * (len(number)-(analysisStep))
        if visibleAngle == -(analysisStep)**2: visibleAngle = 0.0 # obstructed
        
        viewRose = generateSrf(points)
else: print("the _context geometry or the _radius is missing!")