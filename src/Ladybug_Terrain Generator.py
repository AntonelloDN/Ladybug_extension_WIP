# Terrain Generator
#
# Ladybug: A Plugin for Environmental Analysis (GPL) started by Mostapha Sadeghipour Roudsari
# 
# This file is part of Ladybug.
# 
# Copyright (c) 2013-2018, Antonello Di Nunzio <antonellodinunzio@gmail.com>, Mathias Sonderskov Nielsen (mapquest API idea)
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
This component uses Google Maps API or mapquest API to generate terrain models. It works with Rhino 6.
-
This component requires an internet connection and if you use Google API you need an API key from Google.
"Activate the API and get an API key.
To use the Elevation API, you must first activate the API in the Google Cloud Platform Console and obtain the proper authentication credentials. You need to provide an API key in each request (or a client ID if you have a Premium Plan).
Click the button below to flow through a process where you will:
    1.Create or select a project
    2.Enable the API
    3.Get an API key"
ref: https://developers.google.com/maps/documentation/elevation/start
-
If you use mapquest API you need an API key.
Go to this website to obtain an API key of mapquest: https://developer.mapquest.com/
_______________________________________________________________________________________________
IMPORTANT!
"New Pay-As-You-Go Pricing
Effective July 16, 2018
How billing works under the new pay-as-you-go model=
    1.The Google Maps Platform APIs are billed by SKU.
    2.Usage is tracked for each Product SKU, and an API may have more than one Product SKU.
    3.Cost is calculated by: SKU Usage x Price per each use.
    4.For each billing account, for qualifying Google Maps Platform SKUs, a $200 USD Google Maps Platform credit is available each month, and automatically applied to the qualifying SKUs."
ref: https://developers.google.com/maps/documentation/elevation/usage-and-billing
-
Note that each surface is a request, for example if you use a surface made by sub-surfaces 6x6, this will be 36 requests.
-
Special thanks goes to Google Maps.
-
Provided by Ladybug 0.0.66
    
    Args:
        source_: Connect 0 if you want to use Google API otherwise will be used mapquest API.
        APIKEY_: This is necessary for API Google. Connect a panel with your API Key from Google or mapquest. Remember that it is a personal key.
        _location: It accepts two type of inputs.
        a) latitude, longitude and elevation that represent WSG84 coordinates of the base point. You can achieve these type of coordinates from Google Maps or similar.
        e.g. 40.821796, 14.426439, 990
        -
        b) location, you can obtain this type of input from "Ladybug_Construct Location", "Ladybug_Location Finder", "Ladybug_Import epw", "Ladybug_Import Location".
        _radius_: A radius to make the terrain 3D model in Rhino model units. The default is set to 100.
        -
        If you provide a big radius, this could require lots of time (also a couple of minutes).
        _basePoint_: Input a point here to georeference the terrain model.
        type_: Select the type of output:
        0 = rectangular mesh
        1 = rectangular surface
        -
        The default value is 0.
        _numOfTiles_: Set the number of tiles (e.g. 4, that means 4x4). If no input is connected this will be 3 (tiles: 3x3).
        _numDivision_: Set the number of points for each tile. If no input is connected this will be 8 (I suggest max 10).
        _imgResolution_: Connect an integer number which manage the quality of single satellite image.
        -
        The following list shows the approximate level of detail you can expect to see at each _imgResolution_ level:
        1 = World
        5 = Landmass/continent
        10 = City
        15 = Streets
        20 = Buildings
        -
        The default value is 18.
        mapType_: Connect an integer number, from 0 to 3, which manages the formats of map.
        -
        0 = GOOGLE Satellite (default)
        1 = GOOGLE Roadmap, specifies a standard roadmap image
        2 = GOOGLE Terrain, it shows terrain and vegetation 
        3 = GOOGLE Hybrid, it specifies a hybrid of the satellite and roadmap image
        4 = MAPQUEST map
        5 = MAPQUEST hyb
        6 = MAPQUEST sat
        7 = MAPQUEST light
        8 = MAPQUEST dark
        folder_: The folder into which you would like to write the image file. This should be a complete file path to the folder.  If no folder is provided, the images will be written to C:/USERNAME/AppData/Roaming/Ladybug/IMG_Google.
        _runIt: Set to "True" to run the component and generate the 3D terrain model.
        texture_: Set to "True" to enable Google Static Maps as well. Keep in mind the daily quota of requests. Default value is False.
        sleepFactor_: pause in seconds between each iteration, default value is 0.05 s.
    Returns:
        readMe!: ...
        pointsGeo : WSG84 coordinates of the grid points (latitude,longitude).
        pointsXY : Cartesian coordinates of the grid points (X, Y).
        pointsZ : Z values of the grid points. Connect this output to a Z vector to move the pointsXY in the right positions.
        tiles: The area which will be calculated. If you want to visualize Satellite images connect this output to 'G' input of 'Human Custom Preview Material'.
        imagePath: Satellite images from Google Static Maps API. Connect it to 'DB' input of 'Human Custom Preview Material' to apply textures to the 3d model or to the list of input surfaces.
        -----------: ...
        terrain: 3D terrain model.
        origin: The origin (center) point of the "terrain" geometry.
        elevation: Elevation of the origin_ input.
"""

ghenv.Component.Name = "Ladybug_Terrain Generator"
ghenv.Component.NickName = 'TerrainGenerator'
ghenv.Component.Message = 'VER 0.0.66\nMAR_03_2018'
ghenv.Component.IconDisplayMode = ghenv.Component.IconDisplayMode.application
ghenv.Component.Category = "Ladybug"
ghenv.Component.SubCategory = "7 | WIP"
#compatibleLBVersion = VER 0.0.62\nJUN_07_2016
try: ghenv.Component.AdditionalHelpFromDocStrings = "0"
except: pass

import Rhino
import scriptcontext as sc
import socket
import System
System.Net.ServicePointManager.SecurityProtocol = System.Net.SecurityProtocolType.Tls12
import os
import clr
import urllib
import urllib2
import json
import time
from math import pi, log, tan, atan, exp, sqrt
clr.AddReference("Grasshopper")
import Grasshopper.Kernel as gh
from Grasshopper.Kernel.Data import GH_Path
from Grasshopper import DataTree

def mdPath(folder):
    # make a folder for the images
    if folder != None:
        directory = os.path.join(folder, "IMG_Google\\")
        if not os.path.exists(folder):
            try:
                os.mkdir(directory)
            except Exception:
                appdata = os.getenv("APPDATA")
                try:
                    directory = os.path.join(appdata, "Ladybug\IMG_Google\\")
                except:
                    directory = os.path.join(appdata[:3], "Ladybug\IMG_Google\\")
                if not os.path.exists(directory):
                    os.makedirs(directory)
                w = gh.GH_RuntimeMessageLevel.Warning
                ghenv.Component.AddRuntimeMessage(w, "Invalid Folder, you can find images here: {}".format(directory))
    else:
        appdata = os.getenv("APPDATA")
        try:
            directory = os.path.join(appdata, "Ladybug\IMG_Google\\")
        except:
            directory = os.path.join(appdata[:3], "Ladybug\IMG_Google\\")
        if not os.path.exists(directory):
            os.makedirs(directory)

    return directory

class MapSettings ( object ):

    # init
    def __init__( self ):
        self.basePoint = Rhino.Geometry.Point3d.Origin
        self.numDivision = 8
        self.radius = 200
        self.numOfTiles = 3
        self.type = 0
        # location or point3d
        try:
            latitude, longitude, elevation = eval( _location )
            self.location = Rhino.Geometry.Point3d( latitude, longitude, elevation )
        except:
            locationName, latitude, longitude, timeZone, elevation = lb_preparation.decomposeLocation( _location )
            self.location = Rhino.Geometry.Point3d( latitude, longitude, elevation )
        # base point
        base_point = Rhino.DocObjects.EarthAnchorPoint( )
        base_point.EarthBasepointLatitude = self.location.X
        base_point.EarthBasepointLongitude = self.location.Y
        base_point.EarthBasepointElevation = self.location.Z
        base_point.ModelEast = Rhino.Geometry.Vector3d.XAxis
        base_point.ModelNorth = Rhino.Geometry.Vector3d.YAxis
        base_point.ModelBasePoint = self.basePoint
        modelUnit = Rhino.UnitSystem( )
        self.xf = base_point.GetModelToEarthTransform( modelUnit )
        # make sure that basePoint is on the terrain
        self.factor = 1 if elevation >= 0 else -1
    # set tiles
    def setTiles( self, radius ):
        interval_radius = Rhino.Geometry.Interval( -radius, radius )
        rectangle_tiles = Rhino.Geometry.Rectangle3d(Rhino.Geometry.Plane( self.basePoint, Rhino.Geometry.Vector3d.ZAxis ), interval_radius, interval_radius ).ToNurbsCurve( )
        brep_tiles = Rhino.Geometry.Brep.CreatePlanarBreps( rectangle_tiles )[0]
        surface_tiles = brep_tiles.Surfaces[0]
        surface_tiles.SetDomain( 0, Rhino.Geometry.Interval( 0, 1 ) )
        surface_tiles.SetDomain( 1, Rhino.Geometry.Interval( 0, 1 ) )
        edge = brep_tiles.DuplicateEdgeCurves( True )[0]
        t = edge.DivideByCount( self.numOfTiles, True )
        intervals = []
        for i in range( 0, self.numOfTiles ):
            part = Rhino.Geometry.Interval( t[i], t[i+1] )
            intervals.append( part )
        tiles = []
        for part in intervals:
            for part2 in intervals:
                tiles.append( surface_tiles.Trim( part, part2 ) )
        return tiles
    # pre calculation points
    def divideSrf( self, srf ):
        pts = []
        srf.SetDomain( 0, Rhino.Geometry.Interval( 0, 1 ) )
        srf.SetDomain(1, Rhino.Geometry.Interval( 0, 1 ) )
        centre = Rhino.Geometry.AreaMassProperties.Compute(srf).Centroid

        for i in range( 0, self.numDivision + 1 ):
            for j in range( 0, self.numDivision + 1 ):
                pts.append( srf.PointAt( i / self.numDivision, j / self.numDivision ) )
        return pts, centre

class GeoLib( object ):
    # class variables
    earth_radius = 6378137
    equator_circumference = 2 * pi * earth_radius
    initial_resolution = equator_circumference / 256.0
    origin_shift = equator_circumference / 2.0
    
    def __init__( self, pts, xf, zoom ):
        # gen 3d points
        self.pts = pts
        self.points = [xf * pt for pt in self.pts]
        self.list_latitude = [pt.Y for pt in self.pts]
        self.list_longitude = [pt.X for pt in self.pts]
        self.pointsZ = []
        self.zoom = zoom
        
    def elevationAPI(self):
        if (source_ == 0 or source_ == 1):
            list_coordinate = [str(pt.Y) + ',' + str(pt.X) + "|" for pt in self.points]
        else:
            list_coordinate = [str(pt.Y) + ',' + str(pt.X) + "," for pt in self.points]
        
        flat_list_coordinate = ''.join(list_coordinate)
        flat_list_coordinate = flat_list_coordinate[:len(flat_list_coordinate)-1]
        url_part2 = flat_list_coordinate
        
        if len(url_part2) < 4094:
            try:
                if (APIKEY_):
                    if (source_ == 0):
                        response = urllib2.urlopen('https://maps.googleapis.com/maps/api/elevation/json?locations=' + url_part2 + '&key=' + APIKEY_ )
                        jsonData = json.loads( response.read( ) )
                        print(jsonData)
                        elevations = [ jsonData['results'][i]['elevation'] for i in xrange( len(jsonData['results']) ) ]
                    else:
                        response = urllib2.urlopen('https://open.mapquestapi.com/elevation/v1/profile?shapeFormat=raw&latLngCollection=' + url_part2 + '&key=' + APIKEY_ )
                        jsonData = json.loads( response.read( ) )
                        print(jsonData)
                        elevations = [ jsonData['elevationProfile'][i]['height'] for i in xrange( len(jsonData['elevationProfile']) ) ]
                    for pt, el in zip ( self.pts, elevations ):
                        pt.Z = el / unitConversionFactor
                        pt.X = pt.X / unitConversionFactor
                        pt.Y = pt.Y / unitConversionFactor
                        self.pointsZ.append( pt )
                        
                else:
                    warning = 'Please connect an API key.'
                    ghenv.Component.AddRuntimeMessage(gh.GH_RuntimeMessageLevel.Warning, warning)
                    self.pointsZ = [-1]
                    good_to_go = False

            except urllib2.URLError, e:
                warning = 'Please check your internet connection.'
                ghenv.Component.AddRuntimeMessage(gh.GH_RuntimeMessageLevel.Warning, warning)
                print 'No data', e
                self.pointsZ = [-1]
        else:
            warning = 'Please reduce _numDivision_ or increase _numOfTiles_.'
            ghenv.Component.AddRuntimeMessage(gh.GH_RuntimeMessageLevel.Warning, warning)
            self.pointsZ = [-1]

    def findCorner(self, relation):
        relation_latitude = relation(self.list_latitude)
        indices_latitude_relation = {i for i, elem in enumerate(self.list_latitude) if elem == relation_latitude}

        relation_longitude = relation(self.list_longitude)
        indices_longitude_relation = {i for i, elem in enumerate(self.list_longitude) if elem == relation_longitude}

        intersection = indices_latitude_relation.intersection(indices_longitude_relation)
        value = list(intersection)[0]

        return value

    # This Mercator projection function is made by heltonbiker (http://stackoverflow.com/)
    def latlontopixels(self, lat, lon):
        mx = (lon * self.origin_shift) / 180.0
        my = log(tan((90 + lat) * pi/360.0))/(pi/180.0)
        my = (my * self.origin_shift) /180.0
        res = self.initial_resolution / (2**self.zoom)
        px = (mx + self.origin_shift) / res
        py = (my + self.origin_shift) / res
        return px, py

    def mapAPI(self, center, mapType):

        upperleft = self.points[self.findCorner(max)]
        lowerright = self.points[self.findCorner(min)]

        ullon, ullat = (upperleft[0], upperleft[1])
        lrlon, lrlat = (lowerright[0], lowerright[1])
        ulx, uly = self.latlontopixels(ullat, ullon)
        lrx, lry = self.latlontopixels(lrlat, lrlon)

        # calculate total pixel dimensions
        dx, dy = ulx - lrx, uly - lry

        if dx > 640 or dy > 640:
            warning = 'Your request is too big. Change image resolution or make smaller tiles.'
            ghenv.Component.AddRuntimeMessage(gh.GH_RuntimeMessageLevel.Warning, warning)
            return -1
        
        if (source_ == 0):
            urlPart1 = 'https://maps.googleapis.com/maps/api/staticmap?'
            urlPart2 = "center={0},%20{1}&zoom={2}&size={3}x{4}&maptype={5}".format(str(center.Y), str(center.X), str(self.zoom), str(int(dx)), str(int(dy)), str(mapType)) + '&key=' + APIKEY_
        else:
            urlPart1 = 'https://www.mapquestapi.com/staticmap/v5/map?'
            urlPart2 = "center={0},{1}&zoom={2}&size={3},{4}&type={5}".format(str(center.Y), str(center.X), str(self.zoom), str(int(dx)), str(int(dy)), str(mapType)) + '&key=' + APIKEY_
            
        goog_url = urlPart1 + urlPart2
        print(goog_url)

        return goog_url

class Terrain3D( object ):

    def __init__( self, pts, numOfTiles, numDivision, basePoint, factor):

        cull_pts = list( set(pts) )
        cull_pts.sort(key = lambda pt: pt.X)
        cull_pts.sort(key = lambda pt: pt.Y, reverse=True)
        self.cPts = cull_pts
        self.num = (numDivision + 1) * numOfTiles - (numOfTiles - 1)
        self.basePoint = basePoint
        self.factor = factor

    def makeMesh ( self ):
        lb_meshpreparation = sc.sticky["ladybug_Mesh"]()
        terrain = lb_meshpreparation.meshFromPoints(self.num, self.num, self.cPts)
        origin = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes([terrain], [self.basePoint], Rhino.Geometry.Vector3d.ZAxis * self.factor, sc.doc.ModelAbsoluteTolerance)

        return origin, terrain

    def makeSrf ( self ):
        uDegree = min(3, self.num - 1)
        vDegree = min(3, self.num - 1)
        terrain = Rhino.Geometry.NurbsSurface.CreateThroughPoints(self.cPts, self.num, self.num, uDegree, vDegree, False, False)
        origin = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps([terrain.ToBrep()], [self.basePoint], Rhino.Geometry.Vector3d.ZAxis * self.factor, sc.doc.ModelAbsoluteTolerance)

        return origin, terrain

def main (  ):
    
    mapsType = {'0':'satellite', '1':'roadmap', '2':'terrain', '3':'hybrid', '4':'map', '5':'hyb', '6':'sat', '7':'light', '8':'dark'}
    
    if folder_:
        folder = folder_
    else:
        folder = None
    directory = mdPath(folder)
    
    if sleepFactor_:
        sleepFactor = sleepFactor_
    else:
        sleepFactor = 0.05
    if texture_:
        texture = texture_
    else:
        texture = False
    if mapType_ == None and source_ == 0:
        mapType = mapsType['0']
    elif mapType_ == None:
        mapType = mapsType['6']
    else: mapType = mapsType[mapType_]
    if _imgResolution_ == None:
        imgResolution = 18
    else: imgResolution = int(_imgResolution_) # make sure that it is an integer number
    if not _location:
        print('Please connect a location.')
        return -1
    
    tilesTree, tilePoints, tilePointsGeo, imagePath = DataTree[System.Object](), DataTree[System.Object](), DataTree[System.Object](), DataTree[System.Object]()
    flatListPoints = []
    mappa = MapSettings( )
    if _basePoint_: mappa.basePoint = _basePoint_
    if _numDivision_: mappa.numDivision = _numDivision_
    if _radius_: mappa.radius = _radius_
    if _numOfTiles_: mappa.numOfTiles = _numOfTiles_
    
    # visualization
    visualizationTiles = mappa.setTiles( mappa.radius / unitConversionFactor )
    for i, tile in enumerate( visualizationTiles ):
        path = GH_Path( 0, i )
        tilesTree.Add( tile, path )
    
    if _runIt:
        calculationTiles = mappa.setTiles( mappa.radius )
        for i, tile in enumerate( calculationTiles ):
            points, centre = mappa.divideSrf( tile )
            geoLib = GeoLib(points, mappa.xf, imgResolution )
            geoLib.elevationAPI()
            if len(geoLib.pointsZ) != len(points):
                warning = 'Please try again, you lost some data.'
                ghenv.Component.AddRuntimeMessage(gh.GH_RuntimeMessageLevel.Warning, warning)
                return -1
            path = GH_Path(0, i)
            flatListPoints.extend( geoLib.pointsZ )
            
            tilePointsGeo.AddRange( [Rhino.Geometry.Point3d(pt.Y, pt.X, 0) for pt in geoLib.points], path )
            tilePoints.AddRange( geoLib.pointsZ, path )
            time.sleep( sleepFactor )
        tp = Terrain3D( flatListPoints, mappa.numOfTiles, mappa.numDivision, mappa.basePoint, mappa.factor )
        gt = tp.makeSrf( ) if type_ == 1 else tp.makeMesh( )
        
        if texture:
            calculationTiles = mappa.setTiles( mappa.radius )
            for i, tile in enumerate( calculationTiles ):
                points, centre = mappa.divideSrf( tile )
                geoLib = GeoLib( points, mappa.xf, imgResolution )
                pointGeo = mappa.xf * centre
                goog_url = geoLib.mapAPI( pointGeo, mapType )
                try:
                    path = GH_Path( 0, i )
                    name = directory + str( i ) + "elevation.png"
                    client = System.Net.WebClient()
                    written_file = client.DownloadFile( goog_url, name )
                    imagePath.Add( name, path )
                except:
                    pass
                    print("Something went wrong during the request of images. Please, try again.")
                    
            return tilesTree, tilePoints, gt[1], gt[0], gt[0][0].Z, tilePointsGeo, imagePath 
        return tilesTree, tilePoints, gt[1], gt[0], gt[0][0].Z, tilePointsGeo, None
    else:
        # delete image from the folder
        try:
            folderDataList = os.listdir( directory )
            if len( folderDataList ) != 0:
                for image in folderDataList:
                    os.remove( os.path.join( directory, image ) )
        except: pass
    return tilesTree, None, None, None, None, None, None


initCheck = False
if sc.sticky.has_key('ladybug_release'):
    initCheck = True
    try:
        if not sc.sticky['ladybug_release'].isCompatible(ghenv.Component): initCheck = True
    except:
        initCheck = False
        warning = "You need a newer version of Ladybug to use this compoent." + \
        "Use updateLadybug component to update userObjects.\n" + \
        "If you have already updated userObjects drag Ladybug_Ladybug component " + \
        "into canvas and try again."
        w = gh.GH_RuntimeMessageLevel.Warning
        ghenv.Component.AddRuntimeMessage(w, warning)
    lb_preparation = sc.sticky["ladybug_Preparation"]()
else:
    initCheck = False
    print "You should first let the Ladybug fly..."
    w = gh.GH_RuntimeMessageLevel.Warning
    ghenv.Component.AddRuntimeMessage(w, "You should first let the Ladybug fly...")

if initCheck:
    unitConversionFactor = lb_preparation.checkUnits()
    result = main( )
    if result != -1:
        tiles, points, terrain, origin, elevation, pointsGeo, imagePath = result
# hide outputs
ghenv.Component.Params.Output[2].Hidden = True