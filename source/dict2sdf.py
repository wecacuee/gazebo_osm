##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: GetSDF() class
#             Builds a sdf file by adding models and seting their properties,
#             roads and sets spherical coordinates for the world
##############################################################################

import lxml.etree as Et
import xml.dom.minidom as minidom
import numpy

# Copied from here: https://github.com/gazebosim/gazebo-classic/blob/gazebo11/media/materials/scripts/gazebo.material
MATERIALDICT = dict(
    Red=dict(ambient="1  0  0 1",
            diffuse="1 0 0 1",
            specular="0.1 0.1 0.1 1"),
    RedBright=dict(ambient="0.87 0.26 0.07 1",
            diffuse="0.87 0.26 0.07 1",
            specular="0.87 0.26 0.07 1"),
    Purple=dict(ambient="1 0 1 1",
            diffuse="1 0 1 1",
            specular="0.1 0.1 0.1 1"),
    Orange=dict(ambient="1 0.5088 0.0468 1",
            diffuse="1 0.5088 0.0468 1",
            specular="0.5 0.5 0.5 1"),    
    Blue=dict(ambient="0 0 1 1",
            diffuse="0 0 1 1",
            specular="0.1 0.1 0.1 1"),
    Yellow=dict(ambient="1 1 0 1",
            diffuse="1 1 0 1",
            specular="0 0 0 0")
    )

class GetSDF:

    def __init__(self):
        self.sdf = Et.Element('sdf')
        self.sdf.set('version', "1.4")
        world = Et.SubElement(self.sdf, 'world')
        world.set('name', 'default')
        self.modelList = dict()

    def addSphericalCoords(self, latVal, lonVal,
                           elevationVal=0.0, headingVal=0):
        ''' Add the spherical coordinates for the map'''
        spherical_coordinates = Et.SubElement(self.sdf.find('world'),
                                              'spherical_coordinates')

        model = Et.SubElement(spherical_coordinates, 'surface_model')
        model.text = "EARTH_WGS84"

        lat = Et.SubElement(spherical_coordinates, 'latitude_deg')
        lat.text = str(latVal)

        lon = Et.SubElement(spherical_coordinates, 'longitude_deg')
        lon.text = str(lonVal)

        elevation = Et.SubElement(spherical_coordinates, 'elevation')
        elevation.text = str(elevationVal)

        heading = Et.SubElement(spherical_coordinates, 'heading_deg')
        heading.text = str(headingVal)

    def includeModel(self, modelName):
        ''' Include models in gazebo database'''
        includeModel = Et.SubElement(self.sdf.find('world'), 'include')
        includeUri = Et.SubElement(includeModel, 'uri')
        includeUri.text = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/" + modelName
        return includeModel

    def addModel(self, mainModel, modelName, pose):
        '''Add model with pose and the name taken as inputs'''

        includeModel = self.includeModel(mainModel)

        model = Et.SubElement(includeModel, 'name')
        model.text = modelName

        static = Et.SubElement(includeModel, 'static')
        static.text = 'true'

        modelPose = Et.SubElement(includeModel, 'pose')

        modelPose.text = (str(pose[0]) +
                          " " + str(pose[1]) +
                          " " + str(pose[2]) + " 0 0 0")

    def addRoad(self, roadName):
        '''Add road to sdf file'''
        road = Et.SubElement(self.sdf.find('world'), 'road')
        road.set('name', roadName)

    def setRoadWidth(self, width, roadName):
        ''' Set the width of the road specified by the road name'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]

        roadWidth = Et.SubElement(roadWanted[0], 'width')
        roadWidth.text = str(width)

    def addRoadPoint(self, point, roadName):
        '''Add points required to build a road, specified by the roadname'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]
        roadPoint = Et.SubElement(roadWanted[0], 'point')
        roadPoint.text = (str(point[0]) +
                          " " + str(point[1]) +
                          " " + str(point[2]))

    def addBuilding(self, mean, pointList, building_name, color):
        building = Et.SubElement(self.sdf.find('world'), 'model')
        building.set('name', building_name)
        static = Et.SubElement(building, 'static')
        static.text = 'true'
        mainPose = Et.SubElement(building, 'pose')
        mainPose.text = '0 0 0 0 0 0'
        link = Et.SubElement(building, 'link')
        link.set('name', (building_name))
        collision = Et.SubElement(link, 'collision')
        collision.set('name', (building_name))
        visual = Et.SubElement(link, 'visual')
        visual.set('name', (building_name))
        material = Et.SubElement(visual, 'material')
        for col_se in 'ambient diffuse specular'.split():
            Et.SubElement(material, col_se).text = MATERIALDICT[color][col_se]
        Et.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        for colvis in (collision, visual):
            geometry = Et.SubElement(colvis, 'geometry')
            polyline = Et.SubElement(geometry, 'polyline')
            Et.SubElement(polyline, 'height').text = "1"
            for point in range(numpy.size(pointList, 1)):
                Et.SubElement(polyline, 'point').text = \
                    "%f %f" % tuple(pointList[:2, point].tolist())

    def writeToFile(self, filename):
        '''Write sdf file'''
        with open(filename, "wb") as outfile:
            outfile.write(Et.tostring(self.sdf, pretty_print=True,
                                    xml_declaration=True))
