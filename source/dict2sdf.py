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
import numpy as np

# Copied from here: https://github.com/gazebosim/gazebo-classic/blob/gazebo11/media/materials/scripts/gazebo.material
MATERIALDICT = dict(
    Red=dict(ambient="1  0  0 1",
            diffuse="1 0 0 1",
            specular="0.1 0.1 0.1 1"),    
    Blue=dict(ambient="0 0 1 1",
            diffuse="0 0 1 1",
            specular="0.1 0.1 0.1 1"),
    Green=dict(ambient="0 1 0 1",
            diffuse="0 1 0 1",
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
    Yellow=dict(ambient="1 1 0 1",
            diffuse="1 1 0 1",
            specular="0 0 0 0"),
    GroundGray=dict(ambient="0.3 0.3 0.3 1",
            diffuse="0.7 0.7 0.7 1",
            specular="0.01 0.01 0.01 1"),
    
    )

def allclose_index(road_points, point):
    idx = None
    for i, pt in enumerate(road_points):
        if np.allclose(point, pt):
            idx = i
            break
    return idx

def split_roads(roadName, roadPoints):
    # Break if loops are there
    road_points = []
    multi_roads = []
    Npts = np.size(roadPoints, 1)
    for pidx in range(Npts):
        point = roadPoints[:3, pidx]
        idx = allclose_index(road_points, point)
        if idx is not None and pidx != Npts and idx != 0:
            print("Found loops in ", roadName)
            road_points.append(point)
            if idx >= 1:
                multi_roads.append(np.asarray(road_points[:idx]).T)
            if idx < len(road_points):
                multi_roads.append(np.asarray(road_points[idx:]).T)
            road_points = []
        else:
            road_points.append(point)
    if len(road_points):
        multi_roads.append(np.asarray(road_points).T)
    return multi_roads

def dilate_polyline(roadPoints, width):


    closed_polyline_points = []
    return_points = []
    prev_uvec = None
    for pidx in range(np.size(roadPoints, 1)-1):
        point = roadPoints[:2, pidx]
        next_point = roadPoints[:2, pidx+1]
        vec = (next_point - point) 
        uvec = vec / np.linalg.norm(vec)
        perpvec = np.array([-uvec[1], uvec[0]])
        point_left = point - perpvec * width / 2
        point_right = point + perpvec * width / 2
        if len(closed_polyline_points):
            prev_point_left = closed_polyline_points[-1]
            prev_ret_point = return_points[-1]
            if uvec.dot(prev_uvec) >= 0:  # Acute angle change in direction
                closed_polyline_points.append(point_left)
                return_points.append(point_right)
            else:
                # Direction has changed
                theta = np.atan2(prev_uvec[1], prev_uvec[0]) - np.atan2(uvec[1], uvec[0])
                thetan = (theta + np.pi) % (2*np.pi) - np.pi
                if thetan > 0: # direction changed to the left
                    # find the intersection of prev left boundary with left boundary
                    # prev_point + prev_uvec * t = point_left + uvec * q
                    tq = np.linalg.solve(np.vstack((uvec, -prev_uvec)).T, 
                                            prev_point_left - point_left)
                    cont_point = prev_point_left + prev_uvec * tq[1]
                    closed_polyline_points.append(cont_point)
                    return_points.append(point_left)
                    return_points.append(point_right)
                else: # direction changed to right
                    # find the intersection of prev right boundary with right boundary
                    # prev_ret_point + prev_uvec * t = point_right + uvec * q
                    tq = np.linalg.solve(np.vstack((uvec, -prev_uvec)).T, 
                                            prev_ret_point - point_right)
                    cont_ret_point = prev_ret_point + prev_uvec * tq[1]
                    return_points.append(cont_ret_point)
                    closed_polyline_points.append(point_right)
                    closed_polyline_points.append(point_left)

        else:
            closed_polyline_points.append(point_left)
            return_points.append(point_right)
        prev_uvec = uvec

    point_left = next_point - perpvec * width / 2
    point_right = next_point + perpvec * width / 2
    closed_polyline_points.append(point_left)
    return_points.append(point_right)
    closed_polyline_points.extend(reversed(return_points))
    closed_polyline_points.append(closed_polyline_points[0].copy())
    return closed_polyline_points


class GetSDF:

    def __init__(self):
        self.sdf = Et.Element('sdf')
        self.sdf.set('version', "1.5")
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

    def addRoad(self, roadName, width, roadPoints):
        '''Add road to sdf file'''
        if not roadPoints.shape[1]:
            return
        multiroads = split_roads(roadName, roadPoints)
        if len(multiroads) >= 2:
            for i, roads in enumerate(multiroads):
                self.addRoad(roadName + '_p%d' % i, width, roads)
            return

        road = Et.SubElement(self.sdf.find('world'), 'model')
        road.set('name', roadName)
        static = Et.SubElement(road, 'static')
        static.text = 'true'
        mainPose = Et.SubElement(road, 'pose')
        mainPose.text = '0 0 0 0 0 0'
        link = Et.SubElement(road, 'link')
        link.set('name', (roadName))
        Et.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        collision = Et.SubElement(link, 'collision')
        collision.set('name', (roadName))
        ode = Et.SubElement(Et.SubElement(Et.SubElement(collision, 'surface'), 'friction'), 'ode')
        Et.SubElement(ode, 'mu').text = '100'
        Et.SubElement(ode, 'mu2').text = '50'

        visual = Et.SubElement(link, 'visual')
        visual.set('name', (roadName))
        Et.SubElement(visual, 'cast_shadows').text = 'false'
        material = Et.SubElement(visual, 'material')
        for col_se in 'ambient diffuse specular'.split():
            Et.SubElement(material, col_se).text = MATERIALDICT['GroundGray'][col_se]
        
        for colvis in (collision, visual):
            geometry = Et.SubElement(colvis, 'geometry')
            polyline = Et.SubElement(geometry, 'polyline')
            Et.SubElement(polyline, 'height').text = "0.001"
            if np.all(roadPoints[:2, 0] == roadPoints[:2, -1]):
                for pidx in range(roadPoints.shape[1]):
                    Et.SubElement(polyline, 'point').text = \
                        "%f %f" % tuple(roadPoints[:2, pidx])
            else:
                dilated_road_points = dilate_polyline(roadPoints, width)
                if roadName in "footway_143862338".split():
                    np.savez(f"testFiles/{roadName}.npz", 
                                roadPoints=roadPoints, 
                                width=width)
                for point in dilated_road_points:
                    Et.SubElement(polyline, 'point').text = \
                        "%f %f" % tuple(point)

    def setRoadWidth(self, width, roadName):
        ''' Set the width of the road specified by the road name'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]

        roadWidth = Et.SubElement(roadWanted[0], 'width')
        roadWidth.text = str(width)

    def addRoadPoint(self, point, roadName, width):
        '''Add points required to build a road, specified by the roadname'''
        allRoads = self.sdf.find('world').findall('road')

        roadWanted = [road for road in allRoads
                      if road.get('name') == roadName]
        roadPoint = Et.SubElement(roadWanted[0], 'point')
        roadPoint.text = (str(point[0]) +
                          " " + str(point[1]) +
                          " " + str(point[2]))

    def addBuilding(self, mean, pointList, building_name, color, height):
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
        Et.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        for col_se in 'ambient diffuse specular'.split():
            Et.SubElement(material, col_se).text = MATERIALDICT[color][col_se]
        for colvis in (collision, visual):
            geometry = Et.SubElement(colvis, 'geometry')
            polyline = Et.SubElement(geometry, 'polyline')
            Et.SubElement(polyline, 'height').text = "%f" % height
            for point in range(np.size(pointList, 1)):
                Et.SubElement(polyline, 'point').text = \
                    "%f %f" % tuple(pointList[:2, point].tolist())
                
    def addGroundPlane(self, bbox):
        ground_plane_name = 'ground_plane'
        ground_plane = Et.SubElement(self.sdf.find('world'), 'model')
        ground_plane.set('name', ground_plane_name)
        static = Et.SubElement(ground_plane, 'static')
        static.text = 'true'
        mainPose = Et.SubElement(ground_plane, 'pose')
        mainPose.text = '%f %f 0 0 0 0' % ((bbox[0]+bbox[2])/2, (bbox[1]+bbox[3])/2)
        link = Et.SubElement(ground_plane, 'link')
        link.set('name', (ground_plane_name))
        collision = Et.SubElement(link, 'collision')
        collision.set('name', (ground_plane_name))
        ode = Et.SubElement(Et.SubElement(Et.SubElement(collision, 'surface'), 'friction'), 'ode')
        Et.SubElement(ode, 'mu').text = '100'
        Et.SubElement(ode, 'mu2').text = '50'

        visual = Et.SubElement(link, 'visual')
        visual.set('name', (ground_plane_name))
        Et.SubElement(visual, 'cast_shadows').text = 'false'
        material = Et.SubElement(visual, 'material')
        Et.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        for col_se in 'ambient diffuse specular'.split():
            Et.SubElement(material, col_se).text = MATERIALDICT['Yellow'][col_se]
        for colvis in (collision, visual):
            plane = Et.SubElement(Et.SubElement(colvis, 'geometry'), 'plane')
            Et.SubElement(plane, 'normal').text = '0 0 1'
            Et.SubElement(plane, 'size').text = '%f %f' % (
                (bbox[2]-bbox[0]), (bbox[3]-bbox[1]))

        

    def writeToFile(self, filename):
        '''Write sdf file'''
        with open(filename, "wb") as outfile:
            outfile.write(Et.tostring(self.sdf, pretty_print=True,
                                    xml_declaration=True))
