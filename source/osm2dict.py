##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: Osm2Dict() class
#             Output a list of roads and models that need to be simulated in
#             the gazebo form the data it recives from the .osm file
##############################################################################

import numpy as np


class Osm2Dict:

    def __init__(self, lonStart, latStart, data, flags=['a']):

        self.latStart = latStart
        self.lonStart = lonStart
        self.data = data
        self.displayAll = 'a' in flags
        self.displayModels = 'm' in flags
        self.displayRoads = 'r' in flags
        self.displayBuildings = "b" in flags
        self.flags = flags
        #Radius of the Earth
        self.R = 6371e3 # m
        self.bbox = None
        #Dictionaries to store results
        self.records = dict()
        self.models = dict()
        self.buildings = dict()
        #types of highways to be simulated
        self.highwayType = dict({"footway": 0.3, 'pedestrian': 3,
                                 "motorway": 14, "motorway_link": 13,
                                 "trunk": 12, "trunk_link": 11,
                                 "primary": 10, "primary_link": 9,
                                 "secondary": 8, "secondary_link": 7,
                                 "tertiary": 6, "tertiary_link": 5,
                                 "residential": 3,
                                 "steps": 0.8})

        #types of models and buildings to be simulated and a dictionary
        #associating them with models in gazebo and their occurences
        self.modelType = ['highway', 'amenity', 'building', 'emergency']

        self.addModel = dict({"stop": {"modelName": "Stop Sign",
                                       "occurence": -1},
                              "street_lamp": {"modelName": "Lamp Post",
                                              "occurence": -1},
                              "traffic_signals": {"modelName":
                                                  "Construction Cone",
                                                  "occurence": -1},
                              "fire hydrant": {"modelName": "Fire hydrant",
                                               "occurence": -1},
                              "give_way": {"modelName": "Speed limit sign",
                                           "occurence": -1},
                              "bus_stop": {"modelName":
                                           "RoboCup 2014 SPL Goal",
                                           "occurence": -1},
                              "fuel": {'modelName': "Gas Station",
                                       'occurence': -1}
                              })

        self.amenityList = dict({"school": {"color": "Purple",
                                            "occurence": -1,
                                            "height": 3},
                                 "post_office": {'color': 'Orange',
                                                 'occurence': -1,
                                            "height": 3},
                                #  "university": {"color": "Purple",
                                #                 'occurence': -1,
                                            # "height": 3},
                                 "library": {"color": "Purple",
                                             'occurence': -1,
                                            "height": 3},
                                 "bar": {"color": "Blue",
                                         'occurence': -1,
                                            "height": 3},
                                 "cafe": {'color': "Blue",
                                          'occurence': -1,
                                            "height": 3},
                                 "pub": {"color": "Blue",
                                         'occurence': -1,
                                            "height": 3},
                                 "restaurant": {"color": "Blue",
                                                'occurence': -1,
                                            "height": 3},
                                 "fast_food": {"color": "Blue",
                                               'occurence': -1,
                                            "height": 3},
                                 "college": {"color": "Purple",
                                             'occurence': -1,
                                            "height": 3},
                                 "kindergarten": {"color": "Purple",
                                                  'occurence': -1,
                                            "height": 3},
                                "parking": {'color': "GroundGray",
                                            "occurence": -1,
                                            "height": 0.01}
                                 })
        self.landuseList = dict({"grass": {"color": "Green", 
                                           'occurence': -1,
                                            "height": 0.01}})

        self.node = {data[i].get("data").get("id"): data[i].get('data')
                     for i in range(len(data))
                     if data[i].get("type") == "node"}

        self.ways = {data[i].get("data").get("id"): data[i].get('data')
                     for i in range(len(data))
                     if data[i].get("type") == "way"}

    def getPoints(self, coords):
        '''Input : latitude and longitudnal coordinates
           Return the points in gazebo frame with respect
           to the starting coordinates'''
        if not coords.any():
            return []

        lon2 = np.radians(coords[:, 0])
        lat2 = np.radians(coords[:, 1])

        dLat = lat2-np.radians(self.latStart)
        dLon = lon2-np.radians(self.lonStart)

        a = (np.sin(dLat/2) * np.sin(dLat/2) +
             np.sin(dLon/2) * np.sin(dLon/2) *
             np.cos(np.radians(self.latStart)) *
             np.cos(lat2))

        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))

        distance = self.R * c

        angles = (np.arctan2(np.sin(dLon) * np.cos(lat2),
                  np.cos(np.radians(self.latStart)) *
                  np.sin(lat2) -
                  np.sin(np.radians(self.latStart)) *
                  np.cos(lat2) * np.cos(dLon)))

        point = np.array([distance*np.cos(angles),# * 1000,
                          -distance*np.sin(angles),# * 1000,
                          np.zeros(np.shape(distance))#*1000
                          ])
        return point

    def latLonToPoints(self, node_ref):
        '''Pulls out the latitude and longitudes of the nodes in the
           list of nodes and gets the points in the gazebo frame'''
        coords = []
        for node in node_ref:
            coords.append([self.node[node].get("lon"),
                           self.node[node].get("lat")])
        
        pointsXYZ = self.getPoints(np.asarray(coords))
        return pointsXYZ

    def getRoadDetails(self):
        '''Returns a list of roads with corresponding widths'''
         # get the road latitude and longitudes
        for way in self.ways.keys():
            tagData = self.ways[way].get("tag")
            if "highway" in tagData:
                typeHighway = tagData.get("highway")

                if typeHighway in self.highwayType.keys():

                            roadName = tagData.get("name")

                            if roadName is None:
                                roadName = (typeHighway +
                                            "_" +
                                            str(self.ways[way]
                                                    .get("id")))
                            else:
                                roadName += "_" + str(self.ways[way]
                                                          .get("id"))

                            node_ref = self.ways[way].get("nd")
                            if node_ref:
                                location = self.latLonToPoints(node_ref)

                                self.records[roadName] = {'points':
                                                          location,
                                                          'width':
                                                          self.highwayType
                                                          [typeHighway]}
        return self.records

    def getModelDetails(self):
        '''Returns a list of models to be included in the map'''
        models = {element + "$" + str(i): self.data[i].get("data")
                  for i in range(len(self.data))
                  for element in self.addModel.keys()
                  if element in self.data[i].get("data").get("tag").values()}

        for mName, data in models.items():
            modelType = mName.split("$")[0]

            coords = np.array([data.get("lon"),
                               data.get("lat")])
            coords = np.reshape(coords, (len(coords)//2, 2))

            modelLocation = self.getPoints(coords)

            self.addModel[modelType]['occurence'] += 1

            repNum = self.addModel[modelType]['occurence']

            self.models[self.addModel
                        [modelType]
                        ['modelName']
                        + "_" +
                        str(repNum)] = {"points": modelLocation,
                                        "mainModel": self.addModel
                                        [modelType]['modelName']}

    def getBuildingDetails(self):
        '''Returns a list of buildings to be included in the map'''
        building = [d
                    for d in self.data
                    if ("building" in d.get("data").get("tag")
                        and d.get("type") in ("way", "relation"))]

        for element_w_type in building:
            e_type = element_w_type.get("type")
            element = element_w_type.get("data")
            tagData = element.get("tag")
            
            
            tagData = element.get("tag")
            if "name" in tagData:
                buildingName = tagData.get("name")
            else:
                buildingName = ("office_building" +
                                "_" +
                                str(element.get("id")))
            if "name_1" in tagData:
                buildingName += tagData.get("name_1")

            if e_type == "relation":
                members = [self.ways[m['ref']]
                    for m in element.get("member")
                    if m.get('type') == 'way' and m.get('role') == 'outer']
                collected_node_ref = [] 
                for element in members:
                    node_ref = element.get("nd")
                    if len(node_ref) <= 2 or node_ref[0] != node_ref[-1]:
                        if len(collected_node_ref) and collected_node_ref[-1] == node_ref[0]:
                            collected_node_ref.append(node_ref[1:])
                        else:
                            collected_node_ref.append(node_ref)
                if len(collected_node_ref):
                    members = [{'nd': sum(collected_node_ref, [])}]
            else:
                members = [element]
            for i, element in enumerate(members):
                node_ref = element.get("nd")
                location = self.latLonToPoints(node_ref)

                buildingLoc = np.array([[sum(location[0, :]) /
                                         len(location[0, :])],
                                        [sum(location[1, :]) /
                                         len(location[1, :])],
                                        [sum(location[2, :]) /
                                         len(location[2, :])]]
                                       )
                buildingName = ((buildingName + "_p%d" % i) 
                                if len(members) > 1 
                                else buildingName)
                self.buildings[buildingName] = {"mean":
                                                buildingLoc,
                                                "points": location,
                                                "color": "Red",
                                                "height": 3}

        amenity = [self.data[i]
                   for i in range(len(self.data))
                   if self.data[i].get("data").get("tag").get("amenity")
                   in self.amenityList]

        for element_w_type in amenity:
            e_type = element_w_type.get("type")
            element = element_w_type.get("data")
            tagData = element.get("tag")
            thisamenity = tagData.get("amenity")
            default_name = thisamenity +"_" + str(element.get("id"))
            if e_type == "relation":
                members = [self.ways[m['ref']]
                    for m in element.get("member")
                    if m.get('type') == 'way' and m.get('role') == 'outer']
                name = tagData.get("name", default_name)
                print("Parsing relation amenity: ", name)
            else:
                members = [element]
                name = tagData.get("name", default_name)
            for element in members:
                node_ref = element.get("nd")
                if node_ref:
                    location = self.latLonToPoints(node_ref)

                    amenityLocation = np.array([[sum(location[0, :]) /
                                                 len(location[0, :])],
                                                [sum(location[1, :]) /
                                                 len(location[1, :])],
                                                [sum(location[2, :]) /
                                                 len(location[2, :])]]
                                               )

                    self.amenityList[thisamenity]['occurence'] += 1
                    repNum = self.amenityList[thisamenity]['occurence']

                    self.buildings[name + "_%d" %
                                   element.get("id")] = {"mean": amenityLocation,
                                                   "points": location,
                                                   "color":
                                                   self.amenityList
                                                   [thisamenity]
                                                   ['color'],
                                                   "height":
                                                   self.amenityList
                                                   [thisamenity]
                                                   ['height'],
                                                   }
        landuses = [self.data[i]
                   for i in range(len(self.data))
                   if self.data[i].get("data").get("tag").get("landuse") in 
                   self.landuseList]
        
        for element_w_type in landuses:
            e_type = element_w_type.get("type")
            element = element_w_type.get("data")
            tagData = element.get("tag")
            thislanduse = tagData.get("landuse")
            default_name = thislanduse +"_" + str(element.get("id"))
            if e_type == "relation":
                members = [self.ways[m['ref']]
                    for m in element.get("member")
                    if m.get('type') == 'way' and m.get('role') == 'outer']
                name = tagData.get("name", default_name)
                print("Parsing relation Name: ", name)
            else:
                members = [element]
                name = tagData.get("name", default_name)
            for element in members:
                node_ref = element.get("nd")
                if node_ref:
                    location = self.latLonToPoints(node_ref)

                    landuseLocation = np.array([[sum(location[0, :]) /
                                                 len(location[0, :])],
                                                [sum(location[1, :]) /
                                                 len(location[1, :])],
                                                [sum(location[2, :]) /
                                                 len(location[2, :])]]
                                               )

                    self.landuseList[thislanduse]['occurence'] += 1
                    repNum = self.landuseList[thislanduse]['occurence']
                    self.buildings[name + "_%d" % element.get("id")
                                   ] = {"mean": landuseLocation,
                                                   "points": location,
                                                   "color":
                                                   self.landuseList
                                                   [thislanduse]
                                                   ['color'],
                                                   "height": 
                                                   self.landuseList
                                                   [thislanduse]
                                                   ['height']
                                                   } 
        return
        service = [self.data[i].get("data")
                   for i in range(len(self.data))
                   if "service" in self.data[i].get("data").get("tag")]

        for element in service:
            if element.get("tag").get("service") == "parking_aisle":
                node_ref = element.get("nd")
                if node_ref:
                    location = self.latLonToPoints(node_ref)

                    parkingLocation = np.array([[sum(location[0, :]) /
                                                 len(location[0, :])],
                                                [sum(location[1, :]) /
                                                 len(location[1, :])],
                                                [sum(location[2, :]) /
                                                 len(location[2, :])]]
                                               )

                    self.buildings["parking_aisle_" +
                                   str(element
                                       .get("id"))] = {"mean":
                                                       parkingLocation,
                                                       "points": location,
                                                       "color": "Yellow"
                                                       }

    def getMapDetails(self):
        ''' Returns a list of highways with corresponding widths
            and a list of all the models to be included'''
        if 'm' in self.flags or 'a' in self.flags:
            self.getModelDetails()

        if 'b' in self.flags or 'a' in self.flags:
            self.getBuildingDetails()

        if 'r' in self.flags or 'a' in self.flags:
            self.getRoadDetails()

        return self.records, self.models, self.buildings

    def setFlags(self, addFlag):
        '''sets the value for the list of flags'''
        if addFlag in ['a', 'm', 'r', 'b']:
            if addFlag not in self.flags:
                if addFlag == 'a':
                    self.flags.append(addFlag)
                else:
                    self.flags = [addFlag]
            return True
        else:
            print('Error: Invalid flag! [Valid values : "a", "m", "r", "b"]')
            return False

    def getFlags(self):
        '''Returns the list of flags activated'''
        return self.flags

    def getLat(self):
        '''Get the latitude of the start point'''
        return self.latStart

    def getLon(self):
        '''Get the longitude of the start point'''
        return self.lonStart
    
    def getPointBBox(self, latlongbbox):
        pointsXYZ = self.getPoints(np.asarray(latlongbbox).reshape(2, 2))
        minXYZ = np.min(pointsXYZ, axis=1)
        maxXYZ = np.max(pointsXYZ, axis=1)
        self.bbox = [minXYZ[0], minXYZ[1], maxXYZ[0], maxXYZ[1]]
        return self.bbox
