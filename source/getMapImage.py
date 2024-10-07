##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: getMapImage()
#             Uses the data from .osm file to output an image of the area
#             indicated in the .osm file and
#             Stores it in file with the specified name(.png)
##############################################################################

import os
try:
    import mapnik
    HAS_MAPNIK = True
except ImportError:
    HAS_MAPNIK = False

try:
    import matplotlib.pyplot as plt
    import matplotlib as mplb
    HAS_MTPLT = True
except ImportError:
    HAS_MTPLT = False

import numpy as np
        
from dict2sdf import dilate_polyline, split_roads, allclose_index


RGBACOLORS = dict(
    Red=[1,  0,  0, 1],
    Blue=[0, 0, 1, 1],
    Green=[0,1,0,1],
    RedBright=[0.87,0.26,0.07,1],
    Purple=[1,0,1,1],
    Orange=[1,0.5088,0.0468,1],
    Yellow=[1,1,0,1],
    GroundGray=[0.3,0.3,0.3,1]
    )

class MPLBMap():
    def __init__(self, bbox):
        self.fig, self.ax = plt.subplots(
            figsize=((bbox[2]-bbox[0])/100,
                     (bbox[3]-bbox[1])/100),
            subplot_kw=dict(xmargin=0, ymargin=0),
            linewidth=0, dpi=300)
        self.fig.set_layout_engine('tight', pad=0, w_pad=0, h_pad=0)
        self.bbox = bbox
        self.ax.set_xlim(bbox[0], bbox[2])
        self.ax.set_ylim(bbox[1], bbox[3])
        self.ax.axis('off')
        self.ax.axis('equal')

    def add_road(self, road_name, width, road_points):
        if not road_points.shape[1]:
            return
        multiroads = split_roads(road_name, road_points)
        if len(multiroads) >= 2:
            for i, roads in enumerate(multiroads):
                self.add_road(road_name + '_p%d' % i, width, roads)
            return

        if np.all(road_points[:2, 0] == road_points[:2, -1]):
            dilated_road_points = road_points[:2, :].T
        else:
            dilated_road_points = np.asarray(dilate_polyline(road_points, width))
        self.ax.add_patch(mplb.patches.Polygon(
            dilated_road_points[:-1], color=RGBACOLORS['GroundGray']))
            

    def add_roads(self, roadPointWidthMap):
        for road_name, road_data in roadPointWidthMap.items():
            self.add_road(road_name, road_data['width'], road_data['points'])

    def add_buildings(self, buildingLocationMap):
        for building_name, building in buildingLocationMap.items():
            self.add_building(building['mean'],
                        building['points'],
                        building_name,
                        building['color'],
                        building['height'])
    def add_building(self, mean, pointList, building_name, color, height):
        self.ax.add_patch(mplb.patches.Polygon(pointList[:2, :-1].T, 
                                               color=RGBACOLORS[color]))


    def save_image(self, image_file):
        self.fig.savefig(image_file)

def getMapImage(osmFile, map_output):
    '''Uses the data from the osmFile to out a .png image
       of the area depicted by the input file'''
    if not HAS_MAPNIK :
        print ('Error: Mapnik module is missing. ' +
               'Please install for getting the image functionality.')
        return -2

    if osmFile == '':
        print ('Error: getMapImage::No File Recieved')
        return -1

    highwaList = dict({"motorway": {'width': 4,
                                    'color': 'green',
                                    'fontSize': 12},
                       "trunk": {'width': 3,
                                 'color': 'green',
                                 'fontSize': 11},
                       "primary": {'width': 1.5,
                                   'color': '#0090ff',
                                   'fontSize': 10},
                       "secondary": {'width': 0.8,
                                     'color': '#ff00ff',
                                     'fontSize': 8},
                       "tertiary": {'width': 0.42,
                                    'color': '#000000',
                                    'fontSize': 8},
                       "residential": {'width': 0.21,
                                       'color': 'black',
                                       'fontSize': 8},
                       "living_street": {'width': 0.21,
                                         'color': 'black',
                                         'fontSize': 8},
                       "pedestrian": {'width': 0.21,
                                      'color': 'brown',
                                      'fontSize': 8},
                       "footway": {'width': 0.21,
                                   'color': 'brown',
                                   'fontSize': 8}})

    m = mapnik.Map(1024, 1024)
    m.background = mapnik.Color('white')

    for highwayType in highwaList.keys():
        styleType = mapnik.Style()

        rule = mapnik.Rule()

        rule.filter = mapnik.Expression('[highway]=' + "'" + highwayType + "'")


        line_symbolizer = mapnik.LineSymbolizer()
        line_symbolizer.stroke = mapnik.Color(highwaList[highwayType]['color'])
        line_symbolizer.stroke_linecap = mapnik.stroke_linecap.ROUND_CAP
        line_symbolizer.stroke_width = highwaList[highwayType]['width']

        rule.symbols.append(line_symbolizer)

        rule2 = mapnik.Rule()

        rule2.filter = mapnik.Expression('[highway]=' + "'" +
                                         highwayType + "'")

        text_symbolizer = mapnik.TextSymbolizer(mapnik.Expression("[name]"),
                                                "DejaVu Sans Book",
                                                highwaList[highwayType]
                                                          ['fontSize'],
                                                mapnik.Color('black'))
        text_symbolizer.halo_fill = mapnik.Color('white')

        rule2.symbols.append(text_symbolizer)

        styleType.rules.append(rule)
        styleType.rules.append(rule2)

        m.append_style(highwayType, styleType)

    ds = mapnik.Osm(file=osmFile)

    layer = mapnik.Layer('world')
    layer.datasource = ds
    for highwayType in highwaList.keys():
        layer.styles.append(highwayType)

    m.layers.append(layer)
    m.zoom_all()
    mapnik.render_to_file(m, map_output, 'png')

    return os.system('xdg-open ' + map_output)
