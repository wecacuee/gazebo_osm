##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description: getOsmFile()
#             Downloads the .osm file for the stated bounding box and
#             Stores it in file with the specified name
##############################################################################

import urllib
import osmapi


def getOsmFile(box, outputFile='map.osm', inputOsmFile=''):
    '''downloads the data file for the specified bounding box
       stores the file as outputFile, if inputOsmFile is not specified
       and also converts the data in the form of a dictionary'''
    if not box and not inputOsmFile:
        return None

    dataDict = {}
    if inputOsmFile:
        outputFile = inputOsmFile
    else:
        with  urllib.request.urlopen('http://api.openstreetmap.org' +
                                      '/api/0.6/map?bbox='
                                      + str(box)[1:-1].replace(" ", "")) as osmFile:
            with open(outputFile, 'w') as osm:
                osm.write(osmFile.read())

    with open(outputFile, 'r') as osmRead:
        dataDict = osmapi.parser.ParseOsm(osmRead.read())

    return dataDict
