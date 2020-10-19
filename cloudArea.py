from osgeo import gdal, ogr, osr
import numpy as np
import os
import pyproj
from pyproj import Transformer
import shapely.wkt
from shapely.geometry import Polygon
import json

def saveRaster(dataset,datasetPath,cols,rows,projection):
    rasterSet = gdal.GetDriverByName('GTiff').Create(datasetPath, cols, rows, 1, gdal.GDT_Float32)
    rasterSet.SetProjection(projection)
    rasterSet.SetGeoTransform(gt)
    rasterSet.GetRasterBand(1).WriteArray(dataset)
    rasterSet.GetRasterBand(1).SetNoDataValue(-999)
    rasterSet = None
    
# Define nombre de carpeta con imagenes
folderPath = "geotiffs"
folderContent = os.listdir(folderPath)
listImages = []
dataResults = {}
dictInd = 1

# Agrega el nombre de todas las imagenes tiff a una lista
for i in folderContent:
    imgPath = os.path.join(folderPath, i)
    ext = os.path.splitext(i)[-1]
    if ext == ".tiff":
        listImages.append(imgPath)

# itera sobre las imagenes tiff encontradas en el folder 
for img in listImages:
    #print(img)
    rasterImg = gdal.Open(img)

    sceneClassBand = np.array(rasterImg.GetRasterBand(14).ReadAsArray())
    cols = rasterImg.RasterXSize
    rows = rasterImg.RasterYSize
    #print("Imagen tiene %d columnas por %d filas" % (cols, rows))
    rasterArea = cols * rows
    gt = rasterImg.GetGeoTransform()
    cellX = gt[1]
    cellY = -gt[5]
    pixelArea = cellX * cellY

    # calculo del punto medio de la imagen y conversion de las coordenadas de proyectadas a geograficas
    rasterCenter = [(gt[0] + (cols * cellX / 2)), (gt[3] - (rows * cellY / 2))]
    rasterPrj = rasterImg.GetProjection()
    rasterSR = osr.SpatialReference(wkt=rasterPrj)
    if rasterSR.IsProjected:
        srcProj = pyproj.Proj('epsg:32632')
        dstProj = pyproj.Proj(proj='longlat', ellps='WGS84', datum='WGS84')
        transformer = Transformer.from_proj(srcProj, dstProj, always_xy=True)
        long, lat = transformer.transform(rasterCenter[0], rasterCenter[1])
        #print("Parcela esta en %0.4f N, %0.4f E" % (lat, long))

    # Extrae pixeles de nubes de la banda scene classification
    cloudLayer = np.zeros((rows, cols), dtype=int)
    cloudLayer[sceneClassBand == 3] = 3
    cloudLayer[sceneClassBand == 8] = 8
    cloudLayer[sceneClassBand == 9] = 9
    cloudLayer[sceneClassBand == 10] = 10

    # guarda raster con pixeles de nubes
    saveRaster(cloudLayer, "temp/cloudLayer.tif", cols, rows, rasterPrj)
    dataCloud = gdal.Open("temp/cloudLayer.tif")
    bandCloud = dataCloud.GetRasterBand(1)

    # crea shapefile con contornos de areas de nubes
    ogr_ds = ogr.GetDriverByName("ESRI Shapefile").CreateDataSource("temp/cloudContour.shp")
    prj = dataCloud.GetProjectionRef()
    srs = osr.SpatialReference(wkt=prj)
    cloudContourShp = ogr_ds.CreateLayer('contour', srs)
    field_defn = ogr.FieldDefn("sceneClass", ogr.OFTInteger)
    cloudContourShp.CreateField(field_defn)
    gdal.Polygonize(bandCloud, None, cloudContourShp, 0, [], callback=None)
    ogr_ds = None

    # calculo del area de nubes de acuerdo a la banda scene classification
    cloudContour = ogr.Open("temp/cloudContour.shp")
    layer = cloudContour.GetLayer(0)
    clArea = 0
    for l in range(layer.GetFeatureCount()):
        feature = layer.GetFeature(l)
        if feature.GetField("sceneClass") != 0:
            geometry = feature.GetGeometryRef()
            areaF =  shapely.wkt.loads(geometry.ExportToWkt())
            clPoly = Polygon(areaF)
            clArea += clPoly.area
    #print("Imagen tiene %0.4f m2 de nubes" % clArea)
    rasterWorldArea = rasterArea * pixelArea
    cloudPerc = clArea / rasterWorldArea
    #print("%0.6f de la imagen tiene nubes" % cloudPerc)

    # crea un diccionario con los datos obtenidos de la imagen actual
    # y actualiza el general con todos los datos que se agregaran
    # al archivo json
    currImgData = {
        img : {
            "ubicacion" : {
                "lat" : lat,
                "long" : long
                },
            "nubes_m2" : clArea,
            "nubes_porc" : cloudPerc
            }
        }
    dataResults[dictInd] = currImgData
    dictInd += 1
    

    # cierra y elimina archivos generados temporalmente
    cloudContour = None
    dataCloud = None
    temps = os.listdir("temp")
    for files in temps:
        file = os.path.join("temp", files)
        os.remove(file)

#print(dataResults)
# crea el archivo json con todos los resultados obtenidos
with open("prueba_luxelare.json", "w") as write_file:
    json.dump(dataResults, write_file)
