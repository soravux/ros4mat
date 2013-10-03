Header header
uint16 width
uint16 height
uint8 channels
uint8[] image       # Taille = width*height*channels
# image contient les donnees sur la forme [height][width][channel]
# par exemple, si le premier pixel (coordonnees (0,0)) a les valeurs RGB (240,32,111)
# et que le deuxieme (coordonnees (1,0)) a les valeurs RGB (12, 87, 149),
# alors image contiendra :
# [240, 32, 111, 12, 87, 149, ... (longueur d'une ligne = width), 1er pixel de la seconde ligne, etc.)]
uint8 compressionRatio
float64 timestamp
