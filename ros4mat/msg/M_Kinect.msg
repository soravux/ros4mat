Header header
uint16 width_rgb
uint16 height_rgb
uint8 channels
uint16 width_depth
uint16 height_depth
uint8[] rgb       # Taille = width_rgb*height_rgb*channels
uint8[] depth       # Taille = width_depth*height_depth
# image contient les donnees sur la forme [height][width][channel]
# par exemple, si le premier pixel (coordonnees (0,0)) a les valeurs RGB (240,32,111)
# et que le deuxieme (coordonnees (1,0)) a les valeurs RGB (12, 87, 149),
# alors image contiendra :
# [240, 32, 111, 12, 87, 149, ... (longueur d'une ligne = width), 1er pixel de la seconde ligne, etc.)]
float64 timestamp
