# Modo de solucion que utilizo la sbg 
uint8 status

float64 latitud
float64 longitud

bool position_valid # Si se tiene un error <= 10 m
# NED
float64 x # posicion en metros
float64 y # posicion en metros

bool heading_valid # Si se tiene un error <= 1 grado
float64 yaw # heading en radianes