# Modo de solucion que utilizo la sbg 
uint8 status

float64 latitud
float64 longitud

bool position_valid # Si se tiene un error <= 10 m
# NED
float64 x # posicion en metros
float64 y # posicion en metros

bool attitude_valid # Si el error de roll/pitch < 0.5 grados
bool heading_valid # Si se tiene un error <= 1 grado

float64 roll # roll en radianes
float64 pitch # pitch en radianes
float64 yaw # heading en radianes