#include "globals.h"

// Definición del array global
int parametros[ARRAY_PARAMETROS_SIZE] = {
    0,//[0],0: BRAKE 1:changeState 2:DEBUG
    0,//[1]:Estado a probar, ver BREAK en movements
    FORWARD_X,//[2]: vel forward (forward,backward y line retreat)
    TURN_LEFT_SPEED,//[3]: vel giro izq 
    TURN_LEFT_45_DELAY,//[4]:
    TURN_LEFT_90_DELAY,//[5]:
    TURN_RIGHT_SPEED,//[6]: vel giro
    TURN_RIGHT_45_DELAY,//[7]:
    TURN_RIGHT_90_DELAY,//[8]:
    TURN_LEFT_180_DELAY,//[9]:
    SHORT_LEFT_DELAY,//[10]:
    SHORT_RIGHT_DELAY,//[11]:
    THRESHOLD,//[12]:                                                                                                                                                                
    TURKISH_TIME,//[13]:
    TURKISH_DELAY,//[14]:
    TURKISH_SPEED,//[15]:
    GIRO_U_DELAY,//[16]:
    GIRO_U_L_DELAY,//[17]:
    CORRECT_SPEED,//[18]: correccion rueda izquierda
    TURN_RIGHT_180_DELAY//[19]: giro 180 a la derecha (solo calibracion.cpp 0110)
};
