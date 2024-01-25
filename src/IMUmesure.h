#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

/* CONSTANTES */
#define TAILLE_FILTRE 50
#define AXE_X 1

/* DECLARATION DE FONCTIONS */
void ThreadIMU_start(void);
void wait_OBJET_SUIVANT(void);

#endif

