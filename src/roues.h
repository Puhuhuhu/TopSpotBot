#ifndef DISTANCE_H
#define DISTANCE_H


 /* CONSTANTES */
#define VITESSE_ROT 300       // [step/s]
#define VITESSE_MAX 800       // [step/s]
#define TOUR_FAIT 1293        // [steps] (Distance entre roues / Perimètre Roue) * (Nombre steps révolution)
#define DISTANCE_MIN 50       // [mm] Distance minimale avec l'objet.
#define ON 1                  // Valeur pour allumer LED
#define LIM_ERREUR 3          // [mm] limite d'erreur du régulateur PI
#define KP 15
#define KI 10

/* DECLARATION DE FONCTIONS */
void tourner(int);
void ligne_droite(int);
void ThreadMoteur_start(void);
uint8_t pente_max(int16_t pentes[NB_MAX_OBJETS+1]); 
void moteur_stop(void);
void wait_IMU_CAPTURE(void);
bool get_phase_une (void);
int regulateur_pi(uint16_t distance);

#endif