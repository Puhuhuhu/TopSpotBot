#ifndef TOF_H
#define TOF_H

/* CONSTANTES */
#define DIST_BUF_TAILLE 5      // Taille du buffer des msesures de distance (détermine la netteté de la pente pour détecter un objet)
#define LIM_DIST 20            // [mm] Limite de différence de distance au delà de laquelle on considère qu'un objet est présent.
#define RAYON 300              // [mm] Rayon maximum de détection d'un objet.
#define FACTEUR_CORRECTION 20  // [steps] Correction de l'erreur de position.

/* DECLARATION DE FONCTIONS */
void ThreadTOF_start(void);
void TOF_calibrage(void);
int get_distance_TOF(void);

#endif