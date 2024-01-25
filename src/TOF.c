#include "main.h"
#include "TOF.h"
#include "roues.h"
#include <sensors/VL53L0X/VL53L0X.h>
#include <chprintf.h>
#include <motors.h>


/* INITIALISATION DE VARIABLES */
static uint16_t dist_buf[DIST_BUF_TAILLE] = {0};    // Buffer des mesures de distance.

/* DEBUT DU THREAD TraitementTOF */
static THD_WORKING_AREA(waTraitementTOF, 512);
static THD_FUNCTION(TraitementTOF, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    /* Initialisation de variables */
    int32_t position_debut_fin[2] = {0};            // Position moteur de droite debut puis fin.
    int32_t fin_premier_objet = 0;                  // Position de fin du premier objet si le robot commence sur un objet.
    int32_t position_premier_objet = 0;             // Position du premier objet si le robot commence sur un objet.
    int32_t position_trouve[NB_MAX_OBJETS+1] = {0}; // Liste des positions trouvées (position[0] = nombre d'objets trouvés).
    int32_t pos_buf[DIST_BUF_TAILLE] = {0};         // Buffer des positions du moteur.
    uint16_t cpt_obj = 0;                           // Compteur du nombre d'objets.
    uint8_t cpt_buf = 0;                            // Compteur du buffer.
    bool obj_detect = 0;                            // Booléen pour déterminer si un objet est détécté.
    bool commence_sur_objet = 0;                    // Booléen qui indique que le robot à commencé sur un objet.
    bool topic_publie = 0;                          // Booléen qui indique si le topic TOF à été publié.

    /* Initialisation du messagebus pour communiquer avec le thread Moteur */
    messagebus_topic_t TOF_topic;
    MUTEX_DECL(TOF_topic_lock);
    CONDVAR_DECL(TOF_topic_condvar);
    messagebus_topic_init(&TOF_topic, &TOF_topic_lock, &TOF_topic_condvar, &position_trouve, sizeof(position_trouve));
    messagebus_advertise_topic(&bus, &TOF_topic, "/TOF");

    while(1){

        // Attend le signal de départ du thread Moteur.
        if(get_phase_une()){
            dist_buf[cpt_buf] = VL53L0X_get_dist_mm();
            pos_buf[cpt_buf] = right_motor_get_pos();

            // Si le robot à commencé au milieu d'un objet.
            if((cpt_obj == 0) && (dist_buf[cpt_buf] > (dist_buf[(cpt_buf+1)%DIST_BUF_TAILLE] + LIM_DIST)) && (dist_buf[cpt_buf] <= RAYON)){
                fin_premier_objet = pos_buf[(cpt_buf+1)%DIST_BUF_TAILLE];
                commence_sur_objet = 1;
                cpt_obj++;
            }

            if(cpt_obj <= NB_MAX_OBJETS){   
                if(!obj_detect){     
                    // Detection du début d'un objet dans le rayon maximal.
                    if((dist_buf[cpt_buf] <= (dist_buf[(cpt_buf+1)%DIST_BUF_TAILLE] - LIM_DIST)) && (dist_buf[cpt_buf] <= RAYON)){ 
                            // Enregistre la position du moteur de droite au début de l'objet.
                            position_debut_fin[0] = pos_buf[(cpt_buf+1)%DIST_BUF_TAILLE];            
                            obj_detect = 1;
                            cpt_obj++;                                                  
                        }
                }
                else{
                    // Vérifie si la fin d'un objet est détectée .
                    if(dist_buf[cpt_buf] > (dist_buf[(cpt_buf+1)%DIST_BUF_TAILLE] + LIM_DIST)){

                        // Enregistre la position du moteur de droite à la fin de l'objet.
                        position_debut_fin[1] = pos_buf[(cpt_buf+1)%DIST_BUF_TAILLE]; 
                        obj_detect = 0;
                        position_trouve[0] = cpt_obj;     

                        // Enregistre la position moyenne de l'objet.
                        position_trouve[cpt_obj] = ((position_debut_fin[0]+position_debut_fin[1])/2) - FACTEUR_CORRECTION;
                    }
                }
                cpt_buf = (cpt_buf+1)%DIST_BUF_TAILLE;
            }

        }
        else{
            // Si le robot à commencé sur un objet. 
            if (commence_sur_objet){
                commence_sur_objet = 0;
                position_premier_objet = (((position_debut_fin[0]-TOUR_FAIT) + fin_premier_objet)/2) - FACTEUR_CORRECTION;
                // Si position < 0 alors le place en tant que dernier objet trouvé sinon en tant que premier objet trouvé.
                if(position_premier_objet < 0){
                    position_premier_objet = TOUR_FAIT + position_premier_objet;
                    for(int32_t i = 2; i <= position_trouve[0]; i++){
                        position_trouve[i-1] = position_trouve[i];
                    }
                    position_trouve[cpt_obj] = position_premier_objet;                    
                }
                else{
                    position_trouve[1] = position_premier_objet;
                }
            }
            // Publie les résultats sur le topic /TOF.
            if(!topic_publie){
                topic_publie = 1;
                 messagebus_topic_publish(&TOF_topic, &position_trouve, sizeof(position_trouve));
            }
        }
   
    }
}
/* FIN DU THREAD */

/* DEFINITION DE FONCTIONS */
// Lance le thread TraitementTOF.
void ThreadTOF_start(void)
{
	chThdCreateStatic(waTraitementTOF, sizeof(waTraitementTOF), NORMALPRIO, TraitementTOF, NULL);
}

// Remplie le buffer avec des valeurs mesurées.
void TOF_calibrage(void)
{
    for(uint8_t i = 0; i < DIST_BUF_TAILLE; i++){
        dist_buf[i] = VL53L0X_get_dist_mm();
    }
}

// Fonction pour récupérer une mesure de distance depuis un autre fichier.
int get_distance_TOF(void)
{
    return VL53L0X_get_dist_mm();
}