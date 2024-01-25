#include "main.h"
#include "roues.h"
#include "TOF.h"
#include "IMUmesure.h"
#include <chprintf.h>
#include <motors.h>
#include <leds.h>

/* INITIALISATION DE VARIABLES */
static BSEMAPHORE_DECL(IMU_CAPTURE, TRUE); // Synchronisation entre les thread Moteur et CaptureIMU.
static bool phase_une = 0;                 // Exécuter ou stopper le thread TraitementTOF.
static bool avance = 0;                    // Booléen pour déterminer si le robot est entrain d'avancer.

/* DEBUT DU THREAD Moteur */
static THD_WORKING_AREA(waMoteur, 256);
static THD_FUNCTION(Moteur,arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    /* Initialisation de variables */
    int32_t nb_pas=0;                               // Compteur du nombre de pas du moteur.
    int32_t position_trouve[NB_MAX_OBJETS+1] = {0}; // Liste des positions trouvées (position[0] = nombre d'objets trouvés).
    int16_t pentes[NB_MAX_OBJETS+1] = {0};          // Liste des pentes correspondantes (pentes[0] = nombre d'objets trouvés).
    uint8_t objet_max = 0;                          // Numéro de l'objet correspondant à la pente maximum.
    uint16_t distance = 0;                          // Récupère la valeur du capteur de distance.

    /* Attente des topics /TOF et /pentes */
    messagebus_topic_t *TOF_topic = messagebus_find_topic_blocking(&bus, "/TOF"); 
    messagebus_topic_t *pentes_topic = messagebus_find_topic_blocking(&bus, "/pentes"); 

    /* ---- DEBUT PHASE 1 ---- */

    // Le robot fait un tour complet.
    // Exécute le thread TraitementTOF pendant le tour.
    phase_une = 1;
    tourner(VITESSE_ROT);
    while(nb_pas < TOUR_FAIT){
        nb_pas = right_motor_get_pos();
    }
    moteur_stop();
    phase_une = 0;

    // Récupère les données de la position des objets.
    messagebus_topic_read(TOF_topic, &position_trouve, sizeof(position_trouve));

    /* ---- FIN PHASE 1 ---- */

    /* ---- DEBUT PHASE 2 ---- */

    // Réinitialisation du compteur et de la position du moteur.
    nb_pas=right_motor_get_pos()-TOUR_FAIT;
    right_motor_set_pos(nb_pas);

    // Le robot parcoure les différentes positions des objets trouvés.
    // Le thread CaptureIMU récupère les valeurs de l'accélération sur l'axe X.
    for(int i=1; i<=position_trouve[0];i++){
        tourner(VITESSE_ROT); 
        while(nb_pas < position_trouve[i]){
            nb_pas = right_motor_get_pos();
        }
        moteur_stop();
       
        chBSemSignal(&IMU_CAPTURE);
        wait_OBJET_SUIVANT();
    }

    // Récupère les données des accélérations.
    messagebus_topic_read(pentes_topic, &pentes, sizeof(pentes));

    // Renvoie le numéro de l'objet avec l'accélération maximum.
    objet_max = pente_max(pentes);

    /* ---- FIN PHASE 2 ---- */

    /* ---- DEBUT PHASE 3 ---- */

    // Détermine le sens de rotation pour aller au plus vite sur l'objet.
    if((right_motor_get_pos()- position_trouve[objet_max]) < TOUR_FAIT/2){
        tourner(-VITESSE_ROT);
        while(nb_pas > position_trouve[objet_max]){
            nb_pas = right_motor_get_pos();
        }
        moteur_stop();
    }
    else{
        tourner(VITESSE_ROT);
        while(nb_pas < TOUR_FAIT){
            nb_pas = right_motor_get_pos();
        }
        moteur_stop();
        nb_pas = right_motor_get_pos() - TOUR_FAIT;
        right_motor_set_pos(nb_pas);
        tourner(VITESSE_ROT);
        while(nb_pas < position_trouve[objet_max]){
            nb_pas = right_motor_get_pos();
           
        }
        moteur_stop();
    }

    // Avance sur l'objet jusqu'à une distance minimum avec l'objet.

    avance = 1;
    while(avance){
        distance = get_distance_TOF();
        ligne_droite(regulateur_pi(distance));
    }

    /* ---- FIN PHASE 3 ---- */
}
/* FIN DU THREAD */   

/* DEFINITION DE FONCTIONS */

// Fait tourner l'e-puck.
void tourner(int vitesse)
{
    right_motor_set_speed(vitesse);
    left_motor_set_speed(-vitesse);
}

// Fait aller l'e-puck en ligne droite.
void ligne_droite(int vitesse)
{
    right_motor_set_speed(vitesse);
    left_motor_set_speed(vitesse);
}

// Stop les moteurs.
void moteur_stop(void)
{
    right_motor_set_speed(0);
    left_motor_set_speed(0);
}

// Lance le thread Moteur.
void ThreadMoteur_start(void)
{
    chThdCreateStatic(waMoteur,sizeof(waMoteur),NORMALPRIO,Moteur,NULL);
}

// Calcule la pente maximum du vecteur pentes. Renvoie le numéro de l'objet correspondant.
uint8_t pente_max(int16_t pentes[NB_MAX_OBJETS+1])
{
    int16_t pente_max = pentes[1];
    uint8_t objet = 1;

    for(uint8_t i=2; i <= pentes[0]; i++){
        if(pentes[i] > pente_max){
            pente_max = pentes[i];
            objet = i;
        }
    }
    return objet;
}

// Fonction pour attendre le sémaphore IMU_CAPTURE depuis un autre fichier.
void wait_IMU_CAPTURE(void)
{
    chBSemWait(&IMU_CAPTURE);
}

// Fonction pour récupérer la valeur de PhaseUneFlag depuis un autre fichier.
bool get_phase_une (void)
{
    return phase_une;
}

// Régulateur PI pour la ligne droite.
int regulateur_pi(uint16_t distance)
{
    int32_t erreur = 0;
    int16_t vitesse = 0;
    int32_t somme_erreur = 0;

    erreur = distance - DISTANCE_MIN;

    if(abs(erreur) < LIM_ERREUR){
        set_body_led(ON);
        avance = 0;
        return 0;
    }

    somme_erreur += erreur;

    vitesse = KP * erreur + KI * somme_erreur;

    if(vitesse > VITESSE_MAX){
        vitesse = VITESSE_MAX;
    }
    return vitesse;
}

