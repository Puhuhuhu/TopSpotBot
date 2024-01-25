#include "main.h"
#include "IMUmesure.h"
#include "roues.h"
#include <chprintf.h>
#include <sensors/imu.h>

/* INITIALISATION DE VARIABLES */
static BSEMAPHORE_DECL(OBJET_SUIVANT, TRUE); // Synchronisation avec le thread Moteur

/* DEBUT DU THREAD CaptureIMU */
static THD_WORKING_AREA(waCaptureIMU, 256);
static THD_FUNCTION(CaptureIMU, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    /* Initialisation de varibales */
    int16_t pentes[NB_MAX_OBJETS + 1] = {0};   // Liste des pentes correspondantes (pentes[0] = nombre d'objets trouvés).
    uint8_t compteur_objet = 0;                // Compteur d'objets analysés.

    /* Initialisation du messagebus pour communiquer avec le thread Moteur */
    messagebus_topic_t pentes_topic;
    MUTEX_DECL(PENTES_LOCK);
    CONDVAR_DECL(PENTES_CONDVAR);
    messagebus_topic_init(&pentes_topic, &PENTES_LOCK, &PENTES_CONDVAR, &pentes, sizeof(pentes));
    messagebus_advertise_topic(&bus, &pentes_topic, "/pentes");

    while(1){

        // Attend le signal de départ par le thread Moteur.
        wait_IMU_CAPTURE();

        // Récupère l'accélération en X (filtrée sur "TAILLE_FILTRE" échantillons).
        compteur_objet++;
        pentes[0]++;
        pentes[compteur_objet] = get_acc_filtered(AXE_X, TAILLE_FILTRE);

        // Publie le résultat sur le topic /pentes.
        messagebus_topic_publish(&pentes_topic, &pentes, sizeof(pentes));

        // Envoie le signal de départ au thread Moteur.
        chBSemSignal(&OBJET_SUIVANT);
    }
}
/* FIN DU THREAD */

/* DEFINITION DE FONCTIONS */
// Lance le thread CaptureIMU.
void ThreadIMU_start(void)
{
	chThdCreateStatic(waCaptureIMU, sizeof(waCaptureIMU), NORMALPRIO, CaptureIMU, NULL);
}

// Fonction pour attendre le sémaphore OBJET_SUIVANT depuis un autre fichier.
void wait_OBJET_SUIVANT(void)
{
    chBSemWait(&OBJET_SUIVANT);
}