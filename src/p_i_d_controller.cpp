
#include "p_i_d_controller.h"




// Initialisation du constructeur PID avec les gains Kp, Ki, et Kd
PID::PID(float p, float i, float d) : Kp(p), Ki(i), Kd(d), prevError(0), integral(0) {}

// Méthode pour mettre à jour les constantes PID
void PID::setTunings(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}


// Méthode pour calculer la sortie PID
float PID::calculate(float setpoint, float measured, float dt) {
    float error = setpoint - measured;  // Calculer l'erreur
    integral += error * dt;  // Calcul de l'intégrale de l'erreur
    float derivative = (dt == 0) ? 0 : (error - prevError) / dt;  // Calcul de la dérivée de l'erreur avec vérification de dt
    prevError = error;  // Mise à jour de l'erreur précédente
    return Kp * error + Ki * integral + Kd * derivative;  // Calcul de la sortie PID
}

// Méthode pour réinitialiser les constantes PID à zéro
void PID::resetTunings() {
    Kp = 0;
    Ki = 0;
    Kd = 0;
    prevError = 0;
    integral = 0;
}

