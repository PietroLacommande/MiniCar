//
// Created by louis on 2024-11-05.
//


#ifndef P_I_D_CONTROLLER_H
#define P_I_D_CONTROLLER_H

class PID {
public:
    PID(float p, float i, float d);  // Initialisation du constructeur PID
    void setTunings(float p, float i, float d);  // Méthode pour mettre à jour les constantes PID
    float calculate(float setpoint, float measured, float dt);  // Méthode pour calculer la sortie PID
    void resetTunings();  // Méthode pour réinitialiser les constantes PID à zéro

private:
    float Kp;  // Gain proportionnel
    float Ki;  // Gain intégral
    float Kd;  // Gain dérivé
    float prevError;  // Erreur précédente
    float integral;  // Intégrale de l'erreur
};

#endif // P_I_D_CONTROLLER_H