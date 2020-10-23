package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class PID {
    TeamCodeOpGoal robot = new TeamCodeOpGoal();
    VuforiaUltimateGoal robot2 = new VuforiaUltimateGoal();
    AngularVelocity r;

    //Posição atual e posição desejada
    double posicaoM;
    float velocidadeDesejada = 0.6f;
    double velocidadeAnterior;
    double currentVelocity, currenteVelocityError;

    // Aceleração, aceleração antiga, tempo atual e erro de tempo
    double aceleration, aceleracaoError;
    double aceleracaoDesejada = 0.2;
    double currentTime, previusTime;

    double output;

    PIDFCoefficients pidFeed = new PIDFCoefficients(0.5, 0, 0, 0.2);

    double previusTime2;
    double velocidadeAnterior2;
    //Controle de posição por PID
    //Testar com as mecanum
    public void feedForward(double posEs, double posEsBack, double posDi, double posDiBack, double setPoint) {
        //Média das posições atuais
        posicaoM  = (posEs + posEsBack + posDi + posDiBack) / 4.0;

        //Calculos de tempo, velocidade e erro de velocidade
        currentTime = r.acquisitionTime;
        currentVelocity = (posicaoM - setPoint) / (currentTime - previusTime);
        currenteVelocityError = velocidadeDesejada - currentVelocity;

        //Calculos de aceleração
        aceleration = (currentVelocity - velocidadeAnterior) / (currentTime - previusTime);
        aceleracaoError = aceleracaoDesejada - aceleration;

        //Output para velocidade/aceleração do motor
        output = pidFeed.f  + velocidadeDesejada * currenteVelocityError + aceleracaoDesejada * aceleracaoError;

        velocidadeAnterior = currentVelocity;
        previusTime = currentTime;

    }
    public double tracionada(double a) {
        //Calculos de tempo, velocidade e erro de velocidade
        double currentTime = r.acquisitionTime;
        double currentVelocity = (a) / (currentTime - previusTime);
        double currenteVelocityError = velocidadeDesejada - currentVelocity;

        //Calculos de aceleração
        aceleration = (currentVelocity - velocidadeAnterior2) / (currentTime - previusTime2);
        aceleracaoError = aceleracaoDesejada - aceleration;

        previusTime2 = currentTime;
        velocidadeAnterior2 = currentVelocity;

        //Output para velocidade/aceleração do motor
        return output = pidFeed.f  + velocidadeDesejada * currenteVelocityError + aceleracaoDesejada * aceleracaoError;

    }
}
