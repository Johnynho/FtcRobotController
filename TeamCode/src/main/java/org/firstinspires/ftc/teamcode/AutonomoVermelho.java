/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.movimentos.SubSistemas;

/**
 * Inicialmente tentar usar o pacote de movimentos (sem encoder)
 * @see SubSistemas, após adquirir os encoders utilizar e aprimorar a Classe abaixo
 * @see org.firstinspires.ftc.teamcode.movimentos.CalculoEncoder
 */
@Autonomous(name="Autônomo Aliança Vermelha", group="Pushbot")
public class AutonomoVermelho extends LinearOpMode {

    //Objetos
    HardwareClass  robot   = new HardwareClass();   // Use a Pushbot's hardware
    Vuforia vuf = new Vuforia();
    SubSistemas move = new SubSistemas();

    @Override
    public void runOpMode() throws InterruptedException {
        //Envia um telemetria para mostrar que o robô está iniciando
        telemetry.addData("Status", "Iniciando robô");
        telemetry.update();

        robot.hardwareGeral(hardwareMap);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Muda a static variável indicando que aliança o robô está jogando
        //Precisamos disso para configurar o vuforia no teleoperado também
        TeleOperado.ladoO = "Vermelho";

        //Configura o vuforia para iniciar a engine dele (que também é utilizada no Tf)
        //O vuforia por si só nesse método não inicia, ele inicia neste "vuf.ativeVuforia"
        vuf.configureVuforia(TeleOperado.ladoO, hardwareMap);

        /*
         * Caso a precisão das argolas seja baixa, fazer o robô se deslocar antes
         * Inicia o tensor flow para ler a pilha de argolas (desativa e inicia a engine no método abaixo)
         * Variável que indica a quantidade de argolas na pilha
         */

        //Ideia é ler a pilha antes de dar start no robô
        waitForStart();

        //Exemplo de movimentação sem encoder
        move.alinharX(20, 0.8);

        //Código para entregar gol pêndulo aqui

        /*
         * Quando chegar na imagem do gol para alinhar nos Ps
         * (ideia é que ele mantenha toda configuração do vuforia,
         * mesmo não ativado para os trackable)
         * //Importante lembrar que para desativar o vuforia é vuf.deactivate()
         * Procurando trackables agora
         */
        vuf.ativeVuforia();

        //Ao chegar no final do método ele para sozinho
    }
}
