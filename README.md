# Guitar-Auto-Tuner

The purpose of the project hereby presented is to ease the task of funning a guitar.
It is based off of a piezoelectric transducer which is coupled to the inside of the guitar, thus
being able to measure the vibrations produced by a resonant string. The signal produced
by the transducer is conditioned and routed to the input of a microcontroller, which will
use a frequency detection algorithm to determine if the guitar is in tune. This frequency is
then used as an input for a fuzzy controller which generates an optimal shaft rotation angle
for a stepper motor to act on the tunning peg. Through the analysis of this system it was
discovered that the uncertainty of the frequency detection is smaller for the lower frequency
strings. Overall, while the system’s performance is acceptable for the fourth and fifth strings,
it is unsatisfactory for the higher frequency ones.

>O projeto apresentado nesse relatório tem o objetivo de facilitar a tarefa de afinação
de um violão. Ele é baseado em um sensor piezoelétrico acoplado ao instrumento, que
mede a vibração do mesmo quando uma corda é tocada. O sinal do sensor é condicionado
e enviado ao conversor AD de um microcontrolador, que recebe este sinal e aplica um algoritmo
de detecção de frequência para medir se ele está afinado. A frequência medida serve
como parâmetro de entrada de um fuzzy, que calcula o ângulo de rotação de um motor de
passo que atua sobre a tarraxa do violão. Na análise do sistema, foi constatado que a incerteza
da detecção de frequência é menor para as três cordas mais graves. A performance do
sistema como um todo foi razoável para a quarta e quinta corda, enquanto que para as cordas
mais agudas, sobretudo a primeira, sua capacidade de afinação é reduzida.
