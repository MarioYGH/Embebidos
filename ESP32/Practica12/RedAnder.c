/*
Red Neuronal para giroscopio
en este codigo se hara una red neuronal para identificar los weights y biases de una red neuronal
El proyecto es un giroscopio que tiene diferentes aceleraciones en x, y, z y entrenaremos una red neuronal para identificar el valor de la aceleracion y nos muestre el tipo de movimiento de acuerdo a las aceleraciones.

principalemnete se hara para ese objetivo pero se podra adaptar para otro proyecto ya que es la adquisicion mas bien de las matrices de la red.

tndemos una red con tres entradas x1 x2 & x3 que son las aceleraciones en x y & z respectivamente. dos neuronas en la capa oculta con tres (3) entradas cada una (IW) un bias cada una b1 y 5 salidas cada una. hay cinco (5) salidas con un sus respectivos weights de la capa oculta LW y con un bias b2 para cada una para tener y1, y2, y3, y4 & y5

tenemos la ecuacion de nuestra red de la siguiente forma: y = P2(LW * P1(IW*X + b1)+b2)
donde:
X
es la matriz formada por los datos adquiridos del acelerometro de tamanio [3 x n] 3 entradas n datos por entrada [entrys x numb of data ]
IW
son los input weights que tiene cada dato entrando a la capa oculta de diension [3 x 2] 3 entradas 2 neuronas [entrys x num of neurons]
LW
son los weights de las salidas que recibe la capa de salida de cada neurona en la capa oculta de dimension [2 x 5] 2 neuronas con 5 salidas cada una [num of neurons x num of outputs]
b1
el bias que tiene la capa oculta para cada neurona con una dimension de [1 x 2] una 1 capa oculta con 2 neuronas [1 (shell) x num of neurons]
b2
el bias que tendra la salida para dar resultados con dimension [1 x 5] 1 capa de salida 5 salidas posibles [1 capa x num de salidas]
P1
funcion de activacion para las neuronas de la capa oculta en este caso va a ser ReLU, es una funcion del tipo
ReLU: f(x)=max(0,x) donde se tomara el valor de x cuando x >= 0 & 0 cuando x sea x=<0
P2
funcion de activacion para las neuronas de la capa de salida en este caso sera la funcion softmax, es una funcion del tipo
softmax(x_i) = e^x_i / sum(e^(x_1-->n))
Esta es la fórmula de la función Softmax, que toma un vector de valores x como entrada y produce un nuevo vector de la misma longitud, donde cada elemento está en el rango de 0 a 1, y la suma de todos los elementos es igual a 1
y
es la salida resultado de la red o capa de salidas dada por la ecuacion que describe nuestra red

Ander Villalobos Gilling

fecha de creacion 27/04/24
fecha de modificacion 7/05/24 

hecho orignalmnte en julia traducido a c 

*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


///funciones/////
//ahora construiremos nuestras funciones de activacion empezando por ReLU y despues softmax
///////ReLU///////
void ReLU(float X[][1], int rows) {
    for (int i = 0; i < rows; i++) {
        X[i][0] = (X[i][0] >= 0) ? X[i][0] : 0;
    }
}

/////////SoftMax//////////////////
void Softmax_Matrix(float X[][1], int rows) {
    // Calcular la suma de las exponenciales de los elementos de la matriz
    float sum_exp = 0.0;
    for (int i = 0; i < rows; i++) {
        sum_exp += exp(X[i][0]);
    }

    // Aplicar Softmax a cada elemento de la matriz
    for (int i = 0; i < rows; i++) {
        X[i][0] = exp(X[i][0]) / sum_exp;
    }
}

// empezamos a nombrar variables que existen en la red 
//como entradas neuronas etc que afectan las dimensiones de las matrices 


void app_main(void)
{
    //probablemente no se usen los siguientes valores pero podrian funcionar si se quiere editar lared y comrender mejor
int ndat = 1;       // numero de datos
int nentry = 3;     // numero de entradas
int numneu = 2;     // numero de neuronas en la capa oculta
int nout = 5;       // numero de salidas


// ahora si declaramos nuestras variables matrices X IW LW b1 b2 que forman parte de la ecuacion de la red 
//no estoy seguro si se deberia o podria declarar las matrices fura del bucle principal pero lo haremos aqui 
// Definir las matrices X, IW, LW y los vectores b1, b2

 float X[3][1]= {
    {0.00},
    {0.00},
    {0.00}
 }; // hay que verificar este porque va a variar con nuestros datos
 
 float IW[3][2] = {
        {0.9654763, -4.744184},
        {-6.8593006, -4.435503},
        {0.8444871, 2.5016394}
    };

    float LW[2][5] = {
        {-16.870426, -6.9068947, 1.8870002, 6.382451, -2.760659},
        {6.393951, -3.3306818, -1.3395275, -10.625842, 2.7441041}
    };

    float b1[1][2] = {{3.1655333, 3.515173}};
    
    float b2[1][5] = {{-11.002483, 6.1525745, -12.075951, -11.585721, -2.8768528}};

// declaracion de multiplicacion de matrix a la que se le aplicara ReLU
    float IW_X[2][1] = {{0}, {0}}; 
    // multiplication of traspose matrix IW & matrix X 
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
        IW_X[i][0] += IW[j][i] * X[j][0];
    }
}

 // sum del vector de bias b1
    for (int i = 0; i < 2; i++) {
        IW_X[i][0] += b1[0][i];
    }

// vamos a renombrar la matriz a la que le aplicamos ReLU como neuron 
float neuron[2][1];
    
    // aplicamos ReLU a nuestra matrix IW_X para tener el resultado de la capa oculta
     // Aplicar ReLU a la matriz IW_X
    ReLU(IW_X, 2);
    // en este momento se aplica ReLU directamente sibre IW_X y la guarda ahi mismo 
    // voy a renombrarla como neuron para trabajar con ella en el futuro 
    // pasar valores de IW_X a neuron
for (int i = 0; i < 2; i++) {
    neuron[i][0] = IW_X[i][0];
}



}
