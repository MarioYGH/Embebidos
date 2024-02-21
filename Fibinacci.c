/*
Factorial 
Autora: Mario Yahir García Hernández 
marioygh15@gmail.com
created: 07/02/24
last modified: 07/02/24
*/

#include <stdio.h>

int main() {
    int MAX_SIZE;
    
    // Solicitar al usuario que ingrese el valor de MAX_SIZE
    printf("Ingrese el tamaño máximo de la serie Fibonacci: ");
    scanf("%d", &MAX_SIZE);
    
    int fibonacci[MAX_SIZE];
    
    // Inicializar los primeros dos elementos de la serie Fibonacci
    fibonacci[0] = 0;
    fibonacci[1] = 1;
    
    // Generar el resto de la serie Fibonacci
    for (int i = 2; i < MAX_SIZE; i++) {
        fibonacci[i] = fibonacci[i - 1] + fibonacci[i - 2];
    }
    
    // Mostrar la serie Fibonacci
    printf("Serie Fibonacci:\n");
    for (int i = 0; i < MAX_SIZE; i++) {
        printf("%d ", fibonacci[i]);
    }
    
    return 0;
}
