/*
Factorial 
Autora: Mario Yahir García Hernández 
marioygh15@gmail.com
created: 07/02/24
last modified: 07/02/24
*/

#include <stdio.h>


int main () {
//declaramos variables
    int num;
    int i; 
    int fact=1; 

//pedimos el valor del factorial 
    printf("Ingrese el numero a calcular: "); 
    scanf("%d", &num); 

    for(i=1;i<=num;i++)
        fact=fact*i;

    printf("El factorial de %d", num);
    printf(" es: %d", fact);


return 0;

}
