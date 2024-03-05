/*
FUNCIONES, factorial, producto punto, fibonacci
*/

#include <stdio.h>
#define size 5

//functions declaradas//
long int factorial (int num); 
float prod_punt(float vec_u[],float vec_v[]);
void fibonacci(int n);

//main function//
int main () {

//para Factorial 
    int num;
    long int result_fact;
    printf("CALCULO DE FACTORIAL \n");
 //pedir el valor a calcular 
    printf("Ingrese el numero a calcular: "); 
    scanf("%d", &num); 
    result_fact=factorial(num);
    printf("El factorial de %d es: %ld\n", num, result_fact);

//para Producto Punto 
    float vec_u[size];
    float vec_v[size]; 
    float result_pp=0;
    int i;
    //pedir datos
    printf("\nCALCULO DE PRODUCTO PUNTO\n");
    printf("Introduce el vector u: \n" );
    for (i=0; i<size; i++){
        scanf("%f", &vec_u[i]);
    }
    printf("Introduce el vector v: \n" );
    for(i=0; i<size;i++){
        scanf("%f", &vec_v[i]);
    }
    result_pp=prod_punt(vec_u, vec_v);
    printf("El producto punto es: %.2f\n", result_pp);

//para Fibonacci
    int n;
    printf("\nCALCULO SERIE DE FIBONACCI\n"); 
    printf("Ingrese el número de términos: ");
    scanf("%d", &n);
    fibonacci(n);


    return 0;
}

//definition de funciones//
/FACTORIAL/
long int factorial(int num){
    int i; 
    int fact=1; 
    for(i=1;i<=num;i++){
        fact=fact*i;
    }
    return fact;
}
/PRODUCTO PUNTO/
float prod_punt(float vec_u[], float vec_v[]){
    float prod_punt=0; 
    int i;
    for (i=0; i<size; i++){
        prod_punt+=vec_u[i]*vec_v[i];
    }
    return prod_punt;
}
/FIBONACCI/
void fibonacci (int n){
    int ant = 0, act = 1, sig;
    int i; 
    printf("La serie de Fibonacci de %d términos es: ", n);
    for (i = 0; i < n; i++) {
        printf("%d, ", ant); // Print ant inside the loop
        sig = ant + act;
        ant = act;
        act = sig;
    }
}
