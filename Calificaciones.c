/*
*Descripción del programa 
*Autor (es):
*Fecha de creación:
*Fecha de última modificación:
*Correo (s):
*/

#include <stdio.h>

#define MAX_SIZE 10
/*Variables globales*/
/*funciones*/

/*Reservamos Espacio de memoria*/
int main(){
  int calificaciones[MAX_SIZE];
  int k;  
  float suma = 0.0; /* Acumulador, también se puede declarar con con 0.f*/
  

    /*Inicializamos arreglos con cero*/
    for(int k=1; k<MAX_SIZE; k++)
        calificaciones [k] = 0;

    /*Pedimos calificaciones*/
    for(int k=1; k<MAX_SIZE; k++)
        scanf("%d",&calificaciones[k]);

    /*Calculamos la media*/

    for(k=0; k<MAX_SIZE; k++)
        suma += calificaciones [k];

    
  for(int k=1; k<MAX_SIZE; k++);
  suma += calificaciones [k];
  
  suma *= 1.0/MAX_SIZE;
  
  printf("Media: %f");

return 0;
} 
