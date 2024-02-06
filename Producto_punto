/*
producto punto u & v 
Mario Y G
6/2/24
*/

#include <stdio.h>
#define MAX_SIZE 3 

int main(){
   //reservamos espacio en memoria 
   int vec_u [MAX_SIZE];
   int vec_v [MAX_SIZE];
   int vec_res =0;
   int k = 0;  
   // inicializar arreglos 
   for(int k=0; k < MAX_SIZE; k++){
     vec_u[k] = 0;
     vec_v[k] = 0;
   }
   
   for(int k=0; k < MAX_SIZE; k++){ // recuerda tener llaves cuando haces mas de una instruccion en ciclo for 
    scanf("%d", &vec_u[k]);
    scanf("%d", &vec_v[k]);
   }
   for(int k=0; k < MAX_SIZE; k++)
    vec_res += vec_u[k] * vec_v[k];
   
   printf("el producto punto es: %d" , vec_res);
   
    return 0;
}
