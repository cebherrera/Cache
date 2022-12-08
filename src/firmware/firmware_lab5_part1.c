#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// -----------------------------            // número de datos a leer  64kB-48kB= 16kB= 16*1024
#define BITSSTARTUP 0x4000                  // inicio de memoria en uso (48 kB)
#define LIMIT 0x10000                       // Fin de memoria en uso  (64 kB)
#define LED_REGISTERS_MEMORY_ADD 0x10000000 // despliega un valor en los LEDs de 7 SEG
#define LOOP_WAIT_LIMIT 10000000              // delay entre conmutación

// SALIDA para escritura
static void write(uint32_t next, uint32_t data, int addr)
{
    *((volatile uint32_t *)addr) = next; // Dirección avanza cada 0x04
    addr = addr + 0x04;
    *((volatile uint32_t *)addr) = data; // se envia un dato a la dirreción
}
// ENTRADA para lectura
static uint32_t read(uint32_t addr)
{
    return *((volatile uint32_t *)addr);
}

// Para sostener la conmutación
void delay()
{
    uint32_t counter = 0;
    while (counter < LOOP_WAIT_LIMIT)
        counter++;
    counter = 0;
}

// Función principal
void main(void)
{
    char valid = 1;                                           // bandera
    uint32_t data;                                            // variable que contenga el dato
    uint32_t address = BITSSTARTUP;                           // Dirección de Inicio
    // uint32_t direction_stored = address + 0x04;            // dato a guardar: dirrección más 4
    uint32_t next_add_stored = address + 0x08;                                                         // dato a guardar: dirrección más 8
    uint32_t out1 = 0, out2 = 0, out3 = 0, out4 = 0, out5 = 0, out6 = 0, out7 = 0, out8 = 0, out9 = 0; // vañlores iniciales

    //*outbyte = 0x0000;              //validación en 1
    while (valid)
    {
        //*((volatile uint32_t *)LED_REGISTERS_MEMORY_ADD) = 0xF;
        for (uint32_t i = 0; i <  6*1024; i++) // repite los 6 elementos de la tabala por 1024 veces
        {
            write(next_add_stored, i, address); // se manda a escribir
            address = address + 0x08;
            next_add_stored = address + 0x08;
            // printf("%u\n", i); // imprime entero sin signo
        }
        address = BITSSTARTUP; // Siguiente Dirección

        for (uint32_t i = 0; i < 6*1024; i++)
        {
            data = read(address + 0x04); // Para extraer la lectura de los datos
            if ((data % 2) == 0)           // si son pares
            {
                out1 = out2;
                out2 = out3;
                out3 = out4;
                out4 = out5;
                out5 = out6;
                out6 = out7;
                out7 = out8;
                out8 = out9;
                out9 = data;
                // los 8 datos
            }
            address = read(address);
        }

        // Se pasan los datos a los leds
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out1;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out2;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out3;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out4;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out5;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out6;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out7;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out8;
        delay();
        *((uint32_t *)LED_REGISTERS_MEMORY_ADD)=out9;
        delay();
        valid = 0;
    }
}
