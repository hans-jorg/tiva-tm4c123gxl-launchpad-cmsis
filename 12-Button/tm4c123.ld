/**
 * @file     tm4c123.ld
 * @brief    Linker script according CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The blinking frequency depends on core frequency
 * @note     Direct access to registers
 * @note     CMSIS library used
 *
 *
 * Configuration for LM4F120H5QR / Tiva TM4C123GH6PM
 * FLASH: start address 0x00       length 0x40000 (256 KB)
 * SRAM:  start address 0x20000000 length 0x8000  (32 KB)
 * STACK: start address 0x20007FFF lenght 0x0     (Final da RAM)
 **/

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 0x00007000
    STACK (rwx) : ORIGIN = 0x20007000, LENGTH = 0x00000800
}

Stack_Size = LENGTH(STACK);
Heap_Size = 0x400;

/*
 * Definição das secoes para C (para C++ existem secoes adicionais)
 * .text  - instrucoes
 * .data  - dados inicializados
 * .bss   - dados não inicializados
 * .stack - ponteiro para inicio da RAM (endereco mais alto da RAM)
 */

SECTIONS
{
    /* Codigo a ser armazenado a partir do endereco 0: tabela de excecoes/interrupcoes
     * Endereco 0: valor inicial do PC
     * Endereco 1: valor inicial do SP
     *
     * Ver Table 2-8: Exception Types
     */
    .text :
    {
          _text_start = .; /* endereco do inicio (=0) */
     KEEP(*(.nvictable))   /* tabela sera configurada para ficar nesta secao */
          *(.text.*)       /* instrucoes logo a seguir */
          *(.rodata.*)     /* constantes logo a seguir */
          _text_end = .;   /* o endereco deste simbolo e o final do codigo */
    } > FLASH              /* tudo armazenado na memoria flash */

    /*
     * Dados inicializados devem ser ficar na RAM, mas os conteudo inicial
     * deve ficar na memoria flash
     */
    .data :
    {
          _data_start = .; /* endereco do inicio */
          *(.data.*)       /* dados */
          *(vtable)        /* vtables gerados pelo compilador C++ */
          _data_end = .;   /* endereco do final */
    } > RAM AT >FLASH      /* configurado para RAM mas armazenado na flash */

    /*
     * Dados não inicializados ficam na RAM.
     * Conteudo inicial deve ser zerado na inicializacao
     */
    .bss :
    {
        _bss_start = .; /* endereco do inicio */
        *(.bss.*)       /* dados nao inicializados */
        *(COMMON)       /* etc */
        _bss_end = .;   /* endereco do final */
        Heap_Mem = .;
    } > RAM

    /*
     * Definicao da pilha consiste do valor inicial do SP
     */
    .stack :
    {
        . = ALIGN(4);
        _stack_end = .; /* Valor inicial */
        _stack_start = . - Stack_Size;
    }>STACK

}
