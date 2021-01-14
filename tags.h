/******************************************************************************
* CONTROL-UP - Controle de Sistemas Ltda.                                     *
*******************************************************************************
* Projeto : Leitor                                                            *
* Arquivo : tags.H                                                            *
* Descricao : Header do tags.c                                                *
* Plataforma: WINDOWS 10                                                      *
* Compilador: Microchip MPLAB X xc16 v1.24                                    *
* PIC : PIC24FJ256DA210                                                       *
* Versao :                                                                    *
* Data : 28/11/2016                                                           *
* Autor : Luciano Mendes Almeida                                              *
*******************************************************************************/

#ifndef __TAGS_H
#define __TAGS_H

/*--------------------------------------------------------------------*/
/*                    ---+++>>> Constantes <<<+++---	              */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/*                     ---+++>>> Defines <<<+++---	                  */
/*--------------------------------------------------------------------*/

#define FATOR_CORRECAO_TAG 40

/*--------------------------------------------------------------------*/
/*                     ---+++>>> Structs <<<+++---	                  */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/*                    ---+++>>> Prototipagem <<<+++---	              */
/*--------------------------------------------------------------------*/

//void limpa_buffer_tags (void);
void limpa_buffer_tags(unsigned int numDeteccoes);
void troca_bytes(unsigned int numDeteccoes);
//void troca_bytes(void);
void pega_dados_tags(unsigned int numDeteccoes);
//void pega_dados_tags (void);
void trata_tags_excessao(void);
unsigned int verifMaxRSSI(unsigned int tags_detectadas);
unsigned int verifSeEhZonaExclusao(unsigned int indice);
unsigned int verifSeEhOperarioComum(unsigned int indice);
unsigned int verifSeEhCondutor(unsigned int indice);
signed int calculaRSSI(unsigned int indice);
void testeRSSI(void);

/*--------------------------------------------------------------------*/
/*              ---+++>>> Fim do arquivo tags.H <<<+++---             */
/*--------------------------------------------------------------------*/
#endif		
