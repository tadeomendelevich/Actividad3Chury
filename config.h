
#ifndef CONFIG_H
#define	CONFIG_H

#define AUTOCONNECTWIFI 1

/**
 * @brief Cadena constante para configurar el Wifi Automaticamente sin enviar datos 
 * desde la PC
 * 
 */
const unsigned char dataCwmode[]="AT+CWMODE_DEF=1\r\n";
const unsigned char dataCwdhcp[]="AT+CWDHCP_DEF=1,1\r\n";
const unsigned char dataCipmode[]="AT+CIPMODE=1\r\n";
const unsigned char dataCipsend[]="AT+CIPSEND\r\n";
const unsigned char dataCipmux[]="AT+CIPMUX=0\r\n";


/**
 * @brief se debe colocar la SSID y el PASS de la red a la que queremos conectarnos
 * 
 * const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"Galaxy A21sE426\",\"tyva7426\"\r\n";
 * const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"FCAL\",\"fcalconcordia.06-2019\"\r\n";
 * 
 * const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"SSID\",\"PASS\"\r\n";
 * 
 */
//const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"ANM2\",\"anm157523\"\r\n";
//const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"FCAL\",\"fcalconcordia.06-2019\"\r\n";
//const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"MEGACABLE FIBRA-2.4G-ckd0\",\"djg19dlk\"\r\n";  // Departamento Tadeo
//const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"iPhone de Tadeo\",\"mendemende\"\r\n";

const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"MEGACABLE FIBRA-2.4G-ckd0\",\"djg19dlk\"\r\n";  // Departamento Tadeo

//const unsigned char dataCwjap[]="AT+CWJAP_DEF=\"iPhone\",\"hola123\"\r\n";
/**
 * @brief se debe colocar la IP de la PC a la que se quiere conectar
 * const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"IP_de_la:PC\",30010,30001,0\r\n";
 * 
 */
//const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"192.168.0.15\",30010,30001,0\r\n";

//const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"172.20.10.2\",30010,30001,0\r\n";
//const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"192.168.100.5\",30010,30001,0\r\n";  //Departamento Tadeo
//const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"172.20.10.3\",30010,30001,0\r\n";

const unsigned char dataCipstart[]="AT+CIPSTART=\"UDP\",\"192.168.100.5\",30010,30001,0\r\n";  //Departamento Tadeo


#define     ALIVEAUTOINTERVAL   15000


#endif