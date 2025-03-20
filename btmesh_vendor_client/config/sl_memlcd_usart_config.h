#ifndef SL_MEMLCD_CONFIG_H
#define SL_MEMLCD_CONFIG_H

// <<< sl:start pin_tool >>>
// <usart signal=TX,CLK> SL_MEMLCD_SPI
// $[USART_SL_MEMLCD_SPI]
#ifndef SL_MEMLCD_SPI_PERIPHERAL                
#define SL_MEMLCD_SPI_PERIPHERAL                 USART1
#endif
#ifndef SL_MEMLCD_SPI_PERIPHERAL_NO             
#define SL_MEMLCD_SPI_PERIPHERAL_NO              1
#endif

// USART1 TX on PC6
#ifndef SL_MEMLCD_SPI_TX_PORT                   
#define SL_MEMLCD_SPI_TX_PORT                    gpioPortC
#endif
#ifndef SL_MEMLCD_SPI_TX_PIN                    
#define SL_MEMLCD_SPI_TX_PIN                     6
#endif
#ifndef SL_MEMLCD_SPI_TX_LOC                    
#define SL_MEMLCD_SPI_TX_LOC                     11
#endif

// USART1 CLK on PC8
#ifndef SL_MEMLCD_SPI_CLK_PORT                  
#define SL_MEMLCD_SPI_CLK_PORT                   gpioPortC
#endif
#ifndef SL_MEMLCD_SPI_CLK_PIN                   
#define SL_MEMLCD_SPI_CLK_PIN                    8
#endif
#ifndef SL_MEMLCD_SPI_CLK_LOC                   
#define SL_MEMLCD_SPI_CLK_LOC                    11
#endif
// [USART_SL_MEMLCD_SPI]$

// <gpio> SL_MEMLCD_SPI_CS
// $[GPIO_SL_MEMLCD_SPI_CS]
#ifndef SL_MEMLCD_SPI_CS_PORT                   
#define SL_MEMLCD_SPI_CS_PORT                    gpioPortD
#endif
#ifndef SL_MEMLCD_SPI_CS_PIN                    
#define SL_MEMLCD_SPI_CS_PIN                     14
#endif
// [GPIO_SL_MEMLCD_SPI_CS]$

// <gpio optional=true> SL_MEMLCD_EXTCOMIN
// $[GPIO_SL_MEMLCD_EXTCOMIN]
#ifndef SL_MEMLCD_EXTCOMIN_PORT                 
#define SL_MEMLCD_EXTCOMIN_PORT                  gpioPortD
#endif
#ifndef SL_MEMLCD_EXTCOMIN_PIN                  
#define SL_MEMLCD_EXTCOMIN_PIN                   13
#endif
// [GPIO_SL_MEMLCD_EXTCOMIN]$

// <<< sl:end pin_tool >>>

#endif
