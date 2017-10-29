# Test STM32L476VG

+ 4 code eventi gestite da 4 thread con esecuzione periodica
+ 1 task ui input (joystick: push cambia stato UI, up/down cambia asse sensore)
+ 1 task ui output (per lcd)
+ 1 task polling sensori (accelerometro, magnetometro e giroscopio)
+ 1 task di debug/trace (ora blinka il led verde e basta)
+ salvataggio su flash spi stato corrente (quale sensore e quale asse mostrato sulla ui) al cambio stato
+ ricaricamento stato inziale da flash spi