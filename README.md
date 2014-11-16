server_lader
============

Umbau eines ESP120 zu einem Ladegerät

Trivia:
Das ESP120 ist ein 3kW starkes Servernetzteil mit einer Ausgangsspannung von
standartmäßig 52V und bis zu >50A. Das Netzteil kann allerdings leicht auf
andere Spannungen gemoddet werden um dann z.B. als Akkulader oder sehr potentes
Labornetzteil eingesetzt zu werden.


Was tut dieses Projekt?
dieses Projekt soll die zur Spannungsverstellung nötige Hilfsspannung aus den
aus dem Netzteil zusätzlich erzeugten 5V erzeugen, die Ausgangsspannung regeln
und für einen Ladestrom nach einem Ladeprogramm, das extern eingespeist wird,
einstellen. Das Netzteil muss nur minimal gemoddet werden.


Der Mod:
siehe elweb, eigene Bilder folgen noch.

-------------------------------------------------------------------

Umbau auf eine ander overvoltage protection Spannung:

 Dazu ist es notwendig einen zusätzlichen Widerstand parallel zu den bereits vorhandenen
 auf einer der Platinen einzulöten.
 Im Ordner Bilder ist die Stelle zu sehen, wo der Widerstand eingelötet werden muss. 
 
 Es existieren Messwerte für 3 Spannungen
 61V 35,4k Ohm (-1,5V am Poti)
 60V 42,5k Ohm (-1,3V am Poti)
 59V 51,6k Ohm (-1,1V am Poti)
 
 Wenn man die bereits verbauten Widerstände misst kommt man auf 4,096 Ohm.
 Mit einem parallel geschaltenen 51,6k Ohm Widerstand kommt man so auf einen Widerstand von 3,79kOhm
 
 
 
