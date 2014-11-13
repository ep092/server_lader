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


Umbau auf 60V
 Dazu ist es notwendig einen zusätzlichen Widerstand auf einer der Platinen einzulöten.
 Im Ordner Bilder ist die Stelle zu sehen, wo der Widerstand eingelötet werden muss. 
 Aktuell getestet ist ein zusätzlicher 3k Ohm Widerstand, der jedoch die Überspannungsabschaltung
 etwas zu weit anhebt, denn bei 61V lief das Netzteil immer noch.
 TODO: genauen Widerstandswert mit Hilfe eines Potis herausfinden.
 
 
